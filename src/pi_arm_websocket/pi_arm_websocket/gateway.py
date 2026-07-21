from __future__ import annotations

import asyncio
import json
import math
import threading
from itertools import count
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from pi_arm_interfaces.action import DirectMove, MoveJ, MoveJs, MoveL
from pi_arm_interfaces.msg import RobotState
from pi_arm_interfaces.srv import ManageMotor, StopMotion
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import websockets
from websockets.exceptions import ConnectionClosed

from .protocol import (
    CODE_COMMAND_FAILED,
    CODE_INVALID_PARAMS,
    CODE_MOTION_BUSY,
    CODE_STATE_UNAVAILABLE,
    ProtocolError,
    Request,
    integer_param,
    parse_request,
    required_float,
    required_vector,
    response,
)

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


class RosBridge(Node):
    def __init__(self) -> None:
        super().__init__("pi_arm_websocket")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8765)
        self.declare_parameter("state_push_interval_ms", 100)
        self.declare_parameter("max_size_bytes", 32 * 1024 * 1024)
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter(
            "joint_names",
            ["joint_01", "joint_02", "joint_03", "joint_05", "joint_06", "joint_09"],
        )
        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.push_interval = max(
            0.01, float(self.get_parameter("state_push_interval_ms").value) / 1000.0
        )
        self.max_size = max(1024 * 1024, int(self.get_parameter("max_size_bytes").value))
        self.timeout = float(self.get_parameter("request_timeout_sec").value)
        if not self.host or not 1 <= self.port <= 65535 or self.timeout <= 0.0:
            raise ValueError("WebSocket host、port 或 request_timeout_sec 参数非法")
        self.default_joint_names = list(self.get_parameter("joint_names").value)
        self._state: RobotState | None = None
        self._state_lock = threading.Lock()
        # ClientGoalHandle is unhashable in Jazzy rclpy; track with a list.
        self._active_goals: list[Any] = []
        self._goal_lock = threading.Lock()

        self.create_subscription(RobotState, "/pi_arm/state", self._on_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.service_clients = {
            "enable_motor": self.create_client(ManageMotor, "/pi_arm/enable_motor"),
            "disable_motor": self.create_client(ManageMotor, "/pi_arm/disable_motor"),
            "reset_motor": self.create_client(ManageMotor, "/pi_arm/reset_motor"),
            "set_zero": self.create_client(ManageMotor, "/pi_arm/set_zero"),
            "stop_motion": self.create_client(StopMotion, "/pi_arm/stop_motion"),
        }
        self.action_clients = {
            "movej": ActionClient(self, MoveJ, "/pi_arm/movej"),
            "movejs": ActionClient(self, MoveJs, "/pi_arm/movejs"),
            "movel": ActionClient(self, MoveL, "/pi_arm/movel"),
            "direct": ActionClient(self, DirectMove, "/pi_arm/direct_move"),
        }

    def _on_state(self, message: RobotState) -> None:
        with self._state_lock:
            self._state = message

    def state_snapshot(self) -> RobotState | None:
        with self._state_lock:
            return self._state

    async def wait_future(self, future: Any, timeout: float | None = None) -> Any:
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        ok = await asyncio.to_thread(completed.wait, timeout or self.timeout)
        if not ok:
            raise ProtocolError(CODE_COMMAND_FAILED, "ROS 请求超时")
        exception = future.exception()
        if exception is not None:
            raise ProtocolError(CODE_COMMAND_FAILED, f"ROS 请求失败: {exception}")
        return future.result()

    async def call_service(self, command: str, request: Any) -> Any:
        client = self.service_clients[command]
        available = await asyncio.to_thread(client.wait_for_service, self.timeout)
        if not available:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "manager 服务不可用")
        result = await self.wait_future(client.call_async(request))
        if not result.success:
            code = int(result.code) if result.code else CODE_COMMAND_FAILED
            raise ProtocolError(code, result.message)
        return result

    async def send_goal(self, action_name: str, goal: Any) -> int:
        state = self.state_snapshot()
        if state is None:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "机械臂状态尚未就绪")
        if state.state == RobotState.RUNNING:
            raise ProtocolError(CODE_MOTION_BUSY, "已有运动任务正在运行")
        if state.state != RobotState.READY:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, f"当前状态不允许运动: {state.state_name}")
        client = self.action_clients[action_name]
        available = await asyncio.to_thread(client.wait_for_server, self.timeout)
        if not available:
            raise ProtocolError(CODE_STATE_UNAVAILABLE, "manager action 不可用")
        feedback_ready = threading.Event()
        task_id_box: list[int] = []

        def feedback_callback(message: Any) -> None:
            if not task_id_box:
                task_id_box.append(int(message.feedback.task_id))
                feedback_ready.set()

        goal_handle = await self.wait_future(
            client.send_goal_async(goal, feedback_callback=feedback_callback)
        )
        if not goal_handle.accepted:
            raise ProtocolError(CODE_MOTION_BUSY, "运动目标被拒绝")
        result_future = goal_handle.get_result_async()
        with self._goal_lock:
            self._active_goals.append(goal_handle)

        def release(_: Any) -> None:
            with self._goal_lock:
                try:
                    self._active_goals.remove(goal_handle)
                except ValueError:
                    pass

        result_future.add_done_callback(release)
        received = await asyncio.to_thread(feedback_ready.wait, self.timeout)
        if not received:
            raise ProtocolError(CODE_COMMAND_FAILED, "manager 未返回任务 ID")
        return task_id_box[0]


class CommandGateway:
    def __init__(self, bridge: RosBridge) -> None:
        self.bridge = bridge

    async def handle(self, raw: str) -> str:
        msg_id = 0
        try:
            request = parse_request(raw)
            msg_id = request.msg_id
            data = await self.dispatch(request)
            return response(msg_id, 0, "ok", data)
        except ProtocolError as exc:
            return response(exc.msg_id or msg_id, exc.code, exc.message)
        except Exception as exc:  # Keep the wire protocol stable at the process boundary.
            self.bridge.get_logger().error(f"命令处理失败: {exc}")
            return response(msg_id, CODE_COMMAND_FAILED, f"命令执行失败: {exc}")

    async def dispatch(self, request: Request) -> dict[str, Any]:
        command, params = request.command, request.params
        if command in {"enable_motor", "disable_motor", "reset_motor", "set_zero"}:
            motor_id = integer_param(params, "motor_id", allow_zero=True)
            ros_request = ManageMotor.Request()
            ros_request.motor_id = motor_id
            await self.bridge.call_service(command, ros_request)
            motor_ids = [motor_id] if motor_id else list(range(1, len(self._joint_names()) + 1))
            return {"cmd": command, "motor_ids": motor_ids}
        if command == "stop_motion":
            state = self.bridge.state_snapshot()
            task_id = int(state.task.task_id) if state is not None else None
            result = await self.bridge.call_service(command, StopMotion.Request())
            return {
                "cmd": command,
                "stopped": result.message != "no active motion",
                "task_id": task_id if result.message != "no active motion" else None,
            }
        if command == "movej":
            return await self._movej(params)
        if command == "movejs":
            return await self._movejs(params)
        if command == "movel":
            return await self._movel(params)
        if command == "move":
            targets = self._joint_vector(params, "targets")
            speed = required_float(params, "max_speed", positive=True)
            await self._direct(self._joint_names(), targets, speed)
            return {"cmd": "move"}
        if command == "motor_move":
            return await self._motor_move(params, stop=False)
        if command == "motor_stop":
            return await self._motor_move(params, stop=True)
        raise ProtocolError(CODE_INVALID_PARAMS, f"未实现命令: {command}")

    def _joint_names(self) -> list[str]:
        state = self.bridge.state_snapshot()
        if state and state.hardware.joint_names:
            return list(state.hardware.joint_names)
        return list(self.bridge.default_joint_names)

    def _joint_vector(self, params: dict[str, Any], name: str) -> list[float]:
        return required_vector(params, name, len(self._joint_names()))

    async def _movej(self, params: dict[str, Any]) -> dict[str, Any]:
        target = self._joint_vector(params, "target")
        velocity = required_float(params, "vel", positive=True)
        acceleration = required_float(params, "acc", positive=True)
        goal = MoveJ.Goal()
        goal.joint_names = self._joint_names()
        goal.positions = [value * DEG_TO_RAD for value in target]
        goal.max_velocity = velocity * DEG_TO_RAD
        goal.max_acceleration = acceleration * DEG_TO_RAD
        await self.bridge.send_goal("movej", goal)
        return {"cmd": "movej", "target": target, "vel": velocity, "acc": acceleration}

    async def _movejs(self, params: dict[str, Any]) -> dict[str, Any]:
        targets_value = params.get("targets")
        if not isinstance(targets_value, list) or not targets_value:
            raise ProtocolError(CODE_INVALID_PARAMS, "targets 必须是非空数组")
        if len(targets_value) > 50_000:
            raise ProtocolError(CODE_INVALID_PARAMS, "targets 不能超过 50000 组")
        dt_ms = integer_param(params, "dt")
        names = self._joint_names()
        trajectory = JointTrajectory()
        trajectory.joint_names = names
        targets: list[list[float]] = []
        for index, value in enumerate(targets_value):
            target = required_vector({"target": value}, "target", len(names))
            targets.append(target)
            point = JointTrajectoryPoint()
            point.positions = [item * DEG_TO_RAD for item in target]
            nanoseconds = (index + 1) * dt_ms * 1_000_000
            point.time_from_start = Duration(
                sec=nanoseconds // 1_000_000_000,
                nanosec=nanoseconds % 1_000_000_000,
            )
            trajectory.points.append(point)
        goal = MoveJs.Goal()
        goal.trajectory = trajectory
        task_id = await self.bridge.send_goal("movejs", goal)
        return {"cmd": "movejs", "task_id": task_id, "point_count": len(targets), "dt": dt_ms}

    async def _movel(self, params: dict[str, Any]) -> dict[str, Any]:
        target = required_vector(params, "target", 6)
        linear_velocity = required_float(params, "linear_vel", positive=True)
        linear_acceleration = required_float(params, "linear_acc", positive=True)
        angular_velocity = required_float(params, "angular_vel", positive=True)
        angular_acceleration = required_float(params, "angular_acc", positive=True)
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            target[0] / 1000.0,
            target[1] / 1000.0,
            target[2] / 1000.0,
        )
        qx, qy, qz, qw = quaternion_from_rpy(
            target[3] * DEG_TO_RAD, target[4] * DEG_TO_RAD, target[5] * DEG_TO_RAD
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = MoveL.Goal()
        goal.target = pose
        goal.max_linear_velocity = linear_velocity / 1000.0
        goal.max_linear_acceleration = linear_acceleration / 1000.0
        goal.max_angular_velocity = angular_velocity * DEG_TO_RAD
        goal.max_angular_acceleration = angular_acceleration * DEG_TO_RAD
        await self.bridge.send_goal("movel", goal)
        return {
            "cmd": "movel",
            "target": target,
            "linear_vel": linear_velocity,
            "linear_acc": linear_acceleration,
            "angular_vel": angular_velocity,
            "angular_acc": angular_acceleration,
        }

    async def _direct(
        self, names: list[str], targets_deg: list[float], speed_deg_s: float
    ) -> None:
        goal = DirectMove.Goal()
        goal.joint_names = names
        goal.positions = [value * DEG_TO_RAD for value in targets_deg]
        goal.max_velocity = speed_deg_s * DEG_TO_RAD
        await self.bridge.send_goal("direct", goal)

    async def _motor_move(self, params: dict[str, Any], *, stop: bool) -> dict[str, Any]:
        joint_id = integer_param(params, "joint_id")
        names = self._joint_names()
        if joint_id > len(names):
            raise ProtocolError(CODE_INVALID_PARAMS, "joint_id 不在当前机械臂配置中")
        speed = float(integer_param(params, "max_speed", default=10))
        if stop:
            state = self.bridge.state_snapshot()
            if state is None or joint_id > len(state.hardware.positions):
                raise ProtocolError(CODE_STATE_UNAVAILABLE, "当前关节状态不可用")
            angle = state.hardware.positions[joint_id - 1] * RAD_TO_DEG
            if state.state == RobotState.RUNNING:
                await self.bridge.call_service("stop_motion", StopMotion.Request())
                return {
                    "cmd": "motor_stop",
                    "joint_id": joint_id,
                    "target_angle": angle,
                    "max_speed": speed,
                }
        else:
            angle = required_float(params, "target_angle")
        await self._direct([names[joint_id - 1]], [angle], speed)
        return {
            "cmd": "motor_stop" if stop else "motor_move",
            "joint_id": joint_id,
            "target_angle": angle,
            "max_speed": speed,
        }


class WebSocketGateway:
    def __init__(self, bridge: RosBridge) -> None:
        self.bridge = bridge
        self.commands = CommandGateway(bridge)
        self.clients: set[Any] = set()
        self.counter = count(1)

    async def run(self) -> None:
        async with websockets.serve(
            self._client,
            self.bridge.host,
            self.bridge.port,
            max_size=self.bridge.max_size,
            ping_interval=20,
            ping_timeout=20,
        ):
            await self._push_loop()

    async def _client(self, websocket: Any) -> None:
        self.clients.add(websocket)
        try:
            async for message in websocket:
                raw = message if isinstance(message, str) else "{}"
                await websocket.send(await self.commands.handle(raw))
        except ConnectionClosed:
            return
        finally:
            self.clients.discard(websocket)

    async def _push_loop(self) -> None:
        while rclpy.ok():
            await asyncio.sleep(self.bridge.push_interval)
            if not self.clients:
                continue
            payload = self._state_message()
            dead = []
            for client in tuple(self.clients):
                try:
                    await client.send(payload)
                except ConnectionClosed:
                    dead.append(client)
            for client in dead:
                self.clients.discard(client)

    def _state_message(self) -> str:
        state = self.bridge.state_snapshot()
        if state is None:
            body: dict[str, Any] = {
                "current_joint_angles": [],
                "current_speeds": [],
                "enabled": [],
                "has_error": [],
                "in_motion": [],
                "last_update_timestamps": [],
                "all_enabled": False,
                "any_error": False,
                "any_in_motion": False,
                "robot_state": "DISCONNECTED",
            }
        else:
            hardware = state.hardware
            stamp = float(hardware.stamp.sec) + float(hardware.stamp.nanosec) / 1e9
            body = {
                "current_joint_angles": [v * RAD_TO_DEG for v in hardware.positions],
                "current_speeds": [v * RAD_TO_DEG for v in hardware.velocities],
                "enabled": list(hardware.enabled),
                "has_error": list(hardware.errors),
                "in_motion": list(hardware.moving),
                "last_update_timestamps": [
                    max(0.0, stamp - age) if fresh and math.isfinite(age) else 0.0
                    for fresh, age in zip(hardware.fresh, hardware.ages_sec)
                ],
                "all_enabled": bool(hardware.enabled) and all(hardware.enabled),
                "any_error": any(hardware.errors) or any(hardware.error_codes),
                "any_in_motion": any(hardware.moving),
                "robot_state": state.state_name,
                "task": {
                    "task_id": state.task.task_id,
                    "command": state.task.command,
                    "status": state.task.status,
                    "progress": state.task.progress,
                    "code": state.task.code,
                    "message": state.task.message,
                },
            }
        body["pose"] = self._pose()
        return json.dumps(
            {"type": "state", "msg_id": next(self.counter), **body},
            ensure_ascii=False,
            separators=(",", ":"),
        )

    def _pose(self) -> dict[str, float | None]:
        try:
            transform = self.bridge.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform
            roll, pitch, yaw = rpy_from_quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
            return {
                "x": transform.translation.x * 1000.0,
                "y": transform.translation.y * 1000.0,
                "z": transform.translation.z * 1000.0,
                "rx": roll * RAD_TO_DEG,
                "ry": pitch * RAD_TO_DEG,
                "rz": yaw * RAD_TO_DEG,
            }
        except TransformException:
            return {"x": None, "y": None, "z": None, "rx": None, "ry": None, "rz": None}


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, ...]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main() -> None:
    rclpy.init()
    bridge = RosBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        asyncio.run(WebSocketGateway(bridge).run())
    except KeyboardInterrupt:
        bridge.get_logger().info("WebSocket 网关收到退出信号")
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
