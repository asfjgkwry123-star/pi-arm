# Pi Arm ROS 2 Jazzy

本工作区提供 Pi Arm 的 ROS 2 Jazzy 描述、SocketCAN 硬件、`ros2_control`、MoveIt 2、业务管理与 WebSocket 接入。主动关节顺序固定为：

`joint_01, joint_02, joint_03, joint_05, joint_06, joint_09`

`joint_04` 模仿 `joint_03`，`joint_07` 模仿 `joint_06`，`joint_08` 以 `-1` 倍模仿 `joint_05`。MoveIt 规划组为 `pi_arm`，末端坐标系为 `tool0`。

## 环境和构建

要求 Ubuntu 24.04、ROS 2 Jazzy、SocketCAN（真实硬件）和支持 OpenGL 的桌面环境（使用 RViz 时）。

```bash
cd ~/pi_arm
bash scripts/install_dependencies.sh
bash scripts/build.sh
source install/setup.bash
```

依赖脚本安装 ROS 包并执行 `rosdep install`；构建脚本使用 `colcon build --symlink-install`。每个新终端都需 source `/opt/ros/jazzy/setup.bash` 和工作区的 `install/setup.bash`。

## 运行

无硬件调试：

```bash
ros2 launch pi_arm_bringup bringup.launch.py hardware_type:=mock
```

无图形界面的 mock：

```bash
ros2 launch pi_arm_bringup bringup.launch.py hardware_type:=mock use_rviz:=false
```

真实 CAN：

```bash
sudo bash scripts/setup_can.sh real can0 1000000
ros2 launch pi_arm_bringup bringup.launch.py \
  hardware_type:=real can_interface:=can0 bitrate:=1000000
```

真机启动安全：电机可先上电（即使驱动自动使能）。硬件层在反馈首次全部 `fresh` 后一次性把 `command` hold 到实测角（之后由控制器接管，轨迹运行期间不再每拍 hold）；且仅在全关节使能且反馈新鲜时进入位置控制。不会把 command 缓冲初值 0 当作目标下发。

仅检查模型：

```bash
ros2 launch pi_arm_description display.launch.py backend:=mock
```

仅启动 MoveIt 节点或 RViz：

```bash
ros2 launch pi_arm_moveit_config move_group.launch.py hardware_type:=mock
ros2 launch pi_arm_moveit_config moveit_rviz.launch.py hardware_type:=mock
```

统一启动参数：

- `hardware_type:=mock|real`：选择确定性软件后端或真实 SocketCAN 后端。
- `can_interface:=can0`：真实硬件使用的 SocketCAN 网卡。波特率由 `scripts/setup_can.sh` 在启动 ROS 前配置；launch 中的 `bitrate` 仅用于保持统一部署参数，驱动不会在线修改已启动的网络接口。
- `use_rviz:=true|false`：是否启动 MoveIt RViz。
- `use_manager:=true|false`、`use_websocket:=true|false`：是否启动业务管理和 WebSocket 代理。

## SocketCAN 与 vcan

真实总线默认使用标准帧、`can0` 和 1 Mbit/s。关闭并重新配置接口：

```bash
sudo bash scripts/setup_can.sh real can0 1000000
```

没有电机时可创建虚拟总线检查传输层：

```bash
sudo bash scripts/setup_can.sh vcan vcan0
ip -details link show vcan0
python3 scripts/vcan_motor_emulator.py vcan0
```

另开终端可用真实 CAN backend 验证单 Socket、协议轮询和管理命令：

```bash
ros2 launch pi_arm_bringup bringup.launch.py \
  hardware_type:=real can_interface:=vcan0 use_rviz:=false
```

`vcan` 不是 ros2_control 的 `mock` 后端：前者配合仓库内电机响应器验证 SocketCAN 和协议层；一般 MoveIt 与控制器联调应优先使用 `hardware_type:=mock`。

## ROS API

主要接口位于 `/pi_arm` 命名空间：

- 状态：`/pi_arm/hardware_state`、`/pi_arm/state`。
- 电机服务：`/pi_arm/enable_motor`、`disable_motor`、`reset_motor`、`set_zero`。
- 运动 Action：`/pi_arm/movej`、`movejs`、`movel`、`direct_move`。
- 停止服务：`/pi_arm/stop_motion`。
- MoveIt/控制器：`/joint_trajectory_controller/follow_joint_trajectory`。
- WebSocket：默认 `ws://0.0.0.0:8765`，JSON 请求由 `pi_arm_websocket` 转换到上述 ROS 接口，状态由 `/pi_arm/state` 推送。

可用 `ros2 interface show pi_arm_interfaces/action/MoveJ`、`ros2 action list`、`ros2 service list` 和 `ros2 topic echo /pi_arm/state` 查询实际字段与运行状态。

完整 WebSocket 消息格式、11 个命令、单位、错误码和状态字段见 [`docs/websocket-api.md`](docs/websocket-api.md)。

## 配置单一数据源

限位与传动配置各自只有一个来源，修改时只改对应文件：

- 关节位置/速度限位、effort：`pi_arm_description/urdf/pi_arm.urdf.xacro` 的 `<limit>` 标签。硬件插件通过 ros2_control 解析的 `HardwareInfo::limits` 读取，manager 通过 MoveIt robot model 读取，两者都不再持有副本。
- 关节加速度限制：`pi_arm_moveit_config/config/joint_limits.yaml`（URDF 无法表达加速度）。bringup 会把它以 `robot_description_planning` 命名空间同时传给 move_group 和 manager。
- 笛卡尔 TCP 上限：两处须手动对齐——（1）manager MoveL 拒绝/scaling：`manager_node.cpp` 中 `kMaxTransVelMps` 等编译期常量；（2）move_group Pilz：`pilz_cartesian_limits.yaml`。当前测试值为线速度 0.1 m/s、线加速度 0.2 m/s²、角速度 10°/s。页面 `control.html` 的 `MOVEL_*` 与之对应。角加速度由 Pilz 推导，页面只读。
- 规划管道：`ompl`（`movej`）与 `pilz_industrial_motion_planner`（`movel` 使用 `LIN`）并存。依赖安装含 `ros-jazzy-pilz-industrial-motion-planner`。
- 电机映射（`motor_id`、`reduction_ratio`、`motor_direction`、`cable_ratio`、`nonlinear`）：URDF `<ros2_control>` 段内每个 `<joint>` 的 `<param>`，由硬件插件逐关节解析，缺失即启动失败。
- 关节名与顺序：URDF `<ros2_control>` 段的 `<joint>` 顺序即主动关节顺序；`controllers.yaml` 中的控制器关节列表须与其保持一致（ros2_control 约定）。

验收：启动日志含 `MoveL TCP caps (compile-time)`；`ros2 param list /move_group --filter cartesian_limits` 应列出 Pilz 四项。

## 安全警告

当前速度 `0.1745329 rad/s`（10 deg/s，URDF `<limit velocity>`）、加速度 `0.3490659 rad/s²`（20 deg/s²，`joint_limits.yaml`）以及 URDF effort（肩部 20 N·m、腕部 10 N·m）均为临时保守值，不是经过认证的机械、电机或减速器额定参数。上线前必须根据电机、减速器、供电、制动距离、负载和结构强度重新标定，并在硬件驱动中实施独立限幅。

真实运行前必须：

1. 确认机械急停、限位、保险和断电措施有效，并清空运动范围。
2. 校验零位、关节方向、传动比、CAN ID、模仿关节方向及每轴软硬限位。
3. 首次上电使用低电流/低速度、无负载和单轴点动；禁止直接执行未经验证的 MoveIt 轨迹。
4. WebSocket 当前默认监听所有网卡且不代表已具备认证、授权或 TLS；只允许部署在受信隔离网络，生产环境须增加反向代理、鉴权和访问控制。
5. `mock` 成功仅说明软件链路可用，不证明真实机械臂安全或动力学可行。
