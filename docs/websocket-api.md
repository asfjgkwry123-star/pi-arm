# Pi Arm WebSocket API

默认地址为 `ws://<robot-ip>:8765`。消息均为 UTF-8 JSON；角度单位为度，角速度为度/秒，笛卡尔位置为毫米，线速度为毫米/秒，姿态及角速度为度制。

## 通用消息

请求：

```json
{"type":"req","msg_id":1,"cmd":"enable_motor","params":{"motor_id":0}}
```

响应：

```json
{"type":"rsp","msg_id":1,"code":0,"ms":"ok","data":{}}
```

`msg_id` 由客户端生成并在响应中原样返回。错误码：`0` 成功，`1001` JSON 非法，`1002` 请求结构非法，`1003` 命令不支持，`1004` 参数非法，`2001` 状态或服务不可用，`2002` 执行失败，`2003` 已有运动任务。

运动命令的成功响应表示 manager 已接受任务，不表示机械臂已经到位。任务生命周期通过周期 `state` 消息中的 `robot_state` 和 `task` 字段观察。

## 电机管理

- `enable_motor`：`{"motor_id":0..6}`，0 表示全部。仅机械臂为 `DISABLED`、无任务并连续静止时允许。
- `disable_motor`：`{"motor_id":0..6}`。仅 `READY` 时允许。
- `reset_motor`：`{"motor_id":0..6}`。除 `DISCONNECTED` 外可直接请求。
- `set_zero`：`{"motor_id":0..6}`。仅 `READY` 或 `DISABLED`、无任务并连续静止时允许；该命令写入电机 ROM。

## 运动

- `movej`

```json
{"type":"req","msg_id":10,"cmd":"movej","params":{"target":[0,0,0,0,0,0],"vel":10,"acc":20}}
```

- `movejs`：`targets` 为 N 组六轴角度；`dt` 为相邻点间隔毫秒。

```json
{"type":"req","msg_id":11,"cmd":"movejs","params":{"targets":[[0,0,0,0,0,0],[1,0,0,0,0,0]],"dt":100}}
```

- `movel`：`target=[x,y,z,rx,ry,rz]`，使用 MoveIt 笛卡尔路径及碰撞检查。

```json
{"type":"req","msg_id":12,"cmd":"movel","params":{"target":[400,0,300,0,90,0],"linear_vel":20,"linear_acc":40,"angular_vel":10,"angular_acc":20}}
```

- `move`：立即提交六轴受控目标，仍经过 manager 仲裁。

```json
{"type":"req","msg_id":13,"cmd":"move","params":{"targets":[0,0,0,0,0,0],"max_speed":10}}
```

- `motor_move`：单轴受控目标。

```json
{"type":"req","msg_id":14,"cmd":"motor_move","params":{"joint_id":1,"target_angle":20,"max_speed":10}}
```

- `motor_stop`：停止当前统一运动任务并保持反馈位置；无活动任务时为指定轴提交保持目标。

```json
{"type":"req","msg_id":15,"cmd":"motor_stop","params":{"joint_id":1,"max_speed":10}}
```

- `stop_motion`：取消当前 MoveIt/轨迹 goal 并保持当前位置，参数为空。

## 状态推送

服务端默认每 100 ms 推送：

```json
{
  "type":"state",
  "msg_id":1,
  "current_joint_angles":[0,0,0,0,0,0],
  "current_speeds":[0,0,0,0,0,0],
  "enabled":[true,true,true,true,true,true],
  "has_error":[false,false,false,false,false,false],
  "in_motion":[false,false,false,false,false,false],
  "last_update_timestamps":[0,0,0,0,0,0],
  "all_enabled":true,
  "any_error":false,
  "any_in_motion":false,
  "robot_state":"READY",
  "task":{"task_id":1,"command":"movej","status":2,"progress":1.0,"code":0,"message":"ok"},
  "pose":{"x":0,"y":0,"z":0,"rx":0,"ry":0,"rz":0}
}
```

`robot_state` 取值为 `DISABLED`、`READY`、`RUNNING`、`FAULT`、`DISCONNECTED`。`pose` 是 `base_link -> tool0`；TF 尚不可用时各字段为 `null`。
