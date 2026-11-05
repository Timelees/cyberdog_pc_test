# 右前腿踢球动作：实机调用参考

本文面向上层完整链路集成，记录当前实机验证通过的调用顺序。实际发送和监控代码见同目录 `send_fr_kick_robot.py`。任何实机动作前，确认周围无人、急停可用、右前腿前方无障碍物。

## 调用顺序

```text
1. /cyberdog_1/motion_result_cmd
   MotionResultCmd(motion_id=111, cmd_source=APP)  -> 恢复站立
2. /cyberdog_1/motion_status
   确认 motion_id=111 且 order_process_bar >= 95
3. /cyberdog_1/motion_queue_cmd
   MotionQueueCustomCmd(cmds=[MotionCustomCmd(mode=62, gait_id=88, ...)])
4. /cyberdog_1/motion_status
   确认先离开 motion_id=111，动作结束后再回到 111
```

要点：`111` 是 Recovery Stand 的 `motion_id`。踢球不是另一个 `motion_id`，而是通过队列服务提交 `mode=62`、`gait_id=88` 的自定义步态命令。两种请求使用不同服务，不能互换。

## 1. 站立请求：motion_id 111

- 服务：`/cyberdog_1/motion_result_cmd`
- 类型：`protocol/srv/MotionResultCmd`
- `motion_id = 111`
- `cmd_source = MotionResultCmd.Request.APP`

调用方应先等待服务可用，再调用并等待响应。检查 `response.result`；失败时记录 `response.code` 和 `response.motion_id`，不要继续发踢球请求。服务返回成功后还要用状态话题确认站立完成，不能仅凭服务返回判断控制器已就绪。

```python
client = node.create_client(MotionResultCmd, "/cyberdog_1/motion_result_cmd")
if not client.wait_for_service(timeout_sec=8.0):
    raise RuntimeError("motion_result_cmd unavailable")
request = MotionResultCmd.Request()
request.motion_id = 111
request.cmd_source = MotionResultCmd.Request.APP
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=25.0)
if not future.done() or future.result() is None:
    raise RuntimeError("Recovery Stand timed out")
response = future.result()
if not response.result:
    raise RuntimeError(f"Recovery Stand rejected: code={response.code}")
```

当前发送器随后等待 `/cyberdog_1/motion_status`，直到 `motion_id == 111` 且 `order_process_bar >= 95`，最长等待 3 秒以确认状态话题已同步。

## 2. 踢球请求：队列服务 mode 62 / gait 88

- 服务：`/cyberdog_1/motion_queue_cmd`
- 类型：`protocol/srv/MotionQueueCustomCmd`
- `request.cmds`：包含一个 `protocol/msg/MotionCustomCmd`
- 当前验证选择：`mode = 62`、`gait_id = 88`

当前脚本设置的其余字段如下。这些零值是请求字段占位/默认输入；踢腿轨迹来自机器人端对应的 gait 参数配置，不是通过 `foot_pose` 或 `step_height` 在这里指定：

| 字段 | 当前值 |
|---|---|
| `contact` | `0x0F` |
| `vel_des` | `[0.0] * 3` |
| `rpy_des` | `[0.0] * 3` |
| `pos_des` | `[0.0] * 3` |
| `acc_des` | `[0.0] * 6` |
| `ctrl_point` | `[0.0] * 3` |
| `foot_pose` | `[0.0] * 6` |
| `step_height` | `[0.0] * 2` |

```python
command = MotionCustomCmd()
command.mode = 62
command.gait_id = 88
command.contact = 0x0F
command.vel_des = [0.0] * 3
command.rpy_des = [0.0] * 3
command.pos_des = [0.0] * 3
command.acc_des = [0.0] * 6
command.ctrl_point = [0.0] * 3
command.foot_pose = [0.0] * 6
command.step_height = [0.0] * 2

request = MotionQueueCustomCmd.Request()
request.cmds = [command]
client = node.create_client(
    MotionQueueCustomCmd, "/cyberdog_1/motion_queue_cmd"
)
if not client.wait_for_service(timeout_sec=8.0):
    raise RuntimeError("motion_queue_cmd unavailable")
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
if not future.done() or future.result() is None:
    raise RuntimeError("kick queue request timed out")
response = future.result()
if not response.result:
    raise RuntimeError(f"kick queue rejected: code={response.code}")
```

`response.result == true` 只表示队列请求被接受，不保证动作已经开始或完成。提交请求前先创建状态订阅，避免漏掉动作起始状态。

## 3. 状态确认和故障处理

状态源为 `/cyberdog_1/motion_status`，消息 `protocol/msg/MotionStatus`：

- 站立就绪：`motion_id == 111` 且 `order_process_bar >= 95`。
- 踢球启动：状态 `motion_id` 离开 `111`。若服务成功返回但状态一直是 `111`，动作并未启动，必须报告失败，不能当作已踢球。
- 踢球完成：启动后状态回到 `motion_id == 111`，且 `order_process_bar >= 95`。
- `switch_status` 应是 `MotionStatus.NORMAL` 或 `MotionStatus.TRANSITIONING`；其它值应作为控制器异常处理。
- 电机错误依脚本按有效位检测：`value & 0x3FFFFFFF`。不要仅因 motor_error 值为 `-2147483648`（符号位）就误报电机故障。
- 动作异常或超时后，发送器应尝试请求 `motion_id=111` 恢复站立，并独立记录恢复结果。不要自动无限重试踢球队列。

## 4. ROS 2 / DDS 环境

当前发送器通过 SSH 在机器人本机执行 ROS 客户端，并加载以下配置（须与实机运行域一致）：

```bash
source /opt/ros2/cyberdog/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/mi/cyclonedds.xml
```

因此开发机 ROS CLI 看不到服务，并不能直接证明机器人服务不存在。应在机器人端加载相同 ROS/DDS 环境后再检查：

```bash
source /opt/ros2/cyberdog/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/mi/cyclonedds.xml
ros2 service list | grep -E 'motion_(result|queue)_cmd'
ros2 topic echo /cyberdog_1/motion_status protocol/msg/MotionStatus
```

## 5. 曾遇到的队列不启动问题

历史现象是站立成功，队列调用返回后状态仍停在 `motion_id=111`。排查到 motion_manager 队列急停有效性判断曾写反。正确逻辑应为：

```cpp
inline bool IsStateValid()
{
  return !estop_;
}
```

调用方在 `!IsStateValid()` 时拒绝队列，所以正常未急停状态（`estop_ == false`）必须得到 `true`。修复后需确认机器人运行的是包含修正的 `motion_manager` 二进制；只更新调用脚本并不能修复机器人端的队列判断。

队列没启动时依次检查：服务名和请求类型是否正确、`response.result/code`、motion_manager 的 `Receive QueueCmd` 与队列 TOML/底层日志、状态是否离开 111、机器人端运行的二进制版本及 DDS 域。若状态未离开 111，不要继续调步态幅度，那时命令还没有执行到动作控制器。

## 6. 关联文件

- 实机端到端发送和监控：[`send_fr_kick_robot.py`](send_fr_kick_robot.py)
- 右前腿 gait 参数：`cyberdog_locomotion/control/motion_list/cyberdog2/user/parameter/cyberdog2_user88_FR_lift.toml`
- 队列急停检查：`motion/motion_manager/include/motion_manager/motion_decision.hpp`
- 队列决策：`motion/motion_manager/src/motion_decision.cpp`
- 队列执行及响应：`motion/motion_manager/src/motion_handler.cpp`
