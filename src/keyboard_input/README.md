# keyboard_input

用于在 PC 端通过键盘控制 Cyberdog2 的一个 ROS2 节点。

- 1 / 2 通过 motion_result_cmd 服务触发趴下与站立动作。
- 3 / 4 切换单步控制使用的步态 motion_id。
- 按住 w / s / a / d 时会持续发送运动指令，松开按键超过 `step_duration_ms` 后自动停止。
- 按住 j / l 时会持续发送角速度指令完成左转与右转，从而改变机器人朝向。
- 空格键优先级最高，会立即打断当前移动并发送停止指令。
- 机器人命名空间可通过参数 `namespace` 传入，或通过 namespace_config.yaml 配置，例如 cyberdog_1、cyberdog_2。

数据依赖包protocol

初次构建：

```bash
cd /cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-up-to keyboard_input
```

通过参数控制指定机器人：

```bash
ros2 run keyboard_input keyboard_input_node --ros-args -p namespace:={namespace}
# 例如
ros2 run keyboard_input keyboard_input_node --ros-args -p namespace:=cyberdog_1
```


