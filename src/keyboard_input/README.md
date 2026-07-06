# keyboard_input

用于在 PC 端通过键盘控制 Cyberdog2 的一个 ROS2 节点。

- 1 / 2 通过 motion_result_cmd 服务触发趴下与站立动作。
- 3 / 4 切换单步控制使用的步态 motion_id。
- 按住 w / s / a / d 时会持续发送运动指令，松开按键超过 `step_duration_ms` 后自动停止。
- 按住 j / l 时会持续发送角速度指令完成左转与右转，从而改变机器人朝向。
- 空格键优先级最高，会立即打断当前移动并发送停止指令。
- 机器人命名空间可通过参数 `namespace` 传入，或通过 namespace_config.yaml 配置，例如 cyberdog_1、cyberdog_2。

数据依赖包protocol

## 运行前环境（必须）

PC 端需配置 CycloneDDS 多机发现，否则 `motion_result_cmd` 会间歇性报「服务不可用」（`ros2 service list` 偶尔能看到但节点连不上）。

```bash
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/code/cyberdog2_pc_ws/src/env/cyclonedds.xml
export ROS_DOMAIN_ID=42
ros2 daemon stop && ros2 daemon start
```

`env/cyclonedds.xml` 的 `<Peers>` 中填入**所有**待控机器人的 WiFi IP（每台机器人各一个 `<Peer>`）。详见 [doc/多机通信设置.md](../../doc/多机通信设置.md)。

### 多机器人键盘控制

DDS 配置是**整网共享**的，不是「一个 Peer 对应一台机器」。同一套 `CYCLONEDDS_URI` + `ROS_DOMAIN_ID=42` 下，可同时发现 `cyberdog_1`、`cyberdog_2` 等全部机器人。

每个键盘节点用 `namespace` 参数指定控制对象，**各开一个终端**（stdin 不能共用）：

```bash
# 终端 1：控制 cyberdog_1
ros2 run keyboard_input keyboard_input_node --ros-args -p namespace:=cyberdog_1

# 终端 2：控制 cyberdog_2
ros2 run keyboard_input keyboard_input_node --ros-args -p namespace:=cyberdog_2
```

启动前确认两台服务均可见：

```bash
ros2 service list | grep motion_result_cmd
ros2 run keyboard_input keyboard_input_node --ros-args -p namespace:=cyberdog_1
```

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


