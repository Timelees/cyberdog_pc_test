# keyboard_input使用

用于在 PC 端通过键盘控制 Cyberdog2 的一个 ROS2 节点。

- 1 / 2 通过 motion_result_cmd 服务触发趴下与站立动作。
- 3 / 4 只选择下一条 ServoCmd 使用的慢走 / 快走步态，不会额外发送零速 MotionResultCmd。
- 按住 w / s / a / d 时会持续发送运动指令，松开按键超过 `step_duration_ms` 后自动停止。默认值为 `800 ms`，需大于桌面环境的首次按键自动重复延迟，否则按住键时会出现 `START -> END -> START`。
- 按住 j / l 时会持续发送角速度指令完成左转与右转，从而改变机器人朝向。
- 空格键优先级最高，会立即打断当前移动并发送停止指令。
- 机器人命名空间可通过参数 `namespace` 传入，或通过 namespace_config.yaml 配置，例如 cyberdog_1、cyberdog_2。

数据依赖包protocol

## 推荐启动方式

不要直接在未配置 DDS 环境的终端执行 `ros2 run`。使用工作区启动脚本，它会统一设置 ROS Domain、CycloneDDS 配置和机器人命名空间：

```bash
bash src/env/start_keyboard_input.sh
ROBOT_NAMESPACE=cyberdog_2 bash src/env/start_keyboard_input.sh
```

## 运行前环境（必须）

PC 端需配置 CycloneDDS 多机发现，详细见
[doc/多机通信设置.md](../../doc/多机通信设置.md)。

```bash
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/cyclonedds.xml  # 修改为实际路径
export ROS_DOMAIN_ID=42
```

## 编译

初次构建：

```bash
cd /cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-up-to keyboard_input
```

### 多机器人键盘控制

每个键盘节点用 `namespace` 参数指定控制对象，**各开一个终端**

```bash
# 终端 1：控制 cyberdog_1
bash src/env/start_keyboard_input.sh
# 或指定其他机器人：
ROBOT_NAMESPACE=cyberdog_2 bash src/env/start_keyboard_input.sh

# 终端 2：控制 cyberdog_2
ros2 run keyboard_input keyboard_input_node --ros-args -p namespace:=cyberdog_2
```

