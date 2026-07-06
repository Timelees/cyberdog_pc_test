# 多机 VIO 坐标统一

PC 端 ROS2 节点，将多台 Cyberdog2 各自输出的 VIO 里程计（`/{namespace}/odom_global`）转发到统一的全局坐标系下，供多机可视化与后续策略使用。

## 功能

- 按配置的机器人命名空间列表，订阅各机 `/{namespace}/odom_global` 话题。
- 转发里程计话题到 `/global_vio/{namespace}/odom`，由下游节点（如 `topic_visualization`等其他踢足球场景功能）统一订阅。


### 数据流

各机器人需先在狗端启动 [apriltag_ros](../apriltag_ros/) 的 AprilTag 识别与 VIO 坐标转换，才会产生 `odom_global`。话题格式说明见 [doc/数据格式.md](../../doc/数据格式.md)。

默认配置下，两台机器人的输入/输出示例：

| 机器人 | 输入 | 输出 |
|--------|------|------|
| cyberdog_1 | `/cyberdog_1/odom_global` | `/global_vio/cyberdog_1/odom` |
| cyberdog_2 | `/cyberdog_2/odom_global` | `/global_vio/cyberdog_2/odom` |

## 运行前环境（必须）

PC 端需配置 CycloneDDS 多机发现，才能收到各机器人跨网发布的 `odom_global`，详细见 [doc/多机通信设置.md](../../doc/多机通信设置.md)。

```bash
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/code/cyberdog2_pc_ws/src/env/cyclonedds.xml  # 修改为实际路径
export ROS_DOMAIN_ID=42
```

## 编译

```bash
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select mutil_robot_odom
source install/setup.bash
```

## 使用

添加机器人：在参数文件中添加命名空间到robot_namespaces字段

使用默认参数文件 `config/global_vio_odom.yaml`：

**本机ros2 galactic docker中运行**

```bash
ros2 launch mutil_robot_odom global_vio_odom.launch.py
```


## 多机可视化

转发完成后，可启动 `topic_visualization` 包的多机 RViz 可视化：

```bash
ros2 launch topic_visualization mutil_robot_tag_visualize.launch.py
```
