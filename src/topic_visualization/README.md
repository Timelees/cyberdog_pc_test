本机可视化 cyberdog2 实机话题工具

## 功能

- **单机器人模式**：通过 `namespace_index` 选择一台机器人，转发 scan / odom / odom_slam
- **多机器人共享模式**：订阅 `mutil_odom_shared` 输出的 `/cyberdog_N/odom_shared`，在 `shared_odom` 坐标系下同时显示全部机器人位置、轨迹和位姿文本

## 构建

```bash
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-up-to topic_visualization mutil_odom_shared --symlink-install
```

## 多机器人共享可视化

先启动 odom 共享节点，再启动可视化：

```bash
# 终端 1：多机器人 odom 对齐
ros2 launch mutil_odom_shared shared_odom.launch.py

# 终端 2：RViz 多机显示（Fixed Frame = shared_odom）
ros2 launch topic_visualization visualize.launch.py
```

### 可视化输出话题

| 内容 | 话题 |
|------|------|
| 原始共享 odom（可直接给 RViz Odometry 显示） | `/cyberdog_N/odom_shared` |
| 位姿文本 | `/viz/shared/pose_text` |
| 机器人箭头 Marker | `/viz/shared/robot_markers` |
| 轨迹 | `/viz/shared/cyberdog_N/path` |
| 中转 odom | `/viz/shared/cyberdog_N/odom` |
| TF | `map` → `shared_odom`（静态），`shared_odom` → `cyberdog_N_base_link` |

### 无法显示时排查

1. 确认 `ros_topic_visual` 日志中有 `shared_robots=2`（不是 0）
2. 确认收到首帧日志：`shared odom received: cyberdog_1 ...`
3. RViz Fixed Frame 设为 **shared_odom**
4. 使用 launch 默认配置 `multi_robot_shared.rviz`，不要混用 fixed frame 为 `odom` 的 vins 配置
5. 两台配置文件中的 `robot_namespaces` 必须与 `mutil_odom_shared` 一致
6. 重新编译：`colcon build --packages-select mutil_odom_shared topic_visualization --symlink-install`

单机器人模式可将 `shared_odom_enabled` 设为 `false`，并使用 `config/config.rviz`。

## Mivins 视觉里程计可视化

在 `odom` 坐标系下对比显示腿式里程计与 VINS `/odom_slam` 轨迹。

```bash
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

ros2 launch topic_visualization vins_visualize.launch.py
```

### 实机前置条件

1. Mivins 定位已启动，有 `/cyberdog_N/odom_slam`
2. 腿式里程计有 `/cyberdog_N/odom_out`（用于与 VINS 原点对齐）

### 验证

```bash
ros2 topic echo /odom_slam --field header.frame_id
ros2 topic hz /compare/slam_path
```

### 无法显示时排查

1. `relay odom_slam` count > 0（Mivins 在发布）
2. `relay slam_path` count > 0
3. RViz Fixed Frame 设为 **map**（`odom_slam_align_enabled: false` 时）或 **odom**（对齐模式）
4. 确认 `/odom_slam` 发布 QoS 为 Reliable（`odom_slam_publish_best_effort: false`），与 RViz 一致
5. PC 端收不到实机 `/tf_static` 是正常的；节点会在本机发布 `base_link -> camera` 静态链
