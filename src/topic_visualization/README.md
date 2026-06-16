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
