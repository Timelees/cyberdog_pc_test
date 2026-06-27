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

## Mivins + AprilTag 全局坐标可视化

以 apriltag 配置的 tag frame（默认 `base`）作为 RViz Fixed Frame，将 VIO `/odom_slam` 转换到 tag 坐标系下发布，便于多机对齐。

```bash
# 必须在 galactic docker 内，且配置好多机 DDS（ROS_DOMAIN_ID=42）
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/.cyclonedds.xml
export ROS_DOMAIN_ID=42

ros2 launch topic_visualization vins_visualize.launch.py
```

### 实机前置条件

1. Mivins 定位已启动，有 `/cyberdog_N/odom_slam`
2. apriltag 已启动，且 tag 在相机视野内（`/apriltag/detections` 有数据）
3. `tags_36h11.yaml` 中 `pose_estimation_method` 非空，`tag.frames` 与 `tag_frame_id` 一致（默认 `base`）

### 验证

```bash
# 应在 docker 内执行，不要在未配置 DDS 的宿主机 shell 里直接跑
ros2 topic echo /odom_slam --field header.frame_id    # 应为 base
ros2 run tf2_ros tf2_echo base cyberdog_2/base_link   # tag 可见时有输出
```

### 无法显示时排查

1. `vins_visual` 日志中 `tag_align status: pose_valid=true`（需要 tag 可见）
2. `relay tag_detection` count > 0（apriltag 在运行）
3. `relay odom_slam` count > 0（Mivins 在发布）
4. RViz Fixed Frame 设为 **base**
5. PC 端收不到实机 `/tf_static` 是正常的；节点会在本机发布 `base_link -> optical` 静态链
