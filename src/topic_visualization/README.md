# RViz 可视化工具

PC 端 ROS2 可视化功能包，将 Cyberdog2 实机话题转发为 RViz 友好的本地话题，并提供轨迹、位姿文本、机器人 Marker 等显示。

## 节点概览

| 节点 | launch | 用途 |
|------|--------|------|
| `mutil_robot_tag_visual_node` | `mutil_robot_tag_visualize.launch.py` | **推荐** 多机 `tag_global` 坐标系下同时显示全部机器人 |
| `tags_visual_node` | `tags_visualize.launch.py` | 单机 `tag_global` 下 VIO 轨迹与 TF 可视化 |
| `vins_visual_node` | `vins_visualize.launch.py` | Mivins 视觉里程计与腿式里程计对比 |


## 运行前环境（必须）

PC 端需配置 CycloneDDS 多机发现，才能收到各机器人跨网发布的话题，详细见 [doc/多机通信设置.md](../../doc/多机通信设置.md)。

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
colcon build --packages-select topic_visualization
source install/setup.bash
```

---

## 多机器人 tag_global 可视化（推荐）

在 `tag_global` 全局坐标系下同时显示多台机器人的位置、轨迹和位姿文本。坐标统一由 [mutil_robot_odom](../mutil_robot_odom/) 完成，本节点只负责可视化。


### 启动步骤

**终端 1**：多机 VIO 坐标统一

```bash
ros2 launch mutil_robot_odom global_vio_odom.launch.py
```

**终端 2**：多机 RViz 可视化（Fixed Frame = `tag_global`）

```bash
ros2 launch topic_visualization mutil_robot_tag_visualize.launch.py
```

### 可视化输出话题

| 内容 | 话题 |
|------|------|
| 统一坐标系 odom（由 mutil_robot_odom 发布） | `/global_vio/cyberdog_N/odom` |
| 轨迹 | `/global_vio/cyberdog_N/path` |
| 位姿文本 | `/global_vio/pose_text` |
| 机器人箭头 Marker | `/global_vio/robot_markers` |
| TF | `tag_global` → `cyberdog_N_base_link`（由 mutil_robot_odom 发布） |

### 实机前置条件

1. 各机器人已启动 [apriltag_ros](../apriltag_ros/) 的 AprilTag 识别与 VIO 坐标转换（产生 `odom_global`）
2. `config/mutil_robot_topics.yaml` 与 `mutil_robot_odom/config/global_vio_odom.yaml` 中的 `robot_namespaces` 一致


---

## 单机 tag_global 可视化

订阅单台机器人的 `/{namespace}/odom_global`，在 `tag_global` 坐标系下显示轨迹与 TF。

```bash
ros2 launch topic_visualization tags_visualize.launch.py
```

通过 `namespace_index` 选择机器人（0 = cyberdog_1，1 = cyberdog_2），配置文件：`config/tags_topics.yaml`。

### 可视化输出话题

| 内容 | 话题 |
|------|------|
| 中转 odom | `/viz/tags/odom` |
| 轨迹 | `/viz/tags/path` |
| TF | `tag_global` → `base_link` |

若机器人端已运行 `odom_transform_node`，保持 launch 参数 `run_odom_transform:=false`（默认）。若需在本机启动坐标转换：

```bash
ros2 launch topic_visualization tags_visualize.launch.py robot_namespace:=cyberdog_1
```

RViz Fixed Frame 设为 **tag_global**。

---

## Mivins 视觉里程计可视化

在 `odom` 坐标系下对比显示腿式里程计与 VINS `/odom_slam` 轨迹。

```bash
ros2 launch topic_visualization vins_visualize.launch.py
```

配置文件：`config/vins_topics.yaml`，RViz 配置：`config/mivins_rviz2_config.rviz2.rviz`。

### 实机前置条件

1. Mivins 定位已启动，有 `/cyberdog_N/odom_slam`
2. 腿式里程计有 `/cyberdog_N/odom_out`（用于与 VINS 原点对齐）

### 可视化输出话题

| 内容 | 话题 |
|------|------|
| 腿式里程计轨迹 | `/compare/leg_path` |
| VINS 轨迹 | `/compare/slam_path` |
| 中转 odom_slam | `/odom_slam` |
| 中转 odom | `/odom` |


## 依赖

- ROS2 Galactic、`rviz2`
- `nav_msgs`、`geometry_msgs`、`sensor_msgs`、`visualization_msgs`、`tf2_ros`
- 多机模式上游：[mutil_robot_odom](../mutil_robot_odom/)、各机器人 [apriltag_ros](../apriltag_ros/) 的 `odom_global`
- 话题格式说明：[doc/数据格式.md](../../doc/数据格式.md)
