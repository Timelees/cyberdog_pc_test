# Cyberdog2 PC 端工作空间

本仓库为 Cyberdog2 多机协作场景的 **PC 端 ROS2 Galactic 工作空间源码**，用于跨网接收实机话题、统一多机 VIO 坐标、RViz 可视化与键盘控制等。

机器人端功能（AprilTag 识别、Mivins 定位等）在 [`cyberdog2_ws`](../../cyberdog2_ws) 中编译部署；本仓库侧重 PC 侧节点与文档。

## 目录结构

```
src/                          # 本 git 仓库根目录
├── doc/                      # 部署与通信文档
├── env/                      # 部署脚本、VIO 启动脚本
├── scripts/                  # 辅助工具（如 compile_commands 生成）
└── src/                      # ROS2 功能包
    ├── apriltag_ros/         # AprilTag 识别 + VIO 坐标转换（部署到实机）
    ├── apriltag_msgs/
    ├── protocol/             # Cyberdog2 消息定义（keyboard_input 依赖）
    ├── keyboard_input/       # PC 端键盘控制
    ├── mutil_robot_odom/     # 多机 VIO 坐标统一
    └── topic_visualization/  # RViz 可视化
```

colcon 工作空间根目录为 `cyberdog2_pc_ws/`，编译时在上一级目录执行 `colcon build`。

## 快速开始

### 1. 环境配置

PC 端需配置 CycloneDDS 多机发现，才能与 Cyberdog2 实机通信。详见 [doc/多机通信设置.md](doc/多机通信设置.md)。

每次进入 ROS 环境时执行：

```bash
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/.cyclonedds.xml   # 修改为实际路径
export ROS_DOMAIN_ID=42
```

### 2. 编译

仅编译 PC 端常用包：

```bash
colcon build --packages-select keyboard_input mutil_robot_odom topic_visualization
```

### 3. 多机 VIO 可视化（典型流程）

```
机器人端                          PC 端
─────────                        ─────
apriltag_ros                     mutil_robot_odom
  └─ odom_global  ──DDS──►         └─ /global_vio/{ns}/odom
                                   topic_visualization
                                     └─ RViz 多机显示
```

**机器人端**（每台）：启动 AprilTag 识别与 VIO 坐标转换，详见 [doc/apriltag_ros部署.md](doc/apriltag_ros部署.md)。

**PC 端**：

```bash
# 终端 1：多机坐标统一
ros2 launch mutil_robot_odom global_vio_odom.launch.py

# 终端 2：RViz 多机可视化
ros2 launch topic_visualization mutil_robot_tag_visualize.launch.py
```

## 功能包说明

| 功能包 | 运行位置 | 说明 | 文档 |
|--------|----------|------|------|
| `apriltag_ros` | 实机 | Tag 码识别，将 VIO 里程计转换到 `tag_global` 坐标系 | [apriltag部署](doc/apriltag_ros部署.md) |
| `keyboard_input` | PC | 键盘控制机器人运动（趴下/站立/行走/转向） | [README](src/keyboard_input/README.md) |
| `mutil_robot_odom` | PC | 多机 `odom_global` 统一转发到 `/global_vio` | [README](src/mutil_robot_odom/README.md) |
| `topic_visualization` | PC | 实机话题 RViz 可视化（多机/单机/Mivins 对比） | [README](src/topic_visualization/README.md) |
| `protocol` | — | Cyberdog2 控制消息定义，`keyboard_input` 依赖 | — |
| `apriltag_msgs` | — | AprilTag 检测消息定义 | — |

## doc文档

| 文档 | 内容 |
|------|------|
| [多机通信设置](doc/多机通信设置.md) | CycloneDDS 配置、网络连接、多机发现 |
| [数据格式](doc/数据格式.md) | 命名空间约定、重要 ROS2 话题与消息格式 |
| [apriltag_ros 部署](doc/apriltag_ros部署.md) | 实机编译部署 AprilTag 与坐标转换 |
| [Mivins 部署](doc/Mivins部署.md) | 实机 Mivins 视觉里程计部署 |
| [深度相机点云数据获取](doc/深度相机点云数据获取.md) | 深度相机点云话题说明 |
| [cyberdog 算力资源节省](doc/cyberdog算力资源节省.md) | 实机算力优化相关 |
| [aprilTag 图](doc/aprilTag图/) | Tag 码 SVG 源文件，打印后贴于场地 |

## 部署脚本（env/）

| 脚本 | 用途 |
|------|------|
| [deploy_apriltag.sh](env/deploy_apriltag.sh) | 将 apriltag_ros 编译产物部署到实机 |
| [deploy_mivins.sh](env/deploy_mivins.sh) | 将 Mivins 编译产物部署到实机 |
| [start_vio.sh](env/start_vio.sh) | 远程启动各机器人 VIO 相关 lifecycle 节点 |
