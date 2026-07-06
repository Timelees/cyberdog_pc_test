# 多机 VIO 坐标统一使用流程

本文描述从实机 VIO 启动到 PC 端多机坐标统一的完整流程，涵盖 Mivins 视觉里程计、AprilTag 坐标转换与 `mutil_robot_odom` 转发。

## 整体架构

```
┌─────────────────────────────────────────────────────────────────┐
│  机器人端（每台 cyberdog_N）                                      │
│                                                                 │
│  camera ──► Mivins ──► /cyberdog_N/odom_slam                    │
│     │                              │                            │
│     └──► apriltag_node ──► tag TF   │                            │
│              │                      ▼                            │
│              └──► odom_transform_node ──► /cyberdog_N/odom_global│
│                      （坐标系: tag_global）                       │
└───────────────────────────────┬─────────────────────────────────┘
                                │ CycloneDDS 跨网
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  PC 端（cyberdog2_pc_ws）                                        │
│                                                                 │
│  mutil_robot_odom ──► /global_vio/cyberdog_N/odom               │
│                       TF: tag_global → cyberdog_N_base_link     │
│                                                                 │
│  topic_visualization（可选）──► RViz 多机轨迹 / Marker / 位姿文本  │
└─────────────────────────────────────────────────────────────────┘
```

**坐标系约定**

- `tag_global`：以 AprilTag 编号 0 所在位置为原点的全局坐标系，由 `odom_transform_node` 定义。
- 多台机器人需共同观测**同一块 Tag 0**，各自的 `odom_global` 才处于同一物理坐标系下。
- `mutil_robot_odom` 在 PC 端将各机 `odom_global` 统一转发，并将 `child_frame_id` 改为 `cyberdog_N_base_link`，避免多机 TF 冲突。

---

## 前置条件

### 1. 场地 Tag 码

将 [aprilTag 图](./aprilTag图/) 中的 Tag 0 转为 PNG 并打印，按原图方向贴于垂直面，确保各机器人相机均能看到。

### 2. 机器人命名空间

每台机器人需配置唯一命名空间（如 `cyberdog_1`、`cyberdog_2`），详见 [多机通信设置.md](./多机通信设置.md) 中「修改 cyberdog 命名空间」一节。

### 3. 多机 DDS 通信

PC 端与每台机器人均需配置 CycloneDDS，`ROS_DOMAIN_ID` 保持一致（默认 42）。详见 [多机通信设置.md](./多机通信设置.md)。

PC 端每次进入 ROS 环境时执行：

```bash
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/.cyclonedds.xml   # 修改为实际路径
export ROS_DOMAIN_ID=42
```

### 4. 功能包部署（一次性）

| 组件 | 部署文档 | 部署脚本 |
|------|----------|----------|
| Mivins 视觉里程计 | [Mivins部署.md](./Mivins部署.md) | [deploy_mivins.sh](../env/deploy_mivins.sh) |
| apriltag_ros | [apriltag_ros部署.md](./apriltag_ros部署.md) | [deploy_apriltag.sh](../env/deploy_apriltag.sh) |
| mutil_robot_odom（PC 端） | [mutil_robot_odom README](../src/mutil_robot_odom/README.md) | PC 端 colcon 编译 |

Mivins 部署后还需修改实机 `/opt/ros2/cyberdog/share/algorithm_manager/config/Task.toml`，使 `VisionLocalization` 依赖 `camera/camera` 与 `mivinslocalization`，详见 [Mivins部署.md](./Mivins部署.md)。

---

## 第一步：实机启动 VIO 与相机

每台机器人需先产生 `/odom_slam`，后续坐标转换才有输入。

### 方式 A：脚本批量启动（推荐）

在 **PC 端** 执行 [start_vio.sh](../env/start_vio.sh)，通过 DDS 远程激活各机器人的相机与 Mivins：

```bash
# 按需修改脚本内 ROBOT_NAMESPACES 列表
cd ~/cyberdog2_pc_ws/src/env
bash start_vio.sh
```

脚本会依次对每台机器人执行：

1. `camera/camera` lifecycle configure → activate
2. 打开 `realsense_frame_service`
3. `vinslocalization` lifecycle configure → activate

### 方式 B：手动启动（单台调试）

SSH 登录机器人 NX 板，或在 PC 端通过 DDS 执行：

```bash
ros2 lifecycle set /cyberdog_1/camera/camera configure
ros2 lifecycle set /cyberdog_1/camera/camera activate
ros2 service call /cyberdog_1/camera/realsense_frame_service std_srvs/srv/SetBool "{data: true}"

ros2 lifecycle set /cyberdog_1/vinslocalization configure
ros2 lifecycle set /cyberdog_1/vinslocalization activate
```

### 验证 VIO 输出

```bash
# PC 端或机器人端均可
ros2 topic hz /cyberdog_1/odom_slam
ros2 topic echo /cyberdog_1/camera/infra1/image_rect_raw
```

---

## 第二步：实机启动 AprilTag 与坐标转换

VIO 正常后，在**每台机器人**上启动 AprilTag 识别与 `odom_global` 转换。

**TODO：** 后续稳定版本，这个将加入开机自启动

### 启动节点

SSH 登录各机器人 NX 板，分别执行（命名空间由实机 `get_namespace()` 自动获取，也可显式传入）：

```bash
source /opt/ros2/cyberdog/setup.bash

# 终端 1：AprilTag 识别
ros2 launch apriltag_ros apriltag_36h11.launch.py

# 终端 2：VIO 坐标转换（odom_slam → odom_global）
ros2 launch apriltag_ros odom_transform.launch.py
```


### 验证实机输出

```bash
# Tag 检测（需机器人面向 Tag 0）
ros2 topic echo /{namespace}/apriltag/detections 
# 全局坐标系里程计
ros2 topic echo /{namespace}/odom_global
ros2 topic echo /{namespace}/odom_global --field header.frame_id
# 应输出: tag_global
```

**各台机器人均重复以上步骤**，确认每台都有 `/cyberdog_N/odom_global` 数据。

---

## 第三步：PC 端启动坐标统一

实机 `odom_global` 就绪后，在 PC 端 Galactic Docker 中启动 `mutil_robot_odom`。

### 编译（首次或代码变更后）

```bash
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select mutil_robot_odom
source install/setup.bash
```

### 配置机器人列表

编辑 `src/mutil_robot_odom/config/global_vio_odom.yaml`，将在线机器人追加到 `robot_namespaces`：

```yaml
robot_namespaces: ["cyberdog_1", "cyberdog_2"]
```

### 启动节点

```bash
ros2 launch mutil_robot_odom global_vio_odom.launch.py
```

节点会：

- 订阅 `/{namespace}/odom_global`
- 发布 `/global_vio/{namespace}/odom`（`frame_id = tag_global`，`child_frame_id = {namespace}_base_link`）
- 发布 TF：`tag_global → cyberdog_N_base_link`

### 验证 PC 端转发

```bash
ros2 topic echo /global_vio/{namespace}/odom
```


---

## 第四步：多机可视化（可选）

坐标统一完成后，可在 PC 端启动 RViz 多机显示：

```bash
# 终端 1（若未启动）
ros2 launch mutil_robot_odom global_vio_odom.launch.py

# 终端 2
ros2 launch topic_visualization mutil_robot_tag_visualize.launch.py
```

确保 `src/topic_visualization/config/mutil_robot_topics.yaml` 中 `robot_namespaces` 与 `global_vio_odom.yaml` 一致。RViz Fixed Frame 设为 **tag_global**。

详见 [topic_visualization README](../src/topic_visualization/README.md)。

---

## 关键话题汇总

| 话题 | 发布位置 | 坐标系 | 说明 |
|------|----------|--------|------|
| `/{ns}/odom_slam` | 实机 Mivins | map / odom | 原始 VIO 里程计 |
| `/{ns}/odom_global` | 实机 odom_transform | tag_global | 单机全局坐标里程计 |
| `/global_vio/{ns}/odom` | PC mutil_robot_odom | tag_global | 多机统一转发输出 |
| `/global_vio/{ns}/path` | PC topic_visualization | tag_global | 轨迹（可视化） |

完整话题与消息格式见 [数据格式.md](./数据格式.md)。

---

## 相关文档

- [apriltag_ros 部署](./apriltag_ros部署.md)
- [Mivins 部署](./Mivins部署.md)
- [多机通信设置](./多机通信设置.md)
- [数据格式](./数据格式.md)
- [mutil_robot_odom](../src/mutil_robot_odom/README.md)
- [topic_visualization](../src/topic_visualization/README.md)
