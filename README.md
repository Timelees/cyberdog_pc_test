# CyberDog2 PC 工作空间

本仓库是 cyberdog2协同足球 工作空间中的源码目录，面向 CyberDog2 多机协作、足球导航、视觉处理和仿真。下面的编译命令均从工作空间根目录运行。机器人端 **AprilTag**、**Mivins** 和 **motion_manager** 的交叉编译/部署另有流程，见编号文档。

## 目录结构

```text
cyberdog2_pc_ws/                   # colcon 工作空间根目录
├── src/                           # 本仓库
│   ├── doc/                       # 部署文档、图片及部署参考文件
│   ├── env/                       # DDS 配置、部署和启动脚本
│   ├── config/                    # 工作空间级配置
│   ├── launch/                    # 工作空间级 launch/辅助入口
│   ├── thirdparty/                # 第三方依赖或源码
│   └── src/                       # ROS 2 功能包源码
├── build/                         # colcon 构建产物
├── install/                       # 安装空间，需 source 环境
└── log/                           # colcon 日志
```

## 功能包

| 功能包/目录 | 作用与运行位置 |
|---|---|
| `protocol` | CyberDog 控制及状态 ROS 接口定义；PC 控制包依赖。 |
| `apriltag_msgs` | AprilTag 检测结果消息定义。 |
| `apriltag_ros` | AprilTag 检测及 VIO 坐标转换，主要部署在机器人端；输出检测结果和 `odom_global`。 |
| `mutil_robot_odom` | PC 端多机里程计统一转发，将各机器人 `/{ns}/odom_global` 转为 `/global_vio/{ns}/odom` 并发布 TF。 |
| `topic_visualization` | PC 端将机器人话题整理为 RViz 可视化数据，支持多机 Tag、VINS 等显示。 |
| `robot_status_web` | PC 端 ROS 2 状态监测网页，查看相机/VINS/里程计/Tag 状态，并提供生命周期及站立、趴下操作。 |
| `visual_detection` | YOLOv8 ONNX 视觉检测节点；识别足球、球门等，并可结合场地 Tag 转换足球全局坐标。 |
| `football_navigation` | 足球目标适配、单机足球接近/踢球流程、多机动态障碍与角色分配；含实机和仿真 launch。 |
| `keyboard_input` | PC 键盘发布 CyberDog 运动控制指令。 |
| `sim_base`（位于 `cyberdog_sim/`） | CyberDog 仿真的基础启动、控制及 RViz 支持。 |
| `cyber_guide`（位于 `cyberdog_sim/`） | 仿真机器人的 ROS 控制/运控接口。 |
| `ros_bridge`（位于 `cyberdog_sim/`） | 仿真侧 ROS 与底层消息桥接。 |
| `cyberdog_locomotion`（位于 `cyberdog_action_sim/`） | 官方步态/动作控制实现及动作参数，用于步态调试。 |
| `cyberdog_simulator`（位于 `cyberdog_action_sim/`） | 官方动作仿真元包，组织 Gazebo、模型和示例包。 |
| `cyberdog_msg`、`cyberdog_example`（位于 `cyberdog_action_sim/`） | 官方动作仿真使用的消息及示例节点。 |
| `camera_calibrate` | 相机标定相关源码目录 |

## 环境与编译

PC 端通常在 ROS 2 Galactic 环境（项目使用的 Docker 容器亦可）中编译。进入工作空间后：

```bash
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash

# 按需编译一组常用 PC 端多机包及其依赖
colcon build --packages-up-to robot_status_web mutil_robot_odom topic_visualization keyboard_input visual_detection football_navigation --symlink-install

source install/setup.bash
```

`visual_detection` 还需要 OpenCV、`cv_bridge`、ONNX Runtime；GPU 推理需 CUDA/TensorRT provider。完整仿真工作流的依赖与构建命令见 [动作调试与部署](doc/1_3%20动作调试与部署.md) 和 [Gazebo 仿真文档](doc/gazebo仿真使用文档.md)。

## 多机通信环境

启动docker，每个终端进入的docker环境下均需要配置dds环境

PC 与机器人必须处于可互通网络，并使用一致的 ROS_DOMAIN_ID、CycloneDDS 和机器人命名空间。根据实际网卡/IP 编辑 [env/cyclonedds_pc_unicast.xml](env/cyclonedds_pc_unicast.xml)，然后在每个 ROS 终端加载环境：

```bash
cd 工作空间/
source /opt/ros/galactic/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/code/cyberdog2_pc_ws/src/env/cyclonedds_pc_unicast.xml
export ROS_DOMAIN_ID=42
```

CycloneDDS 网卡及 Peer 配置、机器人命名空间和网络连接步骤见 [多机通信设置](doc/0_3%20多机通信设置.md)。命名空间与关键话题约定见 [数据格式定义](doc/0_2%20数据格式定义.md)。

## 常用启动流程

### 多机 VIO 与 RViz

完整前置条件、实机启动及验证步骤见 [多机 VIO 统一使用流程](doc/2%20多机VIO统一使用流程.md)、[Mivins 部署](doc/1_1%20Mivins部署.md) 和 [AprilTag 部署](doc/1_2%20apriltag_ros部署.md)。

### 状态监测网页

```bash
bash src/env/start_robot_status_web.sh
```

默认浏览器地址为 `http://<PC_IP>:8080/`。编译、配置和操作说明见 [状态监测平台使用](doc/3%20状态监测平台使用.md)。

在监测网页对机器人的相机节点和vins节点进行配置与激活

### 全局坐标转发

启动机器人的全局坐标转发节点，转发话题为：`/global_vio/{namespace}/odom`

```bash
cd 工作空间/
ros2 launch mutil_robot_odom global_vio_odom.launch.py
```

### 视觉检测

终端 1，启动视觉检测：

```bash
ros2 run visual_detection visual_detection_node --ros-args \
  --params-file install/visual_detection/share/visual_detection/config/visual_detection.yaml
```

终端 2，启动 AprilTag，并将输入输出重映射到场地话题：

```bash
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/field_camera/image_rect \
  -r camera_info:=/field_camera/camera_info \
  -r detections:=/field_camera/tag_detections \
  --params-file install/visual_detection/share/visual_detection/config/apriltag_field_camera.yaml
```

终端 3，启动全局坐标转换：

```bash
ros2 run visual_detection soccer_global_converter --ros-args \
  --params-file install/visual_detection/share/visual_detection/config/convert.yaml
```

视觉输入、输出话题及足球全局坐标标定见 [视觉检测使用](doc/4%20视觉检测使用.md)。

### 足球导航任务

单机测试

```bash
ros2 launch football_navigation football_real_robot_test.launch.py
```


足球实机启动前须先验证定位、全局 odom、障碍物和急停；实机 launch 默认不启用运动控制，安全要求及参数见 [football_navigation README](src/football_navigation/README.md)。


### 键盘控制/急停

```bash
ros2 run keyboard_input keyboard_input_node --ros-args -p namespace:=cyberdog_1
```

键位和运动指令说明见 [keyboard_input README](src/keyboard_input/README.md)。动作仿真与实机参数部署见 [动作调试与部署](doc/1_3%20动作调试与部署.md)。

## 编号文档索引

| 文档 | 内容 |
|---|---|
| [0_1 cyberdog 算力资源节省](doc/0_1%20cyberdog算力资源节省.md) | 实机关闭非必要节点及资源控制。 |
| [0_2 数据格式定义](doc/0_2%20数据格式定义.md) | 命名空间、控制消息、里程计和 ROS 话题。 |
| [0_3 多机通信设置](doc/0_3%20多机通信设置.md) | PC/机器人网络、CycloneDDS 配置。 |
| [1_1 Mivins 部署](doc/1_1%20Mivins部署.md) | Mivins 编译、部署、启动及故障恢复。 |
| [1_2 apriltag_ros 部署](doc/1_2%20apriltag_ros部署.md) | AprilTag 检测、全局坐标转换及实机部署。 |
| [1_3 动作调试与部署](doc/1_3%20动作调试与部署.md) | 动作仿真、步态参数调试和实机部署。 |
| [2 多机 VIO 统一流程](doc/2%20多机VIO统一使用流程.md) | 从实机 VIO 到 PC 多机里程计/RViz 的完整流程。 |
| [3 状态监测平台](doc/3%20状态监测平台使用.md) | 网页部署、操作、配置及排障。 |
| [4 视觉检测](doc/4%20视觉检测使用.md) | ONNX 检测、话题输出和足球全局坐标转换。 |

