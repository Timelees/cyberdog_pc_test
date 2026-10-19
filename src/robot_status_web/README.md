# robot_status_web

ROS 2 multi-CyberDog health dashboard. The node subscribes to each robot's odometry, camera, AprilTag and global-VIO topics, polls lifecycle state,
and exposes a FastAPI/WebSocket dashboard.

## 部署（PC 端）

节点需要运行在能通过 DDS 发现机器狗话题和 lifecycle 服务的 PC 环境中。主机需安装 ROS 2 Galactic、rmw_cyclonedds_cpp，并准备与多机通信配置相符的 CycloneDDS XML（工作区根目录或用户目录下的 .cyclonedds.xml）。

在工作区根目录执行：

```bash
source /opt/ros/galactic/setup.bash
sudo apt install ros-galactic-rmw-cyclonedds-cpp
python3 -m pip install --user -r src/src/robot_status_web/requirements.txt
colcon build --packages-up-to robot_status_web
```

`--packages-up-to` 会同时构建 robot_status_web 的工作区依赖（包括 apriltag_msgs）。

### 启动

```bash
bash src/env/start_robot_status_web.sh
```

启动脚本默认使用 ROS_DOMAIN_ID=42 和 rmw_cyclonedds_cpp，并自动查找 .cyclonedds.xml。可通过 ROBOT_STATUS_CONFIG 指定自定义配置：

```bash
ROBOT_STATUS_CONFIG=/path/to/robot_status_web.yaml bash src/env/start_robot_status_web.sh
```

默认监听 0.0.0.0:8080，访问 http://0.0.0.0:8080/。网页可访问但状态为 unknown 时，检查 DDS 发现、命名空间和实际话题名称。

The dashboard only changes lifecycle state when the operator presses
“配置并激活” or “激活全部节点”. Topic health is based on message receipt
time and configurable per-topic timeouts.
