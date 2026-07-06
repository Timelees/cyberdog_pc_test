# apriltag_ros 部署方法


apriltag_ros为通过机器狗相机识别tag码的位姿，功能包输出**camera_infra1_optical_frame -> tag_0_observation**的位姿关系，以tag码（编号0）所在位置作为全局坐标系，通过odom_transform_node节点将视觉惯性里程计输出/odom_slam进行转换输出里程计在全局坐标系下的位姿话题/odom_global。


## 说明

`cyberdog_img:1.0` 镜像是 **arm64 原生环境**（在 x86 电脑上通过 QEMU 模拟运行），编译产物可直接部署到机器狗 NX 板。

实机 ROS 安装路径：`/opt/ros2/cyberdog`。

## 启动 Docker

```bash
docker run --privileged=true -it \
  -v /home/lee/code/cyberdog2_ws:/home/builder/cyberdog2_ws \
  cyberdog_img:1.0 bash
```

## 容器内：准备依赖

```bash
source /opt/ros2/galactic/setup.bash
cd /home/builder/cyberdog2_ws

# apriltag C 库（colcon cmake 包）
if [ ! -f src/third_party/apriltag/package.xml ]; then
  git clone --depth 1 --branch v3.4.5 \
    https://github.com/AprilRobotics/apriltag.git src/third_party/apriltag
fi
```

## 容器内：编译

将文件夹apriltag_msgs和apriltag_ros放置在机器狗源码工作空间下

![apriltag相关路径](./images/apriltag相关文件放置路径_image.png)

```bash
source /opt/ros2/galactic/setup.bash    # 一定要source 
cd /home/builder/cyberdog2_ws
colcon build --merge-install --packages-select apriltag apriltag_msgs apriltag_ros
```

## 部署到实机

### 一键部署脚本

工作区路径按实际修改；机器狗 IP 默认 `192.168.44.1`。  
需安装 `sshpass`：`sudo apt install sshpass`

```bash
# 在宿主机执行
cd ~/code/cyberdog2_pc_ws/src/env
bash deploy_apriltag.sh
```

脚本路径：`src/env/deploy_apriltag.sh`

### 验证部署

远程连接机器人NX板，使用如下指令

```bash
ros2 pkg prefix apriltag_msgs
ros2 interface show apriltag_msgs/msg/AprilTagDetectionArray
ros2 pkg executables apriltag_ros
```

出现以下内容即为部署成功

![部署验证](./images/apriltag部署验证_image.png)

---

## 实机使用

红外相机需先启动：

```bash
# 命名空间按照实际情况修改，一定要call 这个service
ros2 lifecycle set /cyberdog_1/camera/camera configure
ros2 lifecycle set /cyberdog_1/camera/camera activate
ros2 service call /cyberdog_1/camera/realsense_frame_service std_srvs/srv/SetBool "{data: true}"  
```

验证相机是否有数据：

```bash
ros2 topic echo /cyberdog_2/camera/infra1/image_rect_raw 
```

### 节点启动

**机器人上**启动 apriltag 和 odom_transform：

```bash
ros2 launch apriltag_ros apriltag_36h11.launch.py 
```

```bash
# 需先在本机工作空间编译 apriltag_ros（含 odom_transform_node）
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
ros2 launch apriltag_ros odom_transform.launch.py 
# ros2 run apriltag_ros odom_transform_node

```


`odom_global` 需 apriltag 识别到 tag 且 TF 链完整后才有输出；节点日志中 `latched=true` 表示已对齐。
如果看到 `waiting for TF chain base_link -> tag_0_observation`，说明还没有识别到 tag 或缺少相机到机身的 TF。若看到 `map -> tag_0_observation`，说明仍在运行旧版节点或启动参数里 `use_latest_tf_on_failure` 未关闭。

如果启动 `odom_transform_node` 后本机收不到 `/odom_slam`，优先检查 DDS 显式 peer：

- 机器人 `/etc/mi/cyclonedds.xml` 的 `<Peers>` 增加本机 WiFi IP，如 `192.168.31.68`
- 本机 `~/.cyclonedds.xml` 的 `<Peers>` 增加机器人 WiFi IP，如 `192.168.31.162`
- 修改后重启机器人端相关 ROS2 节点，并在本机执行 `ros2 daemon stop && ros2 daemon start`

查看检测结果：

```bash
# 命名空间自定
ros2 topic echo /cyberdog_1/apriltag/detections
```

---

## 可视化查看

本机galactic docker中启动

```bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
ros2 launch topic_visualization tags_visualize.launch.py
```

`tags_visual` 订阅 `/cyberdog_1/odom_global`，在本机转发为 `/viz/tags/odom`、`/viz/tags/path` 并发布 TF `tag_global -> base_link`。RViz 请订阅本机转发话题，不要直接订阅机器人端的 `/cyberdog_1/odom_global`（跨机 BestEffort 与 RViz 默认 Reliable 不兼容）。

### 验证

```bash
ros2 topic hz /cyberdog_1/odom_global
ros2 topic hz /viz/tags/odom
```

`tags_visual` 日志中应周期性出现 `odom_global relay stats: count>0`。

### 无法显示时排查

1. `tags_topics.yaml` 中 `namespace_index: 0` 对应 `cyberdog_1`
2. RViz Fixed Frame 设为 **tag_global**
3. Odometry / Path 显示订阅 **/viz/tags/odom**、**/viz/tags/path**，QoS 设为 **Best Effort**
4. 确认本机已 `colcon build --packages-select topic_visualization --symlink-install` 并 source install