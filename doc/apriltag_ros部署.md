# apriltag_ros 部署


apriltag_ros为通过机器狗相机识别tag码的位姿，功能包输出**camera_infra1_optical_frame -> tag_0_observation**的位姿关系，以tag码（编号0）所在位置作为全局坐标系，通过odom_transform_node节点将视觉惯性里程计输出/odom_slam进行转换输出里程计在全局坐标系/tag_global下的位姿话题/odom_global。

## Tag码

使用svg编辑器将[tag码](./aprilTag图/)中转为PNG并打印，以原图的方向贴于垂直面

## 说明

`cyberdog_img:1.0` 镜像是 **arm64 原生环境**（在 x86 电脑上通过 QEMU 模拟运行），编译产物可直接部署到机器狗 NX 板。

实机 ROS 安装路径：`/opt/ros2/cyberdog`。

## 编译

### 启动 Docker

```bash
docker run --privileged=true -it \
  -v /home/lee/code/cyberdog2_ws:/home/builder/cyberdog2_ws \
  cyberdog_img:1.0 bash
```

### docker容器内：准备依赖

```bash
source /opt/ros2/galactic/setup.bash
cd /home/builder/cyberdog2_ws

# apriltag C 库（colcon cmake 包）
if [ ! -f src/third_party/apriltag/package.xml ]; then
  git clone --depth 1 --branch v3.4.5 \
    https://github.com/AprilRobotics/apriltag.git src/third_party/apriltag
fi
```

### docker容器内：编译

将文件夹apriltag_msgs和apriltag_ros放置在机器狗源码工作空间的src目录中，内容如下

![apriltag相关路径](./images/apriltag相关文件放置路径_image.png)

```bash
source /opt/ros2/galactic/setup.bash    # 一定要source 
cd /home/builder/cyberdog2_ws
colcon build --merge-install --packages-select apriltag apriltag_msgs apriltag_ros
```

## 实机部署

### 一键部署脚本

使用[deploy_apriltag.sh](env/deploy_apriltag.sh)将编译产物部署到实机

脚本路径：`src/env/deploy_apriltag.sh`

工作区路径按实际修改；机器狗 IP 默认 `192.168.44.1`。 

需安装 `sshpass`：`sudo apt install sshpass`

**PS: 脚本开头的WS修改为实际的代码工作空间路径**

```bash
# 在宿主机执行
cd ~/code/cyberdog2_pc_ws/src/env
bash deploy_apriltag.sh
```

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

依赖`camera`节点以及`realsense服务`：

```bash
# 命名空间按照实际情况修改，一定要call 这个service
ros2 lifecycle set /{namespace}/camera/camera configure
ros2 lifecycle set /{namespace}/camera/camera activate
ros2 service call /{namespace}/camera/realsense_frame_service std_srvs/srv/SetBool "{data: true}"  
```

可直接使用[start_vio.sh](env/start_vio.sh)脚本直接启动多台机器人的这些内容，这个脚本也会启动VIO程序

验证相机是否有数据,只要相机有数据，即可使用：

```bash
ros2 topic echo /{namespace}/camera/infra1/image_rect_raw 
```

### 节点启动

**机器人上**启动 apriltag 和 odom_transform：

```bash
# 启动tag识别
ros2 launch apriltag_ros apriltag_36h11.launch.py 
# 启动vio坐标系转换
ros2 launch apriltag_ros odom_transform.launch.py 
```

### 验证

查看以下两个话题，有数据即可

```bash
# 命名空间自定
ros2 topic echo /{namespace}}/apriltag/detections

ros2 topic echo /{namespace}}/odom_global
```



