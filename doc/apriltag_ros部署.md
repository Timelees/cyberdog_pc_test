# apriltag_ros 部署方法

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

```bash
source /opt/ros2/galactic/setup.bash
cd /home/builder/cyberdog2_ws

colcon build --merge-install --packages-up-to apriltag_ros
```

等价写法：

```bash
colcon build --merge-install --packages-select apriltag apriltag_msgs apriltag_ros
```

### QEMU 模拟下编译偶发 Segmentation fault

```bash
export MAKEFLAGS="-j1"
while ! colcon build --merge-install --executor sequential \
  --packages-up-to apriltag_ros; do
  echo "===== retrying ====="
  sleep 2
done
```

## 部署到实机

### 一键部署脚本

工作区路径按实际修改；机器狗 IP 默认 `192.168.44.1`。  
需安装 `sshpass`：`sudo apt install sshpass`

> **注意**：若在 Docker 内用 `--symlink-install` 编译，`install/` 里部分文件是指向 `/home/builder/cyberdog2_ws/build/...` 的符号链接，在宿主机上直接 `scp` 会报 `No such file or directory`。请使用仓库内脚本 `src/env/deploy_apriltag.sh`（会自动解析链接）。

```bash
# 在宿主机执行
cd ~/code/cyberdog2_pc_ws/src/env
bash deploy_apriltag.sh
```

脚本路径：`src/env/deploy_apriltag.sh`（自动修复 Docker 符号链接后上传安装）。

### 验证部署

```bash
export SSHPASS=123
sshpass -e ssh -o StrictHostKeyChecking=no mi@192.168.44.1 bash -s <<'EOF'
source /opt/ros2/cyberdog/setup.bash
ros2 daemon stop && ros2 daemon start
ros2 pkg prefix apriltag_msgs
ros2 interface show apriltag_msgs/msg/AprilTagDetectionArray
ros2 pkg executables apriltag_ros
EOF
```

---

## 实机使用

红外相机需先启动：

```bash
ros2 lifecycle set /cyberdog_2/camera/camera configure
ros2 lifecycle set /cyberdog_2/camera/camera activate
ros2 service call /cyberdog_2/camera/realsense_frame_service std_srvs/srv/SetBool "{data: true}"
```

验证相机是否有数据：
```bash
ros2 topic echo /cyberdog_2/camera/infra1/image_rect_raw 
```

启动 apriltag 节点：

```bash
source /opt/ros2/cyberdog/setup.bash
ros2 launch apriltag_ros apriltag_36h11.launch.py
```

查看检测结果：

```bash
ros2 topic echo /apriltag/detections
```

---

## 本机 Docker 订阅 `/apriltag/detections`

**本机 Docker 操作步骤**：

```bash
# 1. 若尚未编译消息包（只需一次）
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select apriltag_msgs

# 2. 每次打开终端都要 source 两层环境
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash

# 3. 多机通信配置（见 doc/多机通信设置.md）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lee/.cyclonedds.xml
export ROS_DOMAIN_ID=42

ros2 daemon stop && ros2 daemon start

# 4. 验证消息是否可用，再 echo
ros2 interface show apriltag_msgs/msg/AprilTagDetectionArray
ros2 topic echo /apriltag/detections
```

`ros2 interface show` 能显示消息定义即表示本机环境配置正确。
