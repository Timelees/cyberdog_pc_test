# visual_detection

ROS 2 Galactic 视觉检测节点。模块使用 YOLOv8 ONNX Runtime 对 RTSP 网络摄像头或 ROS 图像话题进行检测，识别以下类别：

- `dog1`
- `dog2`
- `soccer`
- `goal`

检测结果包含目标中心的图像坐标，并在检测到球门时提供以球门中心为原点的相对坐标。原有的离线视频检测、muduo TCP 服务端和客户端功能仍然保留。

## 1. 环境

编译和运行使用当前的 Docker 容器：

```bash
docker exec -it galactic_onnx_gpu bash
```

容器中应已经具备：

- ROS 2 Galactic
- OpenCV
- `cv_bridge`
- ONNX Runtime C++ GPU SDK
- CUDA/TensorRT provider
- muduo


## 2. 编译

进入工作区并加载 ROS 2 环境：

```bash
source /opt/ros/galactic/setup.bash
```

```bash
colcon build --packages-select visual_detection --symlink-install
```

编译完成后加载工作区：

```bash
source install/setup.bash
```

## 3. 启动 RTSP 检测

加载环境后直接运行：

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash

ros2 run visual_detection visual_detection_node
```

节点会自动：

1. 读取 `config/visual_detection.yaml`；
2. 连接 RTSP 摄像头；
3. 读取视频帧并执行 ONNX 检测；
4. 发布 JSON 结果和分类目标位置；
5. 仅在 `preview: true` 时显示检测预览窗口（实时运行默认关闭，避免 GUI 阻塞检测）。

启动成功时可以看到：

```text
RTSP stream connected
RTSP detection enabled
```

RTSP 断开后节点会自动重连。

RTSP 默认通过 TCP 拉流，并请求 FFmpeg 低延迟/无缓冲模式。若推理仍慢于相机帧率，节点会在每轮推理前最多抓取 `max_grab_frames` 帧并只解码最新一帧，避免不断处理旧画面。TCP 不能修复摄像头编码器或网络本身的错误，但可避免 UDP RTP 丢包/乱序造成的常见 HEVC 参考帧错误。

### 关闭预览

不显示 OpenCV 预览窗口：

```bash
ros2 run visual_detection visual_detection_node \
  --ros-args \
  -p preview:=false
```

也可以直接将配置文件中的值改为：

```yaml
preview: false
```

`Gtk-Message: Failed to load module "canberra-gtk-module"` 和 `dbind-WARNING` 通常只是 GUI/容器桌面辅助组件缺失或受限，不影响 ROS 图像发布及检测；实时部署建议关闭预览。`Could not find ref with POC`、`Duplicate POC`、`bad cseq` 则来自 HEVC/RTSP 解码链路，通常意味着 UDP 丢包/乱序或码流参考帧损坏，不是 AprilTag 报错。

### 临时覆盖参数

命令行参数只对当前运行有效，例如：

```bash
ros2 run visual_detection visual_detection_node \
  --ros-args \
  -p device:=cpu \
  -p preview:=false \
  -p confidence_threshold:=0.35
```

## 5. ROS 图像话题模式

如果不需要节点自己连接 RTSP，可以把配置中的 `video_stream` 设为空：

```yaml
video_stream: ""
```

节点会改为订阅：

```text
/image_raw
```

也可以临时指定图像话题：

```bash
ros2 run visual_detection visual_detection_node \
  --ros-args \
  -p video_stream:=\"\" \
  -p image_topic:=/camera/image_raw
```

输入消息类型：

```text
sensor_msgs/msg/Image
```

节点使用 `cv_bridge` 转换为 BGR 图像。

## 6. 检测结果话题

### 6.1 完整 JSON 结果

话题：

```text
/visual_detection/result
```

类型：

```text
std_msgs/msg/String
```

查看：

```bash
ros2 topic echo /visual_detection/result
```

单帧 JSON 包含：

- `frame_index`：帧序号；
- `timestamp_sec`：时间戳；
- `goal_origin`：球门中心的图像坐标，没有可用球门原点时为 `null`；
- `points`：当前帧的所有检测点；
- `class_name`：类别名称；
- `confidence`：置信度；
- `image_point`：目标中心的图像像素坐标；
- `goal_relative_point`：相对球门坐标，x 向右为正、y 向上为正；
- `predicted`：是否为足球轨迹预测点。

`goal_origin` 为 `null` 的常见原因是当前帧没有检测到 `goal`，或者置信度低于阈值。即使球门漏检，当前帧检测到的足球和机器狗仍会出现在 `points` 中；此时它们的 `goal_relative_point` 不具有有效的球门参考意义。

### 6.2 分类目标位置话题

节点还会分别发布以下话题：

```text
/visual_detection/soccer
/visual_detection/goal
/visual_detection/dog1
/visual_detection/dog2
```

消息类型均为：

```text
geometry_msgs/msg/PointStamped
```

字段含义：

```text
point.x       目标中心的图像像素 x 坐标
point.y       目标中心的图像像素 y 坐标
point.z       检测置信度
header.frame_id = image
```

例如：

```bash
ros2 topic echo /visual_detection/soccer
ros2 topic echo /visual_detection/goal
ros2 topic echo /visual_detection/dog1
ros2 topic echo /visual_detection/dog2
```

只有对应类别在当前帧被检测到时，分类话题才会发布消息。

### 6.3 Tag1 与足球全局坐标

场地摄像头无需看到竖直安装的 Tag0。`visual_detection_node` 会将 RTSP 图像和相机内参发布到 `/field_camera/image_rect` 与 `/field_camera/camera_info`；在同一个 ROS 2 Galactic Docker/ROS domain 中运行 `apriltag_ros` 检测水平 Tag1，再运行 `soccer_global_converter`。转换节点使用 `convert.yaml` 中测量得到的 Tag1 在最终 `tag_global` 下的完整位姿，先估计场地相机位姿，再将足球像素射线与地面平面求交。

先在 `visual_detection.yaml` 中填写与 RTSP 分辨率相同的相机内参 `camera_fx/fy/cx/cy` 和 OpenCV `plumb_bob` 顺序的畸变系数 `camera_distortion` (`k1,k2,p1,p2,k3`)。节点会先校正帧，再将同一校正帧用于 YOLO 与 AprilTag，保证足球和 Tag 像素坐标一致。内参必须对应 RTSP 实际输出分辨率。

在 `convert.yaml` 中填写：

- `tag_size_m`：Tag1 黑白码实际边长，单位米；应与 AprilTag 节点配置一致。
- `tag1_in_tag_global`：Tag1 原点和方向相对于最终 `tag_global` 的实测位姿；平移单位米，四元数顺序为 `qx, qy, qz, qw`。
- `output_axis_mapping`：全局点发布前的行优先 3×3 分量映射矩阵。当前机器人与场地坐标分量需要互换时配置为 `[0,1,0; 1,0,0; 0,0,1]`，即 `x_out=y_in, y_out=x_in, z_out=z_in`。它是坐标分量重排，不等价于 TF 刚体旋转。
- `ground_z_m`：地面在 `tag_global` 中的 Z 值。只有 Tag0 定义的原点/轴使地面确实为 `Z=0` 时才使用默认值。
- `use_tag_plane_homography`：默认开启。转换器用 Tag1 四角建立图像到地面的单应变换，并把足球检测框底边中心作为接地点；这比用检测框中心和单 Tag PnP 倾角推算球心平面更不易放大远距离误差。
- `tag_calibration_samples`：固定相机初始化时平均的 Tag1 检测帧数，默认 30；初始化期间不要移动相机或 Tag1。

`/visual_detection/soccer` 继续发布足球框中心；新增的 `/visual_detection/soccer_ground_contact` 发布框底边中心，并作为实时全局转换的默认输入。`/field/soccer_global` 表示足球接地点在场地平面上的位置，因此默认输出地面高度，不是球心高度。关闭 `use_tag_plane_homography` 时，应把 `soccer_pixel_topic` 改回 `/visual_detection/soccer`；旧方法使用框中心射线和 `soccer_radius_m` 求球心平面交点。

Tag0 竖直、Tag1 水平时，`tag1_in_tag_global` 不能仅填写两者的平面距离。机器人端最终 `tag_global` 经过墙面 Tag 轴对齐；必须测量 Tag1 中心在该最终坐标系中的三维平移，并根据 Tag1 印刷方向填写完整四元数。Tag0 中心若高于地面，则地面在 `tag_global` 中通常也不是 `Z=0`。
- `calibration_valid`：完成实测并核验轴向后才改为 `true`。占位参数下节点不会发布全局足球点。

Tag1 的位姿不能只填写它与 Tag0 的距离；必须测量三维平移和姿态，并按照 `odom_transform_node` 最终输出的 `tag_global` 轴向表达。若场地使用第二枚地面 Tag 做校验，可将其加入 AprilTag 配置用于观察，但当前转换以配置的 Tag1 为位姿锚点。

在 Docker 中构建后，按顺序运行：

```bash
source /opt/ros/galactic/setup.bash
source install/setup.bash

# 终端 1：YOLO 检测，并把相同相机帧提供给 AprilTag。
ros2 run visual_detection visual_detection_node \
  --ros-args --params-file install/visual_detection/share/visual_detection/config/visual_detection.yaml

# 终端 2：检测场地水平 Tag1。相机图像和 CameraInfo 来自上面的节点。
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/field_camera/image_rect \
  -r camera_info:=/field_camera/camera_info \
  -r detections:=/field_camera/tag_detections \
  --params-file install/visual_detection/share/visual_detection/config/apriltag_field_camera.yaml

# 终端 3：将足球像素位置转换为 Tag0 全局坐标。
ros2 run visual_detection soccer_global_converter --ros-args \
  --params-file install/visual_detection/share/visual_detection/config/convert.yaml
```

输出为 `/field/soccer_global` (`geometry_msgs/msg/PointStamped`)，`header.frame_id` 为 `tag_global`，xyz 单位米。启动后首次有效 Tag1 检测用于解算并锁存固定相机位姿；之后 Tag1 被遮挡不会停止转换。足球短时漏检时会使用 tracker 最多预测 15 个处理帧，超过后停止发布，避免无限外推。检查方式：

```bash
ros2 topic echo /field_camera/tag_detections
ros2 topic echo /field/soccer_global
```

如果 Tag 检测或全局点没有输出，先逐段确认：

```bash
ros2 topic hz /field_camera/image_rect
ros2 topic echo --once /field_camera/camera_info
ros2 topic hz /field_camera/tag_detections
ros2 topic echo --once /field_camera/tag_detections
ros2 topic hz /visual_detection/soccer
ros2 topic echo /field/soccer_global
```

`CameraInfo` 的 `width/height` 应与图像一致，且 `p[0]`、`p[5]`、`p[2]`、`p[6]` 非零；它们分别对应 `fx/fy/cx/cy`。如果图像话题有频率而 detections 没频率，检查场地 Tag 是否确为配置的 `family/id`、边长是否与实际一致、Tag 是否正面可见且清晰，以及 `apriltag_field_camera.yaml` 的 `tag.ids/tag.sizes`。转换器日志出现 `Tag1 initialized` 后即可遮挡 Tag1；如果初始化成功但 `/field/soccer_global` 没输出，检查 `/visual_detection/soccer`、Tag1 实测位姿/Tag 尺寸、`ground_z_m`，以及日志中是否出现射线无法与地面正向相交的情况。

要测试摄像头实时性，短时执行 `ros2 topic delay /field_camera/image_rect`（Galactic 若没有该子命令，可用 `ros2 topic echo` 对照消息 stamp 与接收时间）并观察视频预览。预览本身关闭后检测仍继续；若位置控制要求低延迟，保持 `preview: false`。若 TCP 后 HEVC POC/cseq 错误仍持续，优先在摄像头端改用 H.264、降低码率/帧率或检查网线/交换机；编码包已经丢失时，软件只能丢弃损坏帧，不能恢复画面内容。

在 RViz 中将 Fixed Frame 设为 `tag_global`，同时显示机器人 `/global_vio/<robot>/odom` 和 `/field/soccer_global`。先将球静置在多个已知场地点验证坐标和误差，再进行动态测试。

## 7. 预览窗口

预览窗口名称：

```text
visual_detection
```

预览画面会显示：

- 原始检测框；
- 类别和置信度；
- 球门原点；
- 足球、球门和机器狗中心点；
- 足球预测点。

按 `Esc` 可以请求退出节点。无图形界面的环境建议设置：

```yaml
preview: false
```

## 8. 离线视频检测

原有命令行检测程序仍然可用：

```bash
ros2 run visual_detection dog_detect \
  --video /path/to/input.mp4 \
  --model /path/to/best.onnx \
  --labels /path/to/data.yaml \
  --device cuda \
  --json result.json \
  --out annotated.mp4
```

也可以直接运行安装后的程序：

```bash
install/visual_detection/lib/visual_detection/dog_detect --help
```

## 9. TCP 服务端和客户端

启动服务端：

```bash
ros2 run visual_detection dog_detect \
  --server \
  --port 9090 \
  --model /path/to/best.onnx \
  --labels /path/to/data.yaml \
  --device cuda
```

启动客户端：

```bash
ros2 run visual_detection dog_client \
  --host 127.0.0.1 \
  --port 9090 \
  --video /path/to/input.mp4
```

## 10. 常见问题

### 参数文件解析失败

`--params-file` 必须指向 YAML 文件，不能只写目录：

```bash
# 正确
ros2 run visual_detection visual_detection_node \
  --ros-args \
  --params-file install/visual_detection/share/visual_detection/config/visual_detection.yaml
```

### 节点连接 RTSP 失败

检查：

- 容器是否能访问 `192.168.31.105`；
- 用户名和密码是否正确；
- 摄像头是否启用 RTSP；
- URL 是否为正确的主码流地址；
- OpenCV/FFmpeg 是否支持 RTSP。

### 没有检测结果

可以先降低阈值：

```bash
ros2 run visual_detection visual_detection_node \
  --ros-args \
  -p confidence_threshold:=0.15
```

同时确认模型类别与 `data.yaml` 一致：

```text
0: dog1
1: dog2
2: soccer
3: goal
```

如果预览窗口能显示图像但没有框，通常是模型置信度、模型类别或摄像头画面内容的问题，而不是 ROS 话题连接问题。
