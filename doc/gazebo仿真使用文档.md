# CyberDog2 十机足球 Gazebo 仿真使用文档

## 运行环境

运行环境使用含 ROS 2 Galactic的 docker 容器中完成过编译和运行验证。

加载环境：

```bash
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
source install/setup.bash
```


## 编译

修改了 Gazebo 模型、插件、控制桥或足球导航代码后，执行：

首次编译需编译四足控制器相关代码，后续修改可只编译`football_navigation`

```bash
cd /home/lee/code/cyberdog2_pc_ws
source /opt/ros/galactic/setup.bash
colcon build --packages-select ros_bridge sim_base cyber_guide football_navigation --symlink-install
source install/setup.bash
```


## 启动方式

### 协同模式运行

该模式启动十台机器人、角色分配器以及每台机器人的内置足球控制和轻量导航器。适合直接验证仓库内的协同、路径规划、动态避障和踢球流程。

```bash
ros2 launch football_navigation football_gazebo_simulation.launch.py 
```

## 启动时序与正常现象

launch 会错开生成机器人，避免十个 IMU 和底层插件同时初始化：

1. 约 `1.5 s` 时生成足球；
2. 约 `1.8 s` 时生成 `cyberdog_1`；
3. 此后每隔约 `0.35 s` 生成下一台机器人；
4. 约 `6 s` 时启动状态桥、角色分配和十套底层运控链；
5. 约 `7 s` 时启动内置规划控制节点和可视化节点；
6. 每台机器人连接到 `/cyberdog_N/usr_cmd` 后，先执行约 `4 s` 恢复站立，再进入 locomotion。

因此从执行 launch 到所有机器人可接收运动指令通常需要约 `10～15 s`。看到如下日志说明对应机器人已经完成导航步态切换：

```text
navigation gait ready: motion_id=309 switch_status=0
```

## 主要 ROS 2 接口

### 外部协同程序常用接口

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/cyberdog_N/cmd_vel` | `geometry_msgs/msg/Twist` | 输入 | 第 N 台机器人速度指令 |
| `/global_vio/cyberdog_N/odom` | `nav_msgs/msg/Odometry` | 输出 | Gazebo 中的全局位姿和速度 |
| `/football/ball_pose` | `geometry_msgs/msg/PoseStamped` | 输出 | 真实 Gazebo 足球位姿，坐标系为 `world` |
| `/cyberdog_N/football/role` | `std_msgs/msg/String` | 输出 | 角色分配结果 |
| `/cyberdog_N/football/tactical_target` | `geometry_msgs/msg/PoseStamped` | 输出 | 角色战术目标 |
| `/cyberdog_N/football/state` | `std_msgs/msg/String` | 输出 | 足球控制状态机状态 |
| `/cyberdog_N/local_plan` | `nav_msgs/msg/Path` | 输出 | 内置导航器局部规划轨迹 |
| `/cyberdog_N/football/control_valid` | `std_msgs/msg/Bool` | 输出 | 底层控制链是否可用 |
| `/cyberdog_N/motion_status` | `protocol/msg/MotionStatus` | 输出 | 步态和 MotionServo 状态 |

### 底层接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/cyberdog_N/motion_servo_cmd` | `protocol/msg/MotionServoCmd` | `cmd_vel` 转换后的运动命令 |
| `/cyberdog_N/usr_cmd` | `ros_bridge/srv/UsrCmd` | `cyber_guide` 控制服务 |
| `/cyberdog_N/legged_command` | `ros_bridge/msg/LeggedCommand` | 发往 Gazebo 关节插件的底层命令 |
| `/cyberdog_N/imu_data` | `ros_bridge/msg/SimImu` | Gazebo IMU 数据 |
| `/cyberdog_N/joint_data` | `ros_bridge/msg/SimJoint` | 12 个关节状态 |
| `/cyberdog_N/base_data` | `ros_bridge/msg/SimBase` | Gazebo 机身状态 |

通常协同程序只需要使用 `cmd_vel`、全局 odom、足球位姿、角色和战术目标，不应直接绕过适配器发布 `LeggedCommand`。

## 启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_namespace` | `cyberdog_1` | 主机器人映射，团队模式保持默认即可 |
| `robot_x` | `0.0` | `cyberdog_1` 初始 X |
| `robot_y` | `-2.0` | `cyberdog_1` 初始 Y |
| `robot_z` | `0.42` | 所有机器人生成高度 |
| `robot_yaw` | `0.0` | `cyberdog_1` 初始偏航角，单位 rad |
| `ball_x` | `2.0` | 足球初始 X |
| `ball_y` | `-2.0` | 足球初始 Y |
| `ball_z` | `0.115` | 足球球心初始高度 |
| `gazebo_gui` | `true` | 是否启动 Gazebo 客户端窗口 |
| `use_rviz` | `false` | 是否启动 RViz |
| `enable_gazebo_markers` | `true` | 是否把规划 Marker 显示到 Gazebo |
| `enable_team_coordination` | `true` | 是否启动团队角色分配器 |
| `enable_builtin_team_control` | `true` | 是否启动十套内置规划和控制节点 |
| `enable_contact_wrench_fallback` | `false` | 是否额外注入随机腿部接触力，仅用于压力测试 |

例如修改主机器人和足球初始位置：

```bash
ros2 launch football_navigation football_gazebo_simulation.launch.py \
  robot_x:=-1.0 robot_y:=-2.5 robot_yaw:=0.3 \
  ball_x:=1.5 ball_y:=0.0
```

## 修改十台机器人的初始位置

`cyberdog_1` 的位置由 launch 参数设置。`cyberdog_2`～`cyberdog_10` 的位置来自：

```text
src/src/football_navigation/params/football_team_simulation.yaml
```

对应参数位于 `football_fake_other_robot_publisher.ros__parameters` 下：

```yaml
initial_pose_cyberdog_2: [0.0, -1.0, 0.0]
initial_pose_cyberdog_3: [0.0,  1.0, 0.0]
# 格式：[x, y, yaw]
```

注意：

- 坐标单位为米，yaw 单位为弧度；
- 建议保持机器人中心之间至少 `0.8 m` 的距离；
- 场地有效范围约为 `x ∈ [-8, 8]`、`y ∈ [-4, 4]`；
- 不要把机器人直接生成在足球、边线、球门或另一台机器人内部；
- 修改 YAML 后重新启动仿真才能应用新位置。

## 足球和地面物理参数

足球模型文件：

```text
src/src/cyberdog_sim/sim_base/model/football/model.sdf
```

当前关键参数：

| 参数 | 数值 | 说明 |
| --- | --- | --- |
| 半径 | `0.11 m` | 标准 5 号球约 `0.22 m` 直径 |
| 质量 | `0.43 kg` | 标准足球质量范围内 |
| 转动惯量 | `0.0020812 kg·m²` | 均匀球体近似 |
| 球面滑动摩擦 | `0.35` | 控制触地滑动 |
| 扭转摩擦 | `0.018` | 提供滚动阻力 |
| 回弹系数 | `0.72` | 控制碰撞回弹 |
| 线性速度衰减 | `0.01` | 小幅空气/模型阻力 |

场地模型文件：

```text
src/src/cyberdog_sim/sim_base/model/soccer_playground/model.sdf
```

场地接触面的 ODE 摩擦系数为 `mu=0.80`、`mu2=0.80`，用于近似干燥短人工草皮。球的运动默认完全来自腿部、球和地面的刚体接触，不通过脚本直接修改球速。

`enable_contact_wrench_fallback:=true` 会额外随机施加接触力，只适用于压力测试，不应用于正式物理效果验收。


## 路径显示参数

路径显示参数位于：

```text
src/src/football_navigation/params/football_gazebo_sim.yaml
```

当前默认值：

```yaml
football_visualization_node:
  ros__parameters:
    global_path_width_m: 0.07
    local_trajectory_width_m: 0.10
```

- `global_path_width_m`：全局规划路径宽度；
- `local_trajectory_width_m`：局部轨迹宽度。

如果只使用 Gazebo，不使用 RViz，保持 `enable_gazebo_markers:=true` 即可让 Marker 经桥接后显示到 Gazebo。



## 相关文件

| 文件 | 用途 |
| --- | --- |
| `src/src/football_navigation/launch/football_gazebo_simulation.launch.py` | 十机 Gazebo 总启动文件 |
| `src/src/football_navigation/params/football_gazebo_sim.yaml` | Gazebo 专用控制、可视化和足球参数覆盖 |
| `src/src/football_navigation/params/football_team_simulation.yaml` | 十机初始位置、角色和轻量导航参数 |
| `src/src/cyberdog_sim/sim_base/model/actors.urdf` | 可驱动机器人 URDF |
| `src/src/cyberdog_sim/sim_base/model/football/model.sdf` | 足球质量、尺寸和接触参数 |
| `src/src/cyberdog_sim/sim_base/model/soccer_playground/model.sdf` | 足球场几何、材质和地面摩擦 |
| `src/src/cyberdog_sim/sim_base/world/soccer_playground.world` | Gazebo 世界及物理求解参数 |
| `src/src/cyberdog_sim/sim_base/src/actors_gazebo/legged_plugin.cpp` | 机器人 Gazebo 底层关节插件 |
| `src/src/football_navigation/src/control/football_cmd_vel_to_servo.cpp` | `cmd_vel` 到 MotionServoCmd 的转换和看门狗 |
| `src/src/football_navigation/src/control/football_servo_to_cyber_guide.cpp` | MotionServoCmd 到 cyber_guide 服务的桥接 |


