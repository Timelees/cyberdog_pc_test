# 足球导航

后续开发 Agent 请先阅读 [AGENT_HANDOFF.md](AGENT_HANDOFF.md)，其中集中记录当前节点拓扑、已验证能力、功能边界、未完成事项和修改守则。

参数分为 [football_runtime_common.yaml](params/football_runtime_common.yaml)（可复用运行参数）、[football_single_robot_simulation.yaml](params/football_single_robot_simulation.yaml)（单机仿真覆盖项）和 [football_team_simulation.yaml](params/football_team_simulation.yaml)（十机职责仿真）。单机 launch 会按此顺序合并，后者优先；实机 bringup 应复用前者并提供自己的机器人、传感器和 Nav2 参数文件。

## 功能边界

本包提供足球后方接近与单次踢球状态机、球目标适配、多机器人代价地图障碍层、动态障碍预测、球融合和角色分配。striker 到达球后目标并对准后，调用 motion_manager 执行一次右前腿踢球，不通过导航速度推球。`football_single_robot_simulation.launch.py` 使用轻量级动作服务器验证单台 striker 的闭环接口；实机使用 `football_real_robot.launch.py`，由标准 Nav2 controller/planner 和 `football_multi_robot_obstacle_layer` 负责局部/全局 rolling costmap。若机器人产品导航栈已由外部 bringup 启动，则只启动 `football_robot_runtime.launch.py`，避免重复启动 Nav2。

实机入口默认 `enable_motion:=false`。先确认定位、10 路全局 odom、球/角色输入和两个 costmap 正常，再显式使用 `enable_motion:=true`。本包不启动 VIO、底盘驱动、球融合或角色权威；这些必须由实机/PC authority 系统提供。

## 运行前检查

每次场景测试前均应确认：

- 所有机器人在同一 `tag_global` 坐标系，且 `/global_vio/<namespace>/odom` 时间戳新鲜。
- `football_goal_adapter` 已收到 9 台其他机器人的里程计；默认 `minimum_other_robot_count: 9`，缺失数据时应 fail-closed。
- 记录 `/football/ball_pose`、`/<robot>/football/state`、`/<robot>/football/approach_pose`、`/<robot>/speed_limit`、`/<robot>/cmd_vel`、局部/全局 costmap 及 RViz 视频。

单机闭环冒烟测试：

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch football_navigation football_single_robot_simulation.launch.py use_rviz:=true
```

单机和团队仿真的 Action Client 会固定使用 `require_slow_walk=false`；仿真不订阅
实机 `motion_status`，规划算法验证不会等待 303 状态。

仿真入口默认使用 `isolate_dds:=true`，不会继承实机脚本设置的
`CYCLONEDDS_URI`。只有确实需要让仿真加入指定 DDS 网络时，才传入
`isolate_dds:=false`，并确保环境中的 XML 路径有效。实机入口不清理该变量。

实机安全启动与正式启动：

```bash
ros2 launch football_navigation football_real_robot.launch.py \
  robot_namespace:=cyberdog_1 enable_motion:=false
ros2 launch football_navigation football_real_robot.launch.py \
  robot_namespace:=cyberdog_1 enable_motion:=true
```

已有外部 Nav2 时只启动足球控制链：

```bash
ros2 launch football_navigation football_robot_runtime.launch.py \
  robot_namespace:=cyberdog_1 enable_motion:=true
```

职责分配仿真使用独立 launch，不会启动或修改单机 striker 控制链：

```bash
ros2 launch football_navigation football_team_simulation.launch.py use_rviz:=true
```

Gazebo 物理验证入口会生成一只标准 5 号球和十台 CyberDog，并将
Gazebo 位姿映射为十路全局 odom，供 Nav2 多机器人障碍层和角色分配使用：

```bash
ros2 launch football_navigation football_gazebo_simulation.launch.py \
  gazebo_gui:=true use_rviz:=false
```

十台机器人都使用带关节控制插件的动态 URDF，每台分别启动独立的
`cyber_guide`、`cmd_vel` 转换器和足球控制链。协同程序可向
`/cyberdog_N/cmd_vel` 发布 `geometry_msgs/Twist`。底层状态、关节命令和
`usr_cmd` 服务全部位于各自
`cyberdog_N` 命名空间，不会串控。默认不注入人工踢球力，
球的运动由腿部—球—地面刚体接触决定。另外九台机器人的初始位置复用
`football_team_simulation.yaml` 中的 `initial_pose_cyberdog_2`～`initial_pose_cyberdog_10`。

使用自己的协同程序时应关闭内置十机规划器，避免两个节点同时发布速度；
底层十套 `cmd_vel -> MotionServoCmd -> cyber_guide -> Gazebo` 运控链仍会运行：

```bash
ros2 launch football_navigation football_gazebo_simulation.launch.py \
  gazebo_gui:=true enable_builtin_team_control:=false
```

该 launch 启动十机假位姿、可配置移动球、唯一 `football_team_role_assigner` 和可视化。策略实现分别位于 `src/coordination/*_role_strategy.cpp`，角色权威只负责安全门控、前锋迟滞、身份绑定和消息发布。

团队仿真默认将球固定在场地中心 `(3.0, 0.0)`，`cyberdog_1` 至 `cyberdog_5` 放在同一侧，`cyberdog_6` 至 `cyberdog_10` 沿远端边线布置。A 队五台机器人分别启动与单机仿真一致的 GoalAdapter、轨迹适配器、Action Client 和轻量导航器：距离球最近者成为 striker，规划到球后方并对准后提交一次踢球动作；其他角色进入对应的 `TACTICAL_*` 状态并跟踪各自 tactical target。团队可视化不固定 ego/striker，机器人标签直接显示角色权威发布的职责和当前位置。假世界和导航器共同保持机器人椭圆碰撞净空，默认安全净空为 `0.12 m`。

该启动文件会由 `football_simulation_input_publisher` 持续发布球、射门目标、比赛状态和 striker 身份。单障碍优先使用低开销的单峰平滑曲线；当多台机器人同时封住左右候选曲线时，规划器会沿前进方向搜索可切换绕行侧的多层横向走廊，并将结果插值为连续轨迹发布到 `/cyberdog_1/local_plan`。striker 只导航到球后方并完成对准，不通过导航速度接触或推动足球。

其他机器人初始位置均可在 [football_single_robot_simulation.yaml](params/football_single_robot_simulation.yaml) 中修改：

```yaml
initial_pose_cyberdog_2: [1.25, 1.08, 0.0]  # [x, y, yaw]
initial_pose_cyberdog_3: [-4.0, -2.5, 0.0]
# ...一直到 cyberdog_10
```

将某个 `initial_pose_cyberdog_N` 删除或设为 `[]`，并设置 `randomize_initial_poses: true`，可只随机生成该机器人的位置。配置会检查场地边界和 `minimum_initial_separation_m`，重叠位置会拒绝启动。

默认动态仿真中，`cyberdog_1` 由导航器发布的 `cmd_vel` 驱动，九台 peer 静止，便于重复验证避障行为。`random_peer_motion_enabled` 默认是 `false`；设为 `true` 后，除键盘干扰机器人外的 peer 使用固定随机种子的随机航点运动。每台机器人采用仅前进加转向的非完整约束模型，到达航点、航点超时、触碰场地边界或进入其他机器人的安全净空后会重新选择航点。

`cyberdog_2` 默认是红色键盘干扰机器人；其初始位姿由 `football_fake_other_robot_publisher.initial_pose_cyberdog_2: [x, y, yaw]` 配置。先启动单机仿真，再在另一个交互终端执行：

```bash
ros2 run football_navigation football_keyboard_robot_controller
```

按 `w/s` 前进/后退，`a/d` 左/右转，`x` 停止平移，`z` 停止转向，空格完全停止，`q` 退出。可通过两个节点同名的 `keyboard_controlled_namespace`（fake publisher）和 `keyboard_robot_namespace`（visualization）改用另一台 peer；键盘节点的 `robot_namespace` 也需改为相同名称。

仿真避障使用随机器人 yaw 旋转的矩形外接椭圆。对于默认 `0.562 x 0.339 m`
碰撞矩形，先取最小外接椭圆半轴 `length/sqrt(2)`、`width/sqrt(2)`，再分别
增加可配置的 `collision_ellipse_expansion_m`。默认扩展 `0.05 m` 后，单台机器人
椭圆半轴约为 `0.447 x 0.290 m`。RViz 会在每台机器人的独立 MarkerArray 话题中
同时显示实体和半透明椭圆区域，坐标文本使用紧凑的 `(x,y)` 格式。

## 待办测试清单

状态说明：`[x]` 表示已有自动化/闭环覆盖；`[ ]` 表示仍需现场证据。

- [ ] 单机接近与踢球闭环：从起点规划到足球后方，稳定对准后执行一次右前腿踢球动作。
  - 验收：出现 `NAV_TRANSIT`/`BALL_APPROACH`、`ALIGN_TO_GOAL`、`KICK_APPROACH`/`CONTACT_ALIGN`、`KICK_PREPARING`、`KICKING`、`KICK_COMPLETE`；motion_manager 仅接收一条踢球队列，踢球过程中导航控制保持无效。异常或超时进入 `KICK_FAILED` 并停止导航，不自动重试踢球。
  - 仿真假球固定不动，仅用于导航与动作服务流程验证；真实足球运动需在 Gazebo 刚体接触或实机中验证。

- [ ] 多机环境下，单机静态接近路径避障测试（轻量仿真规划器）。
  - 场景：`cyberdog_2` 位于 `(1.25,1.08)`，`cyberdog_10` 位于 `(1.4,0.3)`，两台机器人分别封住原直线和单峰曲线的候选侧；其余机器人保持可观测且不运动。
  - 验收：规划器识别阻挡机器人并选择安全绕行路径，机器人最终抵达足球后方；不以接触或推动足球作为通过条件。
  - 关联参数：除椭圆净空和跟踪参数外，多障碍搜索由 `multi_obstacle_lattice_stations`、`multi_obstacle_lateral_step_m`、`multi_obstacle_max_lateral_m`、`multi_obstacle_max_lane_change_m` 和 `multi_obstacle_turn_penalty` 控制，见 [football_single_robot_simulation.yaml](params/football_single_robot_simulation.yaml)。实机 costmap 场景仍需单独验收。

- [ ] 多机环境下动态避障接近测试。
  - 场景：`cyberdog_1` 作为 striker，其余九台机器人随机航点运动，移动机器人可横穿规划路线。
  - 验收：障碍变化时安全重规划或进入 `BLOCKED_RECOVERY`，障碍离开后重新规划并抵达足球后方；不以足球移动作为验收条件。
  - 关联实现：peer 随机航点、边界/椭圆碰撞约束和 `random_peer_min_clearance_m`，以及仿真导航器的障碍位姿变化触发重规划。实机 `multi_robot_obstacle_layer` 的预测扫掠区仍需在实机/完整 Nav2 场景单独验收。

- [ ] 多机环境下，双方 striker 竞争球场景。
  - 同时给 A、B 队发布同一球位置，并让两个候选 striker 到球距离相近；重复测试距离相等、当前 striker 失联和球位置突变。
  - 验收：每队每个周期只发布一个有效 striker，角色切换受保持/超时逻辑约束；失联者降为 `STOP`，另一候选可接管；双方的代价地图均把对方视为障碍，不能因角色竞争相互穿透。
  - 关联实现：[football_team_role_assigner.cpp](src/coordination/football_team_role_assigner.cpp) 的唯一角色发布与 striker 重选逻辑。

- [ ] 多机环境下，足球轨迹预测及其他身份机器人调度。
  - 以至少两台观察机器人发布带时间戳的球观测，使球以不同速度和方向移动；同时注入一帧过期、时间偏差过大和离群观测。
  - 验收：`football_ball_fusion` 仅融合满足时间同步/内点条件的观测；`football_team_role_assigner` 使用预测球位更新 striker，并为 SUPPORT、DEFENDER_LEFT、DEFENDER_RIGHT、GOALKEEPER 发布不冲突的战术目标。球停下、观测超时或比赛状态非 `PLAY` 时，所有运动命令安全收敛。

- [ ] 实机测试。
  - 完成定位、急停、限速、足球方位及右前腿踢球区域检查后，先在空旷区域低速验证到球后方的路径，再验证单次踢球。
  - 保留 rosbag、参数快照、日志和现场视频；确认动作失败时不会重复提交踢球请求。
  - 球位置突变与观测抖动：注入单帧跳变、左右高频摆动、丢帧、过期时间戳、未来时间戳，以及球速超过 `max_ball_speed_mps` 的观测。验收：非法观测不得清除最近有效球状态或触发危险目标更新；有效的新球位应受目标刷新阈值/保持时间约束，避免动作目标来回跳变。
  - 球在边界、角落或球门附近：将球放在场地边线、角点、球门线前和目标点重合处。验收：接近点始终在场地安全边界内；无可行的球后方站位时进入安全停止/恢复。
  - 障碍物与球重叠：在球周围、机器人与球之间及球后方规划路线上分别放置静态/动态机器人。验收：不将机器人障碍物误识别为球；障碍阻塞时按动态安全距离减速或停止，障碍离开后重新评估球后方接近位。
  - 关联参数：`max_ball_jump_m`、`max_ball_speed_mps`、`ball_protection_radius_m`、`ball_boundary_margin_m`。


## 实机路径规划测试入口

GoalAdapter 的实机测试配置根据 gait 88 TOML 的累计足端增量和 FR hip 横向坐标，将踢球目标设为足端扫掠终点约 `(0.394, -0.056) m`，并仅在估算扫掠段进入足球球心半径 `0.09 m`（球半径 `0.11 m` 减 `0.02 m` 模型余量）时触发踢球。对机器人基座与足球的相对位姿仍使用 odom 和机身 yaw 变换计算，CONTACT_ALIGN 不再以机身前缘近似代替足端可达性。JSONL 按 `0.10 s` 记录球/机器人相对位姿、足端扫掠距离/端点、各项放行标志、速度指令及数据时效，并追加踢球生命周期事件，包括 112/111 等待、队列发送/响应、motion_id/进度、姿态/足端/电机错误和失败原因。

实机测试将 `behind_capture_tolerance_m` 单独设为 `0.20 m`，处理球后点附近约 `0.16-0.18 m` 的实测到位偏差，让机器人可先锁定站位并原地对准。触球位置门限为 `0.09 m`，略高于 Nav2 的 `0.08 m` XY goal tolerance；航向门限为 `0.12 rad`，略高于 Nav2 的 `0.10 rad` yaw tolerance，避免控制器已判定到位但 GoalAdapter 永远等待更严门限。放行仍须同时满足足端扫掠 `0.09 m` 门槛及其他球后/前向条件。gait 88 的监控超时为 `20 s`，覆盖 TOML 各阶段约 4760 个运控循环的名义执行时间。踢球生命周期事件同时记录触发时球位、动作期球位及位移，便于确认实际冲量和偏转方向。

这仍是静态轨迹模型，不等于实机必然接触：`TransBody` 的支撑脚补偿、实时站立关节角、控制器限幅和地面/球体接触误差需要实机或含真实运控的仿真核验。若 CONTACT_ALIGN 到达足端距离门槛却未触球，应依据新增的足端扫掠距离、motion 状态和错误字段定位差异，不应继续放宽门槛。

在暂时没有足球感知时，使用单机测试入口。它只发布静态球、射门目标、`cyberdog_1` striker 和 `PLAY` 状态，不发布或覆盖实时 `/global_vio/cyberdog_1/odom`：

```bash
ros2 launch football_navigation football_real_robot_test.launch.py \
  ball_x:=2.0 ball_y:=0.0 kick_target_x:=6.0 kick_target_y:=0.0
```

该入口消费实时全局 VIO（激光默认关闭），启动 Nav2、路径规划和 `cmd_vel`
看门狗适配器。`ball_x/ball_y` 是足球坐标，`kick_target_x/kick_target_y`
定义踢球方向；机器人首先导航到足球反方向、距球约 `approach_distance_m`
的接近点，并不是直接导航到 `kick_target`。Nav2 速度使用私有话题
`/<robot>/football/cmd_vel`，避免被机器人端默认 `velocity_adaptor` 同时转换。
适配器参考 `cyberdog_nav2` 将速度做 0.25 秒滑动平均，再转换为
跟随步态 309 的 `MotionServoCmd`；实机 DWB 默认只采样前进和转向。
测试前必须确认键盘急停终端在线。该适配器
默认关闭；确认 RViz 路径无误后显式传入 `use_cmd_vel_adapter:=true` 才会驱动
实机。

主要风险：静态球不是感知结果，机器人移动或足球被推动后目标不会自动更新；
测试入口关闭了其他机器人位姿要求且默认不使用激光，单机模式不能证明静态
障碍或多机避障有效；`football_cmd_vel_to_servo` 与键盘节点或机器人端其他
速度适配器同时运行会形成多个 `MotionServoCmd` 发布源，测试时只能保留一个
运动控制源。急停按键和实体急停必须保持可用。

## 结果记录模板

每次执行在下表新增一行，只有“证据”完整时才将相应待办勾选为 `[x]`。

| 日期 | 场景 | 软件/参数版本 | 结果 | 证据（rosbag、日志、视频） | 问题与后续动作 |
| --- | --- | --- | --- | --- | --- |
|  |  |  | 待执行 |  |  |
