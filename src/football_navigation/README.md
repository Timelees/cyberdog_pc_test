# 足球导航

后续开发 Agent 请先阅读 [AGENT_HANDOFF.md](AGENT_HANDOFF.md)，其中集中记录当前节点拓扑、已验证能力、功能边界、未完成事项和修改守则。

参数分为 [football_runtime_common.yaml](params/football_runtime_common.yaml)（可复用运行参数）、[football_single_robot_simulation.yaml](params/football_single_robot_simulation.yaml)（单机仿真覆盖项）和 [football_team_simulation.yaml](params/football_team_simulation.yaml)（十机职责仿真）。单机 launch 会按此顺序合并，后者优先；实机 bringup 应复用前者并提供自己的机器人、传感器和 Nav2 参数文件。

## 功能边界

本包提供足球接近/推球状态机、球目标适配、多机器人代价地图障碍层、动态障碍预测、球融合和角色分配。`football_single_robot_simulation.launch.py` 使用轻量级动作服务器验证单台 striker 的闭环接口；实机使用 `football_real_robot.launch.py`，由标准 Nav2 controller/planner 和 `football_multi_robot_obstacle_layer` 负责局部/全局 rolling costmap。若机器人产品导航栈已由外部 bringup 启动，则只启动 `football_robot_runtime.launch.py`，避免重复启动 Nav2。

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

该 launch 启动十机假位姿、可配置移动球、唯一 `football_team_role_assigner` 和可视化。策略实现分别位于 `src/coordination/*_role_strategy.cpp`，角色权威只负责安全门控、前锋迟滞、身份绑定和消息发布。

团队仿真默认将球固定在场地中心 `(3.0, 0.0)`，`cyberdog_1` 至 `cyberdog_5` 放在同一侧，`cyberdog_6` 至 `cyberdog_10` 沿远端边线布置。A 队五台机器人分别启动与单机仿真一致的 GoalAdapter、轨迹适配器、Action Client 和轻量导航器：距离球最近者成为 striker，先导航到球后方，再完成接触和推球；其他角色进入对应的 `TACTICAL_*` 状态并跟踪各自 tactical target。团队可视化不固定 ego/striker，机器人标签直接显示角色权威发布的职责和当前位置。假世界和导航器共同保持机器人椭圆碰撞净空，默认安全净空为 `0.12 m`。

该启动文件会由 `football_simulation_input_publisher` 持续发布球、射门目标、比赛状态和 striker 身份。单障碍优先使用低开销的单峰平滑曲线；当多台机器人同时封住左右候选曲线时，规划器会沿前进方向搜索可切换绕行侧的多层横向走廊，并将结果插值为连续轨迹发布到 `/cyberdog_1/local_plan`。非推球阶段控制器只发布前向速度和转向速度，不发布横向侧移速度。

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

状态说明：`[x]` 表示已具备自动化/闭环覆盖；`[ ]` 表示必须保留现场记录后才能关闭，不能仅因代码路径存在而标记通过。

- [x] 单机路径规划验证：起点 -> 足球后位置 -> 推球到目标点。
  - 验收：状态依次出现 `NAV_TRANSIT`/`BALL_APPROACH`、`ALIGN_TO_GOAL`、`CONTACT_ACQUIRE`、`PUSH_BALL`；球沿 ball-to-target 方向前进，且推进阶段速度受 `push_speed_limit_mps` 限制。推球中球横向偏离超过 `push_realign_lateral_error_m` 时进入 `PUSH_REALIGN`，striker 暂停前推并左右移动，对准至进入阈值后重新触球继续推球。
  - 覆盖实现：`football_goal_adapter` 的接近/对齐/接触状态机，`football_simulation_input_publisher` 的低速推球模型，以及 `football_simulation_navigator` 的动作闭环。

- [x] 多机环境下，单机静态接近路径避障测试（轻量仿真规划器）。
  - 场景：`cyberdog_2` 位于 `(1.25,1.08)`，`cyberdog_10` 位于 `(1.4,0.3)`，两台机器人分别封住原直线和单峰曲线的候选侧；其余机器人保持可观测且不运动。
  - 已验证：规划器识别两个阻挡机器人并选择 `multi_obstacle_lattice`，生成 65 点平滑轨迹，规划椭圆边缘最小净空 `0.20 m`，大于 `collision_path_clearance_m: 0.08`；机器人完成绕障并依次进入 `BALL_APPROACH`、`CONTACT_ACQUIRE` 和 `PUSH_BALL`。
  - 关联参数：除椭圆净空和跟踪参数外，多障碍搜索由 `multi_obstacle_lattice_stations`、`multi_obstacle_lateral_step_m`、`multi_obstacle_max_lateral_m`、`multi_obstacle_max_lane_change_m` 和 `multi_obstacle_turn_penalty` 控制，见 [football_single_robot_simulation.yaml](params/football_single_robot_simulation.yaml)。实机 costmap 场景仍需单独验收。

- [ ] 推球走廊静态障碍专项测试。
  - 场景：在 ball-to-goal 走廊中放置静止机器人，球前 0.8--1.5 m；不得以穿过障碍物的方式继续推球。
  - 当前安全行为：进入减速/硬停车区时不再发布向障碍物推进的线速度，状态机可进入 `BLOCKED_RECOVERY`。后续需单独设计带球绕行或清障策略后再关闭此项。

- [x] 多机环境下，单机动态避障推球测试（轻量仿真规划器）。
  - 场景：`cyberdog_1` 作为 striker，其余九台机器人以 `0.08--0.18 m/s` 随机航点运动；移动机器人可横穿接近路径和推球走廊。
  - 已验证：35 秒内收到全部 10 路全局 odom，九台 peer 均有效移动；局部路径产生 150 种采样形状，状态进入 `PUSH_BALL`，控制命令最大横向速度为 `0.0 m/s`。全体外接椭圆最小净空 `0.024 m`，striker 与 peer 最小净空 `0.029 m`，未发生碰撞。近距离阻挡时状态进入 `BLOCKED_RECOVERY`，障碍离开后恢复接近或推球。
  - 关联实现：peer 随机航点、边界/椭圆碰撞约束和 `random_peer_min_clearance_m`，以及仿真导航器的障碍位姿变化触发重规划。实机 `multi_robot_obstacle_layer` 的预测扫掠区仍需在实机/完整 Nav2 场景单独验收。

- [ ] 多机环境下，双方 striker 竞争球场景（可选，建议在前两项稳定后执行）。
  - 同时给 A、B 队发布同一球位置，并让两个候选 striker 到球距离相近；重复测试距离相等、当前 striker 失联和球位置突变。
  - 验收：每队每个周期只发布一个有效 striker，角色切换受保持/超时逻辑约束；失联者降为 `STOP`，另一候选可接管；双方的代价地图均把对方视为障碍，不能因角色竞争相互穿透。
  - 关联实现：[football_team_role_assigner.cpp](src/coordination/football_team_role_assigner.cpp) 的唯一角色发布与 striker 重选逻辑。

- [ ] 多机环境下，足球轨迹预测及其他身份机器人调度。
  - 以至少两台观察机器人发布带时间戳的球观测，使球以不同速度和方向移动；同时注入一帧过期、时间偏差过大和离群观测。
  - 验收：`football_ball_fusion` 仅融合满足时间同步/内点条件的观测；`football_team_role_assigner` 使用预测球位更新 striker，并为 SUPPORT、DEFENDER_LEFT、DEFENDER_RIGHT、GOALKEEPER 发布不冲突的战术目标。球停下、观测超时或比赛状态非 `PLAY` 时，所有运动命令安全收敛。

- [ ] 实机测试。
  - 在每台狗完成定位、急停和限速检查后，按“静态 -> 动态 -> 竞争 -> 轨迹预测”顺序执行；每个场景先低速、低人数、空旷区域复现。
  - 验收：与仿真一致的状态转移和安全停车；TF、里程计、球观测及角色消息均无持续超时；保留 rosbag、参数快照、日志和现场视频。任一安全停车、定位或障碍观测异常均记为未通过，而非重试后覆盖结果。



- [ ] 推球接触偏差、球偏移及异常恢复测试。
  - 接触前横向偏差：将球分别放在机器人推进轴线左/右 `0.05`、`0.10`、`0.15`、`0.20 m`；同时覆盖机器人偏航左/右 `5°`、`10°`、`15°`。验收：偏差小于进入阈值时可稳定进入 `CONTACT_ACQUIRE`/`PUSH_BALL`；超过 `push_enter_lateral_error_m` 或 `push_enter_yaw_error_rad` 时必须先重新对齐，不能侧向硬推球。
  - 推送中球向左/右偏移：在 `PUSH_BALL` 后人为使球横向偏离目标走廊 `0.05`、`0.10`、`0.15`、`0.20 m`，并分别在低速和最大 `push_speed_limit_mps` 下测试。验收：小偏差应通过限速前进和偏航修正收敛；超过 `push_exit_lateral_error_m` 或 `push_exit_yaw_error_rad` 时退出推进，回到 `ALIGN_TO_GOAL`/`BALL_APPROACH` 重新获得站位；不得持续横移挤压球或将球推向错误球门方向。
  - 接触丢失与卡球：推进中使球突然前滚、被障碍物挡住、夹在机器人与边界之间，或机器人轮腿打滑导致球不动。验收：`push_contact_loss_sec`、`push_contact_progress_timeout_sec` 和 `push_contact_hard_timeout_sec` 到期后停止推进并进入恢复流程；恢复前不应继续输出推球速度。
  - 球位置突变与观测抖动：注入单帧跳变、左右高频摆动、丢帧、过期时间戳、未来时间戳，以及球速超过 `max_ball_speed_mps` 的观测。验收：非法观测不得清除最近有效球状态或触发危险目标更新；有效的新球位应受目标刷新阈值/保持时间约束，避免动作目标来回跳变。
  - 球在边界、角落或球门附近：将球放在场地边线、角点、球门线前和目标点重合处。验收：接近点和局部推球目标始终在场地安全边界内；无可行的球后方站位时进入安全停止/恢复，不能把机器人或球推出边界。
  - 障碍物与球重叠：在球周围、机器人与球之间、以及球前推进走廊分别放置静态/动态机器人。验收：不将机器人障碍物误识别为球；障碍阻塞时按动态安全距离减速或停止，障碍离开后重新评估球后方接近位，而非沿旧目标盲推。
  - 关联参数：`push_enter_lateral_error_m`、`push_exit_lateral_error_m`、`push_enter_yaw_error_rad`、`push_exit_yaw_error_rad`、`push_contact_*`、`max_ball_jump_m`、`max_ball_speed_mps`、`ball_protection_radius_m`、`ball_boundary_margin_m`。


## 实机路径规划测试入口

在暂时没有足球感知时，使用单机测试入口。它只发布静态球、射门目标、`cyberdog_1` striker 和 `PLAY` 状态，不发布或覆盖实时 `/global_vio/cyberdog_1/odom`：

```bash
ros2 launch football_navigation football_real_robot_test.launch.py \
  ball_x:=2.0 ball_y:=0.0 kick_target_x:=6.0 kick_target_y:=0.0
```

该入口消费实时全局 VIO（激光默认关闭），启动 Nav2、路径规划和 `cmd_vel`
看门狗适配器。`ball_x/ball_y` 是足球坐标，`kick_target_x/kick_target_y`
定义踢球方向；机器人首先导航到足球反方向、距球约 `approach_distance_m`
的接近点，并不是直接导航到 `kick_target`。适配器将 Nav2 的 `cmd_vel`
限幅后转换为慢走 `MotionServoCmd`，命令超过 0.25 秒未更新会发送一次零速
结束帧。测试前必须确认机器人处于慢走 303，且键盘急停终端在线。该适配器
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
