# 足球导航

## 功能边界

本包提供足球接近/推球状态机、球目标适配、多机器人代价地图障碍层、动态障碍预测、球融合和角色分配。`football_single_robot_simulation.launch.py` 使用轻量级动作服务器验证单台 striker 的闭环接口，并包含基于 10 路全局 odom 的局部绕障航点、减速和硬停车逻辑。该轻量规划器用于仿真功能验证；实机仍由加载 `football_multi_robot_obstacle_layer` 的 Nav2 局部规划器负责。

## 运行前检查

每次场景测试前均应确认：

- 所有机器人在同一 `tag_global` 坐标系，且 `/global_vio/<namespace>/odom` 时间戳新鲜。
- 代价地图同时加载 `obstacle_layer`、`football_multi_robot_obstacle_layer` 和 `inflation_layer`，参考 [football_costmap_reference.yaml](params/football_costmap_reference.yaml)。
- `football_goal_adapter` 已收到 9 台其他机器人的里程计；默认 `minimum_other_robot_count: 9`，缺失数据时应 fail-closed。
- 记录 `/football/ball_pose`、`/<robot>/football/state`、`/<robot>/football/approach_pose`、`/<robot>/speed_limit`、`/<robot>/cmd_vel`、局部/全局 costmap 及 RViz 视频。

单机闭环冒烟测试：

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch football_navigation football_single_robot_simulation.launch.py use_rviz:=true
```

该启动文件会由 `football_simulation_input_publisher` 持续发布球、射门目标、比赛状态和 striker 身份。单障碍优先使用低开销的单峰平滑曲线；当多台机器人同时封住左右候选曲线时，规划器会沿前进方向搜索可切换绕行侧的多层横向走廊，并将结果插值为连续轨迹发布到 `/cyberdog_1/local_plan`。非推球阶段控制器只发布前向速度和转向速度，不发布横向侧移速度。

其他机器人初始位置均可在 [football_single_robot_simulation.yaml](params/football_single_robot_simulation.yaml) 中修改：

```yaml
initial_pose_cyberdog_2: [1.25, 1.08, 0.0]  # [x, y, yaw]
initial_pose_cyberdog_3: [-4.0, -2.5, 0.0]
# ...一直到 cyberdog_10
```

将某个 `initial_pose_cyberdog_N` 删除或设为 `[]`，并设置 `randomize_initial_poses: true`，可只随机生成该机器人的位置。配置会检查场地边界和 `minimum_initial_separation_m`，重叠位置会拒绝启动。

仿真避障使用随机器人 yaw 旋转的矩形外接椭圆。对于默认 `0.562 x 0.339 m`
碰撞矩形，先取最小外接椭圆半轴 `length/sqrt(2)`、`width/sqrt(2)`，再分别
增加可配置的 `collision_ellipse_expansion_m`。默认扩展 `0.05 m` 后，单台机器人
椭圆半轴约为 `0.447 x 0.290 m`。RViz 会在每台机器人的独立 MarkerArray 话题中
同时显示实体和半透明椭圆区域，坐标文本使用紧凑的 `(x,y)` 格式。

## 待办测试清单

状态说明：`[x]` 表示已具备自动化/闭环覆盖；`[ ]` 表示必须保留现场记录后才能关闭，不能仅因代码路径存在而标记通过。

- [x] 单机路径规划验证：起点 -> 足球后位置 -> 推球到目标点。
  - 验收：状态依次出现 `NAV_TRANSIT`/`BALL_APPROACH`、`ALIGN_TO_GOAL`、`CONTACT_ACQUIRE`、`PUSH_BALL`；球沿 ball-to-target 方向前进，且推进阶段速度受 `push_speed_limit_mps` 限制。
  - 覆盖实现：`football_goal_adapter` 的接近/对齐/接触状态机，`football_simulation_input_publisher` 的低速推球模型，以及 `football_simulation_navigator` 的动作闭环。

- [x] 多机环境下，单机静态接近路径避障测试（轻量仿真规划器）。
  - 场景：`cyberdog_2` 位于 `(1.25,1.08)`，`cyberdog_10` 位于 `(1.4,0.3)`，两台机器人分别封住原直线和单峰曲线的候选侧；其余机器人保持可观测且不运动。
  - 已验证：规划器识别两个阻挡机器人并选择 `multi_obstacle_lattice`，生成 65 点平滑轨迹，规划椭圆边缘最小净空 `0.20 m`，大于 `collision_path_clearance_m: 0.08`；机器人完成绕障并依次进入 `BALL_APPROACH`、`CONTACT_ACQUIRE` 和 `PUSH_BALL`。
  - 关联参数：除椭圆净空和跟踪参数外，多障碍搜索由 `multi_obstacle_lattice_stations`、`multi_obstacle_lateral_step_m`、`multi_obstacle_max_lateral_m`、`multi_obstacle_max_lane_change_m` 和 `multi_obstacle_turn_penalty` 控制，见 [football_single_robot_simulation.yaml](params/football_single_robot_simulation.yaml)。实机 costmap 场景仍需单独验收。

- [ ] 推球走廊静态障碍专项测试。
  - 场景：在 ball-to-goal 走廊中放置静止机器人，球前 0.8--1.5 m；不得以穿过障碍物的方式继续推球。
  - 当前安全行为：进入减速/硬停车区时不再发布向障碍物推进的线速度，状态机可进入 `BLOCKED_RECOVERY`。后续需单独设计带球绕行或清障策略后再关闭此项。

- [ ] 多机环境下，单机动态避障推球测试。
  - 让一台非 striker 机器人以 0.08--0.25 m/s 横穿接近路径和推球走廊；先执行远离球的横穿，再执行迎面/横向接近。
  - 验收：`multi_robot_obstacle_layer` 同时标记当前位置和预测扫掠区域；动态安全距离随闭合速度增大；机器人在安全距离内减速或停车，障碍离开并满足退出阈值后才恢复。不得发生碰撞、持续震荡或使用过期障碍数据。
  - 关联实现：障碍层的 `enable_prediction`、`prediction_horizon_sec`、`max_prediction_distance_m`，以及目标适配器的闭合速度和制动距离计算。

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


## 结果记录模板

每次执行在下表新增一行，只有“证据”完整时才将相应待办勾选为 `[x]`。

| 日期 | 场景 | 软件/参数版本 | 结果 | 证据（rosbag、日志、视频） | 问题与后续动作 |
| --- | --- | --- | --- | --- | --- |
|  |  |  | 待执行 |  |  |
