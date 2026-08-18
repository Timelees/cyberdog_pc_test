# football_navigation 当前能力与边界交接

> 面向后续 Codex/Agent 的代码交接文档。内容基于 2026-08-18 当前工作区源码，描述“代码已经实现什么、当前仿真验证了什么、哪些能力仍依赖外部系统”。修改功能前应先阅读本文件、[README.md](README.md) 和对应参数文件。

## 1. 当前目标和系统边界

`football_navigation` 面向 10 台 CyberDog 足球场景：

- A 队：`cyberdog_1`～`cyberdog_5`。
- B 队：`cyberdog_6`～`cyberdog_10`。
- 当前单机仿真固定使用 `cyberdog_1` 作为 striker。
- 统一全局坐标系为 `tag_global`。
- 机器人全局位姿接口为 `/global_vio/{namespace}/odom`。
- 当前包负责足球目标生成、状态机、导航目标适配、仿真闭环、多机器人障碍插件、角色分配、球融合和可视化。
- 当前包不负责 VIO/AprilTag 定位本身；全局 odom 由外部 `mutil_robot_odom`/全局位姿链路提供。
- 当前包不负责定位、PC authority 或底盘驱动 bringup；现有 launch 分为 `football_single_robot_simulation.launch.py`、仅启动足球控制链的 `football_robot_runtime.launch.py` 和带可选标准 Nav2 的 `football_real_robot.launch.py`。

必须区分四类完成度：

1. **已实现并有轻量仿真闭环记录**：单 striker 接近、基于最新 peer 位姿的反应式重规划、对齐、触球和推球；历史记录详见第 12 节。
2. **当前轻量仿真代码已实现但缺少独立留证**：足球禁入圆弧、lattice 失败后的二维 A* 和默认全向近障恢复；JSONL 已记录短时全向 lattice 跟踪，但不构成这些分支的闭环验收。
3. **代码已实现但未由当前单机 launch 启动**：球融合、10 机角色权威、Nav2 多机器人 costmap 插件及实机动作链路。
4. **尚未完成现场验收**：完整 Nav2 动态预测避障、双方 striker 竞争、全角色协同和实机测试。

## 2. 代码分层

| 目录 | 职责 | 关键文件 |
| --- | --- | --- |
| `src/core`、`include/.../core` | 坐标、角度、球后接近位、触球几何、外接椭圆等纯算法 | `football_geometry.*` |
| `src/control`、`include/.../control` | 足球状态机、目标转换、Nav2 Action 桥接 | `football_goal_adapter.*`、`football_trajectory_adapter.*`、`football_tracking_action_client.*` |
| `src/coordination`、`include/.../coordination` | 多机球融合、角色和战术目标分配 | `football_ball_fusion.*`、`football_team_role_assigner.*`、`*_role_strategy.*` |
| `src/simulation`、`include/.../simulation` | 单机轻量仿真、假球、假机器人、轻量导航 Action Server | `football_*simulation*`、`football_fake_*` |
| `src/plugins`、`include/.../plugins` | Nav2 costmap、进度检查和 DWB critic | `multi_robot_obstacle_layer.*`、`dynamic_obstacle_marker.*` |
| `src/visualization`、`include/.../visualization` | RViz MarkerArray 分话题显示 | `football_visualization_node.*` |
| `params` | 当前仿真覆盖参数和后续实机可复用运行参数 | 见第 11 节 |

类声明放在同层级 `include/football_navigation/<responsibility>/`，实现和 `_node.cpp` 入口放在对应 `src/<responsibility>/`。后续新增节点应继续遵循该结构。

## 3. 当前单机仿真拓扑

启动文件：`launch/football_single_robot_simulation.launch.py`。

它启动以下节点：

| 节点 | 当前仿真职责 |
| --- | --- |
| `football_simulation_input_publisher` | 发布 cyberdog_1 odom、固定初始球、kick target、比赛状态和 striker；积分 cyberdog_1 的 `cmd_vel`，并模拟低速推球 |
| `football_fake_other_robot_publisher` | 发布 cyberdog_2～10 的假里程计/全局里程计；默认静止，随机航点可由配置开启 |
| `football_goal_adapter` | 根据球、kick target、striker 位姿和 9 台障碍位姿生成 approach/tracking 目标、控制状态和速度上限 |
| `football_trajectory_adapter` | 校验并转发目标，提供新鲜 heartbeat；不负责规划 |
| `football_tracking_action_client` | 将有效 tracking pose 发送给 `NavigateToPose` Action Server |
| `football_simulation_navigator` | 仿真专用轻量 `NavigateToPose` Action Server，生成局部绕障路径并发布 `cmd_vel` |
| `football_visualization_node` | 将场地、10 台机器人、球、目标、路径、命令和状态拆分到多个 MarkerArray 话题 |

单机仿真**不会启动**：

- `football_ball_fusion`；仿真输入直接发布最终 `/football/ball_pose`。
- `football_team_role_assigner`；仿真输入直接把 cyberdog_1 发布为 striker。
- 真正的 Nav2 Controller/Planner/Costmap；由轻量 `football_simulation_navigator` 代替。
- `football_fake_ball_publisher`；单机 launch 使用 `football_simulation_input_publisher` 内置的简化推球模型。

## 3.1 十机职责分配仿真

启动文件为 `launch/football_team_simulation.launch.py`，配置为
`params/football_team_simulation.yaml`。它是独立仿真入口，不改变上面的单机拓扑，启动：

- `football_fake_other_robot_publisher`：一次性模拟十台机器人并输出 `/global_vio/{namespace}/odom`。
- `football_team_simulation_input_publisher`：发布 `tag_global` 下的固定或正弦移动球位姿。
- `football_team_role_assigner`：执行两队五人 roster 校验、前锋迟滞切换、完整十机数据安全门控和角色话题发布。
- `football_visualization_node`：显示十台机器人和球；不参与决策。

角色行为计算已从权威节点拆成独立、无 ROS 状态的策略类，每个角色一个实现文件：

| 角色 | 实现 |
| --- | --- |
| `STRIKER` | `coordination/striker_role_strategy.cpp` |
| `SUPPORT` | `coordination/support_role_strategy.cpp` |
| `DEFENDER_LEFT` | `coordination/defender_left_role_strategy.cpp` |
| `DEFENDER_RIGHT` | `coordination/defender_right_role_strategy.cpp` |
| `GOALKEEPER` | `coordination/goalkeeper_role_strategy.cpp` |

`football_role_strategies` 是可复用库；节点仍保留现有 fail-closed 逻辑和角色绑定顺序，因此单机仿真行为不受影响。当前策略目标保持既有公式：支援位于球后侧方、两后卫固定在本方防守线两侧、守门员随球横向移动；复杂匹配和实机闭环仍需单独验收。

团队仿真默认球位于 `(3.0, 0.0)` 且不做随机运动。A 队五台机器人启动完整足球控制链，最近球的机器人会按单机 striker 状态机执行球后接近、接触和推球；其他四台保留 `require_striker_role: true`，因此分别进入 `TACTICAL_SUPPORT`、`TACTICAL_DEFENDER_LEFT`、`TACTICAL_DEFENDER_RIGHT`、`TACTICAL_GOALKEEPER`，而不是被错误地全部视为 striker。团队 RViz 不指定 `self_namespace`，标签显示角色权威发布的真实职责和位置。`football_fake_other_robot_publisher.random_peer_min_clearance_m` 默认设为 `0.12 m`，同时轻量导航器按全部十台机器人 odom 做路径净空检查。

## 4. 核心话题和坐标约束

| 接口 | 类型/语义 | 边界约束 |
| --- | --- | --- |
| `/global_vio/{namespace}/odom` | `nav_msgs/Odometry`，10 台机器人全局位姿 | `header.frame_id` 必须是 `tag_global`；必须带有效时间戳和合法四元数 |
| `/football/ball_pose` | `geometry_msgs/PoseStamped`，下游认可的全局球位姿 | 必须在 `tag_global`、场地边界内且未超时；实机建议只由 fusion 输出 |
| `/football/team_a/striker`、`team_b/striker` | 当前前锋 namespace | 真实多机模式应由唯一角色权威发布；单机仿真直接固定 cyberdog_1 |
| `/football/team_<a|b>/kick_target` | 球希望到达的全局位置 | 它不是机器人导航目标；GoalAdapter 会据此计算球后点和推进方向 |
| `/<robot>/football/approach_pose` | 当前机器人导航目标 | 由 GoalAdapter 输出；坐标系由 `target_frame` 配置决定 |
| `/<robot>/football/control_valid` | 当前是否允许执行控制 | 为 false 时 trajectory adapter 清空缓存，action client 取消目标 |
| `/<robot>/football/state` | 足球控制状态字符串 | 用于速度限制、仿真控制和诊断，不应被当成通用 Nav2 状态 |
| `/<robot>/speed_limit` | `nav2_msgs/SpeedLimit` | GoalAdapter 按状态和动态障碍距离发布 |
| `/<robot>/cmd_vel` | 机器人速度命令 | 当前仿真默认允许非接触阶段使用 `linear.x`、`linear.y` 和 `angular.z` 脱困；`CONTACT_ACQUIRE`/`PUSH_BALL` 强制 `linear.y = 0` |
| `/<robot>/local_plan` | 轻量仿真局部路径 | 只用于仿真验证，不等价于实机 Nav2 local plan |

禁止仅修改 `frame_id` 字符串来伪装坐标转换。`tag_global`、`base_link`/机器人本地系以及实机使用的 `vodom` 必须通过真实变换或明确的几何转换连接。

## 5. 足球控制状态机

主要实现位于 `control/football_goal_adapter.cpp`。典型流程：

```text
SEARCH_BALL / SAFE_STOP
          |
          v
NAV_TRANSIT -> OBSTACLE_APPROACH -> BALL_APPROACH
                                      |
                                      v
                               ALIGN_TO_GOAL
                                      |
                                      v
                               CONTACT_ACQUIRE
                                      |
                                      v
                                  PUSH_BALL

任意运动阶段 --障碍过近/无进展--> BLOCKED_RECOVERY
任意阶段 --输入无效/角色丢失/比赛停止--> SAFE_STOP
```

状态含义与边界：

| 状态 | 已实现行为 | 约束 |
| --- | --- | --- |
| `SEARCH_BALL` | 等待合法球数据 | 不发布有效控制目标 |
| `NAV_TRANSIT` | 前往球后方接近区域 | 球、odom、角色和队友数据必须新鲜 |
| `OBSTACLE_APPROACH` | 障碍附近降速接近 | 不是独立全局规划器，只改变目标/速度和安全状态 |
| `BALL_APPROACH` | 低速接近球后点 | 带进入/退出迟滞，避免状态抖动 |
| `ALIGN_TO_GOAL` | 原地或小范围调整推球方向 | 有最大持续时间、累计旋转和进展 watchdog |
| `CONTACT_ACQUIRE` | 低速建立机器人前端与球接触 | 使用矩形机器人与圆形球的几何接触判断 |
| `PUSH_BALL` | 沿 ball-to-kick-target 方向推进 | 必须满足横向偏差、航向偏差和接触稳定条件 |
| `BLOCKED_RECOVERY` | 停止/让行并等待重规划或障碍离开 | 当前不是完整的带球绕障策略 |
| `SAFE_STOP` | 撤销控制有效性和运动目标 | 输入过期、比赛不允许、角色丢失等均可触发 |

GoalAdapter 自身不直接驱动机器人。它发布目标、状态、`control_valid` 和速度上限；最终运动命令由 Nav2 或仿真 navigator 产生。

### Fail-closed 条件

默认实机参数要求：

- 本机 odom 新鲜。
- 球位姿新鲜、时间戳不在未来且速度/跳变合法。
- striker/role 信息新鲜且本机确实是 striker。
- 比赛状态允许当前队运动。
- 10 机模式下收到 9 台其他机器人的新鲜 odom；默认 `minimum_other_robot_count: 9`。
- kick target 合法并与球保持最小距离。
- 目标、球后点和推进目标保持在场地安全边界内。

不要为“让仿真继续跑”绕过这些检查；仿真 launch 应显式提供所需数据。

## 6. 路径规划与动态避障

### 6.1 轻量仿真规划器

`football_simulation_navigator` 是仿真专用 Action Server：

- 每个控制周期先检查近障净空恢复，再检查非接触阶段的足球禁入圆，最后运行常规机器人绕障规划。
- 常规规划依次尝试直线、单峰平滑曲线和多层横向 lattice；前向 lattice 无解时，使用有界二维 A* 搜索，允许路径先横移/后退绕到障碍群外侧，再朝目标前进；全部无解才进入 `hard_stop`。
- 足球位姿新鲜且尚未进入 `CONTACT_ACQUIRE`/`PUSH_BALL` 时，将足球视为半径 `ball_avoidance_radius_m` 的禁入圆。直线路径切入禁区时同时搜索顺/逆时针圆弧，并逐级扩大半径至 `ball_avoidance_max_radius_m`；目标本身位于禁区内或没有无碰撞圆弧时使用 `ball_hard_stop`。
- 使用 lookahead 点跟踪局部路径；绕球圆弧使用更短的 `ball_avoidance_lookahead_m`，避免控制器切过圆弧内侧碰球。
- `enable_holonomic_avoidance: true` 是当前默认配置：非接触阶段将世界系路径方向投影为机体系 `linear.x/linear.y`，可倒退或横移，并同时跟踪路径航向。只有关闭该参数时才恢复“大角度先原地转向、`linear.y = 0`”的非完整约束控制。
- `CONTACT_ACQUIRE` 和 `PUSH_BALL` 不执行绕球/净空恢复，且始终禁止横移，只沿机体前向推进并用角速度修正推球走廊。
- 缓存局部路径，但其他机器人平移超过 `dynamic_replan_translation_m`、旋转超过 `dynamic_replan_yaw_rad` 或障碍数量变化时重新规划。
- `dynamic_replan_min_period_sec` 限制重规划频率。
- 椭圆净空低于 slowdown/hard-stop 阈值时减速或停车。
- 当前净空低于 `clearance_recovery_trigger_clearance_m` 时，非接触阶段会在 24 个方向和多个搜索距离中寻找逐采样点净空不减、终点达到退出净空的路径。恢复期间保持机身 yaw，允许倒退/横移，使用进入/退出迟滞并复用仍安全的逃逸方向，避免在狭窄通道两侧来回切换；没有合格路径时才维持硬停车。
- 默认把路径变化或每 `diagnostic_state_period_sec` 的状态写入 `/tmp/football_navigation_dynamic_avoidance.jsonl`。记录包含状态、机器人/目标/球、最近净空、策略、速度命令、选定 peer 位姿以及路径更新时的完整路径；每次节点启动以截断模式覆盖旧文件。

边界：

- 这是用于接口和行为验证的轻量规划器，不具备完整 Nav2 地图、传感器融合、全局可达性或复杂动态预测能力。
- 轻量规划器只使用各 peer 最新且未超时的位姿快照；所谓“动态避障”是位姿变化触发的反应式重规划，不估计 peer 速度，也不预测未来占用。速度/角速度预测只存在于第 6.2 节的 Nav2 costmap 插件。
- 曲线、lattice 和二维 A* 都是有限采样搜索，不保证任意密集障碍场景存在解，也没有多机器人互惠避让或死锁消解协议。
- GoalAdapter 的 `BLOCKED_RECOVERY` 与 navigator 的 `clearance_recovery` 是两套不同机制：前者是足球状态机的停止/让行状态，后者是仿真局部控制器的主动增距路径，不能混为同一能力。
- 推球走廊被动态机器人持续占用时会暂停推球，不会自动设计复杂的带球绕行战术。
- 当前 A* 成功后路径确实会被采用，但 `buildSmoothLocalPath()` 末尾会把除 `multi_obstacle_lattice` 外的策略名统一改成 `single_smooth_curve`；因此现有 JSONL 的 `strategy` 字段不能可靠区分二维 A* 与单曲线，后续若依赖策略统计应先修正该标签。

### 6.2 实机 Nav2 插件

`football_navigation::MultiRobotObstacleLayer` 已实现并注册为 Nav2 costmap layer：

- 从 `/global_vio/{namespace}/odom` 读取其他机器人。
- 支持有向矩形 footprint 或圆形 fallback。
- 标记机器人当前位置。
- 根据估计线速度/角速度标记预测位置和扫掠区域。
- 支持障碍超时、消息年龄、未来时间容差、最大速度/角速度和最小障碍数量。
- 应与激光 `obstacle_layer` 和 `inflation_layer` 一起使用。

同时注册：

- `FootballProgressChecker`：将有限原地转向视为有效进展，避免足球对齐被误判为 stalled。
- `FootballPreferForwardCritic`：DWB 前进偏好评分。

边界：

- 当前仅保留轻量仿真参数；实机 Nav2 costmap 的参数仍需在后续实机 bringup 中另行维护。
- 当前工作区未通过实机或完整 Nav2 launch 对动态预测扫掠区进行闭环验收。
- 轻量仿真通过不等于 Nav2 插件实机通过。

### 6.3 实机启动入口

`launch/football_real_robot.launch.py` 是实机测试入口，默认启动本包控制链和标准 Nav2 的 controller、planner、recoveries、BT navigator、lifecycle manager。它不启动以下外部系统：

- VIO/AprilTag 或 `/global_vio/{namespace}/odom` 发布器。
- `football_ball_fusion`、`football_team_role_assigner`、比赛状态/角色权威和球检测。
- 激光、底盘驱动、急停或速度仲裁节点。

启动前必须确认：

- 机器人 base frame 与 `tag_global` 之间存在可用 TF，且每台机器人的 odom 消息 `header.frame_id` 为 `tag_global`。
- 本机和其余 9 台机器人持续发布 `/global_vio/{namespace}/odom`；默认 `minimum_obstacle_count: 9`，缺失数据会令 costmap 不 current，Action Client 不发送目标。
- `scan_topic` 指向本机 `sensor_msgs/LaserScan`；默认相对话题 `scan` 会解析为 `/<robot>/scan`，底盘发布全局话题时可启动传入 `scan_topic:=/scan`。
- 外部系统发布 `/football/ball_pose`、队伍 striker、角色、kick target 和 `/football/match_state`；本包不会为了实机测试伪造这些输入。

建议先启动安全模式：

```bash
ros2 launch football_navigation football_real_robot.launch.py \
  robot_namespace:=cyberdog_1 enable_motion:=false
```

确认 TF、odom、动态障碍层日志和两个 costmap 均正常后，再显式启用运动：

```bash
ros2 launch football_navigation football_real_robot.launch.py \
  robot_namespace:=cyberdog_1 enable_motion:=true
```

`enable_motion` 只控制本包 `football_tracking_action_client` 是否发送 `NavigateToPose`；不会绕过 GoalAdapter 的输入校验、角色校验、比赛状态或 costmap 就绪检查。若机器人已有外部 Nav2，可直接使用 `football_robot_runtime.launch.py`，或使用总入口并传入 `start_nav2:=false`，避免重复启动 controller/action server。

外部 CyberDog 产品导航栈可能使用 `local_costmap_tracking/costmap` 和 `rolling_window_costmap/costmap`，而本包标准 Nav2 配置使用 `local_costmap/costmap` 和 `global_costmap/costmap`。接入外部栈时应复制 `football_real_robot.yaml`，通过 `robot_params_file` 覆盖两个 readiness 话题、Action 名称和 frame；不要用 `require_costmap_ready:=false` 掩盖接口不匹配。

实机参数文件职责：

| 文件 | 用途 |
| --- | --- |
| `params/football_real_robot.yaml` | 本包实机速度、目标心跳、Action Client readiness 和可视化覆盖 |
| `params/football_nav2_real.yaml` | 本工作区标准 Nav2 的 rolling local/global costmap、peer 动态预测层、激光障碍层、DWB 和 lifecycle 参数 |

该标准 Nav2 配置是无地图 rolling costmap 验证配置，不替代 CyberDog 产品导航栈的完整参数；使用产品导航栈时保留 `football_robot_runtime.launch.py` 的控制链，并在其产品参数中加载 `football_navigation/MultiRobotObstacleLayer`。

## 7. 机器人碰撞模型

默认机器人矩形尺寸为 `0.562 x 0.339 m`。碰撞规划和可视化使用随 yaw 旋转的外接椭圆：

```text
semi_major = length / sqrt(2) + collision_ellipse_expansion_m
semi_minor = width  / sqrt(2) + collision_ellipse_expansion_m
```

默认 `collision_ellipse_expansion_m = 0.05 m`，半轴约为 `0.447 x 0.290 m`。

仿真约束：

- `football_fake_other_robot_publisher` 在全局坐标系计算场地边界和有向椭圆净空。
- 9 台 peer 之间、peer 与外部驱动的 cyberdog_1 之间均进行净空约束。
- `random_peer_min_clearance_m` 默认额外保留 `0.03 m` 动态净空。
- 接近边界或机器人后会停止该周期运动并重新采样随机航点。
- striker 仍必须由 navigator 主动避障；peer 让行修正是仿真世界的防穿透保险，不应替代规划器安全逻辑。

## 8. 九台 peer 随机运动仿真

实现位于 `simulation/football_fake_other_robot_publisher.cpp`，默认配置在 `football_single_robot_simulation.yaml`：

- `random_peer_motion_enabled: false`，默认保持静态障碍，方便可重复验证。
- 固定 `random_seed: 2026`，场景可复现。
- 每台 peer 独立随机航点、速度和航点有效期。
- 默认速度范围 `0.08～0.18 m/s`。
- 运动模型只前进和转向；航向误差过大时先旋转。
- 航点不得太近，并尽量避开其他机器人的当前位置。
- 随机、scripted 和 continuous demo 三种 peer 运动模式互斥，最多只能启用一种。
- `simulate_selected_robot: false` 时 fake publisher 不生成 cyberdog_1，而是订阅 cyberdog_1 的全局 odom 参与碰撞约束。
- 默认 `cyberdog_2` 是键盘干扰机器人：它订阅 `/cyberdog_2/cmd_vel`，显示为红色，并且不受随机、scripted 或 continuous demo 运动模式驱动。其初始位置使用 `initial_pose_cyberdog_2` 配置；通过独立终端运行 `ros2 run football_navigation football_keyboard_robot_controller` 控制。

要恢复静态障碍测试：

```yaml
random_peer_motion_enabled: false
scripted_peer_motion_enabled: false
continuous_demo_enabled: false
```

初始位置通过 `initial_pose_cyberdog_2`～`initial_pose_cyberdog_10` 配置。删除某项或设为 `[]`，并启用 `randomize_initial_poses`，可对缺失项随机布置；非法边界或初始重叠会拒绝启动。

## 9. 协调能力边界

### 9.1 球融合

`football_ball_fusion` 支持：

- `global_external`：校验已经在 `tag_global` 的球输入，然后发布 `/football/ball_pose`。
- `local_multi_robot`：接收各机器人本地球检测及对应全局 odom，按时间匹配转换到全局坐标，使用内点集合融合。
- 校验检测/odom 年龄、未来时间、检测与 odom 时间差、融合时间差、场地边界和最大球速。
- 进球事件后清空历史。

边界：当前单机 launch 不启动 fusion；多机器人本地感知融合尚未完成闭环场景验证。

### 9.2 角色权威

`football_team_role_assigner` 支持：

- 强制校验 A/B 两队各 5 台且 namespace 范围正确。
- 订阅 10 路全局 odom 和融合球位姿。
- 每队分配 `STRIKER`、`SUPPORT`、`DEFENDER_LEFT`、`DEFENDER_RIGHT`、`GOALKEEPER`；无效时发布 `STOP`。
- 发布每台机器人的 tactical target、每队 striker 和 kick target。
- striker 保持时间、收益阈值和切换确认，抑制频繁换人。
- 支持 `STOP`、`READY`、`PLAY`、`KICKOFF_A`、`KICKOFF_B`、`FINISHED`。
- 使用 `/football/role_authority` 检测重复权威；冲突时 fail-closed。
- 队伍数据过期、时间偏差过大或没有合法 striker 时安全停止角色输出。

边界：

- 当前角色节点只接受 `odom_source_mode: pc_forwarded`。
- 当前单机 launch 不启动角色权威，而是固定 cyberdog_1。
- 双方 striker 竞争、完整五角色协同和权威切换仍未做完整现场验收。

## 10. 可视化

可视化已按职责拆分话题：

| 内容 | MarkerArray 话题 |
| --- | --- |
| 场地边界、中线、中心圆、球门 | `/football/markers/field` |
| 每台机器人实体、文本、碰撞椭圆 | `/football/markers/robots/{namespace}` |
| 足球实体和坐标文本 | `/football/markers/ball` |
| approach pose | `/football/markers/approach_pose` |
| tracking pose | `/football/markers/tracking_pose` |
| 全局/局部路径 | `/football/markers/paths` |
| kick/goal 标记 | `/football/markers/goals` |
| costmap | `/football/markers/costmap` |
| cmd_vel/运动命令面板 | `/football/markers/commands` |
| 状态面板 | `/football/markers/status` |

边界和布局：

- 场地中心默认 `(0,0)`，尺寸默认 `16 x 8 m`。
- 机器人/足球文字与实体绑定并随位置移动，格式为紧凑 `(x,y)`。
- 命令和状态文本固定在场地外，避免遮挡场内数据。
- 每台机器人矩形实体与其外接椭圆发布到同一个该机器人 MarkerArray 话题，不另拆碰撞话题。
- 默认显示足球禁入圆；其半径必须与 navigator 的 `ball_avoidance_radius_m` 同步。
- RViz 只是显示，不参与碰撞判定或控制。

## 11. 参数文件职责

| 文件 | 用途和注意事项 |
| --- | --- |
| `params/football_single_robot_simulation.yaml` | 单机仿真场景、假世界、轻量规划器、动态避障诊断和 RViz 专属参数；由仿真 launch 直接加载 |
| `params/football_runtime_common.yaml` | GoalAdapter、轨迹适配与 Nav2 Action 桥接的通用运行默认值；由实机 runtime launch 先加载，再以实机参数覆盖 |
| `params/football_real_robot.yaml` | 实机足球控制链覆盖，不包含底盘或定位参数 |
| `params/football_nav2_real.yaml` | 本包提供的标准 Nav2 无地图实机动态避障验证配置 |
| `params/football_team_simulation.yaml` | 十机职责分配仿真、移动球和全局假 odom 配置；仅由团队仿真 launch 加载 |

单机仿真参数集中在 `football_single_robot_simulation.yaml`；职责仿真参数集中在 `football_team_simulation.yaml`。修改碰撞尺寸、场地边界、超时或速度时，先核对当前 launch 实际加载的文件。

## 12. 已完成验证

在 `galactic_docker` 中已完成：

- `colcon build --symlink-install --packages-select football_navigation` 编译通过。
- 单机从原点到球后点、对齐、触球并进入 `PUSH_BALL`。
- 多静态障碍使用 `multi_obstacle_lattice` 生成 65 点平滑轨迹并绕开阻挡。
- 10 路 `/global_vio/.../odom` 全部可观测。
- cyberdog_2～10 九台 peer 全部随机移动，cyberdog_1 同时执行动态避障推球。
- 35 秒动态测试中出现 150 种局部路径采样形状，证明动态重规划触发。
- 该次历史动态测试中 `cmd_vel.linear.y` 最大值为 `0.0 m/s`；它验证的是当时的前进/转向控制，不覆盖当前默认开启的 `enable_holonomic_avoidance` 横移/倒退恢复路径。
- 全体机器人外接椭圆最小净空 `0.024 m`；striker 与 peer 最小净空 `0.029 m`，未发生碰撞。
- 状态到达 `BALL_APPROACH -> ALIGN_TO_GOAL -> CONTACT_ACQUIRE -> PUSH_BALL`；动态近距离阻挡时进入 `BLOCKED_RECOVERY`。
- Docker 内现存 2026-08-18 的 JSONL 短时记录共 7 行、约 3.2 秒：策略为 `multi_obstacle_lattice`，首条全向命令为 `[0.4694, -0.1304, -0.6504]`，证明当前 `enable_holonomic_avoidance` 配置会实际输出非零 `linear.y` 并跟踪 lattice 路径。

验证边界：

- 上述数据来自轻量仿真，不是实机或完整 Nav2 验收。
- 当前 JSONL 只覆盖短时 `multi_obstacle_lattice` 跟踪，未到目标，也未出现 `ball_arc_*`、`outer_grid_bypass` 或 `clearance_recovery`。不能据此声称足球圆弧、二维 A* 或全向净空恢复已完成独立闭环验收；这些仍属于“代码已实现、待按当前参数留证”。
- 当前源码树没有 `test/` 目录；CMake 中测试目标均由 `if(EXISTS ...)` 条件创建。因此 `colcon test` 很可能不执行任何当前包测试，不能把“命令快速结束”报告为测试通过。
- 工作区 `build/football_navigation/test_results` 可能包含历史 XML；读取结果前先检查时间戳，不能将旧结果归入当前修改。

## 13. 尚未完成或明确不保证的能力

- 推球走廊被持续占用时的主动带球绕行/战术清障。
- 任意密集动态障碍下的完备路径规划和无死锁保证。
- 当前默认全向净空恢复、足球圆弧保护和二维 A* fallback 的独立可复现实验及日志留证。
- 完整 Nav2 costmap 动态预测、Controller 和真实机器人闭环验收。
- 双方 striker 同时争球的碰撞安全和角色切换现场验证。
- SUPPORT、后卫、守门员的完整运动执行链路验证。
- 多机器人本地视觉球检测的融合闭环验证。
- 球观测大跳变、长时间丢失、角落/边界、侧面触球和卡球的系统测试矩阵。
- 实机急停、定位漂移、网络延迟、DDS 丢包和时钟不同步测试。
- 当前包内部的一键 PC authority launch；角色权威、球融合和外部运行仓库仍需单独启动。
- 当前容器未完成标准 Nav2 全量构建时，不能把 `football_real_robot.launch.py` 的 Nav2 分支视为已运行验收；缺少 `cyberdog_debug`/`lcm` 或 GraphicsMagick 等外部依赖时，应先补齐对应机器人环境，或使用已运行的外部 Nav2 配合 `football_robot_runtime.launch.py`。

## 14. 后续 Agent 修改守则

1. 先确定修改针对轻量仿真、实机 Nav2，还是两者；不要用仿真实现替代实机插件。
2. 所有机器人位姿继续使用 `/global_vio/{namespace}/odom`，不要恢复聚合的 `other_robot_poses` 接口。
3. 保持 `tag_global` 语义统一，不通过改 header 欺骗 TF。
4. 保持 fail-closed：输入过期、角色不符、比赛停止、队友数据不足时必须停止，而不是沿旧目标继续。
5. 修改尺寸/净空时同步检查 GoalAdapter、simulation navigator、fake world、costmap layer 和 visualization。
6. 修改速度时同时核对状态机速度上限、navigator 最大速度、动态制动距离和 fake world 积分上限。
7. 修改随机运动时保留 `random_seed` 的确定性，方便复现回归。
8. `linear.y` 的约束按状态区分：`CONTACT_ACQUIRE`/`PUSH_BALL` 必须保持为零；非接触阶段当前默认允许全向避障。修改 `enable_holonomic_avoidance` 时同步检查仿真积分模型、近障恢复、诊断和实机底盘可执行性，不能把仿真横移能力直接外推到实机 Nav2。
9. 新增验证后记录场景、参数、状态序列、最小椭圆净空和证据，不只写“RViz 看起来正常”。
10. 如果重新加入测试文件，确认 CMake 确实创建并执行测试目标，再更新完成状态。

## 15. 常用命令

Docker 内编译：

```bash
docker exec galactic_docker bash -lc \
  'source /opt/ros/galactic/setup.bash && \
   cd /home/lee/code/cyberdog2_pc_ws && \
   colcon build --symlink-install --packages-select football_navigation'
```

启动 RViz 仿真：

```bash
docker exec -it galactic_docker bash
source /opt/ros/galactic/setup.bash
source /home/lee/code/cyberdog2_pc_ws/install/setup.bash
ros2 launch football_navigation football_single_robot_simulation.launch.py use_rviz:=true
```

无 RViz 冒烟测试：

```bash
ros2 launch football_navigation football_single_robot_simulation.launch.py use_rviz:=false
```

建议运行时至少检查：

```bash
ros2 topic echo /cyberdog_1/football/state
ros2 topic echo /cyberdog_1/cmd_vel
ros2 topic echo /cyberdog_1/local_plan
ros2 topic hz /global_vio/cyberdog_2/odom
tail -f /tmp/football_navigation_dynamic_avoidance.jsonl
```
