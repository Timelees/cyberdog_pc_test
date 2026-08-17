# football_navigation 当前能力与边界交接

> 面向后续 Codex/Agent 的代码交接文档。内容基于 2026-08-17 当前工作区源码，描述“代码已经实现什么、当前仿真验证了什么、哪些能力仍依赖外部系统”。修改功能前应先阅读本文件、[README.md](README.md) 和对应参数文件。

## 1. 当前目标和系统边界

`football_navigation` 面向 10 台 CyberDog 足球场景：

- A 队：`cyberdog_1`～`cyberdog_5`。
- B 队：`cyberdog_6`～`cyberdog_10`。
- 当前单机仿真固定使用 `cyberdog_1` 作为 striker。
- 统一全局坐标系为 `tag_global`。
- 机器人全局位姿接口为 `/global_vio/{namespace}/odom`。
- 当前包负责足球目标生成、状态机、导航目标适配、仿真闭环、多机器人障碍插件、角色分配、球融合和可视化。
- 当前包不负责 VIO/AprilTag 定位本身；全局 odom 由外部 `mutil_robot_odom`/全局位姿链路提供。
- 当前包不包含完整实机 bringup。源码中现存的唯一 launch 是 `football_single_robot_simulation.launch.py`。

必须区分三类完成度：

1. **已实现并在轻量仿真闭环验证**：单 striker 接近、动态避障、对齐、触球和推球。
2. **代码已实现但未由当前单机 launch 启动**：球融合、10 机角色权威、Nav2 多机器人 costmap 插件及实机动作链路。
3. **尚未完成现场验收**：完整 Nav2 动态预测避障、双方 striker 竞争、全角色协同和实机测试。

## 2. 代码分层

| 目录 | 职责 | 关键文件 |
| --- | --- | --- |
| `src/core`、`include/.../core` | 坐标、角度、球后接近位、触球几何、外接椭圆等纯算法 | `football_geometry.*` |
| `src/control`、`include/.../control` | 足球状态机、目标转换、Nav2 Action 桥接 | `football_goal_adapter.*`、`football_trajectory_adapter.*`、`football_tracking_action_client.*` |
| `src/coordination`、`include/.../coordination` | 多机球融合、角色和战术目标分配 | `football_ball_fusion.*`、`football_team_role_assigner.*` |
| `src/simulation`、`include/.../simulation` | 单机轻量仿真、假球、假机器人、轻量导航 Action Server | `football_*simulation*`、`football_fake_*` |
| `src/plugins`、`include/.../plugins` | Nav2 costmap、进度检查和 DWB critic | `multi_robot_obstacle_layer.*`、`dynamic_obstacle_marker.*` |
| `src/visualization`、`include/.../visualization` | RViz MarkerArray 分话题显示 | `football_visualization_node.*` |
| `params` | 仿真、实机节点、PC 权威和 costmap 参考参数 | 见第 10 节 |

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
| `/<robot>/cmd_vel` | 机器人速度命令 | 当前仿真非推球导航只使用 `linear.x` 和 `angular.z`，不使用侧移 `linear.y` |
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

- 直线路径无阻挡时直接跟踪目标。
- 单障碍优先生成连续单峰平滑曲线。
- 多台机器人封堵候选曲线时，使用多层横向 lattice 搜索可切换左右侧的走廊，再插值为平滑路径。
- 使用 lookahead 点控制航向；大角度时先转向，再前进。
- 非推球阶段保持 `linear.y = 0`，避免“侧走”绕障。
- 缓存局部路径，但其他机器人平移超过 `dynamic_replan_translation_m`、旋转超过 `dynamic_replan_yaw_rad` 或障碍数量变化时重新规划。
- `dynamic_replan_min_period_sec` 限制重规划频率。
- 椭圆净空低于 slowdown/hard-stop 阈值时减速或停车。
- 当前净空低于 `clearance_recovery_trigger_clearance_m` 时，非推球阶段会在 24 个方向中搜索一条净空单调增大的短路径，以 `clearance_recovery_speed_mps` 主动脱离近距离区域；达到 `clearance_recovery_exit_clearance_m` 后恢复正常绕行。若没有这样的路径，才维持硬停车。

边界：

- 这是用于接口和行为验证的轻量规划器，不具备完整 Nav2 地图、传感器融合、全局可达性或复杂动态预测能力。
- lattice 是有限横向采样，不保证任意密集障碍场景存在解。
- `BLOCKED_RECOVERY` 主要等待/让行，不保证主动脱困。
- 推球走廊被动态机器人持续占用时会暂停推球，不会自动设计复杂的带球绕行战术。

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

- `params/football_costmap_reference.yaml` 只是参考片段，注释明确要求复制/合并到真实 tracking costmap；当前 football launch 不会自动加载它。
- 当前工作区未通过实机或完整 Nav2 launch 对动态预测扫掠区进行闭环验收。
- 轻量仿真通过不等于 Nav2 插件实机通过。

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
- RViz 只是显示，不参与碰撞判定或控制。

## 11. 参数文件职责

| 文件 | 用途和注意事项 |
| --- | --- |
| `params/football_single_robot_simulation.yaml` | 当前唯一可直接 launch 的 10 机轻量仿真参数；随机 peer、striker 状态机、轻量规划器和可视化均在此 |
| `params/football_robot_runtime.yaml` | 单机器人真实运行侧 GoalAdapter、TrajectoryAdapter、TrackingActionClient 等参数；需要外部 bringup 启动 |
| `params/football_pc_authority.yaml` | PC 侧全局 odom 汇总、球融合、角色权威和团队可视化参数；需要外部 PC authority launch |
| `params/football_costmap_reference.yaml` | Nav2 local costmap 集成参考，不能直接当作本包 launch 参数加载 |
| `params/football_navigation_params.yaml` | 导航相关通用参数，修改前核对当前外部 bringup 是否实际引用 |
| `params/football_robot_dimensions.yaml` | 统一机器人尺寸 |
| `params/football_speed_limit_params.yaml` | 速度限制参考/相关参数 |
| `params/fastdds_shm.xml`、`fastdds_udp_only.xml` | Fast DDS 传输配置，不属于规划算法 |

参数在 simulation、robot runtime、PC authority 三套配置中可能有同名项。修改碰撞尺寸、场地边界、超时或速度时，必须检查所有实际运行节点的配置是否一致。

## 12. 已完成验证

在 `galactic_docker` 中已完成：

- `colcon build --symlink-install --packages-select football_navigation` 编译通过。
- 单机从原点到球后点、对齐、触球并进入 `PUSH_BALL`。
- 多静态障碍使用 `multi_obstacle_lattice` 生成 65 点平滑轨迹并绕开阻挡。
- 10 路 `/global_vio/.../odom` 全部可观测。
- cyberdog_2～10 九台 peer 全部随机移动，cyberdog_1 同时执行动态避障推球。
- 35 秒动态测试中出现 150 种局部路径采样形状，证明动态重规划触发。
- `cmd_vel.linear.y` 最大值为 `0.0 m/s`。
- 全体机器人外接椭圆最小净空 `0.024 m`；striker 与 peer 最小净空 `0.029 m`，未发生碰撞。
- 状态到达 `BALL_APPROACH -> ALIGN_TO_GOAL -> CONTACT_ACQUIRE -> PUSH_BALL`；动态近距离阻挡时进入 `BLOCKED_RECOVERY`。

验证边界：

- 上述数据来自轻量仿真，不是实机或完整 Nav2 验收。
- 当前源码树没有 `test/` 目录；CMake 中测试目标均由 `if(EXISTS ...)` 条件创建。因此 `colcon test` 很可能不执行任何当前包测试，不能把“命令快速结束”报告为测试通过。
- 工作区 `build/football_navigation/test_results` 可能包含历史 XML；读取结果前先检查时间戳，不能将旧结果归入当前修改。

## 13. 尚未完成或明确不保证的能力

- 推球走廊被持续占用时的主动带球绕行/战术清障。
- 任意密集动态障碍下的完备路径规划和无死锁保证。
- 完整 Nav2 costmap 动态预测、Controller 和真实机器人闭环验收。
- 双方 striker 同时争球的碰撞安全和角色切换现场验证。
- SUPPORT、后卫、守门员的完整运动执行链路验证。
- 多机器人本地视觉球检测的融合闭环验证。
- 球观测大跳变、长时间丢失、角落/边界、侧面触球和卡球的系统测试矩阵。
- 实机急停、定位漂移、网络延迟、DDS 丢包和时钟不同步测试。
- 当前包内部的一键实机/PC authority launch；这些仍依赖外部运行仓库或后续补充。

## 14. 后续 Agent 修改守则

1. 先确定修改针对轻量仿真、实机 Nav2，还是两者；不要用仿真实现替代实机插件。
2. 所有机器人位姿继续使用 `/global_vio/{namespace}/odom`，不要恢复聚合的 `other_robot_poses` 接口。
3. 保持 `tag_global` 语义统一，不通过改 header 欺骗 TF。
4. 保持 fail-closed：输入过期、角色不符、比赛停止、队友数据不足时必须停止，而不是沿旧目标继续。
5. 修改尺寸/净空时同步检查 GoalAdapter、simulation navigator、fake world、costmap layer 和 visualization。
6. 修改速度时同时核对状态机速度上限、navigator 最大速度、动态制动距离和 fake world 积分上限。
7. 修改随机运动时保留 `random_seed` 的确定性，方便复现回归。
8. 不要让非推球路径控制重新使用 `linear.y`；当前设计通过转向绕障，避免侧走姿态。
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
```
