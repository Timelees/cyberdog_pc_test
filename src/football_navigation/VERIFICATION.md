# Football No-Map runtime contract

足球模块只负责球验证、角色、接近点、轨迹、动态机器人障碍和 tracking。
生产模式通过足球专用集成 Launch 启用队友实时定位和多机坐标接口，但不修改
队友实现；禁止 MIVINS mapping、Miloc 历史地图、map_server、AMCL、SLAM 和
StaticLayer。

## 唯一入口

真机机器人（tracking 与 velocity adaptor 已包含）：

```bash
export CYBERDOG_NAMESPACE=cyberdog_2
ros2 launch navigation_bringup football_runtime.launch.py \
  mode:=robot_competition namespace:=$CYBERDOG_NAMESPACE
```

PC 端权威节点（全场只能运行一个）：

```bash
ros2 launch navigation_bringup football_runtime.launch.py mode:=pc_authority
```

真机默认比赛状态为 `STOP`；只有裁判/上层明确将 `initial_match_state:=PLAY`
或开球状态后才允许追球。仿真入口显式使用 `PLAY`。

单机软件仿真（PC 权威与机器人正式业务链均包含，真实运动接口不启动）：

```bash
ros2 launch navigation_bringup football_runtime.launch.py \
  mode:=simulation_single_host namespace:=cyberdog_2 use_rviz:=true
```

## 固定接口

```text
/football/ball_pose_raw -> football_ball_fusion -> /football/ball_pose
/global_vio/cyberdog_N/odom -> football_team_role_assigner
/football/team_{a,b}/striker
/football/team_{a,b}/kick_target
/cyberdog_N/football/role
/cyberdog_N/football/tactical_target  frame_id=tag_global
/cyberdog_N/football/other_robot_poses  frame_id=base_link
/cyberdog_N/football/approach_pose
/cyberdog_N/tracking_pose -> /cyberdog_N/tracking_target
/cyberdog_N/cmd_vel
```

真机必须由真实感知节点发布 `/football/ball_pose_raw`：
`geometry_msgs/msg/PoseStamped`、`frame_id=tag_global`、原始图像采集时间戳、
SensorData/BestEffort，建议不低于 10 Hz。足球运行入口绝不启动 fake ball
替代该输入。

十机模式为每队分配 `STRIKER`、`SUPPORT`、`DEFENDER_LEFT`、
`DEFENDER_RIGHT`、`GOALKEEPER`；非 striker 接收不同的 `tactical_target`，
不会复用同一个球后目标。真实球检测节点不在本仓库：未按上面接口发布 raw ball 时，
真机足球闭环不能宣称完成，系统应保持 `SEARCH_BALL` / `control_valid=false`。

仿真额外模拟传感器边界：

```text
/cyberdog_N/odom_out
/cyberdog_N/odom_slam
/cyberdog_N/tf: map -> vodom -> base_link
/tf_static: base_link -> cyberdog_N/tag_0_observation
```

本机仿真位姿只能由各自 `/<namespace>/cmd_vel` 积分；fake publisher 不发布
`odom_global`、`/global_vio` 或最终 PoseArray。最终里程计仍由队友
`odom_transform` 与 `global_vio_odom_node` 产生，PoseArray 只由 PC 角色权威生成。

## No-Map 不变量

- 正式入口不 include通用 `navigation.launch.py`，但会启动 pure-VIO
  `vinslocalization`、AprilTag、`odom_transform` 和 PC `mutil_robot_odom` relay。
- 不启动 MIVINS mapping、map server、AMCL、SLAM、StaticLayer 或 VoxelLayer；
  不需要 `/map` OccupancyGrid、`map->odom` 或人为伪造的世界/机身 TF。
- `football_goal_adapter` 使用同步 `odom_global` 完成 `tag_global` 到本机
  `base_link` 的数学坐标转换，不依赖世界 TF。
- planner、controller、BT、recovery 与两个 rolling costmap 使用 `vodom`；
  机器人机身和相对目标保持 `base_link`。
- costmap 插件只有 LaserScan ObstacleLayer、MultiRobotObstacleLayer 和
  InflationLayer；MultiRobotObstacleLayer 只消费 namespaced PoseArray。
- 一个坏包只被忽略；超时由 watchdog 统一让控制失效。

## 不编译静态验收

```bash
python3 -m unittest \
  cyberdog_nav2/football_navigation/test/test_football_contract.py -v
python3 -m py_compile \
  cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py \
  cyberdog_nav2/football_navigation/launch/*.launch.py
```

真机运行验收还必须确认外部 `odom_global`、DDS、lifecycle、雷达与运动执行器
真实在线；静态检查不能替代 ARM64 和真机证明。
