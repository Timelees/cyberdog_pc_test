# football_navigation 依赖说明

本文档根据 `football_navigation/package.xml`、`CMakeLists.txt`、源码头文件、Launch 文件及运行脚本整理。

## 1. ROS 2 基础依赖

| 功能类别 | 依赖包 |
| --- | --- |
| 构建系统 | `ament_cmake` |
| C++ / Python 节点 | `rclcpp`、`rclpy` |
| Action | `rclcpp_action`、`action_msgs` |
| Lifecycle | `rclcpp_lifecycle`、`lifecycle_msgs` |
| 参数接口 | `rcl_interfaces` |
| 包索引 | `ament_index_cpp`、`ament_index_python` |
| Launch | `launch`、`launch_ros` |
| TF | `tf2`、`tf2_ros`、`tf2_msgs`、`tf2_geometry_msgs` |
| 消息 | `builtin_interfaces`、`geometry_msgs`、`nav_msgs`、`std_msgs`、`visualization_msgs` |
| 服务 | `std_srvs` |
| 插件系统 | `pluginlib` |
| 可视化 | `rviz2` |
| Python YAML | `python3-yaml` |
| 测试 | `ament_cmake_gtest`、`ament_cmake_pytest`、`ament_lint_auto`、`ament_lint_common` |

## 2. Navigation2 依赖

这些依赖属于 ROS 2 Navigation2 功能栈，本工作区中也包含相应源码。

| 依赖包 | 主要用途 | 本项目路径 |
| --- | --- | --- |
| `nav2_costmap_2d` | 实现多机器人动态障碍物 Costmap Layer | `src/thirdparty/navigation2/nav2_costmap_2d` |
| `nav2_core` | 使用 Nav2 插件接口、异常和进度检查器 | `src/thirdparty/navigation2/nav2_core` |
| `nav2_msgs` | 使用导航 Action、目标和路径接口 | `src/thirdparty/navigation2/nav2_msgs` |
| `nav2_behavior_tree` | 实现 Football 行为树 Action 插件 | `src/thirdparty/navigation2/nav2_behavior_tree` |
| `nav2_util` | 使用 Nav2 节点、参数和机器人位姿工具 | `src/thirdparty/navigation2/nav2_util` |
| `nav2_smac_planner` | 检查 SmacPlanner2D 运行库并记录构建身份 | `src/thirdparty/navigation2/nav2_smac_planner` |
| `nav2_common` | Nav2 构建及运行支持 | `src/thirdparty/navigation2/nav2_common` |
| `dwb_core` | 实现 DWB trajectory critic 插件 | `src/thirdparty/navigation2/nav2_dwb_controller/dwb_core` |
| `dwb_msgs` | 使用局部轨迹评价消息 | `src/thirdparty/navigation2/nav2_dwb_controller/dwb_msgs` |
| `behaviortree_cpp_v3` | BehaviorTree.CPP 节点实现和插件注册 | 外部 ROS 2 / BehaviorTree.CPP 依赖 |

## 3. 本项目其他功能包依赖

### 3.1 已在 package.xml 中直接声明

| 功能包 | 项目路径 | 用途 | 声明形式 |
| --- | --- | --- | --- |
| `nav2_msgs` | `src/thirdparty/navigation2/nav2_msgs` | 使用 `NavigateToPose` Action，向 Galactic Nav2 发送足球目标 | `<depend>` |
| `protocol` | `bridges/protocol` | 使用 `MotionServoCmd` 消息，读取底层运动控制命令用于可视化 | `<depend>` |
| `mutil_robot_odom` | `tools/cyberdog_pc_test/src/mutil_robot_odom` | PC 端转发多台机器人的全局 VIO 里程计 | `<exec_depend>` |
| `topic_visualization` | `tools/cyberdog_pc_test/src/topic_visualization` | PC 端多机器人 Tag/位置可视化 | `<exec_depend>` |

### 3.2 实际使用但未在本包 package.xml 中声明

| 功能包 | 项目路径 | 用途 | 引用位置 |
| --- | --- | --- | --- |
| `cyberdog_bringup` | `cyberdog_bringup` | 获取并解析机器人命名空间 | `launch/football_navigation.launch.py`、部分验收脚本 |
| `mcr_bringup` | `cyberdog_tracking_base/mcr_bringup` | 提供目标跟踪参数、行为树和生命周期管理 | `src/control/football_tracking_action_client.cpp`、运行及验收脚本 |
| `navigation_bringup` | `cyberdog_nav2/navigation_bringup` | 足球导航顶层入口，组合机器人端、PC 端和目标跟踪栈 | 构建身份检查、验收脚本和 `VERIFICATION.md` |
| `mcr_controller` | `cyberdog_tracking_base` 下对应功能包 | 足球目标跟踪控制器 | 足球运行脚本 |
| `mcr_planner` | `cyberdog_tracking_base` 下对应功能包 | 足球目标跟踪规划器 | 足球运行脚本 |
| `bt_navigators` | 工作区内对应功能包 | 目标跟踪行为树导航 | 足球运行脚本 |
| `nav2_recoveries` | `cyberdog_nav2/navigation2/nav2_recoveries` | 跟踪导航恢复行为 | 足球运行脚本 |
| `vins` | 工作区内定位功能包 | 真机 VIO 定位 | 真机/仿真启动脚本 |
| `apriltag_ros` | 工作区内视觉定位功能包 | AprilTag 全局定位 | 真机/仿真启动脚本 |

其中最明确的未声明直接依赖是：

- `football_navigation.launch.py` 通过包索引读取 `cyberdog_bringup`。
- `control/football_tracking_action_client.cpp` 通过 `get_package_share_directory("mcr_bringup")` 读取 `mcr_bringup`。
- 构建身份检查、运行入口及验收工具引用 `navigation_bringup`。

建议至少在 `package.xml` 中补充以下运行依赖：

```xml
<exec_depend>cyberdog_bringup</exec_depend>
<exec_depend>mcr_bringup</exec_depend>
<exec_depend>navigation_bringup</exec_depend>
```

## 4. 依赖关系概览

```text
football_navigation
├── ROS 2 基础设施
│   ├── rclcpp / rclpy / launch
│   ├── geometry_msgs / nav_msgs / std_msgs
│   └── tf2 / pluginlib / rviz2
├── Navigation2
│   ├── nav2_costmap_2d / nav2_core
│   ├── nav2_msgs / nav2_behavior_tree / nav2_util
│   ├── nav2_smac_planner
│   └── dwb_core / dwb_msgs
├── cyberdog_tracking_base
│   └── mcr_bringup（运行脚本仍可能使用，未声明）
├── bridges
│   └── protocol
├── tools/cyberdog_pc_test
│   ├── mutil_robot_odom
│   └── topic_visualization
├── cyberdog_bringup（实际使用，未声明）
└── navigation_bringup（顶层组合运行入口，未声明）
```

## 5. package.xml 中的完整声明清单

### buildtool_depend

- `ament_cmake`

### depend

- `ament_index_cpp`
- `action_msgs`
- `behaviortree_cpp_v3`
- `builtin_interfaces`
- `dwb_core`
- `dwb_msgs`
- `geometry_msgs`
- `nav2_behavior_tree`
- `nav2_costmap_2d`
- `nav2_core`
- `nav2_msgs`
- `nav2_smac_planner`
- `nav2_util`
- `nav_msgs`
- `pluginlib`
- `rcl_interfaces`
- `std_msgs`
- `rclcpp`
- `rclcpp_action`
- `rclcpp_lifecycle`
- `tf2`
- `tf2_geometry_msgs`
- `tf2_msgs`
- `tf2_ros`
- `protocol`
- `visualization_msgs`

### exec_depend

- `nav2_common`
- `ament_index_python`
- `launch`
- `launch_ros`
- `mutil_robot_odom`
- `python3-yaml`
- `rclpy`
- `lifecycle_msgs`
- `rviz2`
- `std_srvs`
- `topic_visualization`

### test_depend

- `ament_cmake_gtest`
- `ament_cmake_pytest`
- `ament_lint_auto`
- `ament_lint_common`

## 6. 总结

`football_navigation` 不是一个完全独立的足球节点包。它是建立在 ROS 2、Navigation2、CyberDog 目标跟踪栈、底层运动协议、多机器人里程计以及项目 Bringup 编排之上的足球导航集成包。
