#!/usr/bin/python3
"""Real-robot football entry point with an optional self-contained Nav2 stack."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


DEFAULT_ROBOTS = (
    'cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5,'
    'cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10'
)


def enabled(value, argument_name='start_nav2'):
    normalized = str(value).strip().lower()
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError('{} must be true or false'.format(argument_name))


def launch_nav2(context):
    if not enabled(LaunchConfiguration('start_nav2').perform(context)):
        return []

    robot_namespace = LaunchConfiguration(
        'robot_namespace').perform(context).strip().strip('/')
    odom_template = LaunchConfiguration(
        'odom_topic_template').perform(context).strip()
    use_scan = enabled(
        LaunchConfiguration('use_scan').perform(context), 'use_scan')
    if not robot_namespace or '{namespace}' not in odom_template:
        raise RuntimeError(
            'robot_namespace must not be empty and odom_topic_template '
            'must contain {namespace}')

    required_packages = (
        'nav2_controller',
        'nav2_planner',
        'nav2_recoveries',
        'nav2_bt_navigator',
        'nav2_lifecycle_manager',
        'nav2_navfn_planner',
        'dwb_plugins',
        'dwb_critics',
    )
    missing_packages = []
    for package in required_packages:
        try:
            get_package_prefix(package)
        except Exception:  # ament raises a distro-specific PackageNotFoundError
            missing_packages.append(package)
    if missing_packages:
        raise RuntimeError(
            'start_nav2:=true requires installed packages: {}. '
            'Install the robot Nav2 runtime or use start_nav2:=false with an '
            'already running navigation stack.'.format(', '.join(missing_packages)))

    rewritten = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params_file').perform(context),
        root_key=robot_namespace,
        param_rewrites={
            'use_sim_time': 'false',
            'odom_topic': odom_template.replace(
                '{namespace}', robot_namespace),
            'global_frame': LaunchConfiguration(
                'field_frame').perform(context),
            'target_frame': LaunchConfiguration(
                'field_frame').perform(context),
            'robot_base_frame': LaunchConfiguration(
                'base_frame').perform(context),
            'self_namespace': robot_namespace,
            'robot_namespaces_csv': LaunchConfiguration(
                'robot_namespaces_csv').perform(context),
            'robot_odom_topic_template': odom_template,
            'minimum_obstacle_count': LaunchConfiguration(
                'minimum_obstacle_count').perform(context),
            'topic': LaunchConfiguration('scan_topic').perform(context),
            # With no observation source ObstacleLayer remains current and
            # PlannerServer does not block forever in waitForCostmap().
            'observation_sources': 'scan' if use_scan else '',
        },
        convert_types=True,
    ).perform(context)

    nav2_nodes = (
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_recoveries', 'recoveries_server', 'recoveries_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    )
    actions = []
    for package, executable, name in nav2_nodes:
        remappings = []
        if name == 'bt_navigator':
            # The robot firmware also exposes NavAB clients on the generic
            # navigate_to_pose name. Keep this PC football action private.
            remappings = [
                ('navigate_to_pose', 'football/navigate_to_pose'),
                ('navigate_through_poses', 'football/navigate_through_poses'),
            ]
        actions.append(Node(
            package=package,
            executable=executable,
            name=name,
            namespace=robot_namespace,
            output='screen',
            parameters=[rewritten],
            remappings=remappings,
        ))
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=robot_namespace,
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': enabled(
                LaunchConfiguration('nav2_autostart').perform(context),
                'nav2_autostart'),
            'node_names': [name for _, _, name in nav2_nodes],
        }],
    ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('football_navigation')
    runtime_launch = os.path.join(
        share, 'launch', 'football_robot_runtime.launch.py')
    declarations = [
        DeclareLaunchArgument(
            'robot_namespace', default_value='cyberdog_1',
            description='Robot namespace matching cyberdog_<1..10>.'),
        DeclareLaunchArgument('team_id', default_value='auto'),
        DeclareLaunchArgument('field_frame', default_value='tag_global'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'odom_topic_template',
            default_value='/global_vio/{namespace}/odom',
            description='Global odom topic template; must contain {namespace}.'),
        DeclareLaunchArgument(
            'odom_timeout_sec', default_value='0.45',
            description='Maximum accepted age of the global odometry.'),
        DeclareLaunchArgument(
            'max_ball_odom_skew_sec', default_value='0.12',
            description='Maximum timestamp skew between ball and global odometry.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='scan',
            description='LaserScan topic used by both rolling costmaps.'),
        DeclareLaunchArgument(
            'use_scan', default_value='true',
            description='Enable the LaserScan observation source in both costmaps.'),
        DeclareLaunchArgument(
            'robot_namespaces_csv', default_value=DEFAULT_ROBOTS),
        DeclareLaunchArgument(
            'runtime_params_file',
            default_value=os.path.join(
                share, 'params', 'football_runtime_common.yaml')),
        DeclareLaunchArgument(
            'robot_params_file',
            default_value=os.path.join(
                share, 'params', 'football_real_robot.yaml')),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                share, 'params', 'football_nav2_real.yaml')),
        DeclareLaunchArgument(
            'start_nav2', default_value='true',
            description='Start the bundled map-free Nav2 nodes.'),
        DeclareLaunchArgument(
            'nav2_autostart', default_value='true',
            description='Automatically activate the bundled Nav2 lifecycle nodes.'),
        DeclareLaunchArgument(
            'enable_motion', default_value='false',
            description='Allow the football Action Client to send NavigateToPose goals.'),
        DeclareLaunchArgument(
            'require_slow_walk', default_value='true',
            description='Only allow goals while motion_status reports slow walk (303).'),
        DeclareLaunchArgument(
            'use_test_inputs', default_value='false',
            description='Publish static test ball/target/striker inputs without publishing odom.'),
        DeclareLaunchArgument('test_ball_x', default_value='2.0'),
        DeclareLaunchArgument('test_ball_y', default_value='0.0'),
        DeclareLaunchArgument('test_kick_target_x', default_value='6.0'),
        DeclareLaunchArgument('test_kick_target_y', default_value='0.0'),
        DeclareLaunchArgument(
            'test_input_params_file',
            default_value=os.path.join(share, 'params', 'football_real_robot_test.yaml')),
        DeclareLaunchArgument(
            'use_cmd_vel_adapter', default_value='false',
            description='Convert Nav2 cmd_vel to robot MotionServoCmd with a watchdog.'),
        DeclareLaunchArgument('require_striker_role', default_value='true'),
        DeclareLaunchArgument('require_other_robot_poses', default_value='true'),
        DeclareLaunchArgument('minimum_other_robot_count', default_value='9'),
        DeclareLaunchArgument('minimum_obstacle_count', default_value='9'),
        DeclareLaunchArgument('require_costmap_ready', default_value='true'),
        DeclareLaunchArgument('use_visualization', default_value='true'),
        DeclareLaunchArgument('ball_pose_topic', default_value='/football/ball_pose'),
        DeclareLaunchArgument('kick_target_topic', default_value=''),
    ]
    runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(runtime_launch),
        launch_arguments={
            name: LaunchConfiguration(name) for name in (
                'robot_namespace',
                'team_id',
                'field_frame',
                'base_frame',
                'odom_topic_template',
                'odom_timeout_sec',
                'max_ball_odom_skew_sec',
                'robot_namespaces_csv',
                'runtime_params_file',
                'robot_params_file',
                'enable_motion',
                'require_slow_walk',
                'require_striker_role',
                'require_other_robot_poses',
                'minimum_other_robot_count',
                'require_costmap_ready',
                'use_visualization',
                'ball_pose_topic',
                'kick_target_topic',
            )
        }.items(),
    )
    test_input = Node(
        condition=IfCondition(LaunchConfiguration('use_test_inputs')),
        package='football_navigation',
        executable='football_simulation_input_publisher',
        name='football_real_test_input_publisher',
        output='screen',
        parameters=[LaunchConfiguration('test_input_params_file'), {
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'field_frame': LaunchConfiguration('field_frame'),
            'odom_topic_template': LaunchConfiguration('odom_topic_template'),
            'publish_odom': False,
            'publish_path': False,
            'ball_x': LaunchConfiguration('test_ball_x'),
            'ball_y': LaunchConfiguration('test_ball_y'),
            'kick_target_x': LaunchConfiguration('test_kick_target_x'),
            'kick_target_y': LaunchConfiguration('test_kick_target_y'),
            'simulate_ball_push': False,
            'stop_at_ball': False,
            'ball_topic': LaunchConfiguration('ball_pose_topic'),
            'kick_target_topic': LaunchConfiguration('kick_target_topic'),
        }])
    cmd_adapter = Node(
        condition=IfCondition(LaunchConfiguration('use_cmd_vel_adapter')),
        package='football_navigation',
        executable='football_cmd_vel_to_servo',
        name='football_cmd_vel_to_servo',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        parameters=[{
            'cmd_vel_topic': 'cmd_vel',
            'motion_servo_cmd_topic': 'motion_servo_cmd',
            'control_valid_topic': 'football/control_valid',
            'motion_id': 303,
            'cmd_vel_timeout_sec': 0.25,
        }])
    return LaunchDescription(declarations + [
        OpaqueFunction(function=launch_nav2),
        runtime,
        test_input,
        cmd_adapter,
    ])
