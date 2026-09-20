#!/usr/bin/python3
"""Real-robot football entry point with an optional self-contained Nav2 stack."""

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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


def load_node_params(path, node_name):
    if not path:
        return {}
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def launch_nav2(context):
    if not enabled(LaunchConfiguration('start_nav2').perform(context)):
        return []

    robot_namespace = LaunchConfiguration(
        'robot_namespace').perform(context).strip().strip('/')
    odom_template = LaunchConfiguration(
        'odom_topic_template').perform(context).strip()
    use_scan = enabled(
        LaunchConfiguration('use_scan').perform(context), 'use_scan')
    use_sim_time = enabled(
        LaunchConfiguration('use_sim_time').perform(context), 'use_sim_time')
    if not robot_namespace or '{namespace}' not in odom_template:
        raise RuntimeError(
            'robot_namespace must not be empty and odom_topic_template '
            'must contain {namespace}')
    odom_topic = odom_template.replace('{namespace}', robot_namespace)

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

    param_rewrites = {
        'use_sim_time': str(use_sim_time).lower(),
        'odom_topic': odom_topic,
        'global_frame': LaunchConfiguration('field_frame').perform(context),
        'target_frame': LaunchConfiguration('field_frame').perform(context),
        'robot_base_frame': LaunchConfiguration('base_frame').perform(context),
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
    }
    rewritten = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params_file').perform(context),
        root_key=robot_namespace,
        param_rewrites=param_rewrites,
        convert_types=True,
    ).perform(context)
    parameter_files = [rewritten]
    overrides_path = LaunchConfiguration(
        'nav2_overrides_file').perform(context).strip()
    if overrides_path:
        parameter_files.append(RewrittenYaml(
            source_file=overrides_path,
            root_key=robot_namespace,
            # This is the final, user-editable YAML layer. Do not rewrite its
            # values with launch defaults: parameters written here must win.
            param_rewrites={},
            convert_types=True,
        ).perform(context))

    nav2_nodes = (
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_recoveries', 'recoveries_server', 'recoveries_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    )
    actions = []
    for package, executable, name in nav2_nodes:
        remappings = []
        if name in ('controller_server', 'recoveries_server'):
            # Isolate football velocity commands from the robot-side
            # cyberdog_nav2 velocity_adaptor, which subscribes to cmd_vel.
            # Only football_cmd_vel_to_servo may bridge this private topic to
            # MotionServoCmd, preventing two gait adapters from publishing at
            # the same time.
            remappings.append(('cmd_vel', 'football/cmd_vel'))
        if name == 'controller_server':
            # Galactic's controller_server defaults to the relative `odom`
            # topic when older parameter files omit odom_topic. Keep an
            # explicit remap as a compatibility guard in addition to the
            # rewritten absolute parameter.
            remappings.append(('odom', odom_topic))
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
            parameters=parameter_files,
            remappings=remappings,
        ))
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=robot_namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': enabled(
                LaunchConfiguration('nav2_autostart').perform(context),
                'nav2_autostart'),
            'node_names': [name for _, _, name in nav2_nodes],
        }],
    ))
    return actions


def launch_test_input(context):
    if not enabled(
            LaunchConfiguration('use_test_inputs').perform(context),
            'use_test_inputs'):
        return []

    # Launch arguments provide portable fallbacks. The named YAML section is
    # merged last so field coordinates and test behavior edited in YAML always
    # take precedence.
    params = {
        'use_sim_time': enabled(
            LaunchConfiguration('use_sim_time').perform(context), 'use_sim_time'),
        'robot_namespace': LaunchConfiguration('robot_namespace').perform(context),
        'field_frame': LaunchConfiguration('field_frame').perform(context),
        'odom_topic_template': LaunchConfiguration(
            'odom_topic_template').perform(context),
        'publish_odom': False,
        'publish_ball': enabled(
            LaunchConfiguration('test_publish_ball').perform(context),
            'test_publish_ball'),
        'publish_path': False,
        'ball_x': float(LaunchConfiguration('test_ball_x').perform(context)),
        'ball_y': float(LaunchConfiguration('test_ball_y').perform(context)),
        'kick_target_x': float(
            LaunchConfiguration('test_kick_target_x').perform(context)),
        'kick_target_y': float(
            LaunchConfiguration('test_kick_target_y').perform(context)),
        'simulate_ball_push': False,
        'stop_at_ball': False,
        'ball_topic': LaunchConfiguration('ball_pose_topic').perform(context),
        'kick_target_topic': LaunchConfiguration('kick_target_topic').perform(context),
    }
    params.update(load_node_params(
        LaunchConfiguration('test_input_params_file').perform(context),
        'football_simulation_input_publisher'))
    return [Node(
        package='football_navigation',
        executable='football_simulation_input_publisher',
        name='football_real_test_input_publisher',
        output='screen',
        parameters=[params],
    )]


def launch_cmd_adapter(context):
    if not enabled(
            LaunchConfiguration('use_cmd_vel_adapter').perform(context),
            'use_cmd_vel_adapter'):
        return []

    params = {
        'use_sim_time': enabled(
            LaunchConfiguration('use_sim_time').perform(context), 'use_sim_time'),
        'cmd_vel_topic': 'football/cmd_vel',
        'motion_servo_cmd_topic': 'motion_servo_cmd',
        'control_valid_topic': 'football/control_valid',
        'motion_id': int(
            LaunchConfiguration('navigation_motion_id').perform(context)),
        'cmd_source': 4,
        'gait_value': 2,
        'cmd_vel_timeout_sec': 0.25,
        'smoothing_window_sec': 0.25,
        'max_linear_x': 0.26,
        'max_linear_y': 0.20,
        'max_angular_z': 1.0,
    }
    for file_argument in (
            'runtime_params_file', 'robot_params_file', 'robot_overrides_file'):
        path = LaunchConfiguration(file_argument).perform(context).strip()
        params.update(load_node_params(path, 'football_cmd_vel_to_servo'))
    return [Node(
        package='football_navigation',
        executable='football_cmd_vel_to_servo',
        name='football_cmd_vel_to_servo',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    share = get_package_share_directory('football_navigation')
    runtime_launch = os.path.join(
        share, 'launch', 'football_robot_runtime.launch.py')
    declarations = [
        DeclareLaunchArgument(
            'robot_namespace', default_value='cyberdog_1',
            description='Robot namespace matching cyberdog_<1..10>.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
            'robot_overrides_file', default_value='',
            description='Optional per-node parameter overrides applied after robot_params_file.'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                share, 'params', 'football_nav2_real.yaml')),
        DeclareLaunchArgument(
            'nav2_overrides_file', default_value='',
            description='Optional second Nav2 parameter file for environment-specific overrides.'),
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
            description='Only allow goals while motion_status reports the expected navigation gait.'),
        DeclareLaunchArgument(
            'navigation_motion_id', default_value='309',
            description='CyberDog follow-gait motion ID used for path tracking.'),
        DeclareLaunchArgument(
            'use_test_inputs', default_value='false',
            description='Publish static test ball/target/striker inputs without publishing odom.'),
        DeclareLaunchArgument('test_ball_x', default_value='2.0'),
        DeclareLaunchArgument('test_ball_y', default_value='0.0'),
        DeclareLaunchArgument('test_kick_target_x', default_value='6.0'),
        DeclareLaunchArgument('test_kick_target_y', default_value='0.0'),
        DeclareLaunchArgument(
            'test_publish_ball', default_value='true',
            description='Publish the synthetic test ball (disable for Gazebo physics).'),
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
                'use_sim_time',
                'team_id',
                'field_frame',
                'base_frame',
                'odom_topic_template',
                'odom_timeout_sec',
                'max_ball_odom_skew_sec',
                'robot_namespaces_csv',
                'runtime_params_file',
                'robot_params_file',
                'robot_overrides_file',
                'enable_motion',
                'require_slow_walk',
                'navigation_motion_id',
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
    return LaunchDescription(declarations + [
        OpaqueFunction(function=launch_nav2),
        runtime,
        OpaqueFunction(function=launch_test_input),
        OpaqueFunction(function=launch_cmd_adapter),
    ])
