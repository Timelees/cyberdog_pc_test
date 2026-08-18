#!/usr/bin/python3
"""Real-robot football entry point with an optional self-contained Nav2 stack."""

import os

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


def launch_nav2(context):
    if not enabled(LaunchConfiguration('start_nav2').perform(context)):
        return []

    robot_namespace = LaunchConfiguration(
        'robot_namespace').perform(context).strip().strip('/')
    odom_template = LaunchConfiguration(
        'odom_topic_template').perform(context).strip()
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
            'topic': LaunchConfiguration('scan_topic').perform(context),
        },
        convert_types=True,
    ).perform(context)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav2_nodes = (
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_recoveries', 'recoveries_server', 'recoveries_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    )
    actions = [Node(
        package=package,
        executable=executable,
        name=name,
        namespace=robot_namespace,
        output='screen',
        parameters=[rewritten],
        remappings=remappings,
    ) for package, executable, name in nav2_nodes]
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
            'scan_topic', default_value='scan',
            description='LaserScan topic used by both rolling costmaps.'),
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
        DeclareLaunchArgument('require_striker_role', default_value='true'),
        DeclareLaunchArgument('require_other_robot_poses', default_value='true'),
        DeclareLaunchArgument('minimum_other_robot_count', default_value='9'),
        DeclareLaunchArgument('require_costmap_ready', default_value='true'),
        DeclareLaunchArgument('use_visualization', default_value='true'),
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
                'robot_namespaces_csv',
                'runtime_params_file',
                'robot_params_file',
                'enable_motion',
                'require_striker_role',
                'require_other_robot_poses',
                'minimum_other_robot_count',
                'require_costmap_ready',
                'use_visualization',
            )
        }.items(),
    )
    return LaunchDescription(declarations + [
        OpaqueFunction(function=launch_nav2),
        runtime,
    ])
