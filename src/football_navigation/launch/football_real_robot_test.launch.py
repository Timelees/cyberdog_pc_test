"""Single-robot real-hardware path test with static football inputs.

This launch relays live per-robot odom_global into tag_global but never creates
fake odometry. It publishes deterministic ball/team inputs for planning tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_global_vio_relay(context):
    """Bridge the robot's namespaced odom_global into the football global frame."""
    if LaunchConfiguration('start_global_vio_relay').perform(context).lower() not in (
            'true', '1', 'yes', 'on'):
        return []
    namespaces = [
        value.strip().strip('/')
        for value in LaunchConfiguration('robot_namespaces_csv').perform(context).split(',')
        if value.strip().strip('/')
    ]
    if not namespaces:
        raise RuntimeError('robot_namespaces_csv must contain at least one robot')
    return [Node(
        package='mutil_robot_odom',
        executable='global_vio_odom_node',
        name='football_global_vio_odom',
        output='screen',
        parameters=[{
            'robot_namespaces': namespaces,
            'input_odom_topic': LaunchConfiguration('global_vio_input_topic'),
            'output_topic_prefix': '/global_vio',
            'output_odom_topic': 'odom',
            'target_frame_id': 'tag_global',
            'base_frame_name': 'base_link',
            'publish_tf': True,
            'publish_map_anchor_tf': False,
            'subscribe_best_effort': True,
            'publish_best_effort': False,
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory('football_navigation')
    real_launch = os.path.join(share, 'launch', 'football_real_robot.launch.py')
    default_rviz_config = os.path.join(
        share, 'rviz', 'football_single_robot_simulation.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('ball_x', default_value='-2.22'),
        DeclareLaunchArgument('ball_y', default_value='-1.24'),
        DeclareLaunchArgument('kick_target_x', default_value='-3.65'),
        DeclareLaunchArgument('kick_target_y', default_value='-0.74'),
        DeclareLaunchArgument(
            'ball_pose_topic', default_value='/football/real_test/ball_pose',
            description='Isolated fixed ball input topic for the real test.'),
        DeclareLaunchArgument(
            'kick_target_topic', default_value='/football/real_test/kick_target',
            description='Isolated fixed kick target topic for the real test.'),
        DeclareLaunchArgument(
            'robot_namespaces_csv', default_value='cyberdog_1',
            description=(
                'Robots shown and treated as dynamic obstacles. Use '
                'cyberdog_1,cyberdog_3 for the two-robot interference test.')),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the football test visualization.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='RViz configuration for the real-robot test.'),
        DeclareLaunchArgument(
            'use_scan', default_value='false',
            description='Use the robot LaserScan for Nav2 costmaps.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/cyberdog_1/football_test_scan',
            description='Nav2 LaserScan topic when use_scan is enabled.'),
        DeclareLaunchArgument(
            'enable_motion', default_value='true',
            description=(
                'Allow the tracking client to send NavigateToPose goals. This '
                'runs planning; the cmd_vel adapter separately controls hardware output.')),
        DeclareLaunchArgument(
            'start_global_vio_relay', default_value='false',
            description=(
                'Start the odom_global relay only when the robot does not already '
                'publish /global_vio/<robot>/odom. The normal real-robot setup '
                'consumes that existing global odom directly.')),
        DeclareLaunchArgument(
            'global_vio_input_topic', default_value='/odom_global',
            description='Per-robot input odom suffix consumed by mutil_robot_odom.'),
        DeclareLaunchArgument(
            'odom_timeout_sec', default_value='5.0',
            description=(
                'Real-test odom age limit. The current robot-to-PC global VIO '
                'stream is commonly 0.8-1.6 s old; values older than this '
                'still force SAFE_STOP.')),
        DeclareLaunchArgument(
            'max_ball_odom_skew_sec', default_value='5.0',
            description=(
                'Real-test synchronization window for the static configured '
                'ball pose and delayed global VIO samples.')),
        DeclareLaunchArgument(
            'use_cmd_vel_adapter', default_value='true',
            description=(
                'Convert Nav2 cmd_vel to namespaced motion_servo_cmd. Motion '
                'still requires a valid slow-walk (303) status. Set false for '
                'a visualization-only dry run.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                'robot_namespace': 'cyberdog_1',
                'base_frame': 'cyberdog_1_base_link',
                'field_frame': 'tag_global',
                # With use_scan=false this topic has no publisher.
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_scan': LaunchConfiguration('use_scan'),
                'robot_namespaces_csv': LaunchConfiguration(
                    'robot_namespaces_csv'),
                'odom_timeout_sec': LaunchConfiguration('odom_timeout_sec'),
                'max_ball_odom_skew_sec': LaunchConfiguration(
                    'max_ball_odom_skew_sec'),
                'enable_motion': LaunchConfiguration('enable_motion'),
                'require_slow_walk': 'true',
                'require_striker_role': 'true',
                'require_other_robot_poses': 'false',
                'minimum_other_robot_count': '0',
                'minimum_obstacle_count': '0',
                # Without LaserScan there may be no marked cells to gate on.
                'require_costmap_ready': 'false',
                'use_test_inputs': 'true',
                'use_cmd_vel_adapter': LaunchConfiguration(
                    'use_cmd_vel_adapter'),
                'use_visualization': 'true',
                'ball_pose_topic': LaunchConfiguration('ball_pose_topic'),
                'kick_target_topic': LaunchConfiguration('kick_target_topic'),
                'test_ball_x': LaunchConfiguration('ball_x'),
                'test_ball_y': LaunchConfiguration('ball_y'),
                'test_kick_target_x': LaunchConfiguration('kick_target_x'),
                'test_kick_target_y': LaunchConfiguration('kick_target_y'),
            }.items()),
        OpaqueFunction(function=launch_global_vio_relay),
        Node(
            condition=IfCondition(LaunchConfiguration('use_scan')),
            package='football_navigation',
            executable='football_scan_frame_relay',
            name='football_scan_frame_relay',
            output='screen',
            parameters=[{
                'input_topic': '/cyberdog_1/scan',
                'output_topic': LaunchConfiguration('scan_topic'),
                'parent_frame': 'cyberdog_1_base_link',
                'output_frame': 'cyberdog_1_laser_frame',
            }]),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='football_real_robot_test_rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
