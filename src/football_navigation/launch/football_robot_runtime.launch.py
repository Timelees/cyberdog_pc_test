#!/usr/bin/python3
"""Per-robot football nodes; platform bringup stays at the top-level entry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    football_share = get_package_share_directory('football_navigation')
    navigation_frame = LaunchConfiguration('navigation_frame')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Robot namespace, e.g. cyberdog_2'),
        DeclareLaunchArgument('runtime_mode', default_value='robot_competition'),
        DeclareLaunchArgument('field_frame', default_value='tag_global'),
        DeclareLaunchArgument('navigation_frame', default_value='vodom'),
        DeclareLaunchArgument('odom_global_topic', default_value=''),
        DeclareLaunchArgument('enable_local_visualization', default_value='false'),
        DeclareLaunchArgument('football_params_file', default_value=os.path.join(
            football_share, 'params', 'football_robot_runtime.yaml')),
        DeclareLaunchArgument('field_geometry_params_file', default_value=os.path.join(
            football_share, 'params', 'football_pc_authority.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                football_share, 'launch', 'football_navigation.launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'params_file': LaunchConfiguration('football_params_file'),
                'field_geometry_params_file': LaunchConfiguration('field_geometry_params_file'),
                'runtime_mode': LaunchConfiguration('runtime_mode'),
                'unified_frame': LaunchConfiguration('field_frame'),
                'navigation_frame': navigation_frame,
                'odom_global_topic': LaunchConfiguration('odom_global_topic'),
                'target_frame': 'base_link',
                'enable_trajectory_adapter': 'true',
                'enable_tracking_action_client': 'true',
                'enable_local_visualization': LaunchConfiguration('enable_local_visualization'),
            }.items()),
    ])