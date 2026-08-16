#!/usr/bin/python3
"""Legacy visualization-only entry; control uses football_runtime.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('football_navigation')
    params = os.path.join(share, 'params', 'football_navigation_params.yaml')
    rviz = os.path.join(share, 'rviz', 'football_no_map.rviz')
    namespace = LaunchConfiguration('namespace')
    field_frame = LaunchConfiguration('field_frame')
    use_rviz = LaunchConfiguration('use_rviz')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('field_frame', default_value='tag_global'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(
            package='football_navigation', executable='football_visualization_node',
            name='football_visualization_node', namespace=namespace, output='screen',
            parameters=[params, {
                'field_frame': field_frame,
                'target_frame': field_frame,
            }]),
        Node(
            condition=IfCondition(use_rviz), package='rviz2', executable='rviz2',
            name='football_rviz', output='screen', arguments=['-d', rviz]),
    ])
