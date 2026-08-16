#!/usr/bin/python3
"""PC teammate odometry relay plus sole football authority."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def node_params(data, node_name):
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def launch_nodes(context):
    share = get_package_share_directory('football_navigation')
    params_path = os.path.join(share, 'params', 'football_pc_authority.yaml')

    with open(params_path, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}

    field_frame = LaunchConfiguration('field_frame').perform(context).strip()
    ball_input_topic = LaunchConfiguration('ball_input_topic').perform(context).strip()
    odom_template = LaunchConfiguration('odom_topic_template').perform(context).strip()
    start_teammate_relay = as_bool(
        LaunchConfiguration('start_teammate_relay').perform(context)
    )
    use_visualization = as_bool(
        LaunchConfiguration('use_visualization').perform(context)
    )
    initial_match_state = LaunchConfiguration(
        'initial_match_state'
    ).perform(context).strip()
    continuous_demo = as_bool(
        LaunchConfiguration('continuous_demo').perform(context)
    )
    forced_team_a_striker = LaunchConfiguration(
        'forced_team_a_striker_namespace'
    ).perform(context).strip().strip('/')

    ball_params = node_params(params, 'football_ball_fusion')
    ball_params.update({
        'mode': 'global_external',
        'field_frame': field_frame,
        'global_input_topic': ball_input_topic,
        'output_topic': '/football/ball_pose',
    })

    role_params = node_params(params, 'football_team_role_assigner')
    role_params.update({
        'field_frame': field_frame,
        'odom_source_mode': 'pc_forwarded',
        'odom_topic_template': odom_template,
        'authority_id': 'football_pc_primary',
        'initial_match_state': initial_match_state,
    })
    if continuous_demo:
        if not forced_team_a_striker:
            raise RuntimeError(
                'continuous_demo requires forced_team_a_striker_namespace'
            )
        role_params.update({
            'forced_team_a_striker_namespace': forced_team_a_striker,
        })

    actions = []

    if start_teammate_relay:
        actions.append(Node(
            package='mutil_robot_odom',
            executable='global_vio_odom_node',
            name='global_vio_odom',
            output='screen',
            parameters=[node_params(params, 'global_vio_odom')],
        ))

    actions.extend([
        Node(
            package='football_navigation',
            executable='football_ball_fusion',
            name='football_ball_fusion',
            output='screen',
            parameters=[ball_params],
        ),
        Node(
            package='football_navigation',
            executable='football_team_role_assigner',
            name='football_team_role_assigner',
            output='screen',
            parameters=[role_params],
        ),
    ])

    if use_visualization:
        actions.append(Node(
            package='topic_visualization',
            executable='mutil_robot_tag_visual_node',
            name='mutil_robot_tag_visual',
            output='screen',
            parameters=[node_params(params, 'mutil_robot_tag_visual')],
        ))

        visualization_params = node_params(params, 'football_visualization_node')
        visualization_params.update({
            'field_frame': field_frame,
            'target_frame': field_frame,
            'show_field_boundary': True,
            'show_ball_marker': True,
            'show_ego_robot_marker': False,
            'show_robot_inflation_markers': False,
            'enable_costmap_markers': False,
        })

        actions.append(Node(
            package='football_navigation',
            executable='football_visualization_node',
            name='football_visualization_node',
            output='screen',
            parameters=[visualization_params],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'field_frame',
            default_value='tag_global',
        ),
        DeclareLaunchArgument(
            'ball_input_topic',
            default_value='/football/ball_pose_raw',
        ),
        DeclareLaunchArgument(
            'odom_topic_template',
            default_value='/global_vio/{namespace}/odom',
        ),
        DeclareLaunchArgument(
            'start_teammate_relay',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'use_visualization',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'initial_match_state',
            default_value='STOP',
        ),
        DeclareLaunchArgument(
            'continuous_demo',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'forced_team_a_striker_namespace',
            default_value='',
        ),
        OpaqueFunction(function=launch_nodes),
    ])
