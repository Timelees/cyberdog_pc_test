#!/usr/bin/python3
"""Ten-robot role-assignment simulation, isolated from the single-robot demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import yaml

from launch.actions import DeclareLaunchArgument, OpaqueFunction, UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def node_params(data, node_name):
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def merged_node_params(runtime, simulation, node_name):
    output = node_params(runtime, node_name)
    output.update(node_params(simulation, node_name))
    return output


def launch_nodes(context):
    share = get_package_share_directory('football_navigation')
    params_path = LaunchConfiguration('params_file').perform(context)
    runtime_path = LaunchConfiguration('runtime_params_file').perform(context)
    with open(params_path, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}
    with open(runtime_path, 'r', encoding='utf-8') as stream:
        runtime = yaml.safe_load(stream) or {}

    team_names = [f'cyberdog_{index}' for index in range(1, 6)]
    field_frame = 'tag_global'
    odom_template = '/global_vio/{namespace}/odom'
    robot_csv = ','.join(f'cyberdog_{index}' for index in range(1, 11))
    nodes = [
        Node(
            package='football_navigation',
            executable='football_fake_other_robot_publisher',
            name='football_fake_other_robot_publisher',
            output='screen',
            parameters=[node_params(params, 'football_fake_other_robot_publisher')],
        ),
        Node(
            package='football_navigation',
            executable='football_team_simulation_input_publisher',
            name='football_team_simulation_input_publisher',
            output='screen',
            parameters=[node_params(params, 'football_team_simulation_input_publisher')],
        ),
        Node(
            package='football_navigation',
            executable='football_team_role_assigner',
            name='football_team_role_assigner',
            output='screen',
            parameters=[node_params(params, 'football_team_role_assigner')],
        ),
    ]

    for robot in team_names:
        goal = merged_node_params(runtime, params, 'football_goal_adapter')
        goal.update({
            'self_namespace': robot,
            'team_id': 'a',
            'field_frame': field_frame,
            'target_frame': field_frame,
            'base_frame': f'{robot}/base_link',
            'odom_global_topic': odom_template.replace('{namespace}', robot),
            'robot_namespaces_csv': robot_csv,
            'robot_odom_topic_template': odom_template,
            'require_striker_role': True,
            'role_topic': f'/{robot}/football/role',
            'tactical_target_topic': f'/{robot}/football/tactical_target',
        })
        trajectory = merged_node_params(runtime, params, 'football_trajectory_adapter')
        trajectory.update({'target_frame': field_frame})
        tracking = merged_node_params(runtime, params, 'football_tracking_action_client')
        tracking.update({
            'enabled': True,
            'require_slow_walk': False,
            'self_namespace': robot,
            'team_id': 'a',
            'expected_tracking_frame': field_frame,
            'require_striker_role': True,
        })
        navigator = node_params(params, 'football_simulation_navigator')
        navigator.update({
            'field_frame': field_frame,
            'odom_topic': odom_template.replace('{namespace}', robot),
            'cmd_vel_topic': f'/{robot}/cmd_vel',
            'self_namespace': robot,
            'robot_namespaces_csv': robot_csv,
            'robot_odom_topic_template': odom_template,
            'control_state_topic': f'/{robot}/football/state',
        })
        nodes.extend([
            Node(package='football_navigation', executable='football_goal_adapter',
                 namespace=robot, name='football_goal_adapter', output='screen',
                 parameters=[goal]),
            Node(package='football_navigation', executable='football_trajectory_adapter',
                 namespace=robot, name='football_trajectory_adapter', output='screen',
                 parameters=[trajectory]),
            Node(package='football_navigation', executable='football_tracking_action_client',
                 namespace=robot, name='football_tracking_action_client', output='screen',
                 parameters=[tracking]),
            Node(package='football_navigation', executable='football_simulation_navigator',
                 namespace=robot, name='football_simulation_navigator', output='screen',
                 parameters=[navigator]),
        ])

    visualization = node_params(params, 'football_visualization_node')
    visualization.update({
        'field_frame': field_frame,
        'target_frame': field_frame,
        'base_frame': 'base_link',
        'self_namespace': '',
        'team_id': '',
        'odom_topic': '/global_vio/cyberdog_1/odom',
        'robot_namespaces_csv': robot_csv,
        'robot_odom_topic_template': odom_template,
        'robot_local_trajectory_topic_template': '/{namespace}/local_plan',
        'expect_motion_cmds': True,
        'show_ego_robot_marker': False,
    })
    nodes.append(Node(package='football_navigation', executable='football_visualization_node',
                      name='football_visualization_node', output='screen',
                      parameters=[visualization]))
    rviz_config = os.path.join(share, 'rviz', 'football_single_robot_simulation.rviz')
    nodes.append(Node(condition=IfCondition(LaunchConfiguration('use_rviz')),
                      package='rviz2', executable='rviz2', name='football_team_simulation_rviz',
                      output='screen', arguments=['-d', rviz_config]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('football_navigation')
    rviz_config = os.path.join(
        share, 'rviz', 'football_single_robot_simulation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'isolate_dds', default_value='true',
            description='Ignore robot-only CYCLONEDDS_URI for simulation.'),
        UnsetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            condition=IfCondition(LaunchConfiguration('isolate_dds'))),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share, 'params', 'football_team_simulation.yaml'),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'runtime_params_file',
            default_value=os.path.join(share, 'params', 'football_runtime_common.yaml'),
        ),
        OpaqueFunction(function=launch_nodes),
    ])
