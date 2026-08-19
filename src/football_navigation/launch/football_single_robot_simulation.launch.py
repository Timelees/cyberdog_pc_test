#!/usr/bin/python3
"""Fixed-ball, single-striker simulation with RViz visualization."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def node_params(data, node_name):
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def merged_node_params(runtime, simulation, node_name):
    """Use reusable runtime defaults, then apply simulation-only overrides."""
    output = node_params(runtime, node_name)
    output.update(node_params(simulation, node_name))
    return output


def launch_nodes(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    runtime_params_file = LaunchConfiguration('runtime_params_file').perform(context)
    with open(params_file, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}
    with open(runtime_params_file, 'r', encoding='utf-8') as stream:
        runtime_params = yaml.safe_load(stream) or {}

    simulation = node_params(params, 'football_simulation_input_publisher')
    robot_namespace = str(simulation.get(
        'robot_namespace', 'cyberdog_1')).strip().strip('/')
    if not robot_namespace:
        raise RuntimeError('robot_namespace must not be empty')

    field_frame = str(simulation.get('field_frame', 'tag_global'))
    odom_template = str(simulation.get(
        'odom_topic_template', '/global_vio/{namespace}/odom'))
    if '{namespace}' not in odom_template:
        raise RuntimeError('odom_topic_template must contain {namespace}')
    odom_topic = odom_template.replace('{namespace}', robot_namespace)
    robot_namespaces_csv = str(simulation.get(
        'robot_namespaces_csv',
        'cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5,'
        'cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10'))

    team_id = 'a' if int(robot_namespace.rsplit('_', 1)[1]) <= 5 else 'b'
    approach_topic = '/{}/football/approach_pose'.format(robot_namespace)
    path_topic = '/{}/plan'.format(robot_namespace)
    cmd_vel_topic = '/{}/cmd_vel'.format(robot_namespace)
    simulation.update({
        'approach_topic': approach_topic,
        'path_topic': path_topic,
        'striker_topic': '/football/team_{}/striker'.format(team_id),
        'kick_target_topic': '/football/team_{}/kick_target'.format(team_id),
        'cmd_vel_topic': cmd_vel_topic,
        'control_state_topic': '/{}/football/state'.format(robot_namespace),
    })

    goal = merged_node_params(runtime_params, params, 'football_goal_adapter')
    goal.update({
        'self_namespace': robot_namespace,
        'team_id': team_id,
        'field_frame': field_frame,
        'target_frame': field_frame,
        'odom_global_topic': odom_topic,
        'robot_namespaces_csv': robot_namespaces_csv,
        'robot_odom_topic_template': odom_template,
    })

    visualization = node_params(params, 'football_visualization_node')
    visualization.update({
        'self_namespace': robot_namespace,
        'team_id': team_id,
        'field_frame': field_frame,
        'target_frame': field_frame,
        'odom_topic': odom_topic,
        'approach_pose_topic': approach_topic,
        'path_topic': path_topic,
        'cmd_vel_topic': cmd_vel_topic,
        'expect_motion_cmds': True,
        'robot_namespaces_csv': robot_namespaces_csv,
        'robot_odom_topic_template': odom_template,
    })

    trajectory = merged_node_params(runtime_params, params, 'football_trajectory_adapter')
    trajectory.update({'target_frame': field_frame})

    tracking = merged_node_params(
        runtime_params, params, 'football_tracking_action_client')
    tracking.update({
        # Simulation has no hardware motion_status publisher.  Keep the
        # planner/action chain active so this launch validates navigation and
        # avoidance rather than the real-robot gait gate.
        'enabled': True,
        'require_slow_walk': False,
        'self_namespace': robot_namespace,
        'team_id': team_id,
        'expected_tracking_frame': field_frame,
    })

    navigator = node_params(params, 'football_simulation_navigator')
    navigator.update({
        'field_frame': field_frame,
        'odom_topic': odom_topic,
        'cmd_vel_topic': cmd_vel_topic,
        'self_namespace': robot_namespace,
        'robot_namespaces_csv': robot_namespaces_csv,
        'robot_odom_topic_template': odom_template,
    })

    fake_robots = node_params(params, 'football_fake_other_robot_publisher')
    fake_robots.update({
        'frame_id': field_frame,
        'robot_namespaces_csv': robot_namespaces_csv,
        'selected_namespace': robot_namespace,
        'simulate_selected_robot': False,
        'selected_initial_global_x': float(simulation.get('robot_x', 0.0)),
        'selected_initial_global_y': float(simulation.get('robot_y', 0.0)),
        'selected_global_odom_topic': odom_topic,
        'acceptance_global_odom_topic_template': odom_template,
    })

    rviz_config = os.path.join(
        get_package_share_directory('football_navigation'),
        'rviz', 'football_single_robot_simulation.rviz')

    return [
        Node(
            package='football_navigation',
            executable='football_fake_other_robot_publisher',
            name='football_fake_other_robot_publisher',
            output='screen',
            parameters=[fake_robots],
        ),
        Node(
            package='football_navigation',
            executable='football_simulation_input_publisher',
            name='football_simulation_input_publisher',
            output='screen',
            parameters=[simulation],
        ),
        Node(
            package='football_navigation',
            executable='football_goal_adapter',
            name='football_goal_adapter',
            namespace=robot_namespace,
            output='screen',
            parameters=[goal],
        ),
        Node(
            package='football_navigation',
            executable='football_trajectory_adapter',
            name='football_trajectory_adapter',
            namespace=robot_namespace,
            output='screen',
            parameters=[trajectory],
        ),
        Node(
            package='football_navigation',
            executable='football_tracking_action_client',
            name='football_tracking_action_client',
            namespace=robot_namespace,
            output='screen',
            parameters=[tracking],
        ),
        Node(
            package='football_navigation',
            executable='football_simulation_navigator',
            name='football_simulation_navigator',
            namespace=robot_namespace,
            output='screen',
            parameters=[navigator],
        ),
        Node(
            package='football_navigation',
            executable='football_visualization_node',
            name='football_visualization_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[visualization],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='football_simulation_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('football_navigation')
    return LaunchDescription([
        DeclareLaunchArgument(
            'isolate_dds', default_value='true',
            description='Ignore robot-only CYCLONEDDS_URI for simulation.'),
        # Simulation must not inherit a robot-only CycloneDDS XML path from
        # start_vio.sh or a previously sourced hardware shell.  The default
        # DDS implementation then uses its local defaults and stays isolated
        # from the real-robot launch.
        UnsetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            condition=IfCondition(LaunchConfiguration('isolate_dds'))),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share, 'params', 'football_single_robot_simulation.yaml'),
        ),
        DeclareLaunchArgument(
            'runtime_params_file',
            default_value=os.path.join(
                share, 'params', 'football_runtime_common.yaml'),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=launch_nodes),
    ])
