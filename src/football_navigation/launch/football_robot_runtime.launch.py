#!/usr/bin/python3
"""Start the per-robot football control pipeline without owning Nav2 or PC authority."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_ROBOTS = (
    'cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5,'
    'cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10'
)


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def node_params(data, node_name):
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def merged_node_params(common, robot, node_name):
    output = node_params(common, node_name)
    output.update(node_params(robot, node_name))
    return output


def as_bool(value, argument_name):
    normalized = str(value).strip().lower()
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError('{} must be true or false'.format(argument_name))


def normalize_robot_namespace(value):
    output = str(value).strip().strip('/')
    parts = output.rsplit('_', 1)
    if len(parts) != 2 or parts[0] != 'cyberdog':
        raise RuntimeError('robot_namespace must match cyberdog_<1..10>')
    try:
        index = int(parts[1])
    except ValueError as error:
        raise RuntimeError('robot_namespace must match cyberdog_<1..10>') from error
    if index < 1 or index > 10:
        raise RuntimeError('robot_namespace must match cyberdog_<1..10>')
    return output, index


def launch_nodes(context):
    common = load_yaml(LaunchConfiguration('runtime_params_file').perform(context))
    robot = load_yaml(LaunchConfiguration('robot_params_file').perform(context))
    robot_namespace, robot_index = normalize_robot_namespace(
        LaunchConfiguration('robot_namespace').perform(context))

    configured_team = LaunchConfiguration('team_id').perform(context).strip().lower()
    inferred_team = 'a' if robot_index <= 5 else 'b'
    team_id = inferred_team if configured_team in ('', 'auto') else configured_team
    if team_id not in ('a', 'b'):
        raise RuntimeError('team_id must be auto, a, or b')

    field_frame = LaunchConfiguration('field_frame').perform(context).strip()
    base_frame = LaunchConfiguration('base_frame').perform(context).strip()
    odom_template = LaunchConfiguration('odom_topic_template').perform(context).strip()
    if not field_frame or not base_frame:
        raise RuntimeError('field_frame and base_frame must not be empty')
    if '{namespace}' not in odom_template:
        raise RuntimeError('odom_topic_template must contain {namespace}')
    odom_topic = odom_template.replace('{namespace}', robot_namespace)
    robot_namespaces_csv = LaunchConfiguration(
        'robot_namespaces_csv').perform(context).strip()

    enable_motion = as_bool(
        LaunchConfiguration('enable_motion').perform(context), 'enable_motion')
    require_striker = as_bool(
        LaunchConfiguration('require_striker_role').perform(context),
        'require_striker_role')
    require_other_robots = as_bool(
        LaunchConfiguration('require_other_robot_poses').perform(context),
        'require_other_robot_poses')
    require_costmaps = as_bool(
        LaunchConfiguration('require_costmap_ready').perform(context),
        'require_costmap_ready')
    minimum_other_robots = int(
        LaunchConfiguration('minimum_other_robot_count').perform(context))
    if minimum_other_robots < 0 or minimum_other_robots > 9:
        raise RuntimeError('minimum_other_robot_count must be between 0 and 9')

    goal = merged_node_params(common, robot, 'football_goal_adapter')
    kick_target_topic = LaunchConfiguration('kick_target_topic').perform(context).strip()
    goal.update({
        'self_namespace': robot_namespace,
        'team_id': team_id,
        'field_frame': field_frame,
        'target_frame': field_frame,
        'base_frame': base_frame,
        'odom_global_topic': odom_topic,
        'robot_namespaces_csv': robot_namespaces_csv,
        'robot_odom_topic_template': odom_template,
        'ball_pose_topic': LaunchConfiguration('ball_pose_topic').perform(context),
        'require_striker_role': require_striker,
        'require_other_robot_poses': require_other_robots,
        'minimum_other_robot_count': minimum_other_robots,
        'cmd_vel_topic': 'cmd_vel',
        'odom_timeout_sec': float(
            LaunchConfiguration('odom_timeout_sec').perform(context)),
        'max_ball_odom_skew_sec': float(
            LaunchConfiguration('max_ball_odom_skew_sec').perform(context)),
    })
    if kick_target_topic:
        goal.update({
            'kick_target_topic': kick_target_topic,
            'use_team_kick_target': False,
        })

    trajectory = merged_node_params(common, robot, 'football_trajectory_adapter')
    trajectory.update({'target_frame': field_frame})

    tracking = merged_node_params(common, robot, 'football_tracking_action_client')
    tracking.update({
        'enabled': enable_motion,
        'self_namespace': robot_namespace,
        'team_id': team_id,
        'expected_tracking_frame': field_frame,
        'require_striker_role': require_striker,
        'require_costmap_ready': require_costmaps,
        'expected_costmap_frame': field_frame,
        'require_slow_walk': as_bool(
            LaunchConfiguration('require_slow_walk').perform(context),
            'require_slow_walk'),
    })

    actions = [
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
    ]

    if as_bool(
            LaunchConfiguration('use_visualization').perform(context),
            'use_visualization'):
        visualization = merged_node_params(
            common, robot, 'football_visualization_node')
        visualization.update({
            'self_namespace': robot_namespace,
            'team_id': team_id,
            'field_frame': field_frame,
            'target_frame': field_frame,
            'base_frame': base_frame,
            'odom_topic': odom_topic,
            'approach_pose_topic': 'football/approach_pose',
            'path_topic': 'plan',
            'cmd_vel_topic': 'cmd_vel',
            'expect_motion_cmds': enable_motion,
            'robot_namespaces_csv': robot_namespaces_csv,
            'robot_odom_topic_template': odom_template,
        })
        actions.append(Node(
            package='football_navigation',
            executable='football_visualization_node',
            name='football_visualization_node',
            namespace=robot_namespace,
            output='screen',
            parameters=[visualization],
        ))

    return actions


def generate_launch_description():
    share = get_package_share_directory('football_navigation')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace', default_value='cyberdog_1',
            description='Robot namespace matching cyberdog_<1..10>.'),
        DeclareLaunchArgument('team_id', default_value='auto'),
        DeclareLaunchArgument('field_frame', default_value='tag_global'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'odom_topic_template',
            default_value='/global_vio/{namespace}/odom'),
        DeclareLaunchArgument('odom_timeout_sec', default_value='0.45'),
        DeclareLaunchArgument('max_ball_odom_skew_sec', default_value='0.12'),
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
            'enable_motion', default_value='false',
            description='Allow the football Action Client to send NavigateToPose goals.'),
        DeclareLaunchArgument(
            'require_slow_walk', default_value='true',
            description='Only allow goals while motion_status reports slow walk (303).'),
        DeclareLaunchArgument('require_striker_role', default_value='true'),
        DeclareLaunchArgument('require_other_robot_poses', default_value='true'),
        DeclareLaunchArgument('minimum_other_robot_count', default_value='9'),
        DeclareLaunchArgument('require_costmap_ready', default_value='true'),
        DeclareLaunchArgument('use_visualization', default_value='true'),
        DeclareLaunchArgument('ball_pose_topic', default_value='/football/ball_pose'),
        DeclareLaunchArgument('kick_target_topic', default_value=''),
        OpaqueFunction(function=launch_nodes),
    ])
