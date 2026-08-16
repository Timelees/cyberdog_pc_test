#!/usr/bin/python3
"""Robot-side football business nodes; role authority is intentionally absent."""

import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.join(
    get_package_share_directory('cyberdog_bringup'),
    'bringup',
))
from namespace_util import resolve_robot_namespace


def as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes')


def infer_team(namespace):
    normalized = str(namespace).strip().strip('/')
    prefix = 'cyberdog_'
    if not normalized.startswith(prefix):
        raise RuntimeError(
            'robot namespace must match cyberdog_<1..10>'
        )

    suffix = normalized[len(prefix):]
    if not suffix.isdigit():
        raise RuntimeError(
            'robot namespace must match cyberdog_<1..10>'
        )

    robot_id = int(suffix)
    if normalized != '{}{}'.format(prefix, robot_id) or not 1 <= robot_id <= 10:
        raise RuntimeError(
            'robot namespace must match cyberdog_<1..10>'
        )

    return 'a' if robot_id <= 5 else 'b'


def node_params(params, name):
    return [params.get(name, {}).get('ros__parameters', {})]


def merge_geometry_params(params, geometry):
    for name in (
            'football_goal_adapter',
            'football_visualization_node'):
        values = geometry.get(name, {}).get(
            'ros__parameters',
            {},
        )
        if values:
            destination = params.setdefault(
                name,
                {},
            ).setdefault(
                'ros__parameters',
                {},
            )
            for key, value in values.items():
                destination.setdefault(key, value)


def launch_nodes(context):
    namespace = LaunchConfiguration(
        'namespace'
    ).perform(context).strip().strip('/')

    runtime_mode = LaunchConfiguration(
        'runtime_mode'
    ).perform(context)

    if runtime_mode not in (
            'robot_competition',
            'simulation_single_host'):
        raise RuntimeError(
            'runtime_mode must be robot_competition or simulation_single_host'
        )

    enable_local_visualization = as_bool(
        LaunchConfiguration(
            'enable_local_visualization'
        ).perform(context)
    )

    with open(
            LaunchConfiguration('params_file').perform(context),
            encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}

    with open(
            LaunchConfiguration(
                'field_geometry_params_file'
            ).perform(context),
            encoding='utf-8') as stream:
        merge_geometry_params(
            params,
            yaml.safe_load(stream) or {},
        )

    field_frame = LaunchConfiguration(
        'unified_frame'
    ).perform(context)

    navigation_frame = LaunchConfiguration(
        'navigation_frame'
    ).perform(context)

    target_frame = LaunchConfiguration(
        'target_frame'
    ).perform(context)

    odom_global_topic = LaunchConfiguration(
        'odom_global_topic'
    ).perform(context).strip()

    team_id = infer_team(namespace)

    goal = params.setdefault(
        'football_goal_adapter',
        {},
    ).setdefault(
        'ros__parameters',
        {},
    )

    goal.update({
        'field_frame': field_frame,
        'target_frame': target_frame,
        'base_frame': 'base_link',
        'self_namespace': namespace,
        'team_id': team_id,
        'require_striker_role': True,
        'use_team_kick_target': True,
    })

    if odom_global_topic:
        goal['odom_global_topic'] = odom_global_topic

    trajectory = params.setdefault(
        'football_trajectory_adapter',
        {},
    ).setdefault(
        'ros__parameters',
        {},
    )

    trajectory.update({
        'target_frame': target_frame,
        'output_tracking_pose_topic': 'tracking_pose',
        'output_tracking_heartbeat_topic':
            'tracking_pose_heartbeat',
        'tracking_pose_heartbeat_hz': 10.0,
    })

    tracking = params.setdefault(
        'football_tracking_action_client',
        {},
    ).setdefault(
        'ros__parameters',
        {},
    )

    tracking.update({
        'enabled': True,
        'tracking_pose_topic': 'tracking_pose_heartbeat',
        'expected_tracking_frame': target_frame,
        'self_namespace': namespace,
        'team_id': team_id,
        'require_striker_role': True,
    })

    local_visualization = params.setdefault(
        'football_visualization_node',
        {},
    ).setdefault(
        'ros__parameters',
        {},
    )

    local_visualization.update({
        'target_frame': navigation_frame,
        'base_frame': 'base_link',
        'field_frame': field_frame,
        'self_namespace': namespace,
        'team_id': team_id,
        'field_marker_topic': 'football/markers/field',
        'robot_marker_topic': 'football/markers/robots',
        'ball_marker_topic': 'football/markers/ball',
        'approach_marker_topic': 'football/markers/approach_pose',
        'tracking_marker_topic': 'football/markers/tracking_pose',
        'path_marker_topic': 'football/markers/paths',
        'goal_marker_topic': 'football/markers/goals',
        'costmap_marker_topic': 'football/markers/costmap',
        'command_marker_topic': 'football/markers/commands',
        'status_marker_topic': 'football/markers/status',
        'ball_topic':
            '/football/ball_pose',
        'approach_pose_topic':
            'football/approach_pose',
        'tracking_pose_topic':
            'tracking_pose',
        'goal_pose_topic':
            'goal_pose',
        'other_robot_poses_topic':
            'football/other_robot_poses',
        'cmd_vel_topic':
            'cmd_vel',
        'motion_servo_cmd_topic':
            'motion_servo_cmd',
        'costmap_topic':
            'local_costmap_tracking/costmap',
        'path_topic':
            'plan',
        'local_trajectory_topic':
            'local_plan',
        'odom_topic':
            'odom_out',
        'show_field_boundary': False,
        'show_ball_marker': False,
        'show_goal_markers': False,
        'show_ego_robot_marker': True,
        'show_robot_inflation_markers': False,
        'enable_costmap_markers': False,
        'expect_motion_cmds': True,
    })

    nodes = [
        Node(
            package='football_navigation',
            executable='football_goal_adapter',
            name='football_goal_adapter',
            namespace=namespace,
            output='screen',
            parameters=node_params(
                params,
                'football_goal_adapter',
            ),
        ),
        Node(
            condition=IfCondition(
                LaunchConfiguration(
                    'enable_trajectory_adapter'
                )
            ),
            package='football_navigation',
            executable='football_trajectory_adapter',
            name='football_trajectory_adapter',
            namespace=namespace,
            output='screen',
            parameters=node_params(
                params,
                'football_trajectory_adapter',
            ),
        ),
        Node(
            condition=IfCondition(
                LaunchConfiguration(
                    'enable_tracking_action_client'
                )
            ),
            package='football_navigation',
            executable='football_tracking_action_client',
            name='football_tracking_action_client',
            namespace=namespace,
            output='screen',
            parameters=node_params(
                params,
                'football_tracking_action_client',
            ),
        ),
    ]

    if enable_local_visualization:
        nodes.append(
            Node(
                package='football_navigation',
                executable='football_visualization_node',
                name='football_local_visualization_node',
                namespace=namespace,
                output='screen',
                parameters=node_params(
                    params,
                    'football_visualization_node',
                ),
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                ],
            )
        )

    return nodes


def generate_launch_description():
    share = get_package_share_directory(
        'football_navigation'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=resolve_robot_namespace(),
        ),
        DeclareLaunchArgument(
            'runtime_mode',
            default_value='robot_competition',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share,
                'params',
                'football_robot_runtime.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'field_geometry_params_file',
            default_value=os.path.join(
                share,
                'params',
                'football_pc_authority.yaml',
            ),
        ),
        DeclareLaunchArgument(
            'unified_frame',
            default_value='tag_global',
        ),
        DeclareLaunchArgument(
            'navigation_frame',
            default_value='vodom',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='vodom',
        ),
        DeclareLaunchArgument(
            'odom_global_topic',
            default_value='',
        ),
        DeclareLaunchArgument(
            'enable_trajectory_adapter',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'enable_tracking_action_client',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'enable_local_visualization',
            default_value='false',
        ),
        OpaqueFunction(
            function=launch_nodes,
        ),
    ])
