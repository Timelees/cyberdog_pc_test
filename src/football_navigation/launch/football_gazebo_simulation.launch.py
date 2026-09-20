#!/usr/bin/python3
"""Gazebo football validation using the real MotionServoCmd navigation path."""

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def load_node_params(path, node_name):
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    return dict(data.get(node_name, {}).get('ros__parameters', {}))


def generate_launch_description():
    football_share = get_package_share_directory('football_navigation')
    sim_share = get_package_share_directory('sim_base')
    sim_prefix = get_package_prefix('sim_base')
    robot_urdf = os.path.join(sim_share, 'model', 'actors.urdf')
    ball_sdf = os.path.join(sim_share, 'model', 'football', 'model.sdf')
    world = os.path.join(sim_share, 'world', 'soccer_playground.world')
    rviz_config = os.path.join(
        football_share, 'rviz', 'football_single_robot_simulation.rviz')
    gazebo_params = os.path.join(
        football_share, 'params', 'football_gazebo_sim.yaml')
    team_params = os.path.join(
        football_share, 'params', 'football_team_simulation.yaml')
    runtime_params = os.path.join(
        football_share, 'params', 'football_runtime_common.yaml')
    process_guard = os.path.join(
        get_package_prefix('football_navigation'), 'lib',
        'football_navigation', 'football_process_group_guard.py')
    team_spawn = load_node_params(
        team_params, 'football_fake_other_robot_publisher')
    gazebo_spawn = load_node_params(gazebo_params, 'football_gazebo_spawn')
    team_role = load_node_params(team_params, 'football_team_role_assigner')
    robot_names = [f'cyberdog_{index}' for index in range(1, 11)]
    active_robots = gazebo_spawn.get('active_robots', robot_names)
    if isinstance(active_robots, str):
        active_robots = [name.strip() for name in active_robots.split(',') if name.strip()]
    if not isinstance(active_robots, list) or not active_robots:
        raise RuntimeError('football_gazebo_spawn.active_robots must contain at least one robot')
    active_robots = [str(name).strip().lstrip('/') for name in active_robots]
    if set(active_robots) - set(robot_names) or len(active_robots) != len(set(active_robots)):
        raise RuntimeError(f'active_robots contains invalid or duplicate names: {active_robots}')
    robot_csv = ','.join(robot_names)
    primary_pose = team_spawn.get('initial_pose_cyberdog_1', [0.0, -2.0, 0.0])
    if not isinstance(primary_pose, list) or len(primary_pose) != 3:
        raise RuntimeError('initial_pose_cyberdog_1 must contain [x, y, yaw]')

    # Gazebo's SDF parser does not consistently resolve ROS package:// mesh
    # URIs when spawn_entity reads a URDF file directly. Resolve this prefix
    # before publishing robot_description so visual meshes are always found.
    with open(robot_urdf, 'r', encoding='utf-8') as stream:
        robot_description = stream.read()
    robot_description = robot_description.replace(
        'package://sim_base/model/', f'file://{sim_share}/model/')

    existing_models = os.environ.get('GAZEBO_MODEL_PATH', '')

    # Keep the package model directory as the primary root so the world can
    # resolve model://soccer_playground and model://football.  The robot SDF
    # uses model://*.dae for its meshes, which live in the nested ``meshes``
    # model directory; Gazebo does not search nested model roots implicitly.
    # Include that directory explicitly or the robot is spawned without its
    # visual geometry.
    model_root = os.path.join(sim_share, 'model')
    mesh_model_root = os.path.join(model_root, 'meshes')
    model_roots = [model_root, mesh_model_root]
    if existing_models:
        model_roots.append(existing_models)
    model_path = os.pathsep.join(model_roots)
    existing_plugins = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    plugin_path = os.path.join(sim_prefix, 'lib')
    if existing_plugins:
        plugin_path += os.pathsep + existing_plugins

    robot_namespace = LaunchConfiguration('robot_namespace')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    enable_gazebo_markers = LaunchConfiguration('enable_gazebo_markers')
    enable_team_coordination = LaunchConfiguration('enable_team_coordination')
    enable_builtin_team_control = LaunchConfiguration(
        'enable_builtin_team_control')
    enable_contact_wrench_fallback = LaunchConfiguration(
        'enable_contact_wrench_fallback')
    use_rviz = LaunchConfiguration('use_rviz')

    # Own Gazebo processes directly. gazebo_ros/launch/gzserver.launch.py uses
    # shell=True, so killing its shell can leave gzserver orphaned. Direct
    # execution lets launch signal the actual server/client PIDs and escalate
    # from SIGINT to SIGTERM/SIGKILL when necessary.
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver', world, '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so',
        ],
        output='screen',
        sigterm_timeout='5.0',
        sigkill_timeout='2.0',
        on_exit=Shutdown(reason='Gazebo server exited'),
    )
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        condition=IfCondition(gazebo_gui),
        sigterm_timeout='3.0',
        sigkill_timeout='2.0',
        on_exit=Shutdown(reason='Gazebo client exited'),
    )
    orphan_guard = ExecuteProcess(
        cmd=[process_guard],
        output='screen',
        sigterm_timeout='1.0',
        sigkill_timeout='1.0',
    )
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_cyberdog2',
        output='screen',
        arguments=[
            '-entity', 'cyberdog_1', '-topic', 'robot_description',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-Y', LaunchConfiguration('robot_yaw'),
        ],
    )
    # Every robot uses the actuated URDF.  Spawning is staggered because each
    # model creates an IMU sensor and a namespaced low-level plugin.
    spawn_peer_robots = []
    for index in range(2, 11):
        robot = f'cyberdog_{index}'
        pose = team_spawn.get(f'initial_pose_{robot}', [])
        if not isinstance(pose, list) or len(pose) != 3:
            raise RuntimeError(
                f'missing three-element initial_pose_{robot} in {team_params}')
        spawn_peer_robots.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot}',
            output='screen',
            arguments=[
                '-entity', robot, '-topic', 'robot_description',
                '-x', str(pose[0]), '-y', str(pose[1]),
                '-z', LaunchConfiguration('robot_z'), '-Y', str(pose[2]),
            ],
        ))
    spawn_ball = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_football',
        output='screen',
        arguments=[
            '-entity', 'football', '-file', ball_sdf,
            '-x', LaunchConfiguration('ball_x'),
            '-y', LaunchConfiguration('ball_y'),
            '-z', LaunchConfiguration('ball_z'),
        ],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )
    state_params = load_node_params(
        gazebo_params, "football_gazebo_state_bridge")
    state_params.update({
        "use_sim_time": True,
        "robot_models_csv": robot_csv,
        "robot_namespaces_csv": robot_csv,
        "odom_topic_template": "/global_vio/{namespace}/odom",
        "ball_topic": "/football/ball_pose",
    })
    gazebo_state_bridge = Node(
        package="football_navigation",
        executable="football_gazebo_state_bridge",
        name="football_gazebo_state_bridge",
        output="screen",
        parameters=[state_params],
    )
    # The role assigner keeps its fixed A/B team definitions (it validates
    # ten-player rosters), but can require only the selected active robots to
    # be online for a lightweight partial-team test.
    team_role['minimum_other_robot_count'] = max(1, len(active_robots) - 1)
    team_role.update({
        'use_sim_time': True,
        'field_frame': 'world',
        'authority_id': 'football_gazebo_simulation',
    })
    team_role_assigner = Node(
        condition=IfCondition(enable_team_coordination),
        package='football_navigation',
        executable='football_team_role_assigner',
        name='football_team_role_assigner',
        output='screen',
        parameters=[team_role],
    )
    # Ten complete and isolated motion chains:
    # navigator -> cmd_vel -> MotionServoCmd -> usr_cmd -> cyber_guide -> plugin.
    motion_nodes = []
    football_nodes = []
    servo_params = load_node_params(
        gazebo_params, 'football_servo_to_cyber_guide')
    servo_params.update({'use_sim_time': True, 'usr_cmd_service': 'usr_cmd'})
    cmd_adapter_params = {
        'use_sim_time': True,
        'cmd_vel_topic': 'cmd_vel',
        'motion_servo_cmd_topic': 'motion_servo_cmd',
        'control_valid_topic': 'football/control_valid',
        'motion_status_topic': 'motion_status',
        'motion_id': 309,
        'cmd_source': 4,
        'gait_value': 2,
        'cmd_vel_timeout_sec': 0.25,
        'smoothing_window_sec': 0.25,
        'max_linear_x': 0.26,
        'max_linear_y': 0.20,
        'max_angular_z': 1.0,
    }
    for robot in active_robots:
        motion_nodes.extend([
            Node(
                package='cyber_guide', executable='cyber_guide',
                namespace=robot, name='cyber_guide', output='screen',
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='football_navigation',
                executable='football_servo_to_cyber_guide',
                namespace=robot, name='football_servo_to_cyber_guide',
                output='screen', parameters=[dict(servo_params)],
            ),
            Node(
                package='football_navigation',
                executable='football_cmd_vel_to_servo',
                namespace=robot, name='football_cmd_vel_to_servo',
                output='screen', parameters=[dict(cmd_adapter_params)],
            ),
        ])

        team_id = 'a' if int(robot.rsplit('_', 1)[1]) <= 5 else 'b'
        goal = load_node_params(runtime_params, 'football_goal_adapter')
        goal.update(load_node_params(gazebo_params, 'football_goal_adapter'))
        goal.update({
            'use_sim_time': True,
            'self_namespace': robot,
            'team_id': team_id,
            'field_frame': 'world',
            'target_frame': 'world',
            'base_frame': 'base_link',
            'odom_global_topic': f'/global_vio/{robot}/odom',
            'robot_namespaces_csv': robot_csv,
            'robot_odom_topic_template': '/global_vio/{namespace}/odom',
            'require_striker_role': True,
            'role_topic': f'/{robot}/football/role',
            'tactical_target_topic': f'/{robot}/football/tactical_target',
        })
        trajectory = load_node_params(
            runtime_params, 'football_trajectory_adapter')
        trajectory.update({'use_sim_time': True, 'target_frame': 'world'})
        tracking = load_node_params(
            runtime_params, 'football_tracking_action_client')
        tracking.update({
            'use_sim_time': True,
            'enabled': True,
            'require_slow_walk': False,
            'self_namespace': robot,
            'team_id': team_id,
            'expected_tracking_frame': 'world',
            'require_striker_role': True,
        })
        navigator = load_node_params(
            team_params, 'football_simulation_navigator')
        navigator.update({
            'use_sim_time': True,
            'field_frame': 'world',
            'odom_topic': f'/global_vio/{robot}/odom',
            'cmd_vel_topic': f'/{robot}/cmd_vel',
            'self_namespace': robot,
            'robot_namespaces_csv': robot_csv,
            'robot_odom_topic_template': '/global_vio/{namespace}/odom',
            'control_state_topic': f'/{robot}/football/state',
        })
        football_nodes.extend([
            Node(condition=IfCondition(enable_builtin_team_control),
                 package='football_navigation', executable='football_goal_adapter',
                 namespace=robot, name='football_goal_adapter', output='screen',
                 parameters=[goal]),
            Node(condition=IfCondition(enable_builtin_team_control),
                 package='football_navigation', executable='football_trajectory_adapter',
                 namespace=robot, name='football_trajectory_adapter', output='screen',
                 parameters=[trajectory]),
            Node(condition=IfCondition(enable_builtin_team_control),
                 package='football_navigation', executable='football_tracking_action_client',
                 namespace=robot, name='football_tracking_action_client', output='screen',
                 parameters=[tracking]),
            Node(condition=IfCondition(enable_builtin_team_control),
                 package='football_navigation', executable='football_simulation_navigator',
                 namespace=robot, name='football_simulation_navigator', output='screen',
                 parameters=[navigator]),
        ])
    marker_bridge_params = load_node_params(gazebo_params, 'football_gazebo_marker_bridge')
    marker_bridge_params.update({
        'use_sim_time': True,
        'marker_topics': marker_bridge_params.get('marker_topics', [
            '/football/markers/ball',
            '/football/markers/approach_pose',
            '/football/markers/tracking_pose',
            '/football/markers/paths',
            '/football/markers/goals',
        ]) + [
            f'/football/markers/robots/{robot}' for robot in robot_names
        ],
        'robot_marker_topic': '',
    })
    gazebo_marker_bridge = Node(
        condition=IfCondition(PythonExpression([
            "'", gazebo_gui, "' == 'true' and '",
            enable_gazebo_markers, "' == 'true'",
        ])),
        package='football_navigation',
        executable='football_gazebo_marker_bridge',
        name='football_gazebo_marker_bridge',
        output='screen',
        parameters=[marker_bridge_params],
    )
    leg_contact_simulator = Node(
        condition=IfCondition(enable_contact_wrench_fallback),
        package='football_navigation',
        executable='football_gazebo_leg_contact_simulator',
        name='football_gazebo_leg_contact_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': ['/global_vio/', robot_namespace, '/odom'],
            'control_state_topic': [
                '/', robot_namespace, '/football/state'],
        }, load_node_params(
            gazebo_params, 'football_gazebo_leg_contact_simulator')],
    )

    visualization_params = load_node_params(
        team_params, 'football_visualization_node')
    visualization_params.update(
        load_node_params(gazebo_params, 'football_visualization_node'))
    visualization_params.update({
        'use_sim_time': True,
        'field_frame': 'world',
        'target_frame': 'world',
        'robot_namespaces_csv': robot_csv,
        'robot_odom_topic_template': '/global_vio/{namespace}/odom',
        'robot_local_trajectory_topic_template': '/{namespace}/local_plan',
        'show_ego_robot_marker': False,
        'show_other_robot_markers': True,
        'expect_motion_cmds': True,
    })
    visualization = Node(
        package='football_navigation', executable='football_visualization_node',
        name='football_gazebo_visualization', output='screen',
        parameters=[visualization_params])
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='football_gazebo_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )
    spawn_actions = [TimerAction(period=1.5, actions=[spawn_ball])]
    spawn_actions.append(TimerAction(period=1.8, actions=[spawn_robot]))
    for offset, peer in enumerate(spawn_peer_robots, start=1):
        spawn_actions.append(TimerAction(
            period=1.8 + 0.35 * offset, actions=[peer]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace', default_value='cyberdog_1',
            description='Controlled Gazebo entity mapping; keep cyberdog_1 in team mode.'),
        DeclareLaunchArgument('robot_x', default_value=str(primary_pose[0])),
        DeclareLaunchArgument('robot_y', default_value=str(primary_pose[1])),
        DeclareLaunchArgument('robot_z', default_value='0.42'),
        DeclareLaunchArgument('robot_yaw', default_value=str(primary_pose[2])),
        DeclareLaunchArgument(
            'ball_x', default_value='2.0',
            description='Initial Gazebo football X coordinate in world.'),
        DeclareLaunchArgument(
            'ball_y', default_value='-2.0',
            description='Initial Gazebo football Y coordinate in world.'),
        DeclareLaunchArgument(
            'ball_z', default_value='0.115',
            description='Initial Gazebo football center height.'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('enable_gazebo_markers', default_value='true'),
        DeclareLaunchArgument(
            'enable_team_coordination', default_value='true',
            description='Run ten-robot role assignment from Gazebo model poses.'),
        DeclareLaunchArgument(
            'enable_builtin_team_control', default_value='true',
            description=(
                'Run the built-in planner/controller for every robot. Set false '
                'when an external coordinator publishes /cyberdog_N/cmd_vel.')),
        DeclareLaunchArgument(
            'enable_contact_wrench_fallback', default_value='true',
            description=(
                'Inject deterministic low-force leg-contact assistance for Gazebo. '
                'Enabled by default because a centred rigid-body contact can trap the ball between the front legs.')),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        # Do not inherit a robot-only CycloneDDS configuration.
        UnsetEnvironmentVariable(name='CYCLONEDDS_URI'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
        # This scene is self-contained. Prevent gzclient from blocking its UI
        # while it tries to update the remote Gazebo model database.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        orphan_guard,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        *spawn_actions,
        TimerAction(period=6.0, actions=[
            gazebo_state_bridge, team_role_assigner, *motion_nodes]),
        TimerAction(period=7.0, actions=[
            *football_nodes, visualization, gazebo_marker_bridge,
            leg_contact_simulator, rviz]),
    ])
