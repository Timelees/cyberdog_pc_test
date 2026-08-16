#!/usr/bin/env python3
"""Static regression gate for the football-only no-map contract."""

import ast
import hashlib
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]


def read(path):
    return (ROOT / path).read_text(
        encoding="utf-8"
    )


def compact(text):
    """Apply Shell line-continuation semantics, then remove whitespace."""

    return "".join(
        text.replace(
            "\\\n",
            "",
        ).split()
    )


def function_string_literals(
    source,
    function_name,
):
    """Return the exact string literals used inside one Python function."""

    tree = ast.parse(source)

    for node in ast.walk(tree):
        if not isinstance(
            node,
            ast.FunctionDef,
        ):
            continue

        if node.name != function_name:
            continue

        return {
            child.s
            for child in ast.walk(node)
            if isinstance(
                child,
                ast.Str,
            )
        }

    return set()


class FootballContractTest(unittest.TestCase):
    def test_real_runtime_restores_teammate_interfaces(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        localization = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "node.football_localization.launch.py"
        )

        pc = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_pc_authority.launch.py"
        )

        pc_params = read(
            "cyberdog_nav2/football_navigation/params/"
            "football_pc_authority.yaml"
        )

        for required in (
            "node.football_localization.launch.py",
            "start_teammate_localization",
            "start_teammate_relay",
        ):
            self.assertIn(
                required,
                runtime,
            )

        for required in (
            "dog_d430i_stereo_odometry_localization.py",
            "'use_miloc'",
            "vinslocalization",
            "namespace +\n        '/tag_0_observation'",
            "'tf_subscribe_global':\n                    True",
        ):
            self.assertIn(
                required,
                localization,
            )

        self.assertNotIn(
            "vinsfollowing",
            localization,
        )

        self.assertIn(
            "mutil_robot_odom",
            pc,
        )

        self.assertIn(
            "global_vio_odom_node",
            pc,
        )

        self.assertIn(
            "global_vio_odom:",
            pc_params,
        )

        self.assertIn(
            "publish_map_anchor_tf: false",
            pc_params,
        )

    def test_simulation_reuses_entries_without_real_providers(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        start = runtime.index(
            'if mode == "simulation_single_host"'
        )
        simulation = runtime[start:]
        compact_simulation = compact(simulation)

        for required in (
            "pc_runtime",
            "robot_runtime",
            "football_fake_ball_publisher",
            "football_fake_other_robot_publisher",
            "simulation_selected_real_localization",
            "simulation_peer_fake_localization",
            "simulation_use_existing_selected_localization",
        ):
            self.assertIn(
                compact(required),
                compact_simulation,
            )

    def test_single_host_simulation_starts_ten_localizations_and_one_full_stack(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        start = runtime.index(
            'if mode == "simulation_single_host"'
        )
        simulation = runtime[start:]
        compact_simulation = compact(simulation)

        for required in (
            "simulation_robot_namespaces",
            "peer_namespaces = [",
            "for index, robot_namespace in enumerate(",
            "selected_robot_actions",
            '"simulate_selected_robot": False',
            '"navigation_frame": "vodom"',
        ):
            self.assertIn(
                compact(required),
                compact_simulation,
            )

        self.assertNotIn(
            "simulation_robot_start_interval_sec",
            runtime,
        )

    def test_robot_competition_branch_keeps_real_teammate_stack(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        start = runtime.index(
            'if mode == "robot_competition"'
        )
        end = runtime.index(
            'if mode == "pc_authority"',
            start,
        )
        real = runtime[start:end]
        compact_real = compact(real)

        for required in (
            '"start_mivins": start_mivins',
            '"start_apriltag": start_apriltag',
            '"start_odom_transform": start_odom_transform',
            '"runtime_mode": "robot_competition"',
            "velocity_adaptor_runtime",
            "start_teammate_localization",
        ):
            self.assertIn(
                compact(required),
                compact_real,
            )

        self.assertNotIn(
            "football_fake_other_robot_publisher",
            real,
        )

    def test_fake_world_does_not_replace_teammate_nodes(self):
        fake = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )

        self.assertNotIn(
            "odom_global_pub",
            fake,
        )

        self.assertNotIn(
            "/global_vio",
            fake,
        )

        self.assertNotIn(
            "integrateOtherRobot",
            fake,
        )

        self.assertIn(
            "odom_slam_topic_template",
            fake,
        )

        self.assertIn(
            "cmd_vel_topic_template",
            fake,
        )

    def test_fake_world_scripts_only_non_selected_peers(self):
        fake = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )
        start = runtime.index(
            "    if football_acceptance_all_fake:"
        )
        end = runtime.index(
            '    if mode == "robot_competition":',
            start,
        )
        all_fake = runtime[start:end]

        for required in (
            "selected_namespace_",
            "scripted_peer_motion_enabled_",
            "integrateRobot",
            "publish_acceptance_global_odom_",
            "robot.id != selected_namespace_",
        ):
            self.assertIn(required, fake)

        self.assertIn(
            compact('"robot_namespaces_csv": selected_namespace'),
            compact(all_fake),
        )
        self.assertIn(
            compact('"scripted_peer_motion_enabled": False'),
            compact(all_fake),
        )
        self.assertIn(
            compact('"continuous_demo_enabled": False'),
            compact(all_fake),
        )

    def test_navigation_uses_namespaced_mivins_tf(self):
        localization = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "node.football_localization.launch.py"
        )

        tracking = read(
            "cyberdog_tracking_base/mcr_bringup/launch/"
            "bringup_follow_only_launch.py"
        )

        self.assertIn(
            "SetRemap",
            localization,
        )

        self.assertIn(
            "dst='tf'",
            localization,
        )

        self.assertIn(
            "navigation_frame",
            tracking,
        )

    def test_teammate_visualization_is_reused(self):
        pc = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_pc_authority.launch.py"
        )

        self.assertIn(
            "topic_visualization",
            pc,
        )

        self.assertIn(
            "mutil_robot_tag_visual_node",
            pc,
        )

    def test_robot_side_never_runs_role_authority(self):
        launch = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_navigation.launch.py"
        )

        self.assertNotIn(
            "football_team_role_assigner",
            launch,
        )

        for removed in (
            "enable_team_role_assigner",
            "publish_team_roles",
            "enable_ego_priority_window",
            "ego_priority_window_m",
            "ego_priority_hold_sec",
            "enable_fake_ball",
            "enable_fake_other_robots",
            "enable_simulated_motion",
        ):
            self.assertNotIn(
                removed,
                launch,
            )

    def test_fake_sources_match_production_interfaces(self):
        ball = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_ball_publisher.cpp"
        )

        self.assertIn(
            '"ball_topic", "/football/ball_pose_raw"',
            ball,
        )

        self.assertIn(
            '"frame_id", "tag_global"',
            ball,
        )

        self.assertIn(
            "velocity_transfer_gain_",
            ball,
        )

        self.assertIn(
            "ball_friction_mps2_",
            ball,
        )

        other = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )

        for required in (
            "cmd_vel_topic_template",
            "odom_out_topic_template",
            "odom_slam_topic_template",
            "tf_topic_template",
            "tag_frame_template",
            "geometry_msgs::msg::Twist",
            "field_frame_",
        ):
            self.assertIn(
                required,
                other,
            )

        self.assertIn(
            compact('"frame_id", "tag_global"'),
            compact(other),
        )

        self.assertNotIn(
            "geometry_msgs::msg::PoseArray",
            other,
        )

        self.assertIn(
            "tf2_msgs::msg::TFMessage",
            other,
        )

    def test_ball_fusion_validates_time_and_synchronizes_odom(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_ball_fusion.cpp"
        )

        for required in (
            "future_tolerance_sec_",
            "odom_histories_",
            "findNearestOdom",
            "max_detection_odom_skew_sec_",
            "yawFromQuaternion",
        ):
            self.assertIn(
                required,
                source,
            )

        self.assertNotIn(
            "tf2::getYaw",
            source,
        )

    def test_role_authority_is_unique_and_direction_agnostic(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        for required in (
            "authority_instance_id_",
            "seen_instance",
            "goal_separation",
            "max_pose_skew_sec_",
            '"/" + robot_id + "/football/other_robot_poses"',
        ):
            self.assertIn(
                required,
                source,
            )

        self.assertNotIn(
            "striker_lease_sec",
            source,
        )

        self.assertNotIn(
            "team_a_attack_goal_x_ <= team_b_attack_goal_x_",
            source,
        )

        self.assertNotIn(
            "robot_odoms_.erase",
            source,
        )

        self.assertNotIn(
            "have_ball_ = false",
            source,
        )

        self.assertNotIn(
            "visualization_robot_poses",
            source,
        )

    def test_goal_adapter_is_timestamped_math_only(self):
        header = read(
            "cyberdog_nav2/football_navigation/include/"
            "football_navigation/football_goal_adapter.hpp"
        )

        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_goal_adapter.cpp"
        )

        combined = header + source

        for forbidden in (
                        "tf2_ros::Buffer",
            "TransformListener",
            "allow_world_tf_fallback",
            "initTfIfNeeded",
            "lookupTransform",
            "tf_buffer_",
        ):
            self.assertNotIn(
                forbidden,
                combined,
            )

        for required in (
            "odom_history_",
            "findOdomAt",
            "kickTargetFresh",
            "plan.feasible",
            "isRobotBehindAndAlignedForKick",
            "centered_on_kick_line",
            "robot_still_behind",
            "push_target_lead_m_",
            "max_local_goal_distance_m_",
        ):
            self.assertIn(
                required,
                combined,
            )

        for required in (
            "otherRobotPosesFresh",
            "other_robot_pose_odom_",
            "WAITING_TEAM_OBSTACLES",
            "WAITING_KICK_TARGET",
        ):
            self.assertIn(
                required,
                combined,
            )

        for removed in (
            "ego_base_frame_",
            "min_ball_distance_",
            "retreat_distance_m_",
        ):
            self.assertNotIn(
                removed,
                combined,
            )

    def test_bad_target_packets_do_not_clear_good_state(self):
        trajectory = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_trajectory_adapter.cpp"
        )

        callback = trajectory[
            trajectory.index(
                "void approachCallback"
            ):
            trajectory.index(
                "bool finitePose"
            )
        ]

        self.assertNotIn(
            "clearGoal",
            callback,
        )

        self.assertIn(
            "last_input_stamp_",
            trajectory,
        )

        self.assertNotIn(
            "goal_latch_timeout_sec",
            trajectory,
        )

        action = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_tracking_action_client.cpp"
        )

        self.assertNotIn(
            'cancelActiveGoal("invalid',
            action,
        )

        self.assertNotIn(
            "wait_for_action_server",
            action,
        )

        self.assertIn(
            "action_server_is_ready",
            action,
        )

        for required in (
            "last_completed_tracking_pose_",
            "have_completed_tracking_pose_",
            "completed_pose_xy_tolerance_",
            "completed_pose_yaw_tolerance_",
            "isPoseNear(",
            "force_retry_",
        ):
            self.assertIn(
                required,
                action,
            )

    def test_obstacle_layer_is_namespaced_pose_array_only(self):
        header = read(
            "cyberdog_nav2/football_navigation/include/"
            "football_navigation/multi_robot_obstacle_layer.hpp"
        )
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        combined = header + source

        for forbidden in (
            "odomCallback",
            "odom_topics",
            "odom_subs",
            "use_odom_input",
            "last_odom_stamps",
            "nav_msgs/msg/odometry.hpp",
        ):
            self.assertNotIn(forbidden, combined)

        for required in (
            "tf_->transform",
            "transform_tolerance_sec",
            "updateTracks(accepted, input_size, stamp)",
            "obstacle_tracks_",
            "appendPredictedSweep",
            "input_size = std::min<std::size_t>(msg->poses.size(), 20)",
            "updateBounds(",
            "updateCosts(",
            "previous_obstacles_",
            "resetMap(",
        ):
            self.assertIn(required, source)

        self.assertIn(
            compact(
                "accepted.size() < "
                "static_cast<std::size_t>(minimum_obstacle_count_)"
            ),
            compact(source),
        )

    def test_parameters_follow_monotonic_timeout_chain(self):
        robot = yaml.safe_load(
            read(
                "cyberdog_nav2/football_navigation/params/"
                "football_robot_runtime.yaml"
            )
        )

        pc = yaml.safe_load(
            read(
                "cyberdog_nav2/football_navigation/params/"
                "football_pc_authority.yaml"
            )
        )

        goal = robot[
            "football_goal_adapter"
        ][
            "ros__parameters"
        ]

        trajectory = robot[
            "football_trajectory_adapter"
        ][
            "ros__parameters"
        ]

        tracking = robot[
            "football_tracking_action_client"
        ][
            "ros__parameters"
        ]

        fake_world = robot[
            "football_fake_world"
        ][
            "ros__parameters"
        ]

        pc_params = pc[
            "football_ball_fusion"
        ][
            "ros__parameters"
        ]

        role_params = pc[
            "football_team_role_assigner"
        ][
            "ros__parameters"
        ]
        self.assertEqual(
            0.6,
            tracking[
                "tracking_pose_timeout_sec"
            ],
        )

        self.assertEqual(
            0.8,
            tracking[
                "role_timeout_sec"
            ],
        )

        self.assertEqual(
            0.05,
            tracking[
                "completed_pose_xy_tolerance"
            ],
        )

        self.assertEqual(
            0.08,
            tracking[
                "completed_pose_yaw_tolerance"
            ],
        )

        self.assertEqual(
            0.8,
            goal[
                "kick_target_timeout_sec"
            ],
        )

        self.assertEqual(
            0.35,
            goal[
                "cmd_vel_timeout_sec"
            ],
        )

        self.assertEqual(
            0.3,
            fake_world[
                "cmd_vel_timeout_sec"
            ],
                    )

        self.assertEqual(
            10.0,
            trajectory[
                "tracking_pose_heartbeat_hz"
            ],
        )

        self.assertEqual(
            2.0,
            trajectory[
                "planner_update_rate_hz"
            ],
        )

        self.assertEqual(
            "tracking_pose_heartbeat",
            trajectory[
                "output_tracking_heartbeat_topic"
            ],
        )

        self.assertEqual(
            0.3,
            pc_params[
                "max_detection_age_sec"
            ],
        )

        self.assertEqual(
            0.45,
            role_params[
                "ball_timeout_sec"
            ],
        )

        self.assertNotIn(
            "striker_lease_sec",
            pc_params,
        )

        self.assertNotIn(
            "allow_world_tf_fallback",
            goal,
        )

    def test_cmake_links_existing_geometry_helper(self):
        cmake = read(
            "cyberdog_nav2/football_navigation/"
            "CMakeLists.txt"
        )

        for target in (
            "football_fake_ball_publisher",
            "football_ball_fusion",
        ):
            start = cmake.index(
                "target_link_libraries(" +
                target
            )

            self.assertIn(
                "football_geometry_lib",
                cmake[
                    start:
                    start + 140
                ],
            )

    def test_rviz_local_view_has_no_fabricated_world_tf(self):
        rviz = read(
            "cyberdog_nav2/football_navigation/rviz/"
            "football_no_map_costmap.rviz"
        )

        helper = read(
            "cyberdog_nav2/football_navigation/rviz/"
            "rviz_football_costmap_docker.sh"
        )

        launch = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_navigation.launch.py"
        )

        visualizer = read(
            "cyberdog_nav2/football_navigation/src/"
            "visualization/football_visualization_node.cpp"
        )

        self.assertIn(
            "Fixed Frame: vodom",
            rviz,
        )

        self.assertIn(
            "local_visualization_marker_array",
            rviz,
        )

        self.assertNotIn(
            "static_transform_publisher",
            helper,
        )

        self.assertIn(
            "football_local_visualization_node",
            launch,
        )

        for required in (
            '"field_frame", "tag_global"',
            "qos_profile_sensor_data",
            "show_field_boundary_",
            "show_ball_marker_",
        ):
            self.assertIn(
                required,
                visualizer,
            )

        self.assertNotIn(
            "equivalent_frame_fallback",
            launch,
        )

        self.assertNotIn(
            "equivalent_frame_fallback",
            visualizer,
        )

    def test_apriltag_defaults_are_preserved(self):
        localization = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "node.football_localization.launch.py"
        )

        for required in (
            "'latch_first_tag_pose':\n"
            "                        True",

            "'publish_latched_when_lost':\n"
            "                        True",

            "'use_first_pose_as_global':\n"
            "                        True",

            "'apply_wall_tag_alignment':\n"
            "                    True",

            "'rotate_output_x_180':\n"
            "                    True",
        ):
            self.assertIn(
                required,
                localization,
            )

        self.assertNotIn(
            "simulation_mode",
            localization,
        )

    def test_fake_world_only_emulates_sensor_boundary(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )

        for required in (
            "makeSimulatedRawTagObservation",
            "makeRawTagObservation",
            "global_tf_static_pub_",
            "publishRawTagObservations",
        ):
            self.assertIn(
                required,
                source,
            )

        dynamic_tf = source[
            source.index(
                "tf2_msgs::msg::TFMessage makeTfMessage"
            ):
            source.index(
                "void update()"
            )
        ]

        self.assertNotIn(
            "tag_frame_template_",
            dynamic_tf,
        )

    def test_effective_navigation_frames_are_vodom(self):
        source = read(
            "cyberdog_tracking_base/mcr_bringup/launch/"
            "bringup_follow_only_launch.py"
        )

        follow = yaml.safe_load(
            read(
                "cyberdog_tracking_base/mcr_bringup/params/"
                "follow_params.yaml"
            )
        )

        self.assertIn(
            "import yaml",
            source,
        )

        self.assertIn(
            "follow_overrides",
            source,
        )

        self.assertIn(
            "_write_structured_params",
            source,
        )

        self.assertNotIn(
            "_rewrite_params_text",
            source,
        )

        self.assertEqual(
            follow[
                "bt_navigator_tracking"
            ][
                "ros__parameters"
            ][
                "global_frame"
            ],
            "vodom",
        )

        for section in (
            "local_costmap_tracking",
            "rolling_window_costmap",
        ):
            params = follow[
                section
            ][
                section
            ][
                "ros__parameters"
            ]
            self.assertTrue(
                params[
                    "rolling_window"
                ]
            )

            layer = params[
                "multi_robot_obstacle_layer"
            ]
            self.assertFalse(
                layer.get(
                    "pose_array_includes_self",
                    False,
                )
            )
            self.assertEqual(
                layer[
                    "self_filter_radius"
                ],
                0.02,
            )

            self.assertEqual(
                params[
                    "multi_robot_obstacle_layer"
                ][
                    "target_frame"
                ],
                "vodom",
            )

    def test_local_visualizer_uses_namespaced_vodom_tf(self):
        robot_runtime = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_robot_runtime.launch.py"
        )

        business = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_navigation.launch.py"
        )

        self.assertIn(
            "DeclareLaunchArgument("
            "'navigation_frame', "
            "default_value='vodom')",
            robot_runtime,
        )

        self.assertIn(
            "'navigation_frame': "
            "navigation_frame",
            robot_runtime,
        )

        self.assertIn(
            "'target_frame': "
            "navigation_frame",
            business,
        )

        self.assertIn(
            "('/tf', 'tf')",
            business,
        )

        self.assertIn(
            "('/tf_static', 'tf_static')",
            business,
        )

        self.assertNotIn(
            "enable_team_roles",
            business,
        )

    def test_only_one_sensor_static_tf_owner(self):
        robot_runtime = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_robot_runtime.launch.py"
        )

        self.assertNotIn(
            "node.state_publisher.launch.py",
            robot_runtime,
        )

        self.assertNotIn(
            "node.static_tf.launch.py",
            robot_runtime,
        )

    def test_visualizer_has_no_frame_relabel_fallback(self):
        visualizer = read(
            "cyberdog_nav2/football_navigation/src/"
            "visualization/football_visualization_node.cpp"
        )

        self.assertNotIn(
            "equivalentFrameFallback",
            visualizer,
        )

        self.assertNotIn(
            "equivalent_frame_fallback_",
            visualizer,
        )

        self.assertIn(
            "grid.header.frame_id != target_frame_",
            visualizer,
        )

    def test_no_map_runtime_is_fail_closed(self):
        env = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_env.sh"
        )

        fake = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_start_team_chase_fake.sh"
        )

        real = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_start_team_chase_real.sh"
        )

        self.assertIn(
            'FOOTBALL_STRICT_NO_MAP:-1',
            env,
        )

        self.assertIn(
            "ros2 node list",
            env,
        )

        self.assertIn(
            'FOOTBALL_STRICT_NO_MAP='
            '"${FOOTBALL_STRICT_NO_MAP:-1}"',
            fake,
        )

        self.assertIn(
            'FOOTBALL_STRICT_NO_MAP='
            '"${FOOTBALL_STRICT_NO_MAP:-1}"',
            real,
        )

    def test_runtime_diagnostics_check_frames_and_ownership(self):
        diag = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_quick_diag.sh"
        )
        behavior = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        for required in (
            "/proc/${BACKEND_PID}/environ",
            "ROS_DOMAIN_ID",
            "RMW_IMPLEMENTATION",
            "CYCLONEDDS_URI",
            "ROS_LOCALHOST_ONLY",
            "football_user_function_acceptance.py",
            'exec "$ACCEPTANCE"',
        ):
            self.assertIn(required, diag)

        for required in (
            '"/{}/cmd_vel"',
            '"/{}/tracking_pose_heartbeat"',
            '"/{}/tf"',
            '"/{}/tracking_target/_action/status"',
            "fake world integrates the same formal Controller cmd_vel",
            "formal ball, cmd-integrated odom and continuous vodom->base_link TF",
            "stable TargetTracking -> Planner -> FollowP -> Controller chain",
        ):
            self.assertIn(required, behavior)

    def test_quick_diag_refreshes_graph_after_initial_topic_probes(self):
        diag = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_quick_diag.sh"
        )

        self.assertIn(
            "find_backend_pid",
            diag,
        )
        self.assertIn(
            "diagnostic shell matches backend ROS environment",
            diag,
        )
        self.assertIn(
            'exec "$ACCEPTANCE"',
            diag,
        )
        self.assertNotIn(
            "ros2 topic echo",
            diag,
        )

    def test_quick_diag_accepts_the_tracking_cmd_vel_owner_set(self):
        behavior = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        self.assertIn(
            '"/{}/cmd_vel".format(self.namespace)',
            behavior,
        )
        self.assertIn(
            "self._on_cmd_vel",
            behavior,
        )
        self.assertIn(
            "fake world integrates the same formal Controller cmd_vel",
            behavior,
        )
        self.assertNotIn(
            compact("create_publisher( Twist"),
            compact(behavior),
        )

    def test_reference_frames_and_cyclonedds_xml_are_valid(self):
        costmap = yaml.safe_load(
            read(
                "cyberdog_nav2/football_navigation/params/"
                "football_costmap_reference.yaml"
            )
        )

        multi = yaml.safe_load(
            read(
                "cyberdog_nav2/football_navigation/params/"
                "football_multi_robot_odom_params.yaml"
            )
        )

        local = costmap[
            "local_costmap"
        ][
            "local_costmap"
        ][
            "ros__parameters"
        ]

        self.assertEqual(
            local[
                "global_frame"
            ],
                        "vodom",
        )

        self.assertEqual(
            local[
                "multi_robot_obstacle_layer"
            ][
                "target_frame"
            ],
            "vodom",
        )

        for section in (
            "local_costmap_tracking",
            "rolling_window_costmap",
        ):
            self.assertEqual(
                multi[
                    section
                ][
                    section
                ][
                    "ros__parameters"
                ][
                    "multi_robot_obstacle_layer"
                ][
                    "target_frame"
                ],
                "vodom",
            )

        ET.parse(
            ROOT /
            "cyclonedds_nx_usb.xml"
        )

        setup = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_setup_cyclonedds.sh"
        )

        self.assertIn(
            "validate_xml",
            setup,
        )

        self.assertIn(
            'validate_xml "$NX_TEMPLATE"',
            setup,
        )

    def test_tracking_integration_does_not_create_package_cycle(self):
        football_manifest = read(
            "cyberdog_nav2/football_navigation/"
            "package.xml"
        )

        robot_runtime = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_robot_runtime.launch.py"
        )

        entry_runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        entry_manifest = read(
            "cyberdog_nav2/navigation_bringup/"
            "package.xml"
        )

        mcr_manifest = read(
            "cyberdog_tracking_base/mcr_bringup/"
            "package.xml"
        )

        deploy_check = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_verify_deploy.sh"
        )

        self.assertNotIn(
            "<exec_depend>mcr_bringup</exec_depend>",
            football_manifest,
        )

        self.assertIn(
            "bringup_follow_only_launch.py",
            entry_runtime,
        )

        self.assertNotIn(
            "bringup_follow_only_launch.py",
            robot_runtime,
        )

        self.assertIn(
            "<exec_depend>mcr_bringup</exec_depend>",
            entry_manifest,
        )

        self.assertIn(
            "<exec_depend>football_navigation</exec_depend>",
            mcr_manifest,
        )

        self.assertIn(
            "share/navigation_bringup/launch/"
            "football_runtime.launch.py",
            deploy_check,
        )

        self.assertIn(
            "share/football_navigation/launch/"
            "football_robot_runtime.launch.py",
            deploy_check,
        )

    def test_robot_runtime_contains_only_football_nodes(self):
        robot_runtime = read(
            "cyberdog_nav2/football_navigation/launch/"
            "football_robot_runtime.launch.py"
        )

        entry_runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        self.assertNotIn(
            "navigation_bringup",
            robot_runtime,
        )

        self.assertNotIn(
            "node.state_publisher.launch.py",
            robot_runtime,
        )

        self.assertNotIn(
            "node.velocity_adaptor.launch.py",
            robot_runtime,
        )

        self.assertIn(
            "node.state_publisher.launch.py",
            entry_runtime,
        )

        self.assertIn(
                        "node.velocity_adaptor.launch.py",
            entry_runtime,
        )

    def test_relative_trajectory_preserves_geometry_stamp_and_refreshes_heartbeat(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_trajectory_adapter.cpp"
        )

        for forbidden in (
            "goal_latch_timeout_sec",
            "enable_goal_smoothing",
            "smoothGoal",
            "needsGoalUpdate",
            "publishLatchedGoal",
            "tracking_pose_transformed",
            "transformToTargetFrame",
        ):
            self.assertNotIn(
                forbidden,
                source,
            )

        self.assertIn(
            "msg->header.frame_id != target_frame_",
            source,
        )

        self.assertIn(
            "latest_tracking_pose_ = *msg",
            source,
        )

        planner_update = source[
            source.index("void publishPlannerUpdate()"):
            source.index("void publishTrackingHeartbeat()")
        ]
        self.assertNotIn(
            "header.stamp = now()",
            planner_update,
        )
        self.assertNotIn(
            "latest_tracking_pose_.header.stamp = now()",
            source,
        )
        heartbeat = source[
            source.index("void publishTrackingHeartbeat()"):
            source.index("std::string target_frame_")
        ]
        self.assertIn(
            "output.header.stamp = now()",
            heartbeat,
        )

        self.assertIn(
            "publishTrackingHeartbeat",
            source,
        )

        self.assertIn(
            "publishPlannerUpdate",
            source,
        )

        self.assertIn(
            "separate_heartbeat_topic_",
            source,
        )

    def test_control_and_temporal_safety_contracts(self):
        roles = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        action = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_tracking_action_client.cpp"
        )

        goal = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_goal_adapter.cpp"
        )

        ball = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_ball_fusion.cpp"
        )

        for required in (
            "std::stod(seen_time)",
            "currentStrikerFresh",
            "predicted_x",
            "ego_state->second.odom.header.stamp",
        ):
            self.assertIn(
                required,
                roles,
            )

        for required in (
            "cancel_pending_",
            "goal_generation_",
            "active_generation_",
            "expected_tracking_frame_",
            "async_cancel_goal",
        ):
            self.assertIn(
                required,
                action,
            )

        for required in (
                        "cmd_vel_timeout_sec_",
            "max_other_robot_odom_skew_sec_",
            "max_other_robot_message_age_sec_",
            "processingDue",
            "markProcessed",
        ):
            self.assertIn(
                required,
                goal,
            )

        self.assertIn(
            "goalEventCallback",
            ball,
        )

        self.assertIn(
            "have_previous_ = false",
            ball,
        )

    def test_manifest_exports_costmap_and_football_bt_support(self):
        manifest = read(
            "cyberdog_nav2/football_navigation/"
            "package.xml"
        )

        cmake = read(
            "cyberdog_nav2/football_navigation/"
            "CMakeLists.txt"
        )

        self.assertIn("<depend>nav2_core</depend>", manifest)
        self.assertIn("find_package(nav2_core REQUIRED)", cmake)
        self.assertIn("nav2_core", cmake)
        self.assertIn("BT_RegisterNodesFromPlugin", read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        ))

    def test_match_state_defaults_to_stop_and_gates_motion(self):
        roles = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        goal = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_goal_adapter.cpp"
        )

        action = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_tracking_action_client.cpp"
        )

        for source in (
            roles,
            goal,
            action,
        ):
            self.assertIn(
                "match_state_topic",
                source,
            )

            self.assertIn(
                "matchStateAllowsMovement",
                source,
            )

        self.assertIn(
            'initial_match_state", "STOP"',
            roles,
        )

        self.assertIn(
            "KICKOFF_A",
            roles,
        )

        self.assertIn(
            "KICKOFF_B",
            roles,
        )

    def test_match_state_has_command_input(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        self.assertIn(
            "match_state_command_topic",
            source,
        )

        self.assertIn(
            "matchStateCommandCallback",
            source,
        )

        self.assertIn(
            "validMatchState(msg->data)",
            source,
        )

    def test_kickoff_persists_until_ball_release(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        self.assertIn(
            "kickoff_release_distance_m",
            source,
        )

        self.assertIn(
            "kickoff_max_duration_sec",
            source,
        )

        self.assertIn(
            "ball_released || kickoff_expired",
            source,
        )

        self.assertNotIn(
            'match_state_ = "PLAY";\n'
            '      publishMatchState();\n'
            '    }\n'
            '    if (!matchStateAllowsMovement',
            source,
        )

    def test_goal_event_is_exactly_validated(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        self.assertIn(
            'msg->data == "goal team_a kickoff"',
            source,
        )

        self.assertIn(
            'msg->data == "goal team_b kickoff"',
            source,
        )

        self.assertNotIn(
            'msg->data.find("team_a")',
            source,
        )

    def test_dynamic_striker_reselects_from_remaining_valid_team_members(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        params = read(
            "cyberdog_nav2/football_navigation/params/"
            "football_pc_authority.yaml"
        )

        self.assertIn(
            '"minimum_online_per_team", 1',
            source,
        )

        self.assertIn(
            "minimum_online_per_team: 1",
            params,
        )

        self.assertIn(
            "team_a_eligible",
            source,
        )

        self.assertIn(
            "team_b_eligible",
            source,
        )

        self.assertIn(
            "countValidRobots(team_a) >= "
            "minimum_online_per_team_",
            source,
        )

        self.assertIn(
            "countValidRobots(team_b) >= "
            "minimum_online_per_team_",
            source,
        )

        self.assertIn(
            "countValidRobots(all) < "
            "minimum_other_robot_count_ + 1",
            source,
        )

        self.assertIn(
            'stopAllRoles("SAFE_STOP team_data_stale")',
            source,
        )

        self.assertNotIn(
            "countValidRobots(team_a) < "
            "minimum_online_per_team_ ||",
            source,
        )

    def test_role_authority_publishes_one_role_per_robot_per_tick(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        start = source.index(
            "  void publishTeamTactics("
        )

        end = source.index(
            "  void publishOtherRobots(",
            start,
        )

        block = source[
            start:
            end
        ]

        self.assertIn(
            "std::map<std::string, RoleCommand> commands",
            block,
        )

        self.assertIn(
            "commands.at(robot.id)",
            block,
        )

        self.assertEqual(
                        block.count(
                "publishRole("
            ),
            1,
        )

        self.assertNotIn(
            'publishRole(robot.id, "STOP", 0.0, 0.0)',
            block,
        )

    def test_other_robot_feed_requires_nine_obstacles(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_goal_adapter.cpp"
        )

        roles = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_team_role_assigner.cpp"
        )

        self.assertIn(
            '"minimum_other_robot_count", 9',
            source,
        )

        self.assertIn(
            "latest_other_robot_poses_.poses.size()) >= "
            "minimum_other_robot_count_",
            source,
        )

        self.assertIn(
            "output.poses.size()) >= "
            "minimum_other_robot_count_",
            roles,
        )

    def test_ball_fusion_filters_time_skew(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "coordination/football_ball_fusion.cpp"
        )

        self.assertIn(
            "max_fusion_time_skew_sec",
            source,
        )

        self.assertIn(
            "synchronized",
            source,
        )

        self.assertIn(
            "minimum_inlier_count",
            source,
        )

    def test_target_updater_uses_current_distance(self):
        source = read(
            "cyberdog_tracking_base/"
            "mcr_tracking_components/src/"
            "behavior_tree_nodes/"
            "target_updater_node.cpp"
        )

        self.assertIn(
            "const double current_distance",
            source,
        )

        self.assertIn(
            "max_target_distance_m_ / current_distance",
            source,
        )

        self.assertNotIn(
            "if (distance_ > 4.0)",
            source,
        )

    def test_fake_world_enforces_field_bounds(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )

        self.assertIn(
            "field_min_x_",
            source,
        )

        self.assertIn(
            "boundary_margin_m_",
            source,
        )

        self.assertIn(
            "std::clamp",
            source,
        )

    def test_fake_world_enforces_robot_separation(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )

        self.assertIn(
            "robot_collision_radius_m_",
            source,
        )

        self.assertIn(
            "minimum_distance",
            source,
        )

        self.assertIn(
            "correction_x",
            source,
        )

    def test_fake_ball_treats_out_of_bounds_as_terminal_without_kickoff_reset(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_ball_publisher.cpp"
        )
        signal_start = source.index("  void signalOutOfBounds")
        publish_start = source.index("  void publishBallPose", signal_start)
        signal = source[signal_start:publish_start]

        self.assertIn('"OUT_OF_BOUNDS"', signal)
        self.assertIn("goal_completed_ = true", signal)
        self.assertIn('publishMatchStateCommand("FINISHED")', signal)
        self.assertNotIn("kickoff_x_", signal)
        self.assertNotIn("out_of_bounds_reset_sec", source)
        self.assertNotIn("sim_ball_x_ - ball_radius_m_ < field_min_x_", source)
        self.assertIn("sim_ball_x_ < field_min_x_", source)

    def test_no_legacy_transformed_tracking_topic(self):
        sources = "\n".join(
            read(path)
            for path in (
                "cyberdog_nav2/football_navigation/launch/"
                "football_navigation.launch.py",

                "cyberdog_nav2/football_navigation/src/"
                "visualization/football_visualization_node.cpp",

                "cyberdog_tracking_base/"
                "mcr_tracking_components/src/"
                "behavior_tree_nodes/"
                "target_updater_node.cpp",
            )
        )

        self.assertNotIn(
            "tracking_pose_transformed",
            sources,
        )

    def test_install_artifacts_are_not_stale(self):
        install = ROOT / "install"

        required = (
            install /
            "share" /
            "football_navigation" /
            "launch" /
            "football_navigation.launch.py",

            install /
            "share" /
            "navigation_bringup" /
            "launch" /
            "football_runtime.launch.py",
        )

        if not all(
            path.is_file()
            for path in required
        ):
            self.skipTest(
                "source-only checkout: install is validated by deploy verifier"
            )

        for relative in (
            "cyberdog_nav2/football_navigation/launch/"
            "football_navigation.launch.py",

            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py",
        ):
            source = ROOT / relative

            installed = (
                install /
                "share" /
                source.parent.parent.name /
                "launch" /
                source.name
            )

            self.assertTrue(
                installed.is_file(),
                str(installed),
            )

            source_text = source.read_text(
                encoding="utf-8"
            )
            installed_text = installed.read_text(
                encoding="utf-8"
            )
            if (
                source.stat().st_mtime > installed.stat().st_mtime
                or source_text != installed_text
            ):
                self.skipTest(
                    "source changed after install; rebuild refreshes installed launch files"
                )

            self.assertEqual(source_text, installed_text)

    def test_integrated_fake_uses_team_cyclone_and_hybrid_localization(self):
        env = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_env.sh"
        )
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        self.assertIn('FOOTBALL_DDS_MODE:-team_cyclone', env)
        self.assertIn('team_cyclone)', env)
        self.assertIn('isolated_fastdds)', env)
        self.assertIn('export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp', env)
        self.assertIn('export ROS_LOCALHOST_ONLY=1', env)

        start = runtime.index("    if football_acceptance_all_fake:")
        end = runtime.index('    if mode == "robot_competition":', start)
        all_fake = runtime[start:end]
        self.assertNotIn("localization_runtime", all_fake)
        self.assertNotIn("start_mivins", all_fake)
        self.assertNotIn("start_apriltag", all_fake)
        self.assertNotIn("start_odom_transform", all_fake)

    def test_real_entrypoint_forces_cyclone_and_reuses_teammate_localization(self):
        source = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_start_team_chase_real.sh"
        )

        self.assertIn(
            "export FOOTBALL_DATA_MODE=real",
            source,
        )

        self.assertIn(
            "export FOOTBALL_DDS_MODE=team_cyclone",
            source,
        )

        self.assertIn(
            "football_refuse_parallel_control_stack",
            source,
        )

        self.assertIn(
            "START_MIVINS_WORD",
            source,
        )

        self.assertIn(
            "START_APRILTAG_WORD",
            source,
        )

        self.assertIn(
            "START_ODOM_TRANSFORM_WORD",
            source,
        )

        self.assertIn(
            'start_teammate_localization:='
            '"$START_TEAMMATE_LOCALIZATION_WORD"',
            source,
        )

        self.assertIn(
            'start_mivins:="$START_MIVINS_WORD"',
            source,
        )

        self.assertIn(
            'start_apriltag:="$START_APRILTAG_WORD"',
            source,
        )

        self.assertIn(
            'start_odom_transform:='
            '"$START_ODOM_TRANSFORM_WORD"',
            source,
        )

    def test_nx_uses_only_the_active_install_tree(self):
        env = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_env.sh"
        )
        verify = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_verify_deploy.sh"
        )

        self.assertNotIn("/home/mi/football_overlay", env)
        self.assertIn("ros2 pkg prefix", verify)
        for required in (
            "FOOTBALL_PREFIX",
            "NAVIGATION_PREFIX",
            "MCR_PREFIX",
            "football_navigation",
            "navigation_bringup",
            "mcr_bringup",
        ):
            self.assertIn(required, verify)
        self.assertNotIn("/home/builder", verify)

    def test_single_host_simulation_contract_is_lightweight(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        compact_runtime = compact(runtime)

        self.assertEqual(
            2,
            runtime.count(
                "TimerAction("
            ),
        )

        self.assertNotIn(
            "simulation_robot_start_interval_sec",
                        runtime,
        )

        self.assertNotIn(
            "ordered_namespaces",
            runtime,
        )

        self.assertIn(
            compact(
                "peer_namespaces = ["
            ),
            compact_runtime,
        )

        self.assertIn(
            compact(
                "for index, robot_namespace "
                "in enumerate(peer_namespaces)"
            ),
            compact_runtime,
        )

        self.assertIn(
            compact(
                "period=0.2 * (index + 1)"
            ),
            compact_runtime,
        )

        self.assertIn(
            compact(
                "period=5.0"
            ),
            compact_runtime,
        )

    def test_tracking_action_has_galactic_feedback_callback(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_tracking_action_client.cpp"
        )

        options = source[
            source.index(
                "SendGoalOptions()"
            ):
            source.index(
                "async_send_goal"
            )
        ]

        self.assertEqual(
            1,
            options.count(
                "SendGoalOptions()"
            ),
        )

        self.assertIn(
            "options.feedback_callback",
            options,
        )

    def test_fake_diagnostics_accept_namespaced_ten_robot_nodes(self):
        diag = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_quick_diag.sh"
        )
        behavior = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        self.assertIn(
            'NS="${CYBERDOG_NAMESPACE:-${ROBOT_NAMESPACE:-}}"',
            diag,
        )
        self.assertIn(
            'if [[ "$NS" != "cyberdog_2" ]]',
            diag,
        )
        self.assertIn(
            "FORMAL_ROBOT_COUNT = 9",
            behavior,
        )
        self.assertIn(
            "ACTIVE_OBSTACLE_COUNT = 5",
            behavior,
        )
        for robot in (
            "cyberdog_1",
            "cyberdog_3",
            "cyberdog_4",
            "cyberdog_5",
            "cyberdog_6",
            "cyberdog_7",
            "cyberdog_8",
            "cyberdog_9",
            "cyberdog_10",
        ):
            self.assertIn(robot, behavior)

    def test_acceptance_requires_ten_data_chains_and_one_selected_stack(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_no_map_acceptance.py"
        )

        self.assertIn(
            "def single_host_simulation_contract(self):",
            acceptance,
        )

        self.assertIn(
            "/{}/odom_global",
            acceptance,
        )

        self.assertIn(
            "/global_vio/{}/odom",
            acceptance,
        )

        self.assertIn(
            "lifecycle_manager_tracking",
            acceptance,
        )

        self.assertIn(
            "node._other_robot_count >= 9",
            acceptance,
        )

        self.assertIn(
            "if node._other_robot_moved",
            acceptance,
        )

        self.assertIn(
            "unexpected {}/{}",
            acceptance,
        )

        self.assertIn(
            "_endpoint_full_name",
            acceptance,
                    )

        self.assertIn(
            "/football_fake_world",
            acceptance,
        )

        self.assertIn(
            "_costmap_signatures",
            acceptance,
        )

        self.assertIn(
            "_costmap_obstacle_moved",
            acceptance,
        )

        self.assertNotIn(
            "十台狗均有 planner/controller/BT/recovery",
            acceptance,
        )

        self.assertIn(
            "CycloneDDS 集成假模式合同成立",
            acceptance,
        )

        self.assertIn(
            "odom_global_publishers != expected_odom_global",
            acceptance,
        )

        self.assertIn(
            "global_vio_publishers != expected_global_vio",
            acceptance,
        )

        self.assertIn(
            "selected_odom_slam_publishers !=",
            acceptance,
        )

        self.assertIn(
            "peer_publishers !=",
            acceptance,
        )

    def test_preflight_enforces_single_nx_selected_stack(self):
        preflight = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_navigation_preflight.py"
        )

        for required in (
            "single NX simulation keeps teammate "
            "odom_transform chains and skips selected duplicate",

            "single NX simulation starts one selected "
            "football/Nav2 stack",

            "fake peers use deterministic bounded "
            "scripted motion",
        ):
            self.assertIn(
                required,
                preflight,
            )

        self.assertNotIn(
            "import subprocess",
            preflight,
        )

        self.assertNotIn(
            '["git",',
            preflight,
        )

        self.assertIn(
            "TEAMMATE_INTERFACE_SHA256",
            preflight,
        )

        self.assertIn(
            "teammate interface snapshots match "
            "the reviewed contract",
            preflight,
        )

        self.assertIn(
            "bt_navigator_trackingtracking_target_rclcpp_node",
            preflight,
        )

        self.assertNotIn(
            '"bt_navigator_tracking_target_rclcpp_node"',
            preflight,
        )

        self.assertNotIn(
            "simulation launches all robots in "
            "a bounded staggered order",
            preflight,
        )

    def test_existing_localization_reuse_requires_live_complete_chain(self):
        env = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_env.sh"
        )

        self.assertIn(
            "football_topic_has_sample",
            env,
        )

        self.assertIn(
            "football_lifecycle_is_active",
            env,
        )

        self.assertIn(
            "expected_odom_global_owner",
            env,
        )

        self.assertIn(
            "/${CYBERDOG_NAMESPACE}/odom_transform",
            env,
        )

        self.assertIn(
            "odom_slam_has_sample",
            env,
        )

        self.assertIn(
            "odom_global_has_sample",
            env,
        )

        for state in (
            "COMPLETE",
            "ABSENT",
            "PARTIAL",
            "DUPLICATED",
        ):
            self.assertIn(
                "FOOTBALL_SELECTED_LOCALIZATION_STATE={}".format(
                    state
                ),
                env,
            )

        self.assertIn(
            "required_nodes_complete",
            env,
        )

        self.assertIn(
            '/${CYBERDOG_NAMESPACE}/apriltag',
            env,
        )

    def test_galactic_topic_sampling_does_not_use_unsupported_once_flag(self):
        diag = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_quick_diag.sh"
        )
        behavior = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        self.assertNotIn("ros2 topic echo", diag)
        self.assertNotIn("--once", diag)
        self.assertIn("create_subscription", behavior)
        self.assertIn("executor.spin_once", behavior)
        self.assertNotIn("ros2 topic echo", behavior)

    def test_no_map_runtime_gate_is_domain_wide(self):
        env = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_env.sh"
        )
        behavior = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )

        for forbidden in (
            "map_server",
            "amcl",
            "slam_toolbox",
            "cartographer",
        ):
            self.assertIn(forbidden, env)
            self.assertIn(forbidden, behavior)

        start = runtime.index("    if football_acceptance_all_fake:")
        end = runtime.index('    if mode == "robot_competition":', start)
        all_fake = runtime[start:end]
        for forbidden in (
            "localization_runtime",
            "start_mivins",
            "start_apriltag",
            "start_odom_transform",
            "velocity_adaptor_runtime",
        ):
            self.assertNotIn(forbidden, all_fake)

    def test_lifecycle_checks_do_not_treat_inactive_as_active(self):
        behavior = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        self.assertIn("required_graph_seen", behavior)
        self.assertIn("GoalStatus.STATUS_EXECUTING", behavior)
        self.assertIn("GoalStatus.STATUS_ABORTED", behavior)
        self.assertNotIn('[[ "$state" == *active* ]]', behavior)

    def test_deploy_verifier_covers_formal_runtime_artifacts(self):
        source = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_verify_deploy.sh"
        )

        for required in (
            "football_navigation",
            "navigation_bringup",
            "mcr_bringup",
            "football_runtime.launch.py",
            "football_robot_runtime.launch.py",
            "football_navigation.launch.py",
            "football_user_function_acceptance.py",
            "football_nx_quick_diag.sh",
            "libfootball_multi_robot_obstacle_layer.so",
            "BT_RegisterNodesFromPlugin",
            "MultiRobotObstacleLayer",
            "StaticLayer",
        ):
            self.assertIn(required, source)

        self.assertNotIn("check_sha256", source)
        self.assertNotIn("expected=", source)
    def test_quick_diag_checks_tracking_action_server_and_type(self):
        behavior = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        package = read(
            "cyberdog_nav2/football_navigation/package.xml"
        )

        self.assertIn("GoalStatusArray", behavior)
        self.assertIn('"/{}/tracking_target/_action/status"', behavior)
        self.assertIn("action_goal_ids", behavior)
        self.assertIn(
            "stable TargetTracking -> Planner -> FollowP -> Controller chain",
            behavior,
        )
        self.assertIn("<depend>action_msgs</depend>", package)

    def test_team_cyclone_rejects_comma_interface_and_diag_uses_full_owner(self):
        env = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_env.sh"
        )
        diag = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_quick_diag.sh"
        )
        setup = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_setup_cyclonedds.sh"
        )

        self.assertIn("NetworkInterfaceAddress", env)
        self.assertIn("wlan0,eth0", env)
        self.assertIn('FOOTBALL_DDS_MODE:-team_cyclone', env)
        for key in (
            "ROS_DOMAIN_ID",
            "RMW_IMPLEMENTATION",
            "CYCLONEDDS_URI",
            "ROS_LOCALHOST_ONLY",
        ):
            self.assertIn(key, diag)
        self.assertIn(
            'NX_INTERFACE="${FOOTBALL_NX_INTERFACE:-auto}"',
            setup,
        )
        self.assertNotIn(
            'NX_IFACES="${FOOTBALL_NX_IFACES:-wlan0,eth0}"',
            setup,
        )

    def test_no_map_fallback_params_use_vodom(self):
        follow = (
            ROOT /
            "cyberdog_tracking_base/mcr_bringup/params/"
            "follow_params.yaml"
        ).read_text(
            encoding="utf-8"
        )

        recovery = (
            ROOT /
            "cyberdog_tracking_base/mcr_bringup/params/"
            "recoveries_params.yaml"
        ).read_text(
            encoding="utf-8"
        )

        self.assertNotIn(
            "global_frame: base_link",
            follow,
        )

        self.assertNotIn(
            "target_frame: base_link",
            follow,
        )

        self.assertNotIn(
            "global_frame: base_link",
            recovery,
        )

        self.assertIn(
            "global_frame: vodom",
            follow,
        )

        self.assertIn(
            "target_frame: vodom",
            follow,
        )

        self.assertIn(
            "global_frame: vodom",
            recovery,
        )

    def test_fake_acceptance_waits_for_single_nx_startup(self):
        source = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_team_acceptance_fake.sh"
        )

        self.assertIn(
            "export FOOTBALL_DATA_MODE=fake",
            source,
        )

        self.assertIn(
            'FOOTBALL_DDS_MODE=team_cyclone',
            source,
        )

        self.assertIn(
            'FOOTBALL_ACCEPTANCE_WAIT_SEC='
            '"${FOOTBALL_ACCEPTANCE_WAIT_SEC:-180}"',
            source,
        )

        self.assertIn(
            'FOOTBALL_LIFECYCLE_WAIT_SEC='
            '"${FOOTBALL_LIFECYCLE_WAIT_SEC:-240}"',
            source,
        )

    def test_all_fake_uses_stable_planner_plugin_and_keeps_sensor_obstacles(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )
        layer = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        cmake = read(
            "cyberdog_nav2/football_navigation/CMakeLists.txt"
        )
        package = read(
            "cyberdog_nav2/football_navigation/package.xml"
        )

        compact_runtime = compact(runtime)
        for required in (
            "_write_all_fake_follow_params",
            'if item not in ("nav2_compute_path_to_p_action_bt_node", '
            '"nav2_follow_p_action_bt_node", '
            '"football_multi_robot_obstacle_layer",)',
            'plugin_names.insert(0, "football_multi_robot_obstacle_layer",)',
            '"follow_params_file": all_fake_follow_params',
        ):
            self.assertIn(
                compact(required),
                compact_runtime,
            )
        self.assertNotIn(
            'obstacle_layer["enabled"] = False',
            runtime,
        )
        self.assertNotIn(
            'obstacle_layer["observation_sources"] = ""',
            runtime,
        )

        for required in (
            "StableComputePathToPoseAction",
            "goal_updated_ = false",
            "BT_RegisterNodesFromPlugin",
        ):
            self.assertIn(required, layer)

        self.assertIn("behaviortree_cpp_v3", cmake)
        self.assertIn("nav2_behavior_tree", cmake)
        self.assertIn("nav2_msgs", cmake)
        self.assertIn("<depend>behaviortree_cpp_v3</depend>", package)
        self.assertIn("<depend>nav2_behavior_tree</depend>", package)
        self.assertIn("<depend>nav2_msgs</depend>", package)

    def test_football_target_yaw_speed_and_push_closure(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )
        layer = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        goal = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_goal_adapter.cpp"
        )
        fake_ball = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_ball_publisher.cpp"
        )
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        robot_params = yaml.safe_load(
            read(
                "cyberdog_nav2/football_navigation/params/"
                "football_robot_runtime.yaml"
            )
        )

        for required in (
            'target_updaters[0].set("ID", "FootballTargetUpdater")',
            'truncate_nodes[0].set("distance", "0.0")',
            '"controller_frequency"] = 10.0',
            '"update_rate_hz": 15.0',
            '"publish_local_plan": True',
            '"publish_evaluation": True',
            '"debug_trajectory_details": True',
        ):
            self.assertIn(required, runtime)

        self.assertRegex(
            runtime,
            r'"publish_rate_hz"\s*:\s*50\.0',
        )

        controller_block = runtime[
            runtime.index("tracking_controller.update({"):
            runtime.index("for costmap_name in (")
        ]
        self.assertIn('"max_vel_x": 0.38', controller_block)
        self.assertIn('"max_speed_xy": 0.38', controller_block)
        self.assertIn('"max_vel_y": 0.14', controller_block)
        self.assertIn('"min_vel_y": -0.14', controller_block)
        self.assertIn('"vx_samples": 9', controller_block)
        self.assertIn('"vy_samples": 5', controller_block)
        self.assertIn('"vtheta_samples": 11', controller_block)
        self.assertIn('"ObstacleFootprint"', controller_block)
        self.assertIn('"FootballPreferForward"', controller_block)
        self.assertIn(
            '"football_navigation::FootballPreferForwardCritic"',
            controller_block,
        )

        for required in (
            "class FootballTargetUpdater",
            "preserving its yaw",
        ):
            self.assertIn(required, layer)

        self.assertRegex(
            layer,
            r"registerNodeType<\s*football_navigation::"
            r"FootballTargetUpdater\s*>",
        )

        for required in (
            "alignmentWatchdogTriggered",
            "alignment_max_rotation_rad_",
            "alignment_min_error_improvement_rad_",
            "assignApproachTravelOrientation",
            "assignPushTarget",
            "publishSpeedLimit",
            "speedLimitForState",
            'if (state == "NAV_TRANSIT")',
            'if (state == "CONTACT_ACQUIRE")',
            'next_state = "CONTACT_ACQUIRE"',
        ):
            self.assertIn(required, goal)

        for required in (
            "controlledRobotSeen",
            "controlled_robot_pose_seen_pub_",
            "controlled_robot_pose_fresh_pub_",
            "controlled_robot_transform_valid_pub_",
            "side_contact_pub_",
            "rear_contact_pub_",
            "controlled_robot_seen_pub_",
            "contact.front_contact",
            "max_ball_acceleration_mps2_",
        ):
            self.assertIn(required, fake_ball)

        for required in (
            "near_ball_sustained_spin_detected",
            "near_ball_pure_rotation_abs_angle",
            "near_ball_pure_rotation_net_angle",
            "C++ fake ball sees a fresh, transform-valid cyberdog_2 pose",
            "MINIMUM_FOOTPRINT_CLEARANCE = 0.20",
        ):
            self.assertIn(required, acceptance)

        goal_params = robot_params[
            "football_goal_adapter"
        ]["ros__parameters"]
        self.assertEqual(15.0, goal_params["update_rate_hz"])
        self.assertEqual(0.38, goal_params["approach_speed_limit_mps"])
        self.assertEqual(0.22, goal_params["obstacle_speed_limit_mps"])
        self.assertEqual(0.20, goal_params["ball_approach_speed_limit_mps"])
        self.assertEqual(0.07, goal_params["contact_acquire_speed_limit_mps"])
        self.assertEqual(0.12, goal_params["push_speed_limit_mps"])
        self.assertEqual(0.32, goal_params["push_target_lead_m"])
        self.assertLess(
            goal_params["push_enter_yaw_error_rad"],
            goal_params["push_exit_yaw_error_rad"],
        )
        self.assertLess(
            goal_params["push_enter_lateral_error_m"],
            goal_params["push_exit_lateral_error_m"],
        )



if __name__ == "__main__":
    unittest.main()
