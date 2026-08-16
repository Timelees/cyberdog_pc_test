#!/usr/bin/env python3
"""Static closure tests for deterministic one-robot single-shot acceptance."""

import ast
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from types import MethodType, SimpleNamespace

import yaml


ROOT = Path(__file__).resolve().parents[3]


def read(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def compact(source):
    return " ".join(source.split())


def all_fake_branch():
    source = read(
        "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
    )
    start = source.index("    if football_acceptance_all_fake:")
    end = source.index("    if mode == \"robot_competition\":", start)
    return source[start:end]


def load_functions(relative, names, globals_map=None):
    tree = ast.parse(read(relative), filename=relative)
    selected = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    assert {node.name for node in selected} == set(names)
    namespace = dict(globals_map or {})
    exec(compile(ast.Module(body=selected, type_ignores=[]), relative, "exec"), namespace)
    return namespace


def class_method_node(relative, class_name, method_name):
    tree = ast.parse(read(relative), filename=relative)
    target_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node
        for node in target_class.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )


class FakeLaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class TestFootballAllFakeContract:
    def test_fake_odom_out_matches_real_motion_bridge_contract(self):
        fake = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )
        real = read("motion/motion_bridge/src/odom_out_publisher.cpp")
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert 'std::string("odom")' in real
        assert 'std::string("base_link_leg")' in real
        assert '"odom_out_frame_id", "odom"' in fake
        assert '"odom_out_child_frame_id", "base_link_leg"' in fake
        assert "latest_vodom_robot_pose" in acceptance
        assert 'msg.header.frame_id == "odom"' in acceptance
        assert 'msg.child_frame_id == "base_link_leg"' in acceptance

    def test_kick_entry_geometry_is_not_equivalent_to_reaching_approach_pose(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        namespace = load_functions(
            relative,
            {"normalize_angle", "kick_entry_geometry"},
            {"math": math},
        )
        geometry = namespace["kick_entry_geometry"]

        valid = geometry((3.6, 0.0, 0.0), (4.0, 0.0), (8.0, 0.0))
        side = geometry((4.0, 0.4, 0.0), (4.0, 0.0), (8.0, 0.0))
        reversed_yaw = geometry((3.6, 0.0, math.pi), (4.0, 0.0), (8.0, 0.0))

        assert all(
            math.isclose(actual, expected, abs_tol=1.0e-12)
            for actual, expected in zip(valid, (0.4, 0.0, 0.0, 0.4))
        )
        assert side[0] == 0.0
        assert side[1] == 0.4
        assert reversed_yaw[2] == math.pi
        assert math.isclose(reversed_yaw[3], -0.4, abs_tol=1.0e-12)

    def test_obstacle_approach_cannot_substitute_for_push_ready_states(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        namespace = load_functions(
            relative,
            {"ordered_football_states"},
            {},
        )
        ordered = namespace["ordered_football_states"]

        assert ordered([
            "OBSTACLE_APPROACH",
            "BALL_APPROACH",
            "ALIGN_TO_GOAL",
            "CONTACT_ACQUIRE",
            "PUSH_BALL",
        ])
        assert not ordered([
            "OBSTACLE_APPROACH",
            "ALIGN_TO_GOAL",
            "CONTACT_ACQUIRE",
            "PUSH_BALL",
        ])
        assert not ordered([
            "NAV_TRANSIT",
            "BALL_APPROACH",
            "ALIGN_TO_GOAL",
            "PUSH_BALL",
        ])

    def test_python_files_parse(self):
        for relative in (
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py",
            "cyberdog_nav2/football_navigation/launch/football_robot_runtime.launch.py",
            "cyberdog_nav2/football_navigation/launch/football_navigation.launch.py",
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py",
        ):
            ast.parse(read(relative), filename=relative)

    def test_acceptance_causal_chain_waits_for_dynamic_costmaps_before_action(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        method_node = class_method_node(
            relative,
            "SingleShotAcceptance",
            "_derive_first_runtime_break",
        )
        namespace = {}
        exec(
            compile(ast.Module(body=[method_node], type_ignores=[]), relative, "exec"),
            namespace,
        )
        acceptance = SimpleNamespace(
            namespace="cyberdog_2",
            global_odom_topic="/cyberdog_2/odom_global",
            event_times={
                "graph_ready": 1.0,
                "fake_global_odom": 2.0,
                "ball_pose": 3.0,
                "approach_pose": 4.0,
                "tracking_pose": 5.0,
            },
            runtime_violations=[],
        )
        acceptance._derive_first_runtime_break = MethodType(
            namespace["_derive_first_runtime_break"],
            acceptance,
        )

        assert (
            acceptance._derive_first_runtime_break()["condition"]
            == "dynamic_costmaps_ready"
        )

    def test_acceptance_runtime_violation_cannot_report_pass(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        report = class_method_node(
            relative,
            "SingleShotAcceptance",
            "_report_unlocked",
        )
        passed_assignment = next(
            node
            for node in ast.walk(report)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "passed"
                for target in node.targets
            )
        )
        expression = ast.Expression(body=passed_assignment.value)
        ast.fix_missing_locations(expression)
        acceptance = SimpleNamespace(
            hard_failure=False,
            runtime_violations=[{"condition": "planner_before_dynamic_costmaps"}],
        )

        assert not eval(
            compile(expression, relative, "eval"),
            {"all": all},
            {"checks": [True], "self": acceptance},
        )

    def test_all_fake_branch_is_single_robot_and_returns_before_production(self):
        branch = all_fake_branch()
        normalized = compact(branch)

        assert "single-shot acceptance controls only cyberdog_2" in branch
        assert '"robot_namespaces_csv": selected_namespace' in normalized
        assert '"simulate_selected_robot": True' in normalized
        assert '"scripted_peer_motion_enabled": False' in normalized
        assert '"continuous_demo_enabled": False' in normalized
        assert '"publish_acceptance_global_odom": True' in normalized
        assert '"publish_odom_slam": False' in normalized
        assert "football_fake_other_robot_publisher" in branch
        assert "football_fake_ball_publisher" in branch
        assert "pc_runtime" not in branch
        assert "localization_runtime" not in branch
        assert "state_publisher_runtime" not in branch
        assert "velocity_adaptor_runtime" not in branch
        assert "start_mivins" not in branch
        assert "start_apriltag" not in branch
        assert "start_odom_transform" not in branch
        assert "return [" in branch

    def test_all_fake_branch_has_only_the_tracking_lifecycle_manager(self):
        branch = all_fake_branch()
        tracking_launch = read(
            "cyberdog_tracking_base/mcr_bringup/launch/"
            "bringup_follow_only_launch.py"
        )

        assert "tracking_runtime" in branch
        assert "football_tracking_lifecycle_compat.sh" not in branch
        assert "ExecuteProcess(" not in branch
        assert "tracking_lifecycle_manager.py" in tracking_launch

    def test_deploy_scope_contains_only_user_rebuilt_packages(self):
        deploy = read("deploy_football_no_map_minimal.sh")
        package_block = deploy[
            deploy.index("readonly -a DEPLOY_PACKAGES=("):
            deploy.index(")\n\nfail()", deploy.index("readonly -a DEPLOY_PACKAGES=("))
        ]

        assert "nav2_smac_planner" in package_block
        assert "football_navigation" in package_block
        assert "navigation_bringup" in package_block
        for teammate_package in (
            "bt_navigators",
            "mcr_bringup",
            "mcr_controller",
            "mcr_planner",
            "mcr_tracking_components",
            "velocity_adaptor",
        ):
            assert teammate_package not in package_block
        assert '${#DEPLOY_PACKAGES[@]} -eq 3' in deploy
        assert "DEPLOY_3_RESULT=PASS" in deploy
        assert "HOST_DEPLOY_3_RESULT=PASS" in deploy

    def test_navigation_bringup_is_resource_only(self):
        cmake = read("cyberdog_nav2/navigation_bringup/CMakeLists.txt")

        assert "add_executable(test_node" not in cmake
        assert "find_package(rclcpp" not in cmake
        assert "install(TARGETS" not in cmake

    def test_lifecycle_compat_is_idempotent(self, tmp_path):
        nodes = (
            "controller_server_tracking",
            "planner_server_tracking",
            "recoveries_server",
            "bt_navigator_tracking",
        )
        states = tmp_path / "states.json"
        transitions = tmp_path / "transitions.log"
        fake_ros2 = tmp_path / "ros2"
        fake_ros2.write_text(
            """#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

states_path = Path(os.environ["FAKE_LIFECYCLE_STATES"])
log_path = Path(os.environ["FAKE_LIFECYCLE_LOG"])
states = json.loads(states_path.read_text(encoding="utf-8"))
fqn = sys.argv[3]
if sys.argv[2] == "get":
    print(states[fqn] + " [3]")
    raise SystemExit(0)
transition = sys.argv[4]
log_path.write_text(
    (log_path.read_text(encoding="utf-8") if log_path.exists() else "")
    + fqn + " " + transition + "\\n",
    encoding="utf-8",
)
states[fqn] = "inactive" if transition == "configure" else "active"
states_path.write_text(json.dumps(states), encoding="utf-8")
print("Transitioning successful")
""",
            encoding="utf-8",
        )
        fake_ros2.chmod(0o755)
        script = (
            ROOT
            / "cyberdog_nav2/football_navigation/scripts/"
            "football_tracking_lifecycle_compat.sh"
        )
        environment = dict(os.environ)
        environment.update({
            "PATH": "{}:{}".format(tmp_path, environment["PATH"]),
            "FAKE_LIFECYCLE_STATES": str(states),
            "FAKE_LIFECYCLE_LOG": str(transitions),
            "FOOTBALL_LIFECYCLE_COMPAT_DELAY_SEC": "0",
            "FOOTBALL_LIFECYCLE_COMPAT_MAX_ATTEMPTS": "4",
            "FOOTBALL_LIFECYCLE_COMPAT_RETRY_SEC": "0",
            "FOOTBALL_LIFECYCLE_COMPAT_CONFIRM_SEC": "0",
        })

        states.write_text(json.dumps({
            "/cyberdog_2/{}".format(node): "active"
            for node in nodes
        }), encoding="utf-8")
        completed = subprocess.run(
            [str(script), "cyberdog_2"],
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 0
        assert not transitions.exists()

        states.write_text(json.dumps({
            "/cyberdog_2/{}".format(node): "unconfigured"
            for node in nodes
        }), encoding="utf-8")
        completed = subprocess.run(
            [str(script), "cyberdog_2"],
            env=environment,
            check=False,
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 0
        assert all(
            state == "active"
            for state in json.loads(states.read_text(encoding="utf-8")).values()
        )
        assert len(transitions.read_text(encoding="utf-8").splitlines()) == 8

    def test_all_fake_tracking_rewrites_keep_no_map_contract(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        for token in (
            '"nav2_compute_path_to_p_action_bt_node",',
            '"football_multi_robot_obstacle_layer",',
            'plugin_names.insert(',
            '"goal_updater_topic": "tracking_pose"',
            '"enabled": True',
            '"enable_prediction": True',
        ):
            assert token in launch
        assert 'obstacle_layer["enabled"] = False' not in launch
        assert 'obstacle_layer["observation_sources"] = ""' not in launch

        params = yaml.safe_load(
            read("cyberdog_tracking_base/mcr_bringup/params/follow_params.yaml")
        )
        for name in ("local_costmap_tracking", "rolling_window_costmap"):
            values = params[name][name]["ros__parameters"]
            assert values["rolling_window"] is True
            assert values["global_frame"] == "vodom"
            assert values["robot_base_frame"] == "base_link"
            assert "multi_robot_obstacle_layer" in values["plugins"]
        assert "StaticLayer" not in read(
            "cyberdog_tracking_base/mcr_bringup/params/follow_params.yaml"
        )

    def test_formal_input_interfaces_are_published_by_one_acceptance_node(self):
        source = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        for topic in (
            '"/football/ball_pose"',
            '"/football/team_a/striker"',
            '"/football/team_a/kick_target"',
            '"/football/match_state"',
            '"/{}/football/role"',
            '"/{}/football/other_robot_poses"',
        ):
            assert topic in source

        for forbidden_publisher in (
            'create_publisher(\n            Path',
            'create_publisher(\n            Twist',
            'create_publisher(\n            Odometry',
            'create_publisher(\n            TFMessage',
        ):
            assert forbidden_publisher not in source

    def test_single_shot_scenario_has_five_active_and_four_far_robots(self):
        source = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "ACTIVE_OBSTACLE_COUNT = 5" in source
        assert "FORMAL_ROBOT_COUNT = 9" in source
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
            assert robot in source
        assert "cyberdog_2" not in source[
            source.index("active = ["):
            source.index("return active + far")
        ]

    def test_ball_physics_is_front_contact_driven_and_low_speed(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        fake_ball = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_ball_publisher.cpp"
        )
        geometry = read(
            "cyberdog_nav2/football_navigation/src/core/football_geometry.cpp"
        )

        for token in (
            "computeBallContactMetrics(",
            "contact.front_contact",
            "max_ball_acceleration_mps2_",
            "max_sim_ball_speed_mps_",
            "front_contact_pub_",
            "side_contact_pub_",
            "rear_contact_pub_",
            "contact_robot_pub_",
            "ball_velocity_pub_",
            "controlled_robot_pose_seen_pub_",
            "controlled_robot_pose_fresh_pub_",
            "controlled_robot_transform_valid_pub_",
            "controlled_robot_seen_pub_",
            "controlledRobotSeen(stamp)",
            "single_shot_enabled_",
        ):
            assert token in fake_ball or token in geometry

        for token in (
            '"/football/fake_ball/front_contact"',
            '"/football/fake_ball/side_contact"',
            '"/football/fake_ball/rear_contact"',
            '"/football/fake_ball/contact_robot"',
            '"/football/fake_ball/velocity"',
            '"/football/fake_ball/controlled_robot_pose_seen"',
            '"/football/fake_ball/controlled_robot_pose_fresh"',
            '"/football/fake_ball/controlled_robot_transform_valid"',
            '"/football/fake_ball/controlled_robot_seen"',
            "self.ball_motion_without_contact",
            "self.pre_push_ball_motion",
            "self.longest_front_contact_sec",
            "self.goal_crossed = True",
        ):
            assert token in acceptance

        assert "def _update_ball_physics" not in acceptance
        assert "ball_contact_metrics(" not in acceptance
        for forbidden in (
            "major_" + "kick",
            "DRIVE_THROUGH_" + "KICK_SPEED",
            "kick " + "impulse",
            "MAJOR_" + "KICK_COUNT",
        ):
            assert forbidden not in acceptance + fake_ball

    def test_single_shot_goal_adapter_allows_ball_center_to_reach_goal_line(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        assert '"ball_boundary_margin_m": 0.0' in launch

    def test_front_contact_push_does_not_fall_behind_the_robot(self):
        branch = all_fake_branch()
        contact_center_distance = 0.302 + 0.11
        push_target_offset = 0.302
        controller_goal_tolerance = 0.08

        assert '"velocity_transfer_gain": 1.0' in branch
        assert (
            contact_center_distance - push_target_offset
            > controller_goal_tolerance
        )

    def test_acceptance_proves_behavior_instead_of_topic_existence_only(self):
        source = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        for token in (
            "formal FollowP/Controller is the motion-command source",
            "fake world integrates the same formal Controller cmd_vel",
            "current, predicted and cleared robot costs in both rolling costmaps",
            "crossing/head-on/same-direction robots cause safe avoidance and recovery",
            "five-stage speed policy uses closing-speed/braking slowdown and low contact/push",
            "actual trajectory is smooth and remains close to Planner Path",
            "ALIGN converges without sustained same-direction spinning or a full turn",
            "C++ fake ball sees a fresh, transform-valid cyberdog_2 pose",
            "ordered NAV_TRANSIT/OBSTACLE_APPROACH -> BALL_APPROACH -> "
            "ALIGN -> CONTACT_ACQUIRE -> PUSH",
            "shared oriented-footprint geometry establishes legal controlled front contact",
            "continuous low-speed contact push keeps robot behind ball toward goal",
            "football center crosses the configured team_a goal line between posts",
            'print("RESULT: {}"',
        ):
            assert token in source

    def test_goal_adapter_and_tracking_action_keep_formal_contract(self):
        goal = read(
            "cyberdog_nav2/football_navigation/src/control/football_goal_adapter.cpp"
        )
        tracking = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_tracking_action_client.cpp"
        )
        trajectory = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_trajectory_adapter.cpp"
        )

        assert '"ball_pose_topic", "/football/ball_pose"' in goal
        assert '"other_robot_poses_topic", "football/other_robot_poses"' in goal
        assert "computeApproachPose(ball, kick_target" in goal
        assert 'next_state = "CONTACT_ACQUIRE"' in goal
        assert 'next_state = "PUSH_BALL"' in goal
        for state in (
            "NAV_TRANSIT",
            "OBSTACLE_APPROACH",
            "BALL_APPROACH",
            "ALIGN_TO_GOAL",
            "CONTACT_ACQUIRE",
            "PUSH_BALL",
        ):
            assert state in goal
        assert '"push_target_lead_m", 0.32' in goal
        assert '"push_contact_hard_timeout_sec", 6.00' in goal
        assert '"robot_front_extent_m", 0.302' in goal
        assert "have_push_contact_ ?" in goal
        assert (
            "robot_front_extent_m_ + ball_radius_m_ - "
            "push_goal_crossing_margin_m_"
            in goal
        )
        assert "-push_contact_offset_m" in goal
        assert "kick_target.pose.position.x - direction_x * target_offset_m" in goal
        assert "kick_target.pose.position.y - direction_y * target_offset_m" in goal
        assert "drive_through_committed_" in goal
        assert "drive_direction_x_" in goal
        assert "kick_norm >= minimum_kick_target_distance_m_" in goal
        assert (
            "push_contact_offset_m = std::min(\n"
            "    push_target_lead_m_, robot_front_extent_m_)" in goal
        )
        assert "push_corridor_x" in goal
        assert "push_corridor_y" in goal

        assert "(goal_active_ || goal_pending_) && !pose_stale" in tracking
        for token in (
            "last_completed_tracking_pose_",
            "have_completed_tracking_pose_",
            "completed_pose_xy_tolerance_",
            "completed_pose_yaw_tolerance_",
            "isPoseNear(",
            "force_retry_",
        ):
            assert token in tracking
        assert "last_completed_pose_stamp_" not in tracking
        assert "goal_timeout_sec" not in tracking

        assert "tracking_pose_heartbeat" in trajectory
        assert "planner_update_rate_hz" in trajectory

        params = yaml.safe_load(
            read(
                "cyberdog_nav2/football_navigation/params/"
                "football_robot_runtime.yaml"
            )
        )["football_tracking_action_client"]["ros__parameters"]
        assert params["completed_pose_xy_tolerance"] == 0.05
        assert params["completed_pose_yaw_tolerance"] == 0.08

    def test_generated_runtime_overrides_dwb_for_smooth_no_map_motion(self):
        helpers = load_functions(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py",
            {"_required_path", "_write_all_fake_follow_params"},
            {"tempfile": tempfile, "yaml": yaml},
        )
        generated = helpers["_write_all_fake_follow_params"](
            str(
                ROOT
                / "cyberdog_tracking_base/mcr_bringup/params/follow_params.yaml"
            ),
            "/tmp/football_target_tracking.xml",
        )
        try:
            data = yaml.safe_load(Path(generated).read_text(encoding="utf-8"))
        finally:
            Path(generated).unlink(missing_ok=True)

        controller_server = data["controller_server_tracking"]["ros__parameters"]
        controller = controller_server[
            "TrackingTarget"
        ]
        baseline = yaml.safe_load(
            read("cyberdog_tracking_base/mcr_bringup/params/follow_params.yaml")
        )["controller_server_tracking"]["ros__parameters"]["TrackingTarget"]
        assert controller_server["controller_frequency"] == 10.0
        tuned_keys = {
            "max_vel_x",
            "max_vel_y",
            "min_vel_y",
            "max_speed_xy",
            "max_vel_theta",
            "min_vel_theta",
            "vx_samples",
            "vy_samples",
            "vtheta_samples",
            "sim_time",
            "linear_granularity",
            "angular_granularity",
            "critics",
            "FootballPreferForward.class",
            "FootballPreferForward.penalty",
            "FootballPreferForward.forward_target",
            "FootballPreferForward.lateral_scale",
            "FootballPreferForward.theta_scale",
            "FootballPreferForward.wrong_turn_penalty",
        }
        for key, value in baseline.items():
            if key == "debug_trajectory_details" or key in tuned_keys:
                continue
            assert controller[key] == value
        assert controller["max_vel_x"] == 0.38
        assert controller["max_speed_xy"] == 0.38
        assert controller["max_vel_y"] == 0.14
        assert controller["min_vel_y"] == -0.14
        assert controller["vx_samples"] == 9
        assert controller["vy_samples"] == 5
        assert controller["vtheta_samples"] == 11
        assert controller["sim_time"] == 1.0
        assert "ObstacleFootprint" in controller["critics"]
        assert "FootballPreferForward" in controller["critics"]
        assert (
            controller["FootballPreferForward.class"]
            == "football_navigation::FootballPreferForwardCritic"
        )
        assert controller["publish_evaluation"] is True
        assert controller["debug_trajectory_details"] is True
        for name in ("local_costmap_tracking", "rolling_window_costmap"):
            layer = data[name][name]["ros__parameters"][
                "multi_robot_obstacle_layer"
            ]
            assert layer["other_robot_length_m"] == 0.603
            assert layer["other_robot_width_m"] == 0.339
            assert layer["footprint_padding"] == 0.17
            assert layer["prediction_horizon_sec"] >= 0.80
            assert layer["max_prediction_distance_m"] >= 0.32

    def test_robot_launch_preserves_all_fake_parameter_overrides(self):
        runtime_helpers = load_functions(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py",
            {"_required_path", "_write_all_fake_football_params"},
            {"tempfile": tempfile, "yaml": yaml},
        )
        generated = runtime_helpers["_write_all_fake_football_params"](
            str(
                ROOT
                / "cyberdog_nav2/football_navigation/params/"
                "football_robot_runtime.yaml"
            ),
            "/tmp/football_target_tracking.xml",
        )
        launch_helpers = load_functions(
            "cyberdog_nav2/football_navigation/launch/"
            "football_navigation.launch.py",
            {
                "as_bool",
                "infer_team",
                "node_params",
                "merge_geometry_params",
                "launch_nodes",
            },
            {
                "IfCondition": lambda value: value,
                "LaunchConfiguration": FakeLaunchConfiguration,
                "Node": lambda **kwargs: kwargs,
                "yaml": yaml,
            },
        )
        context = {
            "namespace": "cyberdog_2",
            "runtime_mode": "simulation_single_host",
            "enable_local_visualization": "false",
            "params_file": generated,
            "field_geometry_params_file": str(
                ROOT
                / "cyberdog_nav2/football_navigation/params/"
                "football_pc_authority.yaml"
            ),
            "unified_frame": "tag_global",
            "navigation_frame": "vodom",
            "target_frame": "base_link",
            "odom_global_topic": "/cyberdog_2/odom_global",
            "enable_trajectory_adapter": "true",
            "enable_tracking_action_client": "true",
        }
        try:
            nodes = launch_helpers["launch_nodes"](context)
        finally:
            Path(generated).unlink(missing_ok=True)

        parameters = {
            node["name"]: node["parameters"][0]
            for node in nodes
        }
        assert parameters["football_goal_adapter"]["ball_boundary_margin_m"] == 0.0
        assert (
            parameters["football_trajectory_adapter"]["planner_update_rate_hz"]
            == 2.0
        )
        assert (
            parameters["football_tracking_action_client"]["costmap_timeout_sec"]
            >= 3.0
        )

    def test_multi_robot_layer_writes_and_clears_master_costmap(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        plugin = read(
            "cyberdog_nav2/football_navigation/football_navigation_plugins.xml"
        )

        for token in (
            "updateBounds(",
            "updateCosts(",
            "resetMap(",
            "markFootprintObstacle(",
            "previous_obstacles_",
            "data_timeout_",
            "BT_RegisterNodesFromPlugin",
            "goal_updated_ = false",
        ):
            assert token in source
        assert "football_navigation/MultiRobotObstacleLayer" in plugin

    def test_tracking_costmap_gate_accepts_galactic_zero_stamp_grids(self):
        publisher = read(
            "cyberdog_nav2/navigation2/nav2_costmap_2d/src/"
            "costmap_2d_publisher.cpp"
        )
        tracking = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_tracking_action_client.cpp"
        )
        costmap_callback = tracking[
            tracking.index("void costmapCallback("):
            tracking.index("bool costmapsReadyLocked(")
        ]

        assert "grid_->header.stamp = rclcpp::Time();" in publisher
        assert "const rclcpp::Time receipt = now();" in costmap_callback
        assert "stamp.nanoseconds() <= 0" not in costmap_callback
        assert "costmap_readiness_epoch_" not in tracking

    def test_active_tracking_goal_survives_normal_costmap_clear_recovery(self):
        tracking = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_tracking_action_client.cpp"
        )
        maybe_send = tracking[
            tracking.index("void maybeSendGoal()"):
            tracking.index("void goalResponseCallback(")
        ]

        assert (
            maybe_send.index("(goal_active_ || goal_pending_)")
            < maybe_send.index("costmapsReadyLocked(now())")
        )
        assert 'cancelActiveGoal("dynamic rolling costmaps not ready")' not in (
            maybe_send[
                maybe_send.index("(goal_active_ || goal_pending_)"):
                maybe_send.index("costmapsReadyLocked(now())")
            ]
        )

    def test_acceptance_samples_tf_and_receive_time_obstacles_without_callback_races(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "self.tf_callback_group = MutuallyExclusiveCallbackGroup()" in acceptance
        tf_subscription = acceptance[
            acceptance.index(
                '"/{}/tf".format(self.namespace),'
            ):
            acceptance.index(
                "self.analysis_timer = self.create_timer("
            )
        ]
        assert "callback_group=self.tf_callback_group" in tf_subscription
        capture = ast.get_source_segment(
            acceptance,
            class_method_node(
                "cyberdog_nav2/football_navigation/scripts/"
                "football_user_function_acceptance.py",
                "SingleShotAcceptance",
                "_capture_deferred_event",
            ),
        )
        assert "fake_cycle = self.latest_fake_cycle" in capture
        assert "obstacles_at_receive=tuple(" in capture
        assert "obstacle_velocity_at_receive=tuple(" in capture

    def test_acceptance_snapshots_global_pose_history_before_iterating(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "self.global_pose_history_lock = threading.Lock()" in acceptance
        odom_callback = acceptance[
            acceptance.index("def _on_global_odom("):
            acceptance.index("def _on_local_odom(")
        ]
        pose_lookup = acceptance[
            acceptance.index("def _robot_pose_at("):
            acceptance.index("def _local_pose_to_world(")
        ]
        assert "with self.global_pose_history_lock:" in odom_callback
        assert "with self.global_pose_history_lock:" in pose_lookup
        assert "pose_history = tuple(self.global_pose_history)" in pose_lookup

    def test_step5_fake_input_timer_isolated_from_acceptance_analysis(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        acceptance = read(relative)
        publish = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_publish_contract_inputs"),
        )
        timer = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_on_timer"),
        )

        assert "self.input_thread = threading.Thread(" in acceptance
        assert "target=self._input_publish_loop" in acceptance
        assert "self.input_thread.start()" in acceptance
        assert "self.analysis_timer = self.create_timer(" in acceptance
        assert 'self._profiled_callback("acceptance_timer", self._on_timer)' in acceptance
        assert "self._publish_contract_inputs()" not in timer
        for heavy_token in (
            "_fail_now(",
            "oriented_rectangle(",
            "polygon_edge_clearance(",
            "_check_graph(",
        ):
            assert heavy_token not in publish

    def test_step5_fake_pose_and_stamp_use_one_odom_snapshot(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        acceptance = read(relative)
        odom = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_on_global_odom"),
        )
        publish = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_publish_contract_inputs"),
        )
        transform = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_world_pose_to_base"),
        )

        assert "class RobotStateSnapshot" in acceptance
        assert "self.robot_state_lock = threading.Lock()" in acceptance
        assert "with self.robot_state_lock:" in odom
        assert "self.robot_state_snapshot = RobotStateSnapshot(" in odom
        assert "robot_state = self._robot_state_snapshot()" in publish
        assert "obstacles.header.stamp = robot_state.stamp" in publish
        assert "target.header.stamp = robot_state.stamp" in publish
        assert "robot_state.stamp_ns - self.scenario_start_ros_ns" in publish
        assert "self._world_pose_to_base(" in publish
        assert "robot_state," in publish
        assert "self.latest_robot_pose" not in transform
        assert "robot_state.x" in transform
        assert "robot_state.y" in transform
        assert "robot_state.yaw" in transform
        assert 'obstacles.header.frame_id = self.BASE_FRAME' in publish
        assert "qos_profile_sensor_data" in acceptance

    def test_step5_stops_executor_before_iterating_report_state(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        main = ast.get_source_segment(
            acceptance,
            next(
                node
                for node in ast.parse(acceptance).body
                if isinstance(node, ast.FunctionDef) and node.name == "main"
            ),
        )

        assert main.index("executor.remove_node(node)") < main.index("node.report()")
        assert main.index("executor.shutdown()") < main.index("node.report()")
        assert main.index("gc.disable()") < main.index("node = SingleShotAcceptance(")
        assert main.index("gc.enable()") < main.index("node.report()")

    def test_step5_defers_heavy_path_and_costmap_acceptance_scans(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        acceptance = read(relative)
        plan = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_on_plan"),
        )
        local_plan = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_on_local_plan"),
        )
        costmap = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_on_costmap"),
        )
        report = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "report"),
        )

        assert "_analyse_path(" not in plan
        assert "_analyse_path(" not in local_plan
        assert "sum(" not in costmap
        assert "_cost_at_world(" not in costmap
        assert "_drain_deferred_costmap_analysis()" in report
        assert "_drain_deferred_path_analysis()" in report

    def test_step5_deferred_analysis_uses_receive_time_event_snapshots(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        acceptance = read(relative)

        for field in (
            "received_wall",
            "source_stamp",
            "goal_crossed_at_receive",
            "scenario_start_ros_ns_at_receive",
            "state_at_receive",
            "robot_pose_at_receive",
            "approach_pose_at_receive",
            "cmd_vel_at_receive",
            "obstacles_at_receive",
            "obstacle_velocity_at_receive",
            "elapsed_at_receive",
        ):
            assert field in acceptance

        plan = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_analyse_plan_message"),
        )
        local_plan = ast.get_source_segment(
            acceptance,
            class_method_node(
                relative,
                "SingleShotAcceptance",
                "_analyse_local_plan_message",
            ),
        )
        follow = ast.get_source_segment(
            acceptance,
            class_method_node(
                relative,
                "SingleShotAcceptance",
                "_analyse_follow_path_status",
            ),
        )
        path = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_analyse_path"),
        )
        costmap = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_analyse_costmap"),
        )

        assert "event.goal_crossed_at_receive" in plan
        assert "self.goal_crossed" not in plan
        assert "event.state_at_receive" in plan
        assert "event.cmd_vel_at_receive" in plan
        assert "time.monotonic()" not in local_plan
        assert "event.received_wall" in local_plan
        assert "event.goal_crossed_at_receive" in follow
        assert "self.goal_crossed" not in follow
        assert "event.scenario_start_ros_ns_at_receive" in follow
        for mutable_latest in (
            "self.latest_robot_pose",
            "self.latest_approach_world",
            "self.latest_obstacles_world",
        ):
            assert mutable_latest not in path
        assert "event.elapsed_at_receive" in costmap
        assert "self.elapsed()" not in costmap
        assert "self.latest_obstacles_world" not in costmap
        assert "self.obstacle_velocity_world" not in costmap
        for outcome in ("received", "queued", "drained", "analysed", "filtered"):
            assert outcome in acceptance

    def test_deferred_costmap_readiness_uses_later_historical_receive_time(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        acceptance_source = read(relative)
        analyse = class_method_node(
            relative,
            "SingleShotAcceptance",
            "_analyse_costmap",
        )
        namespace = {}
        exec(
            compile(ast.Module(body=[analyse], type_ignores=[]), relative, "exec"),
            namespace,
        )
        event_times = {}
        acceptance = SimpleNamespace(
            COST_HIGH=90,
            ACTIVE_OBSTACLE_COUNT=5,
            costmap_signatures={"local": set(), "rolling": set()},
            costmap_ready_wall={"local": None, "rolling": None},
            costmap_empty_after_ready_count={"local": 0, "rolling": 0},
            dynamic_cost_seen={"local": True, "rolling": True},
            predicted_dynamic_cost_seen={"local": False, "rolling": False},
            old_block_cleared={"local": True, "rolling": True},
            _record_event=lambda name, wall_time=None: event_times.setdefault(
                name,
                -1.0 if wall_time is None else wall_time,
            ),
        )
        acceptance._analyse_costmap = MethodType(
            namespace["_analyse_costmap"],
            acceptance,
        )
        msg = SimpleNamespace(
            info=SimpleNamespace(
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))
            ),
            data=[90],
        )

        acceptance._analyse_costmap(
            "local",
            msg,
            SimpleNamespace(
                obstacles_at_receive=(),
                obstacle_velocity_at_receive=(),
                received_wall=11.0,
                elapsed_at_receive=1.0,
            ),
        )
        assert "dynamic_costmaps_ready" not in event_times
        acceptance._analyse_costmap(
            "rolling",
            msg,
            SimpleNamespace(
                obstacles_at_receive=(),
                obstacle_velocity_at_receive=(),
                received_wall=13.0,
                elapsed_at_receive=3.0,
            ),
        )

        assert event_times["dynamic_costmaps_ready"] == 13.0
        snapshot = ast.get_source_segment(
            acceptance_source,
            next(
                node
                for node in ast.parse(acceptance_source).body
                if isinstance(node, ast.ClassDef)
                and node.name == "DeferredEventSnapshot"
            ),
        )
        capture = ast.get_source_segment(
            acceptance_source,
            class_method_node(
                relative,
                "SingleShotAcceptance",
                "_capture_deferred_event",
            ),
        )
        plan = ast.get_source_segment(
            acceptance_source,
            class_method_node(
                relative,
                "SingleShotAcceptance",
                "_analyse_plan_message",
            ),
        )
        assert "dynamic_costmaps_ready_at_receive" not in snapshot
        assert "dynamic_costmaps_ready_at_receive" not in capture
        assert "planner_before_dynamic_costmaps_seen" not in plan
        assert "Planner does not publish an executable Path before" not in plan

    def test_plan_cost_sampling_uses_published_occupancy_grid_semantics(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        acceptance = read(relative)
        path = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_analyse_path"),
        )

        assert "OCCUPANCY_INSCRIBED = 99" in acceptance
        assert "OCCUPANCY_LETHAL = 100" in acceptance
        assert "OCCUPANCY_UNKNOWN = -1" in acceptance
        assert "cost == self.OCCUPANCY_INSCRIBED" in path
        assert "cost == self.OCCUPANCY_LETHAL" in path
        assert "cost == self.OCCUPANCY_UNKNOWN" in path
        assert "cost >= 253" not in path
        assert "plan_inscribed_cost_seen" in acceptance
        assert "plan_unknown_cost_seen" in acceptance
        assert "PLAN_INSCRIBED_COST_SEEN=" in acceptance
        assert "PLAN_UNKNOWN_COST_SEEN=" in acceptance

    def test_plan_continuity_only_counts_follow_path_executing_intervals(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        source = read(relative)
        assert "def _finalize_plan_continuity(" in source
        method = class_method_node(
            relative,
            "SingleShotAcceptance",
            "_finalize_plan_continuity",
        )
        namespace = {}
        exec(
            compile(ast.Module(body=[method], type_ignores=[]), relative, "exec"),
            namespace,
        )

        def maximum_gap(transitions, plans, end):
            violations = []
            acceptance = SimpleNamespace(
                namespace="cyberdog_2",
                MAX_PLAN_INTERVAL_SEC=1.5,
                follow_path_execution_transitions=list(transitions),
                plan_times=list(plans),
                maximum_plan_interval=0.0,
                _record_runtime_violation=lambda *args, **kwargs: violations.append(
                    (args, kwargs)
                ),
            )
            MethodType(namespace["_finalize_plan_continuity"], acceptance)(end)
            return acceptance.maximum_plan_interval, violations

        before, before_failures = maximum_gap(
            [(10.0, True), (11.0, False)],
            [1.0, 10.2],
            20.0,
        )
        during, during_failures = maximum_gap(
            [(2.0, True), (5.0, False)],
            [2.0, 5.0],
            10.0,
        )
        terminal_tail, terminal_failures = maximum_gap(
            [(2.0, True), (3.0, False)],
            [2.5],
            10.0,
        )
        executing_tail, executing_tail_failures = maximum_gap(
            [(2.0, True)],
            [2.5],
            5.0,
        )
        shuffled, shuffled_failures = maximum_gap(
            [(5.0, False), (2.0, True)],
            [5.0, 2.0],
            10.0,
        )

        assert math.isclose(before, 0.8)
        assert not before_failures
        assert during == 3.0
        assert during_failures
        assert math.isclose(terminal_tail, 0.5)
        assert not terminal_failures
        assert math.isclose(executing_tail, 2.5)
        assert executing_tail_failures
        assert math.isclose(shuffled, during)
        assert bool(shuffled_failures) == bool(during_failures)

        drain = ast.get_source_segment(
            source,
            class_method_node(
                relative,
                "SingleShotAcceptance",
                "_drain_deferred_status_messages",
            ),
        )
        analyse_plan = ast.get_source_segment(
            source,
            class_method_node(
                relative,
                "SingleShotAcceptance",
                "_analyse_plan_message",
            ),
        )
        assert ".received_wall" in drain
        assert "sorted(" in drain
        assert "self.follow_path_executing_seen" not in analyse_plan

    def test_exact_occupancy_does_not_expand_plan_footprint_samples(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        source = read(relative)
        assert "def _occupancy_at_world(" in source
        assert "def _max_occupancy_near_world(" in source
        methods = [
            class_method_node(relative, "SingleShotAcceptance", name)
            for name in (
                "_grid_cell_at_world",
                "_occupancy_at_world",
                "_max_occupancy_near_world",
            )
        ]
        namespace = {"math": math, "yaw_from_quaternion": lambda _: 0.0}
        exec(
            compile(ast.Module(body=methods, type_ignores=[]), relative, "exec"),
            namespace,
        )
        acceptance = SimpleNamespace(
            _world_to_vodom=lambda x, y: (x, y),
        )
        for name in namespace:
            if name.startswith("_") and callable(namespace[name]):
                setattr(acceptance, name, MethodType(namespace[name], acceptance))
        data = [0] * 100
        data[13] = 100
        data[22] = 99
        data[23] = -1
        msg = SimpleNamespace(
            info=SimpleNamespace(
                resolution=0.1,
                width=10,
                height=10,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0),
                    orientation=SimpleNamespace(),
                ),
            ),
            data=data,
        )

        assert acceptance._occupancy_at_world(msg, 0.15, 0.15) == (True, 0)
        assert acceptance._max_occupancy_near_world(msg, 0.15, 0.15) == 100
        assert acceptance._occupancy_at_world(msg, 0.35, 0.15) == (True, 100)
        assert acceptance._occupancy_at_world(msg, 0.25, 0.25) == (True, 99)
        assert acceptance._occupancy_at_world(msg, 0.35, 0.25) == (True, -1)
        assert acceptance._occupancy_at_world(msg, -0.01, 0.15) == (False, None)

        path = ast.get_source_segment(
            source,
            class_method_node(relative, "SingleShotAcceptance", "_analyse_path"),
        )
        costmap = ast.get_source_segment(
            source,
            class_method_node(relative, "SingleShotAcceptance", "_analyse_costmap"),
        )
        clearance = ast.get_source_segment(
            source,
            class_method_node(
                relative,
                "SingleShotAcceptance",
                "_update_clearance_sample",
            ),
        )
        assert "_occupancy_at_world(" in path
        assert "_max_occupancy_near_world(" not in path
        assert "plan_cost_out_of_bounds_count" in path
        assert "_max_occupancy_near_world(" in costmap
        assert "_max_occupancy_near_world(" in clearance

    def test_fake_producer_tick_and_unique_source_freshness_are_separate(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        source = read(relative)
        assert "def classify_dynamic_source_stamp(" in source
        namespace = load_functions(
            relative,
            {"classify_dynamic_source_stamp"},
            {},
        )
        classify = namespace["classify_dynamic_source_stamp"]

        newest, outcome = classify(None, 1_000_000_000)
        assert (newest, outcome) == (1_000_000_000, "unique")
        newest, outcome = classify(newest, 1_000_000_000)
        assert (newest, outcome) == (1_000_000_000, "duplicate")
        assert (1_600_000_000 - newest) / 1.0e9 > 0.5
        newest, outcome = classify(newest, 900_000_000)
        assert (newest, outcome) == (1_000_000_000, "out_of_order")
        newest, outcome = classify(newest, 1_650_000_000)
        assert (newest, outcome) == (1_650_000_000, "unique")
        assert (1_700_000_000 - newest) / 1.0e9 < 0.1

        for label in (
            "FAKE_PRODUCER_TICK_HZ=",
            "FAKE_PRODUCER_TICK_P95_INTERVAL_SEC=",
            "FAKE_PRODUCER_TICK_P99_INTERVAL_SEC=",
            "FAKE_PRODUCER_TICK_MAX_GAP_SEC=",
            "DYNAMIC_UNIQUE_SOURCE_COUNT=",
            "DYNAMIC_DUPLICATE_STAMP_COUNT=",
            "DYNAMIC_UNIQUE_OUT_OF_ORDER_COUNT=",
            "DYNAMIC_MAX_SOURCE_STALENESS_SEC=",
        ):
            assert label in source
        assert "MAX_DYNAMIC_SOURCE_STALENESS_SEC = 0.50" in source
        assert "DYNAMIC_INPUT_HZ=" not in source

    def test_best_effort_odom_gap_is_reported_as_step5_observation(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "formal odometry source remains continuous" not in acceptance
        assert "Step5-observed odometry source-stamp continuity" in acceptance
        assert "source_stamp_interval={:.3f}s receipt_interval={:.3f}s" in acceptance
        assert "STEP5_OBSERVED_ODOM_SOURCE_STAMP_MAX_INTERVAL_SEC=" in acceptance
        assert "STEP5_OBSERVED_ODOM_RECEIPT_MAX_INTERVAL_SEC=" in acceptance

    def test_all_fake_local_visualizer_uses_transformable_global_odom(self):
        branch = all_fake_branch()
        visualizer = branch[
            branch.index('name="football_local_visualization_node"'):
        ]

        assert '"target_frame": "vodom"' in visualizer
        assert '"odom_topic": selected_fake_global_odom' in visualizer
        assert '"odom_topic": "odom_out"' not in visualizer

    def test_step5_graph_readiness_survives_transient_dds_discovery_omissions(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        acceptance = read(relative)
        graph = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_check_graph"),
        )

        assert "self.observed_required_nodes = set()" in acceptance
        assert "self.observed_required_nodes.update(required.intersection(names))" in graph
        assert "required.difference(self.observed_required_nodes)" in graph
        assert "counts[name] > 1" in graph

        start = ast.get_source_segment(
            acceptance,
            class_method_node(relative, "SingleShotAcceptance", "_start_scenario_if_ready"),
        )
        graph_gate = start[
            start.index("if not self.required_graph_seen:"):
            start.index("robot_state = self._robot_state_snapshot()")
        ]
        assert "return" not in graph_gate

    def test_dynamic_layer_keeps_valid_peers_when_one_pose_is_rejected(self):
        runtime = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )
        layer = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )

        assert '"minimum_obstacle_count": 1' in runtime
        callback = layer[
            layer.index("void MultiRobotObstacleLayer::poseArrayCallback("):
            layer.index("void MultiRobotObstacleLayer::appendPredictedSweep(")
        ]
        assert "accepted.size() < static_cast<std::size_t>(minimum_obstacle_count_)" in callback
        assert '"minimum_obstacle_count": 9' not in runtime

    def test_dynamic_layer_reports_input_acceptance_and_update_costs(self):
        layer = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )

        for token in (
            "input_count=%zu",
            "accepted_count=%zu",
            "rejected_invalid=%zu",
            "rejected_transform=%zu",
            "updateBounds_ms=%.3f",
            "updateCosts_ms=%.3f",
        ):
            assert token in layer

    def test_follow_path_lifecycle_uses_action_stamps_not_callback_arrival_noise(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        follow_callback = acceptance[
            acceptance.index("def _analyse_follow_path_status(self, msg, event):"):
            acceptance.index("def _on_tf(self, msg):")
        ]

        assert "self.follow_path_goal_stamp_ns = {}" in acceptance
        assert "previous_goal_stamp_ns = max(" in follow_callback
        assert "float(goal_stamp_ns - previous_goal_stamp_ns) / 1.0e9" in (
            follow_callback
        )
        assert "self.follow_path_status_by_goal[goal_id] = status.status" in (
            follow_callback
        )
        assert "latest_follow_stamp" in follow_callback
        assert "GoalStatus.STATUS_ABORTED" in follow_callback
        assert "GoalStatus.STATUS_UNKNOWN" in follow_callback

    def test_obstacle_approach_does_not_flip_between_lateral_targets(self):
        goal = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_goal_adapter.cpp"
        )

        assert (
            'blocked && have_published_approach_ &&\n'
            '    last_approach_state_ == "OBSTACLE_APPROACH"'
        ) in goal
        assert "latched_approach_distance > approach_reached_exit_m_" in goal
        assert "latched_approach_path_clear" in goal
        assert "latched_approach_endpoint_clear" in goal
        assert "plan.pose = last_published_approach_field_;" in goal

    def test_passed_obstacle_staging_target_is_not_latched_behind_robot(self):
        goal = read(
            "cyberdog_nav2/football_navigation/src/"
            "control/football_goal_adapter.cpp"
        )
        geometry = read(
            "cyberdog_nav2/football_navigation/src/"
            "core/football_geometry.cpp"
        )
        latch = goal[
            goal.index("const double latched_approach_distance"):
            goal.index("const double behind_error")
        ]

        assert "approachTargetPassed" in geometry
        assert "lateral_delta" in geometry
        assert "latched_approach_passed" in latch
        assert "!latched_approach_passed" in latch

    def test_zero_command_freeze_is_not_exempt_from_no_progress_watchdog(self):
        geometry = read(
            "cyberdog_nav2/football_navigation/src/"
            "core/football_geometry.cpp"
        )
        watchdog = geometry[
            geometry.index("bool isNoProgress("):
            geometry.index("geometry_msgs::msg::PoseStamped computeApproachPoseFromRobot")
        ]

        assert "metrics.command_speed >= config.min_command_speed" not in watchdog

    def test_progress_checker_does_not_treat_rotation_jitter_as_path_progress(self):
        plugin = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        progressed = plugin[
            plugin.index("bool progressed("):
            plugin.index("std::string plugin_name_", plugin.index("bool progressed("))
        ]

        assert "rotation >= rotation_radius_rad_" not in progressed
        assert "return translation >= movement_radius_m_;" in progressed
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )
        assert '"movement_time_allowance": 4.0' in launch

    def test_follow_path_update_has_time_gate_and_generated_bt_is_forwarded(self):
        plugin = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        branch = all_fake_branch()
        follow = plugin[
            plugin.index("class StableFollowPathAction"):
            plugin.index("class FootballTargetUpdater")
        ]

        assert "path_update_min_interval_sec" in follow
        assert "last_goal_update_time_" in follow
        assert '"default_target_tracking_bt_xml": football_tracking_bt' in branch

    def test_forward_critic_is_enabled_only_for_ahead_contact_and_push_goals(self):
        plugin = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        critic = plugin[
            plugin.index("class FootballPreferForwardCritic"):
            plugin.index("class FootballProgressChecker")
        ]

        assert "bool prepare(" in critic
        assert "goal_forward_m_" in critic
        assert "prefer_forward_ = goal_forward_m_ > 0.05;" in critic
        assert "if (!prefer_forward_)" in critic
        assert "goal_yaw_error_" in critic
        assert "wrong_turn_penalty_" in critic

    def test_dynamic_obstacle_stamp_is_rechecked_under_data_lock(self):
        plugin = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        callback = plugin[
            plugin.index("void MultiRobotObstacleLayer::poseArrayCallback("):
            plugin.index("void MultiRobotObstacleLayer::appendPredictedSweep(")
        ]
        lock = callback.index(
            "std::lock_guard<std::mutex> lock(data_mutex_);"
        )
        freshness = callback.index(
            "if (!stampAcceptable(stamp))",
            lock,
        )
        monotonic = callback.index(
            "stamp <= last_pose_array_stamp_",
            freshness,
        )
        update = callback.index("updateTracks(", monotonic)

        assert lock < freshness < monotonic < update
        assert "current_ = true;" not in callback

    def test_dynamic_obstacle_activate_reads_shared_state_under_data_lock(self):
        plugin = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        activate = plugin[
            plugin.index("void MultiRobotObstacleLayer::activate()"):
            plugin.index("void MultiRobotObstacleLayer::deactivate()")
        ]

        lock = activate.index(
            "std::lock_guard<std::mutex> lock(data_mutex_);"
        )
        read_state = activate.index("current_ = have_received_data_;")

        assert lock < read_state

    def test_acceptance_orders_runtime_breaks_by_wall_time_before_dependency(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        derive = class_method_node(
            relative,
            "SingleShotAcceptance",
            "_derive_first_runtime_break",
        )
        rank = class_method_node(
            relative,
            "SingleShotAcceptance",
            "_runtime_violation_rank",
        )
        namespace = {}
        exec(
            compile(
                ast.Module(body=[rank, derive], type_ignores=[]),
                relative,
                "exec",
            ),
            namespace,
        )
        acceptance = SimpleNamespace(
            namespace="cyberdog_2",
            global_odom_topic="/cyberdog_2/odom_global",
            event_times={
                "graph_ready": 1.0,
                "fake_global_odom": 2.0,
                "ball_pose": 3.0,
                "approach_pose": 4.0,
                "tracking_pose": 5.0,
                "dynamic_costmaps_ready": 6.0,
                "tracking_action_executing": 7.0,
                "planner_path": 8.0,
                "follow_path_executing": 9.0,
                "controller_cmd_nonzero": 10.0,
                "integrated_cmd_nonzero": 11.0,
                "odom_motion": 12.0,
                "tf_feedback": 13.0,
                "state_ALIGN_TO_GOAL": 14.0,
                "state_CONTACT_ACQUIRE": 15.0,
                "front_contact": 16.0,
                "state_PUSH_BALL": 17.0,
                "correct_goal_crossing": 18.0,
            },
            runtime_violations=[
                {
                    "time": 20.0,
                    "condition": "costmap freshness failed",
                    "actual": "late",
                    "phase": "PHASE_3_BEHAVIOR_ACCEPTANCE",
                    "broken_link": "costmap",
                },
                {
                    "time": 10.0,
                    "condition": "cmd_vel freshness failed",
                    "actual": "late",
                    "phase": "PHASE_3_BEHAVIOR_ACCEPTANCE",
                    "broken_link": "cmd_vel",
                },
            ],
        )
        rank_function = namespace["_runtime_violation_rank"]
        if isinstance(rank_function, staticmethod):
            rank_function = rank_function.__func__
        acceptance._runtime_violation_rank = rank_function
        acceptance._derive_first_runtime_break = MethodType(
            namespace["_derive_first_runtime_break"],
            acceptance,
        )

        assert (
            acceptance._derive_first_runtime_break()["broken_link"]
            == "cmd_vel"
        )

    def test_acceptance_correlates_headerless_cmd_by_ros_timestamp(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        callback = acceptance[
            acceptance.index("def _on_integrated_cmd_vel("):
            acceptance.index("def _finish_near_ball_rotation_segment(")
        ]
        assert "self.cmd_vel_stamp_history = deque(" in acceptance
        assert "self.integrated_cmd_stamp_history = deque(" in acceptance
        assert "def _reconcile_integrated_cmd_history(self):" in acceptance
        assert "recent_cmd_samples" not in callback
        assert "self.latest_cmd_vel.linear.x" not in callback

    def test_acceptance_cmd_correlation_accepts_cross_process_receipt_reordering(self):
        relative = (
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        method = class_method_node(
            relative,
            "SingleShotAcceptance",
            "_reconcile_integrated_cmd_history",
        )
        namespace = {}
        exec(
            compile(ast.Module(body=[method], type_ignores=[]), relative, "exec"),
            namespace,
        )
        acceptance = SimpleNamespace(
            scenario_start_ros_ns=0,
            cmd_vel_stamp_history=[
                (90_000_000, 0.09, (0.0, 0.0, 0.0)),
                (150_000_000, 0.15, (0.2, 0.0, 0.0)),
            ],
            integrated_cmd_stamp_history=[
                (100_000_000, 0.10, (0.2, 0.0, 0.0)),
            ],
            integrated_cmd_comparison_count=0,
            integrated_cmd_mismatch_count=0,
        )
        acceptance._reconcile_integrated_cmd_history = MethodType(
            namespace["_reconcile_integrated_cmd_history"],
            acceptance,
        )

        acceptance._reconcile_integrated_cmd_history()

        assert acceptance.integrated_cmd_comparison_count == 1
        assert acceptance.integrated_cmd_mismatch_count == 0

    def test_acceptance_input_timer_does_not_repeat_graph_discovery_after_start(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        method = acceptance[
            acceptance.index("def _start_scenario_if_ready(self):"):
            acceptance.index("def _on_timer(self):")
        ]

        assert (
            method.index("if self.scenario_start_wall is not None:")
            < method.index("self._check_graph()")
        )
        assert (
            "self.graph_timer = self.create_timer(\n"
            "            5.0,"
        ) in acceptance

    def test_all_fake_launch_refuses_existing_runtime_before_starting(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )
        all_fake = all_fake_branch()

        assert "import subprocess" in launch
        assert "def _refuse_existing_all_fake_runtime(namespace):" in launch
        assert '["ros2", "node", "list"]' in launch
        assert "_refuse_existing_all_fake_runtime(selected_namespace)" in all_fake
        assert all_fake.index(
            "_refuse_existing_all_fake_runtime(selected_namespace)"
        ) < all_fake.index(
            'Node(\n                package="football_navigation",\n'
            '                executable="football_fake_other_robot_publisher"'
        )

    def test_acceptance_graph_check_rejects_duplicate_required_nodes(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        method = acceptance[
            acceptance.index("def _check_graph(self):"):
            acceptance.index("def _update_trajectory_metrics(self, current):")
        ]

        assert "from collections import Counter, deque" in acceptance
        assert "def _node_full_name_list(self):" in acceptance
        assert "counts = Counter(name_list)" in method
        assert "self.duplicate_required_nodes = sorted(" in method
        assert "counts[name] > 1" in method
        assert "not self.duplicate_required_nodes" in method
        assert "DUPLICATE_REQUIRED_NODES={}" in acceptance

    def test_multi_robot_layer_logs_master_grid_write_evidence(self):
        source = read(
            "cyberdog_nav2/football_navigation/src/"
            "plugins/multi_robot_obstacle_layer.cpp"
        )
        header = read(
            "cyberdog_nav2/football_navigation/include/"
            "football_navigation/multi_robot_obstacle_layer.hpp"
        )
        obstacle = read(
            "cyberdog_nav2/football_navigation/include/"
            "football_navigation/dynamic_obstacle_marker.hpp"
        )

        assert "std::size_t source_index{0};" in obstacle
        assert "obstacle.source_index = index;" in source
        assert "void logMasterGridDiagnostics(" in header
        assert "++update_bounds_calls_;" in source
        assert "++update_costs_calls_;" in source
        assert "MultiRobotObstacleLayer master_grid" in source
        assert "center_cost=%u" in source
        assert "marked_cells=%u" in source

    def test_dwb_sampling_starts_with_scenario_and_publishes_critic_details(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )
        scenario = acceptance[
            acceptance.index("def _start_scenario_if_ready(self):"):
            acceptance.index("def _on_timer(self):")
        ]
        on_state = acceptance[
            acceptance.index("def _on_state(self, msg):"):
            acceptance.index("def _stop_dwb_evaluation_sampling(self):")
        ]

        assert "def _start_dwb_evaluation_sampling(self):" in acceptance
        assert "self._start_dwb_evaluation_sampling()" in scenario
        assert "def _start_dwb_evaluation_subscription(self):" in acceptance
        plan_callback = acceptance[
            acceptance.index("def _on_dwb_global_plan(self, msg):"):
            acceptance.index("def _on_dwb_evaluation(self, msg):")
        ]
        assert "target_forward < -0.05" in plan_callback
        assert "self._start_dwb_evaluation_subscription()" in plan_callback
        assert "create_subscription(" not in on_state
        assert (
            'if msg.data == "ALIGN_TO_GOAL":\n'
            "            self._start_dwb_evaluation_subscription()"
        ) in on_state
        assert '"publish_evaluation": True' in launch
        assert '"debug_trajectory_details": True' in launch

    def test_visual_status_separates_costmap_data_from_robot_matching(self):
        visualization = read(
            "cyberdog_nav2/football_navigation/src/"
            "visualization/football_visualization_node.cpp"
        )
        status = visualization[
            visualization.index("void appendStatusText("):
            visualization.index("void publishMarkers()")
        ]

        costmap_expression = status[
            status.index("const bool costmap_data_ok"):
            status.index("std::ostringstream ss;")
        ]
        assert "costmap_dynamic_robot_count" not in costmap_expression
        assert '"  robot_match="' in status

    def test_pose_labels_include_coordinates_and_separate_ball_from_approach(self):
        visualization = read(
            "cyberdog_nav2/football_navigation/src/"
            "visualization/football_visualization_node.cpp"
        )
        marker = visualization[
            visualization.index("void appendPoseMarker("):
            visualization.index("void appendGoalMarker(")
        ]

        assert 'ns == "approach_pose"' in marker
        assert "pose.pose.position.x" in marker
        assert "pose.pose.position.y" in marker

    def test_costmap_absence_cannot_satisfy_acceptance(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        costmap_check = acceptance[
            acceptance.index(
                "# 6. Dynamic rolling costmaps and behavior-level avoidance."
            ):
            acceptance.index("# 7.", acceptance.index(
                "# 6. Dynamic rolling costmaps and behavior-level avoidance."
            ))
        ]

        assert "all(self.dynamic_cost_seen.values())" in costmap_check
        assert "self.plan_cost_samples >= 5" in costmap_check
        assert "all(count >= 3 for count in self.costmap_message_count.values())" in costmap_check
        assert "all(count == 0 for count in self.costmap_invalid_count.values())" in costmap_check
        assert "all(count == 0 for count in self.costmap_empty_after_ready_count.values())" in costmap_check
        assert "all(gap <= self.MAX_COSTMAP_RECEIPT_GAP_SEC" in costmap_check

    def test_all_fake_planner_uses_costmap_checked_native_smoothing(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/"
            "football_runtime.launch.py"
        )
        package = read(
            "cyberdog_nav2/navigation_bringup/package.xml"
        )
        override = launch[
            launch.index("def _write_all_fake_follow_params("):
            launch.index("def _write_all_fake_football_params(")
        ]

        assert '"plugin": "nav2_smac_planner/SmacPlanner2D"' in override
        assert '"downsample_costmap": False' in override
        assert '"allow_unknown": False' in override
        assert '"smoother.w_data": 0.18' in override
        assert '"smoother.w_smooth": 0.42' in override
        assert "<exec_depend>nav2_smac_planner</exec_depend>" in package

    def test_smac_has_no_undeployed_openmp_runtime_dependency(self):
        cmake = read(
            "cyberdog_nav2/navigation2/nav2_smac_planner/CMakeLists.txt"
        )

        assert "OpenMP" not in cmake

    def test_acceptance_quantifies_global_path_corner_geometry(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "MAX_PLAN_TURN_ANGLE_RAD" in acceptance
        assert "maximum_plan_turn_angle_rad" in acceptance
        assert "MAX_PLAN_TURN_ANGLE_RAD=" in acceptance
        assert "MAX_PLAN_TURN_POINTS=" in acceptance

    def test_acceptance_uses_timestamped_costmap_for_plan_scoring(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert 'self.costmap_history = {' in acceptance
        assert "nearest_costmap" in acceptance
        assert "received_wall" in acceptance[
            acceptance.index("def _nearest_costmap("):
            acceptance.index("def _analyse_path(")
        ]
        assert "self.latest_costmaps.get(\"rolling\")" not in acceptance[
            acceptance.index("def _analyse_path("):
            acceptance.index("def _on_plan(")
        ]

    def test_fake_peer_contract_runs_at_the_real_role_assigner_rate(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "INPUT_PERIOD_SEC = 0.05" in acceptance
        assert (
            "self.input_stop_event.wait(wait_sec)" in acceptance
        )
        assert "producer_tick_hz >= 18.0" in acceptance
        assert "self.dynamic_max_source_staleness_sec" in acceptance

    def test_acceptance_treats_only_latest_follow_goal_abort_as_bad(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        callback = acceptance[
            acceptance.index("def _analyse_follow_path_status("):
            acceptance.index("def _on_tf(")
        ]

        assert "self.follow_path_status_by_goal" in callback
        assert "latest_follow_stamp" in callback
        assert (
            "FollowPath remains accepted or executing until a valid transition"
            not in callback
        )

    def test_acceptance_serializes_cross_callback_state_snapshots(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "self.snapshot_lock = threading.RLock()" in acceptance
        assert "def _locked_callback(" in acceptance
        assert "with self.snapshot_lock:" in acceptance
        assert "def _report_unlocked(" in acceptance
        assert "commands = sorted(" in acceptance
        assert "integrated_samples = sorted(" in acceptance

    def test_acceptance_keeps_high_rate_motion_callbacks_off_snapshot_lock(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "locked=True" in acceptance
        assert "selected_callback = (\n            self._locked_callback(callback)" in acceptance
        for callback in (
            "self._on_global_odom",
            "self._on_local_odom",
            "self._on_cmd_vel",
            "self._on_integrated_cmd_vel",
            "self._on_tf",
        ):
            subscription = acceptance[
                acceptance.index(callback):
                acceptance.index(
                    "        )",
                    acceptance.index(callback),
                )
            ]
            assert "locked=False" in subscription

    def test_acceptance_transforms_base_pose_at_message_stamp(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "self.global_pose_history = deque(" in acceptance
        assert "def _robot_pose_at(" in acceptance
        assert "_local_pose_to_world(msg.pose, stamp_ns)" in acceptance

    def test_acceptance_separates_source_and_receipt_continuity(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "maximum_odom_source_interval" in acceptance
        assert "maximum_odom_receipt_interval" in acceptance
        assert "maximum_tf_source_interval" in acceptance
        assert "maximum_tf_receipt_interval" in acceptance

    def test_acceptance_reports_nominal_and_receipt_cmd_acceleration(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "CMD_NOMINAL_PERIOD_SEC = 0.10" in acceptance
        assert "max_linear_cmd_receipt_acceleration" in acceptance
        assert "max_angular_cmd_receipt_acceleration" in acceptance

    def test_acceptance_collects_thirty_behind_target_dwb_critic_samples(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "DWB_CRITIC_SAMPLE_TARGET = 30" in acceptance
        assert "dwb_behind_near_zero_evaluation_count" in acceptance
        assert "DWB_BEHIND_CRITIC_MOVING_MINUS_SELECTED" in acceptance
        assert "dwb_behind_all_critic_bias" in acceptance
        assert "DWB_BEHIND_ALL_CRITIC_MOVING_MINUS_SELECTED" in acceptance
        assert "dwb_align_all_critic_bias" in acceptance
        assert "DWB_ALIGN_ALL_CRITIC_CORRECT_MINUS_SELECTED" in acceptance
        normalized_acceptance = " ".join(acceptance.split())
        assert (
            "sample_behind = (" in acceptance
            and "self.dwb_evaluation_sample_count < self.DWB_CRITIC_SAMPLE_TARGET"
            in normalized_acceptance
        )
        behind_counter = acceptance.split(
            "selected = msg.twists[msg.best_index]", 1
        )[1].split("selected_speed =", 1)[0]
        assert "_stop_dwb_evaluation_sampling" not in behind_counter

    def test_build_identity_records_source_hashes_and_acceptance_verifies_it(self):
        cmake = read("cyberdog_nav2/football_navigation/CMakeLists.txt")
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "football_build_identity.json" in cmake
        assert "file(SHA256" in cmake
        assert "CMAKE_CONFIGURE_DEPENDS" in cmake
        assert "smac_planner_2d_sha256" in cmake
        identity_sources = cmake.split("set(football_identity_sources", 1)[1].split(
            "set_property(DIRECTORY", 1
        )[0]
        for runtime_input in (
            "src/simulation/football_fake_ball_publisher.cpp",
            "src/simulation/football_fake_other_robot_publisher.cpp",
            "src/coordination/football_ball_fusion.cpp",
            "src/control/football_trajectory_adapter.cpp",
            "src/control/football_tracking_action_client.cpp",
            "src/coordination/football_team_role_assigner.cpp",
            "launch/football_navigation.launch.py",
            "launch/football_robot_runtime.launch.py",
            "params/football_robot_runtime.yaml",
            "football_navigation_plugins.xml",
            "football_navigation_dwb_plugins.xml",
        ):
            assert runtime_input in identity_sources
        assert "BUILD_SOURCE_DIGEST=" in acceptance
        assert "BUILD_IDENTITY_VALID=" in acceptance
        assert "installed_smac_planner_2d_sha256" in acceptance

    def test_acceptance_enforces_follow_goal_count_and_controller_deadlines(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "MIN_FOLLOW_GOAL_INTERVAL_SEC = 0.45" in acceptance
        assert "MAX_FOLLOW_GOAL_COUNT = 49" in acceptance
        assert "self.controller_cycle_timeout_count" in acceptance
        assert "Control loop missed its desired rate" in acceptance

    def test_acceptance_quantifies_ball_approach_to_push_efficiency(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "MAX_BALL_APPROACH_TO_PUSH_SEC" in acceptance
        assert "BALL_APPROACH_TO_PUSH_SEC=" in acceptance

    def test_acceptance_prints_cmd_series_and_ball_coordinate_sequence(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "self.cmd_vel_samples = deque(" in acceptance
        assert "CMD_VEL_SERIES=" in acceptance
        assert "BALL_TRAJECTORY=" in acceptance

    def test_acceptance_consumes_cpp_ball_and_proves_integrated_command_source(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        fake_world = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_other_robot_publisher.cpp"
        )
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        assert "self.ball_pub" not in acceptance
        for token in (
            '"/football/ball_pose"',
            '"/{}/cmd_vel"',
            '"/{}/football/fake_integrated_cmd_vel"',
            '"/{}/follow_p/_action/status"',
            "local_plan_contract_available",
            "global_odom_twist_by_stamp",
            "integrated_cmd_by_stamp",
            "_compare_odom_and_integrated",
            "odom_integrated_comparison_count",
            "fake world integrates the same formal Controller cmd_vel",
        ):
            assert token in acceptance

        for token in (
            "publish_integrated_cmd_vel_",
            "integrated_cmd_vel_topic_template_",
            "robot.local_vx",
            "robot.local_vy",
            "robot.wz",
        ):
            assert token in fake_world

        assert '"publish_integrated_cmd_vel":True' in ''.join(launch.split())
        assert '"single_shot_enabled":True' in ''.join(launch.split())

    def test_near_ball_control_uses_latches_hysteresis_and_anti_spin_critics(self):
        goal = read(
            "cyberdog_nav2/football_navigation/src/control/football_goal_adapter.cpp"
        )
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        params = read(
            "cyberdog_nav2/football_navigation/params/"
            "football_robot_runtime.yaml"
        )
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        for token in (
            "alignment_position_latched_",
            "approach_reached_exit_m_",
            "alignment_reset_grace_sec_",
            "alignment_ready",
            "assignApproachTravelOrientation",
            "resetPushCommitment",
            "push_enter_lateral_error_m_",
            "push_exit_lateral_error_m_",
            "push_enter_yaw_error_rad_",
            "push_exit_yaw_error_rad_",
        ):
            assert token in goal

        assert (
            "behind_error <= approach_reached_m_ &&\n"
            "      centered_on_kick_line" in goal
        )
        assert (
            "behind_error > approach_reached_exit_m_ ||\n"
            "      !centered_on_kick_line" in goal
        )
        assert (
            "!drive_through_committed_ && alignment_position_ready &&\n"
            "    yaw_error > strict_yaw_tolerance" in goal
        )
        assert "const bool alignment_position_ready =" in goal
        position_ready = goal[
            goal.index("const bool alignment_position_ready ="):
            goal.index("const bool strict_kick_alignment =")
        ]
        strict_alignment = goal[
            goal.index("const bool strict_kick_alignment ="):
            goal.index("bool alignment_reacquire =")
        ]
        assert "ball_forward_projection" not in position_ready
        assert "ball_forward_projection > 0.0" in strict_alignment
        assert "else if (at_behind_pose && !alignment_position_ready)" in goal
        geometry_reacquire = goal.index(
            "else if (at_behind_pose && !alignment_position_ready)"
        )
        frozen_align = goal.index("else if (!alignment_ready)")
        assert geometry_reacquire < frozen_align
        assert (
            "alignment_position_latched_ = false;"
            in goal[geometry_reacquire:frozen_align]
        )
        assert "approach = behind;" in goal[geometry_reacquire:frozen_align]
        assert (
            "std::hypot(robot_front_extent_m_, robot_half_width_m_) +\n"
            "      ball_radius_m_ + contact_tolerance_m_" in goal
        )
        assert "plan = ApproachPlan{behind, true, false};" not in goal
        assert (
            "approach = have_alignment_anchor_ ? alignment_anchor_field_ : robot;"
            in goal
        )
        assert "alignment_heading_probe_m" not in goal
        assert "alignment_heading_probe_m" not in params
        assert "plan_single_pose_count" in acceptance
        assert "MOTION_INTENT_STATES" in acceptance
        assert "motion state does not remain a zero-motion one-pose Path" in acceptance
        transform = goal.index(
            "if (!transformFromFieldToOutput(approach, local))"
        )
        publish_distance = goal.index(
            "const double local_distance",
            transform,
        )
        assert "local.pose.position.x =" not in goal[transform:publish_distance]
        assert "local.pose.position.y =" not in goal[transform:publish_distance]

        for token in (
            '"controller_frequency"] = 10.0',
            '"publish_local_plan": True',
            '"publish_evaluation": True',
            '"debug_trajectory_details": True',
            '"FootballTargetUpdater"',
        ):
            assert token in launch

        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )
        for token in (
            "near_ball_sustained_spin_detected",
            "near_ball_pure_rotation_abs_angle",
            "near_ball_pure_rotation_net_angle",
            "near_ball_pure_rotation_start_error",
            "near_ball_pure_rotation_best_error",
            "near_ball_pure_rotation_start_distance",
            "near_ball_pure_rotation_best_distance",
            "near_ball_full_turn_detected",
            "near_ball_wrong_rotation_direction_samples",
            "max_path_cross_track_error",
            "maximum_robot_curvature",
            "max_linear_cmd_acceleration",
        ):
            assert token in acceptance

    def test_goal_adapter_leaves_transit_obstacle_routing_to_loaded_costmaps(self):
        geometry = read(
            "cyberdog_nav2/football_navigation/src/core/football_geometry.cpp"
        )
        branch_start = geometry.index("const auto endpoint_clear =")
        direct_branch = geometry[
            branch_start:
            geometry.index("const double dx = kick_target_pose", branch_start)
        ]

        assert "endpoint_clear(behind_x, behind_y)" in direct_branch
        assert "distancePointToSegment(" in direct_branch
        assert "isOpponentBlockingPathToPoint(" not in direct_branch

    def test_progress_checker_allows_slow_motion_without_masking_a_stall(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        assert '"required_movement_radius": 0.02' in launch
        assert '"movement_time_allowance": 4.0' in launch

    def test_goal_checker_uses_football_tolerance_without_dwb_tuning(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        assert '"xy_goal_tolerance": 0.08' in launch
        controller_override = launch[
            launch.index("tracking_controller.update({"):
            launch.index("planner = _required_path(")
        ]
        assert '"xy_goal_tolerance"' not in controller_override

    def test_follow_path_updates_are_bounded_for_the_controller_action_server(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        assert 'rate_nodes[0].set("hz", "2.0")' in launch
        assert 'compute_nodes[0].set("server_timeout", "360000")' in launch
        assert 'follow_nodes[0].set("server_timeout", "360000")' in launch

    def test_all_fake_uses_formal_ball_fusion_boundary(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert '"ball_topic": "/football/ball_pose_raw"' in launch
        assert 'executable="football_ball_fusion"' in launch
        assert '"/football_ball_fusion"' in acceptance

    def test_acceptance_covers_target_stability_and_synchronized_feedback(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        for token in (
            "action_status_lock",
            "maximum_tracking_target_jump_m",
            "MAXIMUM_TRACKING_TARGET_JUMP_M",
            "minimum_plan_footprint_clearance_m",
            "MINIMUM_PLAN_FOOTPRINT_CLEARANCE_M",
            "MINIMUM_ODOM_HZ",
            "MINIMUM_TF_HZ",
            "MINIMUM_PLAN_HZ",
            "controlled_front_contact_seen",
        ):
            assert token in acceptance
        assert "obstacles.header.stamp = robot_state.stamp" in acceptance
        assert "obstacles.header.stamp = self.latest_global_odom.header.stamp" not in acceptance

    def test_costmap_sampling_covers_the_oriented_footprint_not_only_path_center(self):
        geometry = load_functions(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py",
            (
                "_orientation",
                "_point_in_convex_polygon",
                "polygon_grid_samples",
            ),
            {"math": math},
        )
        samples = geometry["polygon_grid_samples"](
            [(-0.3, -0.2), (0.3, -0.2), (0.3, 0.2), (-0.3, 0.2)],
            0.1,
        )

        assert (0.0, 0.0) in samples
        assert len(samples) >= 24
        assert all(-0.3 <= x <= 0.3 and -0.2 <= y <= 0.2 for x, y in samples)

    def test_alignment_watchdog_allows_controller_yaw_convergence(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )

        assert '"alignment_progress_window_sec": 3.0' in launch

    def test_acceptance_uses_parallel_callbacks_and_one_obstacle_clock(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert "from rclpy.executors import MultiThreadedExecutor" in acceptance
        assert "executor = MultiThreadedExecutor(num_threads=4)" in acceptance
        assert "robot_state.stamp_ns - self.scenario_start_ros_ns" in acceptance
        assert "latest_obstacles_world = self._obstacle_positions(" in acceptance
        assert "self.latest_obstacles_world = latest_obstacles_world" in acceptance
        assert (
            "self._on_global_odom,\n"
            "            qos_profile_sensor_data,\n"
            "            callback_group=self.global_odom_callback_group,"
        ) in acceptance
        for group in (
            "global_odom_callback_group",
            "local_odom_callback_group",
            "cmd_callback_group",
            "integrated_cmd_callback_group",
        ):
            assert "self.{} = MutuallyExclusiveCallbackGroup()".format(group) in acceptance
        assert "self.motion_history_lock = threading.Lock()" in acceptance

    def test_wrapper_waits_for_active_tracking_lifecycle_before_timing_motion(self):
        wrapper = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.sh"
        )

        assert 'TRACKING_NODE="/$NAMESPACE/bt_navigator_tracking"' in wrapper
        assert 'ros2 lifecycle get "$TRACKING_NODE"' in wrapper
        assert "active \\[3\\]" in wrapper
        assert "Managed nodes are active" in wrapper
        assert "tracking lifecycle did not become active before acceptance" in wrapper

    def test_same_direction_obstacle_clears_the_push_corridor_after_interaction(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert (
            "same_direction_x = move_between(\n"
            "                3.55,\n"
            "                6.50,\n"
            "                112.0,\n"
            "                142.0,\n"
            "            )"
        ) in acceptance
        assert (
            "same_direction_y = move_between(\n"
            "            0.75,\n"
            "            2.20,\n"
            "            112.0,\n"
            "            142.0,\n"
            "        )"
        ) in acceptance
        assert "same_direction_x += 0.15 * math.sin(" in acceptance
        assert "same_direction_y += 0.08 * math.sin(" in acceptance

    def test_dynamic_peers_start_with_open_transit_then_cross_and_keep_yielding(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        assert (
            "crossing_y = move_between(\n"
            "            1.55,\n"
            "            -2.30,\n"
            "            2.0,\n"
            "            22.0,"
        ) in acceptance
        assert '"cyberdog_1",\n                2.50,' in acceptance
        assert (
            "incoming_x = move_between(\n"
            "            5.20,\n"
            "            1.65,\n"
            "            24.0,\n"
            "            62.0,"
        ) in acceptance
        assert "crossing_y += 0.08 * math.sin(" in acceptance
        assert "incoming_x += 0.10 * math.sin(" in acceptance
        assert (
            "incoming_y = move_between(\n"
            "            -0.35,\n"
            "            -1.90,\n"
            "            24.0,\n"
            "            62.0,"
        ) in acceptance
        assert "incoming_y += 0.08 * math.sin(" in acceptance

    def test_five_stage_speed_and_explicit_contact_acquire_contract(self):
        goal = read(
            "cyberdog_nav2/football_navigation/src/control/football_goal_adapter.cpp"
        )
        fake_ball = read(
            "cyberdog_nav2/football_navigation/src/simulation/football_fake_ball_publisher.cpp"
        )
        params = yaml.safe_load(
            read(
                "cyberdog_nav2/football_navigation/params/"
                "football_robot_runtime.yaml"
            )
        )["football_goal_adapter"]["ros__parameters"]

        for token in (
            'next_state = "NAV_TRANSIT"',
            'next_state = "OBSTACLE_APPROACH"',
            'next_state = "BALL_APPROACH"',
            'next_state = "ALIGN_TO_GOAL"',
            'next_state = "CONTACT_ACQUIRE"',
            'next_state = "PUSH_BALL"',
            "speedLimitForState",
            "contact_acquire_started_",
            "front_contact_since_",
            "push_contact_stable_hold_sec_",
        ):
            assert token in goal

        assert 'control_state_ != "CONTACT_ACQUIRE"' in fake_ball
        assert params["approach_speed_limit_mps"] == 0.38
        assert params["obstacle_speed_limit_mps"] == 0.22
        assert params["ball_approach_speed_limit_mps"] == 0.20
        assert params["contact_acquire_speed_limit_mps"] == 0.07
        assert params["push_speed_limit_mps"] == 0.12
        assert params["obstacle_stop_distance_m"] < params["obstacle_slowdown_distance_m"]
        assert params["obstacle_slowdown_distance_m"] < params["obstacle_slowdown_exit_distance_m"]
        assert params["obstacle_reaction_time_sec"] == 0.35
        assert params["obstacle_braking_deceleration_mps2"] == 0.65
        assert params["obstacle_prediction_horizon_sec"] == 0.65
        assert params["ball_approach_distance_m"] < params["ball_approach_exit_distance_m"]
        for token in (
            "updateOpponentKinematics",
            "latest_max_closing_speed_mps_",
            "latest_dynamic_stop_distance_m_",
            "latest_dynamic_slowdown_distance_m_",
            "dynamic_emergency_stop",
            "alignment_anchor_field_",
            "have_alignment_anchor_",
        ):
            assert token in goal
        emergency = goal[
            goal.index("if (dynamic_emergency_stop || approach_unavailable)"):
            goal.index("else if (drive_through_committed_)")
        ]
        assert 'next_state = "BLOCKED_RECOVERY"' in emergency
        assert "next_control_valid = false" not in emergency
        assert 'state == "BLOCKED_RECOVERY"' in goal
        nav_limit = goal[
            goal.index('if (state == "NAV_TRANSIT")'):
            goal.index('if (state == "OBSTACLE_APPROACH")')
        ]
        assert "return 0.0;" in nav_limit
        assert params["push_target_lead_m"] > 0.11

    def test_wrapper_launches_non_continuous_single_shot_mode(self):
        wrapper = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.sh"
        )
        assert "football_acceptance_all_fake:=true" in wrapper
        assert "football_demo_continuous:=false" in wrapper
        assert "football_demo_continuous:=true" not in wrapper
        assert 'FOOTBALL_DDS_MODE=isolated_fastdds' in wrapper
        assert 'FOOTBALL_ISOLATED_DOMAIN_ID="$DOMAIN_ID"' in wrapper
        assert 'FOOTBALL_ACCEPTANCE_DOMAIN_ID:-142' in wrapper
        assert 'FOOTBALL_ACCEPTANCE_STARTUP_WAIT_SEC:-$WAIT_SEC' in wrapper
        assert 'ros2 run football_navigation football_user_function_acceptance.py' in wrapper
        assert 'source "$FOOTBALL_NX_ENV"' in wrapper
        assert "football_reset_ros2_daemon" in wrapper
        assert "FOOTBALL_ACCEPTANCE_KEEP_RUNNING" in wrapper

    def test_install_contract_contains_acceptance_and_diagnostics(self):
        cmake = read("cyberdog_nav2/football_navigation/CMakeLists.txt")
        package = read("cyberdog_nav2/football_navigation/package.xml")

        for script in (
            "scripts/football_user_function_acceptance.py",
            "scripts/football_user_function_acceptance.sh",
            "scripts/football_nx_quick_diag.sh",
            "scripts/football_nx_verify_deploy.sh",
        ):
            assert script in cmake

        assert "<depend>action_msgs</depend>" in package
        assert "<depend>dwb_core</depend>" in package
        assert "<depend>dwb_msgs</depend>" in package
        assert "<exec_depend>rclpy</exec_depend>" in package
        assert "<depend>geometry_msgs</depend>" in package

    def test_deploy_verifier_uses_current_oriented_footprint_clearance(self):
        verifier = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_nx_verify_deploy.sh"
        )

        assert "controlled and dynamic robot footprints stay separated" in verifier
        assert "minimum_footprint_clearance_seen" in verifier
        assert "cyberdog_2 avoids all active dynamic robots" not in verifier
        assert "grep -aFq" in verifier
        assert "'FootballTargetUpdater'" in verifier
        assert "same_direction_x = move_between" in verifier
        assert "frozen ALIGN target and cleared push corridor installed" in verifier
        assert "polygon_grid_samples" in verifier
        assert 'rate_nodes[0].set("hz", "2.0")' in verifier
        assert 'compute_nodes[0].set("server_timeout", "360000")' in verifier
        assert 'follow_nodes[0].set("server_timeout", "360000")' in verifier
        assert "opponent_endpoint_clearance_m: 0.84" in verifier

    def test_acceptance_reports_required_action_motion_and_scoring_metrics(self):
        acceptance = read(
            "cyberdog_nav2/football_navigation/scripts/"
            "football_user_function_acceptance.py"
        )

        for label in (
            "GOAL_SEND_COUNT=",
            "GOAL_ACCEPT_COUNT=",
            "GOAL_CANCEL_COUNT=",
            "GOAL_PREEMPT_COUNT=",
            "ACTION_FINAL_STATUS=",
            "PATH_UPDATE_COUNT=",
            "FOLLOWP_RUNNING_SEC=",
            "CMD_VEL_LINEAR_MAX_MPS=",
            "CMD_VEL_LINEAR_AVERAGE_MPS=",
            "CMD_VEL_ANGULAR_MAX_RPS=",
            "ROBOT_ACTUAL_AVERAGE_SPEED_MPS=",
            "ODOM_UPDATE_HZ=",
            "TF_UPDATE_HZ=",
            "ROBOT_TO_BALL_DISTANCE_M=",
            "KICK_LATERAL_ERROR_M=",
            "KICK_YAW_ERROR_RAD=",
            "CONTACT_ACQUIRE_ENTER_SEC=",
            "PUSH_BALL_ENTER_SEC=",
            "BALL_CUMULATIVE_DISPLACEMENT_M=",
            "BALL_GOAL_DIRECTION_DISPLACEMENT_M=",
            "BALL_CROSSED_CORRECT_GOAL_LINE=",
            "ALIGN_APPROACH_WRONG_DIRECTION_RATIO=",
            "ALIGN_TRACKING_WRONG_DIRECTION_RATIO=",
            "ALIGN_PATH_GOAL_WRONG_DIRECTION_RATIO=",
            "DWB_ALIGN_EVALUATION_COUNT=",
            "DWB_ALIGN_WRONG_SELECTION_COUNT=",
            "DWB_ALIGN_CORRECT_CANDIDATE_AVAILABLE_COUNT=",
            "DWB_ALIGN_CRITIC_CORRECT_MINUS_SELECTED=",
        ):
            assert label in acceptance

        assert "from dwb_msgs.msg import LocalPlanEvaluation" in acceptance
        assert '"/{}/evaluation".format(self.namespace)' in acceptance
        assert '"/{}/received_global_plan".format(self.namespace)' in acceptance
        assert "self.destroy_subscription(subscription)" in acceptance
        assert "self.dwb_evaluation_sample_count" in acceptance
        assert "self.dwb_stop_requested" in acceptance
        assert "callback_group=self.path_callback_group" in acceptance
        assert "self.dwb_align_probe_samples < 8" in acceptance
        assert "self.latest_dwb_goal_vodom" in acceptance
        assert "self.latest_vodom_robot_pose" in acceptance

    def test_all_fake_dwb_contract_prefers_forward_smooth_motion(self):
        launch = read(
            "cyberdog_nav2/navigation_bringup/launch/football_runtime.launch.py"
        )
        controller_override = launch[
            launch.index("tracking_controller.update({"):
            launch.index("planner = _required_path(")
        ]
        assert '"max_vel_x": 0.38' in controller_override
        assert '"max_speed_xy": 0.38' in controller_override
        assert '"max_vel_y": 0.14' in controller_override
        assert '"min_vel_y": -0.14' in controller_override
        assert '"vx_samples": 9' in controller_override
        assert '"vy_samples": 5' in controller_override
        assert '"vtheta_samples": 11' in controller_override
        assert '"sim_time": 1.00' in controller_override
        assert '"ObstacleFootprint"' in controller_override
        assert '"FootballPreferForward"' in controller_override
        assert (
            '"FootballPreferForward.class":\n'
            '            "football_navigation::FootballPreferForwardCritic"'
            in controller_override
        )
        assert '"publish_evaluation": True' in controller_override
        assert '"debug_trajectory_details": True' in controller_override

    def test_visualization_rotates_body_velocity_and_kickoff_state_is_published(self):
        visualization = read(
            "cyberdog_nav2/football_navigation/src/"
            "visualization/football_visualization_node.cpp"
        )
        fake_ball = read(
            "cyberdog_nav2/football_navigation/src/"
            "simulation/football_fake_ball_publisher.cpp"
        )
        arrow_start = visualization.index("void appendTwistArrowAt(")
        arrow = visualization[
            arrow_start:
            visualization.index("void appendTwistArrow(", arrow_start)
        ]
        kickoff_start = fake_ball.index("void resetToKickoff(")
        kickoff = fake_ball[
            kickoff_start:
            fake_ball.index("void finishSingleShot(", kickoff_start)
        ]

        assert "worldVelocityToBody(" in arrow
        assert "publishMatchStateCommand(kickoff_state)" in kickoff
