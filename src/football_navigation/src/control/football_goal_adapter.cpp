// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include "football_navigation/control/football_goal_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace football_navigation
{
namespace
{
std::vector<std::string> splitRobotNamespaces(const std::string & csv)
{
  std::vector<std::string> result;
  std::stringstream stream(csv);
  std::string value;
  while (std::getline(stream, value, ',')) {
    const auto first = value.find_first_not_of(" \t\r\n/");
    const auto last = value.find_last_not_of(" \t\r\n/");
    if (first != std::string::npos) {
      result.push_back(value.substr(first, last - first + 1));
    }
  }
  return result;
}

std::string expandRobotTopic(std::string pattern, const std::string & robot_namespace)
{
  const std::string token = "{namespace}";
  const auto position = pattern.find(token);
  if (position == std::string::npos) {
    throw std::invalid_argument("robot_odom_topic_template must contain {namespace}");
  }
  pattern.replace(position, token.size(), robot_namespace);
  return pattern;
}
}  // namespace

FootballGoalAdapter::FootballGoalAdapter()
: Node("football_goal_adapter"), have_last_output_time_(false), have_kick_target_(false)
{
  const auto zero = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_output_time_ = kickoff_hold_until_ = last_ball_stamp_ = last_kick_target_stamp_ = zero;
  last_striker_assignment_time_ = last_valid_ball_time_ = latest_odom_time_ = zero;
  last_tactical_role_time_ = last_tactical_target_stamp_ = zero;
  last_robot_pose_time_ = zero;
  last_cmd_vel_time_ = progress_anchor_time_ = recovery_hold_until_ = zero;
  previous_opponents_stamp_ = zero;
  kick_alignment_since_ = alignment_bad_since_ = alignment_started_ =
    alignment_progress_window_started_ = drive_through_started_ =
    contact_acquire_started_ = front_contact_since_ =
    last_push_contact_time_ = zero;
  last_push_contact_progress_time_ = push_alignment_lost_since_ = zero;
  last_state_publish_time_ = last_speed_limit_publish_time_ =
    last_approach_publish_time_ = zero;

  output_frame_ = declare_parameter<std::string>("target_frame", "base_link");
  field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  // 足球几何计算的输入（球、全局里程计和射门目标）均使用 field_frame。
  // 选定的机器人目标在发布前转换到 output_frame，通常为 tag_global -> base_link。
  ball_pose_topic_ = declare_parameter<std::string>("ball_pose_topic", "/football/ball_pose");
  kick_target_topic_ = declare_parameter<std::string>("kick_target_topic", "");
  approach_pose_topic_ = declare_parameter<std::string>(
    "approach_pose_topic", "football/approach_pose");
  state_topic_ = declare_parameter<std::string>("state_topic", "football/state");
  control_valid_topic_ = declare_parameter<std::string>(
    "control_valid_topic", "football/control_valid");
  localization_valid_topic_ = declare_parameter<std::string>(
    "localization_valid_topic", "football/localization_valid");
  robot_namespaces_csv_ = declare_parameter<std::string>(
    "robot_namespaces_csv",
    "cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5,"
    "cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10");
  robot_odom_topic_template_ = declare_parameter<std::string>(
    "robot_odom_topic_template", "/global_vio/{namespace}/odom");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  speed_limit_topic_ = declare_parameter<std::string>("speed_limit_topic", "speed_limit");

  self_namespace_ = normalizeRobotNamespace(
    declare_parameter<std::string>("self_namespace", ""));
  if (self_namespace_.empty()) {
    self_namespace_ = normalizeRobotNamespace(get_namespace());
  }
  team_id_ = declare_parameter<std::string>("team_id", "");
  std::transform(team_id_.begin(), team_id_.end(), team_id_.begin(),
    [](unsigned char value) {return static_cast<char>(std::tolower(value));});
  if (team_id_ != "a" && team_id_ != "b") {
    team_id_ = inferTeamIdFromNamespace(self_namespace_);
  }
  if (team_id_ != "a" && team_id_ != "b") {
    throw std::invalid_argument(
            "self_namespace must match cyberdog_<1..10> when team_id is not a or b");
  }
  require_striker_role_ = declare_parameter<bool>("require_striker_role", true);
  use_team_kick_target_ = declare_parameter<bool>("use_team_kick_target", true);
  striker_topic_ = declare_parameter<std::string>("striker_topic", "");
  if (striker_topic_.empty()) {
    striker_topic_ = "/football/team_" + team_id_ + "/striker";
  }
  role_topic_ = declare_parameter<std::string>("role_topic", "football/role");
  tactical_target_topic_ = declare_parameter<std::string>(
    "tactical_target_topic", "football/tactical_target");
  if (use_team_kick_target_) {
    // 团队权威节点发布球应该到达的位置，默认通常是对方球门中心。使用话题
    // 而不是在本节点写死坐标，是为了允许运行时战术动态调整射门点。
    kick_target_topic_ = "/football/team_" + team_id_ + "/kick_target";
  }
  goal_event_topic_ = declare_parameter<std::string>(
    "goal_event_topic", "/football/goal_scored");
  match_state_topic_ = declare_parameter<std::string>("match_state_topic", "/football/match_state");
  odom_global_topic_ = declare_parameter<std::string>("odom_global_topic", "");
  if (odom_global_topic_.empty()) {
    // mutil_robot_odom publishes each robot pose in the shared field frame on
    // /global_vio/<namespace>/odom.  Keep the topic overridable for deployment,
    // but use that normalized PC-side output by default.
    odom_global_topic_ = "/global_vio/" + self_namespace_ + "/odom";
  }

  update_rate_hz_ = declare_parameter<double>("update_rate_hz", 15.0);
  ball_timeout_sec_ = declare_parameter<double>("ball_timeout_sec", 0.45);
  max_ball_age_sec_ = declare_parameter<double>("max_ball_age_sec", 0.35);
  future_tolerance_sec_ = declare_parameter<double>("future_tolerance_sec", 0.08);
  max_ball_speed_mps_ = declare_parameter<double>("max_ball_speed_mps", 8.0);
  max_ball_jump_m_ = declare_parameter<double>("max_ball_jump_m", 0.35);
  odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.45);
  max_ball_odom_skew_sec_ = declare_parameter<double>("max_ball_odom_skew_sec", 0.12);
  role_timeout_sec_ = declare_parameter<double>("role_timeout_sec", 0.80);
  tactical_target_timeout_sec_ = declare_parameter<double>("tactical_target_timeout_sec", 0.80);
  require_other_robot_poses_ = declare_parameter<bool>("require_other_robot_poses", true);
  other_robot_timeout_sec_ = declare_parameter<double>("other_robot_timeout_sec", 0.70);
  minimum_other_robot_count_ = declare_parameter<int>("minimum_other_robot_count", 9);
  kick_target_timeout_sec_ = declare_parameter<double>("kick_target_timeout_sec", 0.80);
  kickoff_hold_sec_ = declare_parameter<double>("kickoff_hold_sec", 3.0);
  approach_reached_m_ = declare_parameter<double>("approach_reached_m", 0.16);
  approach_reached_exit_m_ = declare_parameter<double>(
    "approach_reached_exit_m", 0.22);
  align_yaw_tolerance_ = declare_parameter<double>("align_yaw_tolerance", 0.22);
  alignment_reset_grace_sec_ = declare_parameter<double>(
    "alignment_reset_grace_sec", 0.25);
  alignment_max_duration_sec_ = declare_parameter<double>(
    "alignment_max_duration_sec", 6.0);
  alignment_max_rotation_rad_ = declare_parameter<double>(
    "alignment_max_rotation_rad", 3.50);
  alignment_progress_window_sec_ = declare_parameter<double>(
    "alignment_progress_window_sec", 0.80);
  alignment_min_error_improvement_rad_ = declare_parameter<double>(
    "alignment_min_error_improvement_rad", 0.025);
  alignment_reacquire_lateral_m_ = declare_parameter<double>(
    "alignment_reacquire_lateral_m", 0.18);
  approach_speed_limit_mps_ = declare_parameter<double>(
    "approach_speed_limit_mps", 0.38);
  obstacle_speed_limit_mps_ = declare_parameter<double>(
    "obstacle_speed_limit_mps", 0.22);
  ball_approach_speed_limit_mps_ = declare_parameter<double>(
    "ball_approach_speed_limit_mps", 0.20);
  contact_acquire_speed_limit_mps_ = declare_parameter<double>(
    "contact_acquire_speed_limit_mps", 0.07);
  push_speed_limit_mps_ = declare_parameter<double>(
    "push_speed_limit_mps", 0.12);
  robot_collision_length_m_ = declare_parameter<double>(
    "robot_collision_length_m", 0.562);
  robot_collision_width_m_ = declare_parameter<double>(
    "robot_collision_width_m", 0.339);
  collision_ellipse_expansion_m_ = declare_parameter<double>(
    "collision_ellipse_expansion_m", 0.05);
  // These thresholds are signed edge clearances between the two oriented
  // circumscribed ellipses, rather than robot-center distances.
  obstacle_slowdown_distance_m_ = declare_parameter<double>(
    "obstacle_slowdown_clearance_m", 0.45);
  obstacle_slowdown_exit_distance_m_ = declare_parameter<double>(
    "obstacle_slowdown_exit_clearance_m", 0.60);
  obstacle_stop_distance_m_ = declare_parameter<double>(
    "obstacle_stop_clearance_m", 0.03);
  obstacle_reaction_time_sec_ = declare_parameter<double>(
    "obstacle_reaction_time_sec", 0.35);
  obstacle_braking_deceleration_mps2_ = declare_parameter<double>(
    "obstacle_braking_deceleration_mps2", 0.65);
  obstacle_prediction_horizon_sec_ = declare_parameter<double>(
    "obstacle_prediction_horizon_sec", 0.65);
  ball_approach_distance_m_ = declare_parameter<double>(
    "ball_approach_distance_m", 1.25);
  ball_approach_exit_distance_m_ = declare_parameter<double>(
    "ball_approach_exit_distance_m", 1.40);
  speed_limit_update_threshold_mps_ = declare_parameter<double>(
    "speed_limit_update_threshold_mps", 0.015);
  speed_limit_refresh_sec_ = declare_parameter<double>(
    "speed_limit_refresh_sec", 0.25);
  push_target_lead_m_ = declare_parameter<double>("push_target_lead_m", 0.32);
  push_enter_lateral_error_m_ = declare_parameter<double>(
    "push_enter_lateral_error_m", 0.08);
  push_exit_lateral_error_m_ = declare_parameter<double>(
    "push_exit_lateral_error_m", 0.14);
  push_realign_lateral_error_m_ = declare_parameter<double>(
    "push_realign_lateral_error_m", 0.09);
  push_enter_yaw_error_rad_ = declare_parameter<double>(
    "push_enter_yaw_error_rad", 0.10);
  push_exit_yaw_error_rad_ = declare_parameter<double>(
    "push_exit_yaw_error_rad", 0.18);
  push_contact_acquire_timeout_sec_ = declare_parameter<double>(
    "push_contact_acquire_timeout_sec", 2.00);
  contact_acquire_min_hold_sec_ = declare_parameter<double>(
    "contact_acquire_min_hold_sec", 0.30);
  push_contact_stable_hold_sec_ = declare_parameter<double>(
    "push_contact_stable_hold_sec", 0.20);
  push_contact_progress_timeout_sec_ = declare_parameter<double>(
    "push_contact_progress_timeout_sec", 1.50);
  push_contact_hard_timeout_sec_ = declare_parameter<double>(
    "push_contact_hard_timeout_sec", 6.00);
  push_contact_progress_epsilon_m_ = declare_parameter<double>(
    "push_contact_progress_epsilon_m", 0.01);
  push_alignment_loss_sec_ = declare_parameter<double>(
    "push_alignment_loss_sec", 0.35);
  push_contact_loss_sec_ = declare_parameter<double>(
    "push_contact_loss_sec", 0.45);
  robot_front_extent_m_ = declare_parameter<double>("robot_front_extent_m", 0.302);
  robot_rear_extent_m_ = declare_parameter<double>("robot_rear_extent_m", 0.301);
  robot_half_width_m_ = declare_parameter<double>("robot_half_width_m", 0.170);
  ball_radius_m_ = declare_parameter<double>("ball_radius_m", 0.11);
  contact_tolerance_m_ = declare_parameter<double>("contact_tolerance_m", 0.035);
  push_goal_crossing_margin_m_ = declare_parameter<double>(
    "push_goal_crossing_margin_m", 0.04);
  ball_protection_radius_m_ = declare_parameter<double>("ball_protection_radius_m", 0.28);
  max_local_goal_distance_m_ = declare_parameter<double>("max_local_goal_distance_m", 4.00);
  path_block_radius_m_ = declare_parameter<double>("path_block_radius_m", 0.50);
  field_min_x_ = declare_parameter<double>("field_min_x", -2.0);
  field_max_x_ = declare_parameter<double>("field_max_x", 8.0);
  field_min_y_ = declare_parameter<double>("field_min_y", -3.0);
  field_max_y_ = declare_parameter<double>("field_max_y", 3.0);
  boundary_margin_m_ = declare_parameter<double>("boundary_margin_m", 0.25);
  ball_boundary_margin_m_ = declare_parameter<double>("ball_boundary_margin_m", 0.12);
  approach_boundary_margin_m_ = declare_parameter<double>("approach_boundary_margin_m", 0.25);
  maximum_ball_chase_distance_m_ = declare_parameter<double>(
    "maximum_ball_chase_distance_m", 8.0);
  minimum_kick_target_distance_m_ = declare_parameter<double>(
    "minimum_kick_target_distance_m", 0.30);
  minimum_behind_alignment_m_ = declare_parameter<double>("minimum_behind_alignment_m", 0.18);
  maximum_kick_lateral_error_m_ = declare_parameter<double>(
    "maximum_kick_lateral_error_m", 0.20);
  kick_alignment_hold_sec_ = declare_parameter<double>(
    "kick_alignment_hold_sec", 0.35);
  cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.35);
  goal_update_min_hold_sec_ = declare_parameter<double>("goal_update_min_hold_sec", 0.25);
  goal_update_position_threshold_m_ = declare_parameter<double>(
    "goal_update_position_threshold_m", 0.08);
  goal_update_yaw_threshold_rad_ = declare_parameter<double>(
    "goal_update_yaw_threshold_rad", 0.10);
  goal_update_max_refresh_sec_ = declare_parameter<double>(
    "goal_update_max_refresh_sec", 1.0);

  if (approach_reached_m_ <= 0.0 ||
    approach_reached_exit_m_ < approach_reached_m_ ||
    align_yaw_tolerance_ <= 0.0 ||
    maximum_kick_lateral_error_m_ <= 0.0 ||
    alignment_reset_grace_sec_ <= 0.0 ||
    alignment_max_duration_sec_ <= 0.0 ||
    alignment_max_rotation_rad_ <= 0.0 ||
    alignment_progress_window_sec_ <= 0.0 ||
    alignment_min_error_improvement_rad_ <= 0.0 ||
    alignment_reacquire_lateral_m_ <= 0.0 ||
    !std::isfinite(approach_speed_limit_mps_) ||
    approach_speed_limit_mps_ <= 0.0 ||
    !std::isfinite(obstacle_speed_limit_mps_) ||
    obstacle_speed_limit_mps_ <= 0.0 ||
    obstacle_speed_limit_mps_ >= approach_speed_limit_mps_ ||
    !std::isfinite(ball_approach_speed_limit_mps_) ||
    ball_approach_speed_limit_mps_ <= 0.0 ||
    ball_approach_speed_limit_mps_ > obstacle_speed_limit_mps_ ||
    !std::isfinite(contact_acquire_speed_limit_mps_) ||
    contact_acquire_speed_limit_mps_ <= 0.0 ||
    !std::isfinite(push_speed_limit_mps_) ||
    push_speed_limit_mps_ <= 0.0 ||
    contact_acquire_speed_limit_mps_ >= push_speed_limit_mps_ ||
    push_speed_limit_mps_ >= ball_approach_speed_limit_mps_ ||
    robot_collision_length_m_ <= 0.0 || robot_collision_width_m_ <= 0.0 ||
    collision_ellipse_expansion_m_ < 0.0 ||
    obstacle_slowdown_distance_m_ <= obstacle_stop_distance_m_ ||
    obstacle_slowdown_exit_distance_m_ <= obstacle_slowdown_distance_m_ ||
    obstacle_stop_distance_m_ <= 0.0 ||
    obstacle_reaction_time_sec_ <= 0.0 ||
    obstacle_braking_deceleration_mps2_ <= 0.0 ||
    obstacle_prediction_horizon_sec_ <= 0.0 ||
    ball_approach_distance_m_ <= approach_reached_exit_m_ ||
    ball_approach_exit_distance_m_ <= ball_approach_distance_m_ ||
    speed_limit_update_threshold_mps_ <= 0.0 ||
    speed_limit_refresh_sec_ <= 0.0 ||
    push_target_lead_m_ <= 0.0 ||
    push_enter_lateral_error_m_ <= 0.0 ||
    push_exit_lateral_error_m_ < push_enter_lateral_error_m_ ||
    push_realign_lateral_error_m_ < push_enter_lateral_error_m_ ||
    push_realign_lateral_error_m_ > push_exit_lateral_error_m_ ||
    push_enter_yaw_error_rad_ <= 0.0 ||
    push_exit_yaw_error_rad_ < push_enter_yaw_error_rad_ ||
    push_contact_acquire_timeout_sec_ <= 0.0 ||
    contact_acquire_min_hold_sec_ < 0.0 ||
    contact_acquire_min_hold_sec_ >= push_contact_acquire_timeout_sec_ ||
    push_contact_stable_hold_sec_ <= 0.0 ||
    push_contact_stable_hold_sec_ >= push_contact_acquire_timeout_sec_ ||
    push_contact_progress_timeout_sec_ <= 0.0 ||
    push_contact_hard_timeout_sec_ <= push_contact_acquire_timeout_sec_ ||
    push_contact_progress_epsilon_m_ <= 0.0 ||
    push_alignment_loss_sec_ <= 0.0 ||
    push_contact_loss_sec_ <= 0.0 ||
    robot_front_extent_m_ <= 0.0 ||
    robot_rear_extent_m_ <= 0.0 ||
    robot_half_width_m_ <= 0.0 ||
    ball_radius_m_ <= 0.0 ||
    contact_tolerance_m_ < 0.0 ||
    push_goal_crossing_margin_m_ < 0.0 ||
    push_goal_crossing_margin_m_ >= ball_radius_m_)
  {
    throw std::invalid_argument("invalid low-speed push parameters");
  }

  if (goal_update_min_hold_sec_ < 0.0 || goal_update_position_threshold_m_ <= 0.0 ||
    goal_update_yaw_threshold_rad_ <= 0.0 ||
    goal_update_max_refresh_sec_ <= goal_update_min_hold_sec_)
  {
    throw std::invalid_argument("invalid football goal update stability parameters");
  }

  approach_config_.approach_distance = declare_parameter<double>("approach_distance_m", 0.60);
  approach_config_.default_kick_yaw = declare_parameter<double>("default_kick_yaw", 0.0);
  approach_config_.min_direction_norm = declare_parameter<double>("min_direction_norm", 1e-4);
  lateral_entry_config_.entry_lateral_m = declare_parameter<double>("entry_lateral_m", 0.70);
  lateral_entry_config_.retreat_clearance_m = declare_parameter<double>(
    "retreat_clearance_m", 0.35);
  lateral_entry_config_.opponent_endpoint_clearance_m = declare_parameter<double>(
    "opponent_endpoint_clearance_m", 0.84);
  if (!std::isfinite(lateral_entry_config_.opponent_endpoint_clearance_m) ||
    lateral_entry_config_.opponent_endpoint_clearance_m < path_block_radius_m_)
  {
    throw std::invalid_argument(
            "opponent_endpoint_clearance_m must be finite and at least path_block_radius_m");
  }
  lateral_entry_config_.ball_protection_radius_m = std::max(
    ball_protection_radius_m_,
    std::hypot(robot_front_extent_m_, robot_half_width_m_) +
      ball_radius_m_ + contact_tolerance_m_);
  lateral_entry_config_.field_min_x = field_min_x_;
  lateral_entry_config_.field_max_x = field_max_x_;
  lateral_entry_config_.field_min_y = field_min_y_;
  lateral_entry_config_.field_max_y = field_max_y_;
  lateral_entry_config_.boundary_margin_m = approach_boundary_margin_m_;

  no_progress_config_.window_sec = declare_parameter<double>("no_progress_window_sec", 1.5);
  no_progress_config_.min_distance_improvement = declare_parameter<double>(
    "no_progress_min_distance_improvement", 0.05);
  no_progress_config_.min_path_progress = declare_parameter<double>(
    "no_progress_min_path_progress", 0.08);
  no_progress_config_.max_actual_speed = declare_parameter<double>(
    "no_progress_max_actual_speed", 0.03);
  no_progress_config_.obstacle_near_m = declare_parameter<double>(
    "no_progress_obstacle_near_m", 0.75);
  no_progress_config_.max_ball_motion = declare_parameter<double>(
    "no_progress_max_ball_motion", 0.05);
  no_progress_config_.ball_contact_m = declare_parameter<double>(
    "no_progress_ball_contact_m", 0.25);

  approach_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(approach_pose_topic_, 10);
  control_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
    control_valid_topic_, rclcpp::QoS(1).reliable().transient_local());
  localization_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
        localization_valid_topic_, rclcpp::QoS(1).reliable().transient_local());
  state_pub_ = create_publisher<std_msgs::msg::String>(
    state_topic_, rclcpp::QoS(1).reliable().transient_local());
  speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic_, rclcpp::QoS(1).reliable().transient_local());
  ball_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    ball_pose_topic_, rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&FootballGoalAdapter::ballPoseCallback, this, std::placeholders::_1));
  kick_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    kick_target_topic_, rclcpp::QoS(5).reliable().transient_local(),
    std::bind(&FootballGoalAdapter::kickTargetCallback, this, std::placeholders::_1));
  striker_sub_ = create_subscription<std_msgs::msg::String>(
    striker_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballGoalAdapter::strikerCallback, this, std::placeholders::_1));
  role_sub_ = create_subscription<std_msgs::msg::String>(
    role_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballGoalAdapter::roleCallback, this, std::placeholders::_1));
  tactical_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    tactical_target_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballGoalAdapter::tacticalTargetCallback, this, std::placeholders::_1));
  goal_event_sub_ = create_subscription<std_msgs::msg::String>(
    goal_event_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&FootballGoalAdapter::goalEventCallback, this, std::placeholders::_1));
  match_state_sub_ = create_subscription<std_msgs::msg::String>(
    match_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballGoalAdapter::matchStateCallback, this, std::placeholders::_1));
  odom_global_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_global_topic_, rclcpp::SensorDataQoS().keep_last(30),
    std::bind(&FootballGoalAdapter::odomGlobalCallback, this, std::placeholders::_1));
  robot_namespaces_ = splitRobotNamespaces(robot_namespaces_csv_);
  for (const auto & robot_namespace : robot_namespaces_) {
    if (robot_namespace == self_namespace_) {
      continue;
    }
    other_robot_odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      expandRobotTopic(robot_odom_topic_template_, robot_namespace),
      rclcpp::SensorDataQoS().keep_last(5),
      [this, robot_namespace](const nav_msgs::msg::Odometry::SharedPtr msg) {
        otherRobotOdomCallback(robot_namespace, msg);
      }));
  }
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&FootballGoalAdapter::cmdVelCallback, this, std::placeholders::_1));
  safety_timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&FootballGoalAdapter::safetyWatchdog, this));
  setControlState("SEARCH_BALL", false);
}

bool FootballGoalAdapter::validFinitePose(const geometry_msgs::msg::Pose & pose) const
{
  double yaw = 0.0;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && yawFromQuaternion(pose.orientation, yaw);
}

bool FootballGoalAdapter::validateBallPose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (pose.header.frame_id != field_frame_ || !validFinitePose(pose.pose)) {
    return false;
  }
  const rclcpp::Time stamp(pose.header.stamp, get_clock()->get_clock_type());
  const double age = stamp.nanoseconds() > 0 ?
    (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
  return stamp.nanoseconds() > 0 && age <= max_ball_age_sec_ &&
         age >= -future_tolerance_sec_ &&
         (last_ball_stamp_.nanoseconds() <= 0 || stamp > last_ball_stamp_);
}

bool FootballGoalAdapter::odomGlobalFresh() const
{
  return have_odom_global_ && (now() - latest_odom_time_).seconds() <= odom_timeout_sec_;
}

bool FootballGoalAdapter::findOdomAt(
  const rclcpp::Time & target, nav_msgs::msg::Odometry & output, const double max_skew_sec) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & candidate : odom_history_) {
    const rclcpp::Time stamp(candidate.header.stamp, get_clock()->get_clock_type());
    const double skew = std::fabs((target - stamp).seconds());
    if (skew < best) {
      best = skew;
      output = candidate;
    }
  }
  return best <= max_skew_sec;
}

bool FootballGoalAdapter::kickTargetFresh() const
{
  return have_kick_target_ && last_kick_target_stamp_.nanoseconds() > 0 &&
         (now() - last_kick_target_stamp_).seconds() <= kick_target_timeout_sec_;
}

void FootballGoalAdapter::odomGlobalCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 保存 field_frame 下的短时位姿历史，使每一帧球数据能匹配时间上最近的
  // 机器人位姿，避免混用旧球位置和最新里程计造成几何计算误差。
  if (!msg) {
    return;
  }
  if (msg->header.frame_id != field_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "ignoring global odom with frame_id='%s'; expected '%s'",
      msg->header.frame_id.c_str(), field_frame_.c_str());
    return;
  }
  if (!validFinitePose(msg->pose.pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "ignoring non-finite global odom pose");
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = stamp.nanoseconds() > 0 ?
    (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
  if (stamp.nanoseconds() <= 0 || age > odom_timeout_sec_ ||
    age < -future_tolerance_sec_ ||
    (!odom_history_.empty() && stamp <= rclcpp::Time(
      odom_history_.back().header.stamp, get_clock()->get_clock_type())))
  {
    return;
  }
  latest_odom_global_ = *msg;
  latest_odom_time_ = now();
  have_odom_global_ = true;
  odom_history_.push_back(*msg);
  while (odom_history_.size() > odom_history_limit_) {
    odom_history_.pop_front();
  }
  publishLocalizationValid(true);
}

void FootballGoalAdapter::kickTargetCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // kick_target 是球的目的地，不是机器人的导航目标。计算射门方向之前，
  // 它必须与 ball_pose 位于同一个 field_frame 中。
  if (!msg || msg->header.frame_id != field_frame_ || !validFinitePose(msg->pose)) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = stamp.nanoseconds() > 0 ?
    (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
  if (stamp.nanoseconds() <= 0 || age > kick_target_timeout_sec_ ||
    age < -future_tolerance_sec_ ||
    (last_kick_target_stamp_.nanoseconds() > 0 && stamp <= last_kick_target_stamp_))
  {
    return;
  }
  latest_kick_target_ = *msg;
  last_kick_target_stamp_ = stamp;
  have_kick_target_ = true;
}

void FootballGoalAdapter::otherRobotOdomCallback(
  const std::string & robot_namespace,
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg || msg->header.frame_id != field_frame_ || !validFinitePose(msg->pose.pose)) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = stamp.nanoseconds() > 0 ?
    (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
  if (stamp.nanoseconds() <= 0 || age > other_robot_timeout_sec_ ||
    age < -future_tolerance_sec_) {
    return;
  }
  latest_other_robot_odoms_[robot_namespace] = *msg;
  latest_other_robot_times_.insert_or_assign(robot_namespace, now());
}

void FootballGoalAdapter::updateOpponentKinematics(
  const std::vector<OpponentPoint2D> & positions,
  const nav_msgs::msg::Odometry & robot_odom,
  const rclcpp::Time & stamp)
{
  std::vector<OpponentPoint2D> current;
  current = positions;

  double robot_yaw = 0.0;
  if (!yawFromQuaternion(robot_odom.pose.pose.orientation, robot_yaw)) {
    previous_opponents_field_.clear();
    previous_opponents_stamp_ = rclcpp::Time(
      0, 0, get_clock()->get_clock_type());
    latest_max_closing_speed_mps_ = 0.0;
    return;
  }

  const double cosine = std::cos(robot_yaw);
  const double sine = std::sin(robot_yaw);

  const double self_vx =
    cosine * robot_odom.twist.twist.linear.x -
    sine * robot_odom.twist.twist.linear.y;
  const double self_vy =
    sine * robot_odom.twist.twist.linear.x +
    cosine * robot_odom.twist.twist.linear.y;
  const double self_speed = std::hypot(self_vx, self_vy);

  latest_max_closing_speed_mps_ = 0.0;
  const double dt = previous_opponents_stamp_.nanoseconds() > 0 ?
    (stamp - previous_opponents_stamp_).seconds() : 0.0;
  if (dt >= 0.04 && dt <= other_robot_timeout_sec_ &&
    current.size() == previous_opponents_field_.size())
  {
    for (std::size_t index = 0; index < current.size(); ++index) {
      const double obstacle_vx =
        (current[index].first - previous_opponents_field_[index].first) / dt;
      const double obstacle_vy =
        (current[index].second - previous_opponents_field_[index].second) / dt;
      const double relative_x =
        current[index].first - robot_odom.pose.pose.position.x;
      const double relative_y =
        current[index].second - robot_odom.pose.pose.position.y;
      const double separation = std::hypot(relative_x, relative_y);
      if (separation <= 1e-6) {
        continue;
      }
      const double unit_x = relative_x / separation;
      const double unit_y = relative_y / separation;
      const double closing_speed =
        (self_vx - obstacle_vx) * unit_x +
        (self_vy - obstacle_vy) * unit_y;
      latest_max_closing_speed_mps_ = std::max(
        latest_max_closing_speed_mps_,
        std::max(0.0, closing_speed));
    }
  }

  const double braking_distance =
    self_speed * self_speed /
    (2.0 * obstacle_braking_deceleration_mps2_);
  latest_dynamic_stop_distance_m_ =
    obstacle_stop_distance_m_ +
    latest_max_closing_speed_mps_ * obstacle_reaction_time_sec_ +
    braking_distance;
  latest_dynamic_slowdown_distance_m_ = std::max(
    obstacle_slowdown_distance_m_,
    latest_dynamic_stop_distance_m_ +
    latest_max_closing_speed_mps_ * obstacle_prediction_horizon_sec_ + 0.25);

  previous_opponents_field_ = std::move(current);
  previous_opponents_stamp_ = stamp;
}

bool FootballGoalAdapter::otherRobotPosesFresh() const
{
  int fresh_count = 0;
  const auto current = now();
  for (const auto & entry : latest_other_robot_times_) {
    if ((current - entry.second).seconds() <= other_robot_timeout_sec_) {
      ++fresh_count;
    }
  }
  return fresh_count >= minimum_other_robot_count_;
}

void FootballGoalAdapter::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg || !std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
    !std::isfinite(msg->angular.z))
  {
    return;
  }
  latest_command_linear_speed_ = std::hypot(msg->linear.x, msg->linear.y);
  latest_command_angular_speed_ = std::fabs(msg->angular.z);
  latest_command_speed_ = latest_command_linear_speed_ +
    0.25 * latest_command_angular_speed_;
  last_cmd_vel_time_ = now();
}

void FootballGoalAdapter::strikerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const std::string assigned = normalizeRobotNamespace(msg->data);
  have_striker_assignment_ = !assigned.empty();
  is_striker_ = assigned == self_namespace_;
  last_striker_assignment_time_ = now();
}

void FootballGoalAdapter::roleCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || (msg->data != "STRIKER" && msg->data != "SUPPORT" &&
      msg->data != "DEFENDER_LEFT" && msg->data != "DEFENDER_RIGHT" &&
      msg->data != "GOALKEEPER" && msg->data != "STOP"))
  {
    return;
  }
  tactical_role_ = msg->data;
  last_tactical_role_time_ = now();
}

void FootballGoalAdapter::tacticalTargetCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg || msg->header.frame_id != field_frame_ || !validFinitePose(msg->pose)) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = stamp.nanoseconds() > 0 ? (now() - stamp).seconds() : 1e9;
  if (stamp.nanoseconds() <= 0 || age > tactical_target_timeout_sec_ ||
    age < -future_tolerance_sec_ || stamp <= last_tactical_target_stamp_)
  {
    return;
  }
  latest_tactical_target_ = *msg;
  last_tactical_target_stamp_ = stamp;
  have_tactical_target_ = true;
}

bool FootballGoalAdapter::strikerAssignmentFresh() const
{
  return have_striker_assignment_ && last_striker_assignment_time_.nanoseconds() > 0 &&
         (now() - last_striker_assignment_time_).seconds() <= role_timeout_sec_;
}

bool FootballGoalAdapter::isSelfStriker() const
{
  return !require_striker_role_ || (strikerAssignmentFresh() && is_striker_);
}

bool FootballGoalAdapter::hasActiveTacticalRole() const
{
  return tactical_role_ != "STOP" && last_tactical_role_time_.nanoseconds() > 0 &&
    (now() - last_tactical_role_time_).seconds() <= role_timeout_sec_;
}

bool FootballGoalAdapter::publishTacticalTarget()
{
  if (!hasActiveTacticalRole() || tactical_role_ == "STRIKER") {
    return false;
  }
  if (!have_tactical_target_ || last_tactical_target_stamp_.nanoseconds() <= 0 ||
    (now() - last_tactical_target_stamp_).seconds() > tactical_target_timeout_sec_)
  {
    setControlState("WAITING_TACTICAL_TARGET", false);
    return true;
  }
  if (!odomGlobalFresh() || (require_other_robot_poses_ && !otherRobotPosesFresh())) {
    setControlState("WAITING_TEAM_OBSTACLES", false);
    return true;
  }
  if (!findOdomAt(last_tactical_target_stamp_, synchronized_odom_, max_ball_odom_skew_sec_)) {
    setControlState("SAFE_STOP", false);
    return true;
  }
  have_synchronized_odom_ = true;
  geometry_msgs::msg::PoseStamped local;
  if (!transformFromFieldToOutput(latest_tactical_target_, local)) {
    setControlState("SAFE_STOP", false);
    return true;
  }
  const double distance = std::hypot(local.pose.position.x, local.pose.position.y);
  if (output_frame_ != field_frame_ && distance > max_local_goal_distance_m_) {
    const double scale = max_local_goal_distance_m_ / distance;
    local.pose.position.x *= scale;
    local.pose.position.y *= scale;
    local.pose.orientation = quaternionFromYaw(std::atan2(local.pose.position.y, local.pose.position.x));
  }
  local.header.stamp = latest_tactical_target_.header.stamp;
  setControlState("TACTICAL_" + tactical_role_, true);
  approach_pose_pub_->publish(local);
  return true;
}

void FootballGoalAdapter::goalEventCallback(const std_msgs::msg::String::SharedPtr)
{
  kickoff_hold_until_ = now() + rclcpp::Duration::from_seconds(kickoff_hold_sec_);
  resetGoalStability();
  setControlState("SAFE_STOP", false);
}

void FootballGoalAdapter::matchStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || (msg->data != "STOP" && msg->data != "READY" && msg->data != "PLAY" &&
      msg->data != "KICKOFF_A" && msg->data != "KICKOFF_B" && msg->data != "FINISHED"))
  {
    return;
  }
  match_state_ = msg->data;
}

bool FootballGoalAdapter::matchStateAllowsMovement() const
{
  return match_state_ == "PLAY" ||
    (match_state_ == "KICKOFF_A" && team_id_ == "a") ||
    (match_state_ == "KICKOFF_B" && team_id_ == "b");
}

bool FootballGoalAdapter::inKickoffHold(const rclcpp::Time & stamp) const
{
  return kickoff_hold_until_.nanoseconds() > 0 && stamp < kickoff_hold_until_;
}

void FootballGoalAdapter::safetyWatchdog()
{
  if (!matchStateAllowsMovement() || inKickoffHold(now())) {
    setControlState("SAFE_STOP", false);
  } else if (recovery_hold_until_.nanoseconds() > 0 &&
    now() < recovery_hold_until_)
  {
    setControlState("BLOCKED_RECOVERY", true);
  } else if (!isSelfStriker()) {
    if (hasActiveTacticalRole() && publishTacticalTarget()) {
      return;
    }
    setControlState(strikerAssignmentFresh() ? "YIELD" : "SELECT_ATTACKER", false);
  } else if (use_team_kick_target_ && !kickTargetFresh()) {
    resetGoalStability();
    setControlState("WAITING_KICK_TARGET", false);
  } else if (!odomGlobalFresh()) {
    resetGoalStability();
    publishLocalizationValid(false);
    setControlState("SAFE_STOP", false);
  } else if (last_valid_ball_time_.nanoseconds() <= 0 ||
    (now() - last_valid_ball_time_).seconds() > ball_timeout_sec_)
  {
    resetGoalStability();
    publishLocalizationValid(true);
    setControlState("SEARCH_BALL", false);
  } else if (require_other_robot_poses_ && !otherRobotPosesFresh()) {
    resetGoalStability();
    publishLocalizationValid(true);
    setControlState("WAITING_TEAM_OBSTACLES", false);
  }
}

bool FootballGoalAdapter::transformToFieldFrame(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output)
{
  if (input.header.frame_id == field_frame_) {
    output = input;
    return true;
  }
  if (input.header.frame_id != base_frame_ || !have_synchronized_odom_) {
    return false;
  }
  double robot_yaw = 0.0;
  double local_yaw = 0.0;
  const auto & robot = synchronized_odom_.pose.pose;
  if (!yawFromQuaternion(robot.orientation, robot_yaw) ||
    !yawFromQuaternion(input.pose.orientation, local_yaw))
  {
    return false;
  }
  output = input;
  output.header.frame_id = field_frame_;
  output.pose.position.x = robot.position.x + std::cos(robot_yaw) * input.pose.position.x -
    std::sin(robot_yaw) * input.pose.position.y;
  output.pose.position.y = robot.position.y + std::sin(robot_yaw) * input.pose.position.x +
    std::cos(robot_yaw) * input.pose.position.y;
  output.pose.orientation = quaternionFromYaw(robot_yaw + local_yaw);
  return validFinitePose(output.pose);
}

bool FootballGoalAdapter::transformFromFieldToOutput(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output)
{
  if (input.header.frame_id == output_frame_) {
    output = input;
    return true;
  }
  if (input.header.frame_id != field_frame_ || output_frame_ != base_frame_ ||
    !have_synchronized_odom_)
  {
    return false;
  }
  double robot_yaw = 0.0;
  double input_yaw = 0.0;
  const auto & robot = synchronized_odom_.pose.pose;
  if (!yawFromQuaternion(robot.orientation, robot_yaw) ||
    !yawFromQuaternion(input.pose.orientation, input_yaw))
  {
    return false;
  }
  const double dx = input.pose.position.x - robot.position.x;
  const double dy = input.pose.position.y - robot.position.y;
  output = input;
  output.header.frame_id = output_frame_;
  output.pose.position.x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  output.pose.position.y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  output.pose.position.z = 0.0;
  output.pose.orientation = quaternionFromYaw(input_yaw - robot_yaw);
  return validFinitePose(output.pose);
}

bool FootballGoalAdapter::lookupRobotPoseInFieldFrame(
  geometry_msgs::msg::PoseStamped & output)
{
  if (!have_synchronized_odom_) {
    return false;
  }
  output.header = synchronized_odom_.header;
  output.pose = synchronized_odom_.pose.pose;
  last_robot_pose_time_ = now();
  publishLocalizationValid(true);
  return true;
}

std::vector<OpponentPoint2D> FootballGoalAdapter::collectOpponentPositions()
{
  // 所有机器人直接使用 mutil_robot_odom 提供的共享 field_frame 位姿，
  // 不再通过本机 base_frame 下的聚合 PoseArray 做二次坐标变换。
  std::vector<OpponentPoint2D> output;
  if (!otherRobotPosesFresh()) {
    return output;
  }
  const auto current = now();
  for (const auto & entry : latest_other_robot_odoms_) {
    const auto time = latest_other_robot_times_.find(entry.first);
    if (time != latest_other_robot_times_.end() &&
      (current - time->second).seconds() <= other_robot_timeout_sec_)
    {
      output.emplace_back(
        entry.second.pose.pose.position.x,
        entry.second.pose.pose.position.y);
    }
  }
  if (have_synchronized_odom_) {
    updateOpponentKinematics(output, synchronized_odom_, current);
  }
  return output;
}

bool FootballGoalAdapter::isBallSafelyReachable(
  const geometry_msgs::msg::PoseStamped & ball,
  const geometry_msgs::msg::PoseStamped & robot)
{
  const auto inside = [this](const double x, const double y, const double margin) {
      return x >= field_min_x_ + margin && x <= field_max_x_ - margin &&
             y >= field_min_y_ + margin && y <= field_max_y_ - margin;
    };
  return inside(ball.pose.position.x, ball.pose.position.y, ball_boundary_margin_m_) &&
         inside(robot.pose.position.x, robot.pose.position.y, 0.0) &&
         planarDistance(ball.pose.position.x, ball.pose.position.y,
           robot.pose.position.x, robot.pose.position.y) <= maximum_ball_chase_distance_m_;
}

bool FootballGoalAdapter::processingDue() const
{
  if (update_rate_hz_ <= 0.0) {
    return true;
  }
  const auto stamp = now();
    return !have_last_output_time_ ||
    (stamp - last_output_time_).seconds() >= 1.0 / update_rate_hz_;
}

void FootballGoalAdapter::markProcessed()
{
  last_output_time_ = now();
  have_last_output_time_ = true;
}

void FootballGoalAdapter::ballPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 此消息是融合后位于 field_frame 下的球位姿。真实感知和假球仿真都先向
  // 上游 /football/ball_pose_raw 发布原始数据。
  if (!msg || !validateBallPose(*msg)) {
    return;
  }

  const rclcpp::Time sample_stamp(
    msg->header.stamp,
    get_clock()->get_clock_type());
  const auto ball = *msg;

  if (have_last_ball_) {
    const double dt = (sample_stamp - last_ball_stamp_).seconds();
    const double ball_shift = planarDistance(
      ball.pose.position.x,
      ball.pose.position.y,
      last_ball_in_field_.pose.position.x,
      last_ball_in_field_.pose.position.y);

    if (dt <= 0.0 ||
      ball_shift > max_ball_jump_m_ + max_ball_speed_mps_ * dt)
    {
      resetGoalStability();
      setControlState("REJECTED_BALL_JUMP", false);
      return;
    }
  }

  last_ball_stamp_ = sample_stamp;
  last_ball_in_field_ = ball;
  last_valid_ball_time_ = now();
  have_last_ball_ = true;

  if (!processingDue()) {
    return;
  }

  if (!matchStateAllowsMovement() ||
    inKickoffHold(now()) ||
    !isSelfStriker())
  {
    return;
  }

  if (recovery_hold_until_.nanoseconds() > 0 &&
    now() < recovery_hold_until_)
  {
    setControlState("BLOCKED_RECOVERY", true);
    markProcessed();
    return;
  }

  if (use_team_kick_target_ && !kickTargetFresh()) {
    resetGoalStability();
    setControlState("WAITING_KICK_TARGET", false);
    markProcessed();
    return;
  }

  if (!findOdomAt(sample_stamp, synchronized_odom_, max_ball_odom_skew_sec_)) {
    have_synchronized_odom_ = false;
    resetGoalStability();
    setControlState("SAFE_STOP", false);
    markProcessed();
    return;
  }
  have_synchronized_odom_ = true;

  geometry_msgs::msg::PoseStamped robot;
  // robot 来自与当前球时间戳同步的 odom_global；它既不是 base_link 下的
  // 相对位姿，也不是不考虑时间同步而直接取得的最新里程计样本。
  if (!lookupRobotPoseInFieldFrame(robot) || !isBallSafelyReachable(ball, robot)) {
    resetGoalStability();
    setControlState("SAFE_STOP", false);
    markProcessed();
    return;
  }

  if (require_other_robot_poses_ && !otherRobotPosesFresh()) {
    resetGoalStability();
    setControlState("WAITING_TEAM_OBSTACLES", false);
    markProcessed();
    return;
  }

  const auto opponents = collectOpponentPositions();
  geometry_msgs::msg::PoseStamped kick_target = latest_kick_target_;

  if (!use_team_kick_target_) {
    // 备用模式：从球出发，沿配置的默认射门方向在 1 米处构造 field-frame 目标。
    kick_target = ball;
    kick_target.pose.position.x += std::cos(approach_config_.default_kick_yaw);
    kick_target.pose.position.y += std::sin(approach_config_.default_kick_yaw);
  }

  const double kick_dx = kick_target.pose.position.x - ball.pose.position.x;
  const double kick_dy = kick_target.pose.position.y - ball.pose.position.y;
  const double kick_norm = std::hypot(kick_dx, kick_dy);

  if (!std::isfinite(kick_norm) ||
    (kick_norm < minimum_kick_target_distance_m_ && !drive_through_committed_))
  {
    resetGoalStability();
    setControlState("SAFE_STOP", false);
    markProcessed();
    return;
  }

  const double ux =
    kick_norm >= minimum_kick_target_distance_m_ ?
    kick_dx / kick_norm : drive_direction_x_;
  const double uy =
    kick_norm >= minimum_kick_target_distance_m_ ?
    kick_dy / kick_norm : drive_direction_y_;
  const auto behind = computeApproachPose(ball, kick_target, approach_config_);

  ApproachPlan plan{behind, true, false};
  if (!drive_through_committed_) {
    auto lateral = lateral_entry_config_;
    lateral.preferred_side = deterministicYieldSide(team_id_, self_namespace_, 0.0);
    plan = computeApproachPoseAvoidingBlocker(
      ball,
      kick_target,
      robot,
      opponents,
      approach_config_,
      lateral,
      path_block_radius_m_);

  }

  const bool approach_unavailable = !plan.feasible;
  if (approach_unavailable) {
    plan.pose = have_published_approach_ ? last_published_approach_field_ : robot;
  }
  auto approach = plan.pose;
  const bool blocked = plan.used_detour;
  const double latched_approach_distance = planarDistance(
    robot.pose.position.x,
    robot.pose.position.y,
    last_published_approach_field_.pose.position.x,
    last_published_approach_field_.pose.position.y);
  const bool latched_approach_path_clear = !isOpponentBlockingPathToPoint(
    robot.pose.position.x,
    robot.pose.position.y,
    last_published_approach_field_.pose.position.x,
    last_published_approach_field_.pose.position.y,
    opponents,
    path_block_radius_m_);
  const bool latched_approach_endpoint_clear = std::all_of(
    opponents.begin(), opponents.end(),
    [&](const OpponentPoint2D & opponent) {
      return planarDistance(
        last_published_approach_field_.pose.position.x,
        last_published_approach_field_.pose.position.y,
        opponent.first,
        opponent.second) >=
        lateral_entry_config_.opponent_endpoint_clearance_m;
    });
  const bool latched_approach_passed = approachTargetPassed(
    robot, last_published_approach_field_,
    approach_reached_m_, approach_reached_exit_m_);
  if (
    blocked && have_published_approach_ &&
    last_approach_state_ == "OBSTACLE_APPROACH" &&
    latched_approach_distance > approach_reached_exit_m_ &&
    !latched_approach_passed &&
    latched_approach_path_clear &&
    latched_approach_endpoint_clear)
  {
    // Let the rolling planner avoid moving robots around one stable staging
    // point until it is reached or moving robots make it unsafe.
    plan.pose = last_published_approach_field_;
    approach = plan.pose;
  }
  const double behind_error = planarDistance(
    robot.pose.position.x,
    robot.pose.position.y,
    behind.pose.position.x,
    behind.pose.position.y);
  const double strict_lateral_tolerance = std::min(
    maximum_kick_lateral_error_m_, push_enter_lateral_error_m_);
  const bool centered_on_kick_line = isRobotBehindAndAlignedForKick(
    robot.pose.position.x,
    robot.pose.position.y,
    ball.pose.position.x,
    ball.pose.position.y,
    kick_target.pose.position.x,
    kick_target.pose.position.y,
    minimum_behind_alignment_m_,
    strict_lateral_tolerance);

  if (!drive_through_committed_) {
    if (
      !alignment_position_latched_ &&
      behind_error <= approach_reached_m_ &&
      centered_on_kick_line)
    {
      alignment_position_latched_ = true;
      alignment_anchor_field_ = robot;
      alignment_anchor_field_.header.frame_id = field_frame_;
      alignment_anchor_field_.pose.orientation = behind.pose.orientation;
      have_alignment_anchor_ = true;
      resetProgressWatchdog();
    } else if (
      alignment_position_latched_ &&
      (behind_error > approach_reached_exit_m_ ||
      !centered_on_kick_line))
    {
      alignment_position_latched_ = false;
      have_alignment_anchor_ = false;
      kick_alignment_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      alignment_bad_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
    }
  }

  if (!drive_through_committed_ && !alignment_position_latched_) {
    const bool no_progress = updateNoProgress(robot, ball, plan.pose, opponents);
    if (no_progress && stalled_window_count_ >= 3) {
      recovery_hold_until_ = now() + rclcpp::Duration::from_seconds(0.3);
      resetProgressWatchdog();
      setControlState("BLOCKED_RECOVERY", true);
      markProcessed();
      return;
    }
  } else {
    resetProgressWatchdog();
  }

  double robot_yaw = 0.0;
  double desired_yaw = 0.0;
  if (!yawFromQuaternion(robot.pose.orientation, robot_yaw) ||
    !yawFromQuaternion(behind.pose.orientation, desired_yaw))
  {
    resetGoalStability();
    setControlState("SAFE_STOP_INVALID_ORIENTATION", false);
    markProcessed();
    return;
  }

  const double yaw_error = yawDistance(robot_yaw, desired_yaw);
  const double robot_to_ball_x = ball.pose.position.x - robot.pose.position.x;
  const double robot_to_ball_y = ball.pose.position.y - robot.pose.position.y;
  const double robot_ball_distance = std::hypot(robot_to_ball_x, robot_to_ball_y);
  latest_robot_ball_distance_m_ = robot_ball_distance;
  latest_nearest_obstacle_distance_m_ = nearestOpponentDistance(robot, opponents);
  if (!std::isfinite(latest_dynamic_stop_distance_m_)) {
    latest_dynamic_stop_distance_m_ = obstacle_stop_distance_m_;
  }
  if (!std::isfinite(latest_dynamic_slowdown_distance_m_)) {
    latest_dynamic_slowdown_distance_m_ = obstacle_slowdown_distance_m_;
  }
  const bool dynamic_emergency_stop =
    std::isfinite(latest_nearest_obstacle_distance_m_) &&
    latest_nearest_obstacle_distance_m_ <= latest_dynamic_stop_distance_m_;
  if (std::isfinite(latest_nearest_obstacle_distance_m_)) {
    if (latest_nearest_obstacle_distance_m_ <=
      latest_dynamic_slowdown_distance_m_)
    {
      obstacle_slowdown_latched_ = true;
    } else if (latest_nearest_obstacle_distance_m_ >=
      std::max(
        obstacle_slowdown_exit_distance_m_,
        latest_dynamic_slowdown_distance_m_ + 0.15))
    {
      obstacle_slowdown_latched_ = false;
    }
  } else {
    obstacle_slowdown_latched_ = false;
  }
  if (robot_ball_distance <= ball_approach_distance_m_) {
    ball_approach_latched_ = true;
  } else if (robot_ball_distance >= ball_approach_exit_distance_m_) {
    ball_approach_latched_ = false;
  }
  const double ball_forward_projection =
    std::cos(robot_yaw) * robot_to_ball_x +
    std::sin(robot_yaw) * robot_to_ball_y;
  const double kick_lateral_error = std::fabs(
    (robot.pose.position.x - ball.pose.position.x) * (-uy) +
    (robot.pose.position.y - ball.pose.position.y) * ux);

  const double strict_yaw_tolerance = std::min(
    align_yaw_tolerance_, push_enter_yaw_error_rad_);
  const bool at_behind_pose = alignment_position_latched_;
  const bool alignment_position_ready =
    centered_on_kick_line &&
    at_behind_pose &&
    robot_ball_distance > ball_protection_radius_m_ &&
    kick_lateral_error <= strict_lateral_tolerance;
  const bool strict_kick_alignment =
    alignment_position_ready &&
    ball_forward_projection > 0.0 &&
    yaw_error <= strict_yaw_tolerance;

  bool alignment_reacquire = false;
  if (
    !drive_through_committed_ && alignment_position_ready &&
    yaw_error > strict_yaw_tolerance)
  {
    alignment_reacquire = alignmentWatchdogTriggered(robot_yaw, yaw_error);
  } else if (!drive_through_committed_) {
    resetAlignmentWatchdog();
  }

  const auto alignment_now = now();
  if (!drive_through_committed_ && at_behind_pose) {
    if (strict_kick_alignment) {
      alignment_bad_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      if (kick_alignment_since_.nanoseconds() <= 0) {
        kick_alignment_since_ = alignment_now;
      }
    } else if (kick_alignment_since_.nanoseconds() > 0) {
      if (alignment_bad_since_.nanoseconds() <= 0) {
        alignment_bad_since_ = alignment_now;
      }
      if ((alignment_now - alignment_bad_since_).seconds() >
        alignment_reset_grace_sec_)
              {
        kick_alignment_since_ = rclcpp::Time(
          0, 0, get_clock()->get_clock_type());
        alignment_bad_since_ = rclcpp::Time(
          0, 0, get_clock()->get_clock_type());
      }
    }
  } else if (!drive_through_committed_) {
    kick_alignment_since_ = rclcpp::Time(
      0, 0, get_clock()->get_clock_type());
    alignment_bad_since_ = rclcpp::Time(
      0, 0, get_clock()->get_clock_type());
  }

  const bool alignment_held =
    kick_alignment_since_.nanoseconds() > 0 &&
    (alignment_now - kick_alignment_since_).seconds() >=
    kick_alignment_hold_sec_;
  const bool alignment_ready = strict_kick_alignment && alignment_held;

  const double kick_corridor_radius = std::max(
    path_block_radius_m_,
    ball_protection_radius_m_ + 0.14);
  const double push_contact_offset_m = std::min(
    push_target_lead_m_, robot_front_extent_m_);

  auto assignPushTarget =
    [&](const double direction_x, const double direction_y)
    {
      // 尚未形成稳定前向接触时，只推进到球后方很近的位置以获取接触。确认
      // 稳定接触后，将机器人中心目标设在 kick_target 前方一段偏移处，使
      // 机器人前脸和球能够越过期望的射门目标点。
      const double target_offset_m =
        robot_front_extent_m_ + ball_radius_m_ - push_goal_crossing_margin_m_;
      approach = have_push_contact_ ? kick_target : ball;
      const double target_x =
        have_push_contact_ ?
        kick_target.pose.position.x - direction_x * target_offset_m :
        ball.pose.position.x + direction_x * (-push_contact_offset_m);
      const double target_y =
        have_push_contact_ ?
        kick_target.pose.position.y - direction_y * target_offset_m :
        ball.pose.position.y + direction_y * (-push_contact_offset_m);
      approach.pose.position.x = std::clamp(
        target_x,
        field_min_x_ + boundary_margin_m_,
        field_max_x_ - boundary_margin_m_);
      approach.pose.position.y = std::clamp(
        target_y,
        field_min_y_ + boundary_margin_m_,
        field_max_y_ - boundary_margin_m_);
      approach.pose.orientation = quaternionFromYaw(
        std::atan2(direction_y, direction_x));
    };

  auto pushCorridorBlocked =
    [&](const double direction_x, const double direction_y)
    {
      const double push_corridor_x =
        ball.pose.position.x + direction_x * push_target_lead_m_;
      const double push_corridor_y =
        ball.pose.position.y + direction_y * push_target_lead_m_;
      return isOpponentBlockingPathToPoint(
        robot.pose.position.x,
        robot.pose.position.y,
        push_corridor_x,
        push_corridor_y,
        opponents,
        kick_corridor_radius);
    };

  auto assignApproachTravelOrientation =
    [&](geometry_msgs::msg::PoseStamped & target)
    {
      // PoseStamped.orientation is the terminal heading, not the instantaneous
      // path bearing.  Encoding the current travel bearing here leaves an
      // already-active NavigateToPose action with a stale final yaw: after the
      // robot reaches the behind-ball point it first turns toward that old
      // bearing, reports success, and only then receives the kick heading.
      // The local/Nav2 controller already derives its travel heading from the
      // path, so every staging/approach goal must carry the kick direction as
      // its terminal orientation from the moment it is published.
      target.pose.orientation = quaternionFromYaw(desired_yaw);
    };

  std::string next_state = "NAV_TRANSIT";

  if (dynamic_emergency_stop || approach_unavailable) {
    next_state = "BLOCKED_RECOVERY";
  } else if (drive_through_committed_) {
    resetAlignmentWatchdog();
    assignPushTarget(drive_direction_x_, drive_direction_y_);

    const double push_yaw = std::atan2(drive_direction_y_, drive_direction_x_);
    const double push_yaw_error = yawDistance(robot_yaw, push_yaw);
    const double push_lateral_error = std::fabs(
      (robot.pose.position.x - ball.pose.position.x) * (-drive_direction_y_) +
      (robot.pose.position.y - ball.pose.position.y) * drive_direction_x_);

    const bool robot_still_behind = isRobotBehindAndAlignedForKick(
      robot.pose.position.x,
      robot.pose.position.y,
      ball.pose.position.x,
      ball.pose.position.y,
      ball.pose.position.x + drive_direction_x_,
      ball.pose.position.y + drive_direction_y_,
      0.02,
      push_exit_lateral_error_m_);

    const auto contact = computeBallContactMetrics(
      robot.pose.position.x,
      robot.pose.position.y,
      robot_yaw,
      robot_front_extent_m_,
      robot_rear_extent_m_,
      robot_half_width_m_,
      ball.pose.position.x,
      ball.pose.position.y,
      ball_radius_m_,
      contact_tolerance_m_);

    const auto push_now = now();
    if (!have_push_contact_progress_ ||
      contact.separation_m <=
      push_contact_best_separation_m_ - push_contact_progress_epsilon_m_)
    {
      push_contact_best_separation_m_ = contact.separation_m;
      last_push_contact_progress_time_ = push_now;
      have_push_contact_progress_ = true;
    }

    if (contact.front_contact) {
      last_push_contact_time_ = push_now;
      if (front_contact_since_.nanoseconds() <= 0) {
        front_contact_since_ = push_now;
      }
    } else {
      front_contact_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
    }

    const double contact_acquire_elapsed =
      contact_acquire_started_.nanoseconds() > 0 ?
      (push_now - contact_acquire_started_).seconds() : 0.0;
    const bool stable_front_contact =
      contact.front_contact &&
      front_contact_since_.nanoseconds() > 0 &&
      contact_acquire_elapsed >= contact_acquire_min_hold_sec_ &&
      (push_now - front_contact_since_).seconds() >=
      push_contact_stable_hold_sec_;
    if (stable_front_contact) {
      have_push_contact_ = true;
    }

    const double push_elapsed =
      drive_through_started_.nanoseconds() > 0 ?
      (push_now - drive_through_started_).seconds() : 0.0;
    const bool contact_acquire_window_elapsed =
      !have_push_contact_ &&
      push_elapsed > push_contact_acquire_timeout_sec_;
    const bool contact_progress_stalled =
      contact_acquire_window_elapsed &&
      (!have_push_contact_progress_ ||
      last_push_contact_progress_time_.nanoseconds() <= 0 ||
      (push_now - last_push_contact_progress_time_).seconds() >
      push_contact_progress_timeout_sec_);
    const bool contact_acquire_hard_expired =
      !have_push_contact_ &&
      push_elapsed > push_contact_hard_timeout_sec_;
    const bool contact_lost =
      have_push_contact_ &&
      last_push_contact_time_.nanoseconds() > 0 &&
      (push_now - last_push_contact_time_).seconds() > push_contact_loss_sec_;
    const bool alignment_lost_now =
      !robot_still_behind ||
      ball_forward_projection <= 0.0 ||
      push_lateral_error > push_realign_lateral_error_m_ ||
      push_yaw_error > push_exit_yaw_error_rad_;

    if (alignment_lost_now) {
      if (push_alignment_lost_since_.nanoseconds() <= 0) {
        push_alignment_lost_since_ = push_now;
      }
    } else {
      push_alignment_lost_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
    }

    const bool alignment_lost =
      push_alignment_lost_since_.nanoseconds() > 0 &&
      (push_now - push_alignment_lost_since_).seconds() >
      push_alignment_loss_sec_;
    const bool corridor_blocked =
      pushCorridorBlocked(drive_direction_x_, drive_direction_y_);
    const bool lateral_realign_requested =
      push_lateral_error > push_realign_lateral_error_m_;

    if (corridor_blocked) {
      RCLCPP_WARN(
        get_logger(),
        "football push paused: corridor_blocked nearest=%.3f stop=%.3f",
        latest_nearest_obstacle_distance_m_,
        latest_dynamic_stop_distance_m_);
      resetGoalStability();
      approach = behind;
      next_state = "BLOCKED_RECOVERY";
    } else if (push_realign_active_ || lateral_realign_requested) {
      if (!push_realign_active_) {
        RCLCPP_WARN(
          get_logger(),
          "football push lateral realign: error=%.3f enter=%.3f trigger=%.3f",
          push_lateral_error,
          push_enter_lateral_error_m_,
          push_realign_lateral_error_m_);
        push_realign_active_ = true;
        have_push_contact_ = false;
        have_push_contact_progress_ = false;
        front_contact_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        last_push_contact_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      }

      const bool lateral_alignment_restored =
        push_lateral_error <= push_enter_lateral_error_m_ &&
        push_yaw_error <= push_exit_yaw_error_rad_ &&
        robot_still_behind && ball_forward_projection > 0.0;
      if (lateral_alignment_restored) {
        push_realign_active_ = false;
        assignPushTarget(drive_direction_x_, drive_direction_y_);
        contact_acquire_started_ = push_now;
        drive_through_started_ = push_now;
        last_push_contact_progress_time_ = push_now;
        push_contact_best_separation_m_ = std::numeric_limits<double>::infinity();
        push_alignment_lost_since_ = rclcpp::Time(
          0, 0, get_clock()->get_clock_type());
        next_state = "CONTACT_ACQUIRE";
      } else {
        // Preserve the current longitudinal ball separation and remove only
        // the lateral offset. This produces a short left/right correction
        // instead of sending the striker back to the normal behind-ball pose.
        const double longitudinal_from_ball =
          (robot.pose.position.x - ball.pose.position.x) * drive_direction_x_ +
          (robot.pose.position.y - ball.pose.position.y) * drive_direction_y_;
        const double behind_separation = std::clamp(
          -longitudinal_from_ball,
          push_contact_offset_m,
          approach_config_.approach_distance);
        approach = ball;
        approach.pose.position.x = std::clamp(
          ball.pose.position.x - drive_direction_x_ * behind_separation,
          field_min_x_ + boundary_margin_m_,
          field_max_x_ - boundary_margin_m_);
        approach.pose.position.y = std::clamp(
          ball.pose.position.y - drive_direction_y_ * behind_separation,
          field_min_y_ + boundary_margin_m_,
          field_max_y_ - boundary_margin_m_);
        approach.pose.orientation = quaternionFromYaw(push_yaw);
        next_state = "PUSH_REALIGN";
      }
    } else if (contact_acquire_hard_expired ||
      contact_progress_stalled || contact_lost || alignment_lost)
    {
      RCLCPP_WARN(
        get_logger(),
        "football push reacquire: hard_timeout=%d progress_stall=%d "
        "contact_lost=%d alignment_lost=%d",
        contact_acquire_hard_expired,
        contact_progress_stalled,
        contact_lost,
        alignment_lost);
      resetPushCommitment(behind_error <= approach_reached_exit_m_);
      kick_alignment_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      alignment_bad_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      approach = behind;
      next_state = "ALIGN_TO_GOAL";
    } else if (have_push_contact_) {
      next_state = "PUSH_BALL";
    } else {
      // CONTACT_ACQUIRE is deliberately latched. A single false contact
      // sample does not return to ALIGN; the robot advances at a bounded
      // straight-line speed until stable front contact, finite timeout,
      // corridor blockage, or sustained alignment loss.
      next_state = "CONTACT_ACQUIRE";
    }
  } else if (at_behind_pose && !alignment_position_ready) {
    // A zero-translation ALIGN goal is valid only while the complete
    // behind-ball contact geometry is still valid. Otherwise Nav2 sees a
    // one-pose path at the robot and repeatedly reports success without
    // restoring the position required to contact the ball.
    alignment_position_latched_ = false;
    have_alignment_anchor_ = false;
    kick_alignment_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    alignment_bad_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    approach = behind;
    assignApproachTravelOrientation(approach);
    next_state = "BALL_APPROACH";
    resetAlignmentWatchdog();
  } else if (alignment_reacquire) {
    alignment_position_latched_ = false;
    have_alignment_anchor_ = false;
    kick_alignment_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    alignment_bad_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    const double error_sign =
      signedYawError(desired_yaw, robot_yaw) >= 0.0 ? 1.0 : -1.0;
    approach = behind;
    approach.pose.position.x = std::clamp(
      behind.pose.position.x - error_sign * uy * alignment_reacquire_lateral_m_,
      field_min_x_ + approach_boundary_margin_m_,
      field_max_x_ - approach_boundary_margin_m_);
    approach.pose.position.y = std::clamp(
      behind.pose.position.y + error_sign * ux * alignment_reacquire_lateral_m_,
      field_min_y_ + approach_boundary_margin_m_,
      field_max_y_ - approach_boundary_margin_m_);
    assignApproachTravelOrientation(approach);
    next_state = "BALL_APPROACH";
    resetAlignmentWatchdog();
  } else if (blocked) {
    alignment_position_latched_ = false;
    have_alignment_anchor_ = false;
    kick_alignment_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    alignment_bad_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    approach = plan.pose;
    assignApproachTravelOrientation(approach);
    next_state = "OBSTACLE_APPROACH";
  } else if (!at_behind_pose) {
    approach = behind;
    assignApproachTravelOrientation(approach);
    if (obstacle_slowdown_latched_) {
      next_state = "OBSTACLE_APPROACH";
    } else if (ball_approach_latched_) {
      next_state = "BALL_APPROACH";
    } else {
      next_state = "NAV_TRANSIT";
    }
  } else if (!alignment_ready) {
    // Freeze the ALIGN target position in the stable field frame. Only yaw
    // remains the kick direction, so the controller cannot keep translating
    // or chase a target that rotates with base_link.
    approach = have_alignment_anchor_ ? alignment_anchor_field_ : robot;
    approach.header = behind.header;
    approach.pose.orientation = behind.pose.orientation;
    next_state = "ALIGN_TO_GOAL";
  } else {
    assignPushTarget(ux, uy);
    const bool corridor_blocked = pushCorridorBlocked(ux, uy);

    if (corridor_blocked) {
      RCLCPP_WARN(
        get_logger(),
        "football contact delayed: corridor_blocked nearest=%.3f stop=%.3f",
        latest_nearest_obstacle_distance_m_,
        latest_dynamic_stop_distance_m_);
      kick_alignment_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      alignment_bad_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      approach = behind;
      next_state = "BLOCKED_RECOVERY";
    } else {
      drive_direction_x_ = ux;
      drive_direction_y_ = uy;
      drive_through_committed_ = true;
      resetAlignmentWatchdog();
      have_push_contact_ = false;
      have_push_contact_progress_ = false;
      push_contact_best_separation_m_ = std::numeric_limits<double>::infinity();
      drive_through_started_ = now();
      contact_acquire_started_ = drive_through_started_;
      front_contact_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      last_push_contact_time_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      last_push_contact_progress_time_ = drive_through_started_;
      push_alignment_lost_since_ = rclcpp::Time(
        0, 0, get_clock()->get_clock_type());
      next_state = "CONTACT_ACQUIRE";
    }
  }

  setControlState(next_state, true);

  if (!shouldPublishApproach(approach, next_state)) {
    markProcessed();
    return;
  }

  geometry_msgs::msg::PoseStamped local;
  // 上述状态机目标始终在稳定的 field_frame 中计算；下游跟踪接口要求目标
  // 相对于 base_link 表达，因此发布前在这里转换为局部目标。
  if (!transformFromFieldToOutput(approach, local)) {
    resetGoalStability();
    setControlState("SAFE_STOP", false);
    markProcessed();
    return;
  }

  const double local_distance = std::hypot(
    local.pose.position.x,
    local.pose.position.y);
  // max_local_goal_distance_m bounds robot-relative goals. Applying it to a
  // field-frame target incorrectly projects valid global coordinates onto a
  // circle around the field origin (for example 7.65 m -> 4.00 m).
  if (output_frame_ != field_frame_ && local_distance > max_local_goal_distance_m_) {
    const double scale = max_local_goal_distance_m_ / local_distance;
    local.pose.position.x *= scale;
    local.pose.position.y *= scale;
    // Keep the state-machine yaw. Replacing it with the bearing of the
    // clamped local vector would re-introduce a robot-relative yaw target.
  }

  local.header.stamp = msg->header.stamp;
  approach_pose_pub_->publish(local);
  markProcessed();
}

bool FootballGoalAdapter::updateNoProgress(
  const geometry_msgs::msg::PoseStamped & robot,
  const geometry_msgs::msg::PoseStamped & ball,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<OpponentPoint2D> & opponents)
{
  if (!have_progress_anchor_) {
    progress_anchor_time_ = now();
    progress_anchor_robot_ = robot;
    progress_anchor_ball_ = ball;
    have_progress_anchor_ = true;
    return false;
  }
  NoProgressMetrics metrics;
  metrics.elapsed_sec = (now() - progress_anchor_time_).seconds();
  if (metrics.elapsed_sec < no_progress_config_.window_sec) {
    return false;
  }
  const double anchor_ball = planarDistance(
    progress_anchor_robot_.pose.position.x, progress_anchor_robot_.pose.position.y,
    progress_anchor_ball_.pose.position.x, progress_anchor_ball_.pose.position.y);
  metrics.ball_distance = planarDistance(
    robot.pose.position.x, robot.pose.position.y,
    ball.pose.position.x, ball.pose.position.y);
  metrics.ball_distance_improvement = anchor_ball - metrics.ball_distance;
  metrics.goal_distance_improvement = planarDistance(
    progress_anchor_robot_.pose.position.x, progress_anchor_robot_.pose.position.y,
    goal.pose.position.x, goal.pose.position.y) - planarDistance(
    robot.pose.position.x, robot.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);
  metrics.path_progress = planarDistance(
    progress_anchor_robot_.pose.position.x, progress_anchor_robot_.pose.position.y,
    robot.pose.position.x, robot.pose.position.y);
  metrics.actual_speed = std::hypot(
    synchronized_odom_.twist.twist.linear.x, synchronized_odom_.twist.twist.linear.y);
  metrics.command_speed = last_cmd_vel_time_.nanoseconds() > 0 &&
    (now() - last_cmd_vel_time_).seconds() <= cmd_vel_timeout_sec_ ?
    latest_command_linear_speed_ : 0.0;
  metrics.ball_motion = planarDistance(
    progress_anchor_ball_.pose.position.x, progress_anchor_ball_.pose.position.y,
    ball.pose.position.x, ball.pose.position.y);
  metrics.obstacle_distance = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : opponents) {
    metrics.obstacle_distance = std::min(metrics.obstacle_distance, planarDistance(
      robot.pose.position.x, robot.pose.position.y, obstacle.first, obstacle.second));
  }
  const bool stalled = isNoProgress(metrics, no_progress_config_);
  stalled_window_count_ = stalled ? stalled_window_count_ + 1 : 0;
  progress_anchor_time_ = now();
  progress_anchor_robot_ = robot;
  progress_anchor_ball_ = ball;
  return stalled;
}

void FootballGoalAdapter::resetProgressWatchdog()
{
  have_progress_anchor_ = false;
  stalled_window_count_ = 0;
}

void FootballGoalAdapter::resetAlignmentWatchdog()
{
  alignment_watchdog_active_ = false;
  alignment_last_yaw_ = 0.0;
  alignment_accumulated_rotation_rad_ = 0.0;
  alignment_window_start_error_rad_ = 0.0;
  alignment_best_error_rad_ = 0.0;
  alignment_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  alignment_progress_window_started_ = rclcpp::Time(
    0, 0, get_clock()->get_clock_type());
}

bool FootballGoalAdapter::alignmentWatchdogTriggered(
  const double robot_yaw,
  const double yaw_error)
{
  const auto stamp = now();
  if (!alignment_watchdog_active_) {
    alignment_watchdog_active_ = true;
    alignment_last_yaw_ = robot_yaw;
    alignment_accumulated_rotation_rad_ = 0.0;
    alignment_window_start_error_rad_ = yaw_error;
    alignment_best_error_rad_ = yaw_error;
    alignment_started_ = stamp;
    alignment_progress_window_started_ = stamp;
    return false;
  }

  alignment_accumulated_rotation_rad_ += std::fabs(
    signedYawError(robot_yaw, alignment_last_yaw_));
  alignment_last_yaw_ = robot_yaw;
  alignment_best_error_rad_ = std::min(alignment_best_error_rad_, yaw_error);

  const bool duration_exceeded =
    (stamp - alignment_started_).seconds() > alignment_max_duration_sec_;
  const bool rotation_exceeded =
    alignment_accumulated_rotation_rad_ > alignment_max_rotation_rad_;

  bool progress_stalled = false;
  if ((stamp - alignment_progress_window_started_).seconds() >=
    alignment_progress_window_sec_)
  {
    const double improvement =
      alignment_window_start_error_rad_ - alignment_best_error_rad_;
    progress_stalled = improvement < alignment_min_error_improvement_rad_;
    alignment_progress_window_started_ = stamp;
    alignment_window_start_error_rad_ = yaw_error;
    alignment_best_error_rad_ = yaw_error;
  }

  if (duration_exceeded || rotation_exceeded || progress_stalled) {
    RCLCPP_WARN(
      get_logger(),
      "ALIGN watchdog reacquire: elapsed=%.2f accumulated_rotation=%.2f "
      "yaw_error=%.3f stalled=%s",
      (stamp - alignment_started_).seconds(),
      alignment_accumulated_rotation_rad_,
      yaw_error,
      progress_stalled ? "true" : "false");
    return true;
  }
  return false;
}

void FootballGoalAdapter::resetPushCommitment(const bool keep_alignment_position)
{
  drive_through_committed_ = false;
  push_realign_active_ = false;
  drive_direction_x_ = 1.0;
  drive_direction_y_ = 0.0;
  have_push_contact_ = false;
  have_push_contact_progress_ = false;
  push_contact_best_separation_m_ = 0.0;
  drive_through_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  contact_acquire_started_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  front_contact_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_push_contact_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_push_contact_progress_time_ = rclcpp::Time(
    0, 0, get_clock()->get_clock_type());
  push_alignment_lost_since_ = rclcpp::Time(
    0, 0, get_clock()->get_clock_type());
  alignment_position_latched_ = keep_alignment_position;
  if (!keep_alignment_position) {
    have_alignment_anchor_ = false;
  }
}

void FootballGoalAdapter::resetGoalStability()
{
  resetProgressWatchdog();
  resetAlignmentWatchdog();
  have_published_approach_ = false;
  last_approach_state_.clear();
  obstacle_slowdown_latched_ = false;
  ball_approach_latched_ = false;
  resetPushCommitment(false);
  kick_alignment_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  alignment_bad_since_ = rclcpp::Time(
    0, 0, get_clock()->get_clock_type());
}

bool FootballGoalAdapter::shouldPublishApproach(
  const geometry_msgs::msg::PoseStamped & field_goal,
  const std::string & state)
{
  const auto stamp = now();
  bool due = !have_published_approach_ || state != last_approach_state_;

  if (!due) {
    const double elapsed = (stamp - last_approach_publish_time_).seconds();
    if (elapsed < goal_update_min_hold_sec_) {
      return false;
    }

    double current_yaw = 0.0;
    double previous_yaw = 0.0;
    if (!yawFromQuaternion(field_goal.pose.orientation, current_yaw) ||
      !yawFromQuaternion(last_published_approach_field_.pose.orientation, previous_yaw))
    {
      return false;
    }

    due = planarDistance(
      field_goal.pose.position.x,
      field_goal.pose.position.y,
      last_published_approach_field_.pose.position.x,
      last_published_approach_field_.pose.position.y) >=
      goal_update_position_threshold_m_ ||
      yawDistance(current_yaw, previous_yaw) >= goal_update_yaw_threshold_rad_ ||
      elapsed >= goal_update_max_refresh_sec_;
  }

  if (due) {
    last_published_approach_field_ = field_goal;
    last_approach_state_ = state;
    last_approach_publish_time_ = stamp;
    have_published_approach_ = true;
  }

  return due;
}

double FootballGoalAdapter::nearestOpponentDistance(
  const geometry_msgs::msg::PoseStamped & robot,
  const std::vector<OpponentPoint2D> & opponents) const
{
  (void)opponents;
  double nearest = std::numeric_limits<double>::infinity();
  double robot_yaw = 0.0;
  if (!yawFromQuaternion(robot.pose.orientation, robot_yaw)) {
    return nearest;
  }
  const auto collision_ellipse = makeCircumscribedCollisionEllipse(
    robot_collision_length_m_, robot_collision_width_m_, collision_ellipse_expansion_m_);
  const auto stamp = now();
  for (const auto & entry : latest_other_robot_odoms_) {
    const auto received = latest_other_robot_times_.find(entry.first);
    double obstacle_yaw = 0.0;
    if (received == latest_other_robot_times_.end() ||
      (stamp - received->second).seconds() > other_robot_timeout_sec_ ||
      !yawFromQuaternion(entry.second.pose.pose.orientation, obstacle_yaw))
    {
      continue;
    }
    nearest = std::min(nearest, orientedEllipseClearance(
      robot.pose.position.x, robot.pose.position.y, robot_yaw, collision_ellipse,
      entry.second.pose.pose.position.x, entry.second.pose.pose.position.y,
      obstacle_yaw, collision_ellipse));
  }
  return nearest;
}

double FootballGoalAdapter::speedLimitForState(const std::string & state) const
{
  if (state == "NAV_TRANSIT") {
    // Nav2 uses 0.0 as NO_SPEED_LIMIT. Publishing the configured base maximum
    // does not restore DWB after a lower absolute limit because its setter
    // only applies values strictly below that maximum.
    return 0.0;
  }
  if (state == "OBSTACLE_APPROACH") {
    if (!std::isfinite(latest_nearest_obstacle_distance_m_)) {
      return contact_acquire_speed_limit_mps_;
    }
    const double stop_distance = std::isfinite(latest_dynamic_stop_distance_m_) ?
      latest_dynamic_stop_distance_m_ : obstacle_stop_distance_m_;
    const double slowdown_distance = std::max(
      obstacle_slowdown_distance_m_,
      std::isfinite(latest_dynamic_slowdown_distance_m_) ?
      latest_dynamic_slowdown_distance_m_ : obstacle_slowdown_distance_m_);
    const double ratio = std::clamp(
      (latest_nearest_obstacle_distance_m_ - stop_distance) /
      std::max(1e-6, slowdown_distance - stop_distance),
      0.0,
      1.0);
    const double interpolated = contact_acquire_speed_limit_mps_ +
      ratio * (obstacle_speed_limit_mps_ - contact_acquire_speed_limit_mps_);
    const double braking_available = std::max(
      0.0,
      latest_nearest_obstacle_distance_m_ - stop_distance);
    const double braking_limited_speed = std::sqrt(
      2.0 * obstacle_braking_deceleration_mps2_ * braking_available);
    return std::clamp(
      std::min(interpolated, braking_limited_speed),
      contact_acquire_speed_limit_mps_,
      obstacle_speed_limit_mps_);
  }
  if (state == "BALL_APPROACH" || state == "APPROACH_BEHIND_BALL") {
    return ball_approach_speed_limit_mps_;
  }
  if (state == "CONTACT_ACQUIRE" || state == "PUSH_REALIGN") {
    return contact_acquire_speed_limit_mps_;
  }
  if (state == "PUSH_BALL") {
    return push_speed_limit_mps_;
  }
  if (state == "AVOID_BLOCKER") {
    return obstacle_speed_limit_mps_;
  }
  if (state == "BLOCKED_RECOVERY") {
    return contact_acquire_speed_limit_mps_;
  }

  // ALIGN_TO_GOAL intentionally publishes Nav2 NO_SPEED_LIMIT (0.0).
  // The target remains at the latched behind-ball position and the
  // RotateToGoal critic produces zero translation while retaining enough
  // angular authority to converge by the shortest wrapped yaw error.
  return 0.0;
}

void FootballGoalAdapter::publishSpeedLimit(const std::string & state)
{
  if (!speed_limit_pub_) {
    return;
  }

  const auto stamp = now();
  const double requested_limit = speedLimitForState(state);
  const bool first_publish = !std::isfinite(last_published_speed_limit_mps_);
  const bool changed = first_publish ||
    std::fabs(requested_limit - last_published_speed_limit_mps_) >=
    speed_limit_update_threshold_mps_;
  const bool refresh_due =
    last_speed_limit_publish_time_.nanoseconds() <= 0 ||
    (stamp - last_speed_limit_publish_time_).seconds() >= speed_limit_refresh_sec_;
  if (!changed && !refresh_due) {
    return;
  }

  nav2_msgs::msg::SpeedLimit message;
  message.percentage = false;
  message.speed_limit = requested_limit;
  speed_limit_pub_->publish(message);
  last_published_speed_limit_mps_ = requested_limit;
  last_speed_limit_publish_time_ = stamp;
}

void FootballGoalAdapter::setControlState(const std::string & state, const bool valid)
{
  const bool changed = state != control_state_ || !have_published_control_state_ ||
    valid != last_published_control_valid_;
  control_state_ = state;
  control_valid_ = valid;
  // Speed can change continuously inside OBSTACLE_APPROACH even when the
  // symbolic state does not change, so the publisher performs its own
  // value/refresh throttling on every control update.
  publishSpeedLimit(state);
  if (changed) {
    RCLCPP_INFO(
      get_logger(),
      "football state=%s valid=%d ball_distance=%.3f nearest_obstacle=%.3f "
      "dynamic_stop=%.3f",
      state.c_str(),
      valid,
      latest_robot_ball_distance_m_,
      latest_nearest_obstacle_distance_m_,
      latest_dynamic_stop_distance_m_);
  }
  if (!changed && (now() - last_state_publish_time_).seconds() < 1.0) {
    return;
  }
  std_msgs::msg::Bool valid_message;
  valid_message.data = valid;
  control_valid_pub_->publish(valid_message);
  std_msgs::msg::String state_message;
  state_message.data = state;
  state_pub_->publish(state_message);
  have_published_control_state_ = true;
  last_published_control_valid_ = valid;
  last_state_publish_time_ = now();
}

void FootballGoalAdapter::publishLocalizationValid(const bool valid)
{
  std_msgs::msg::Bool message;
  message.data = valid;
  localization_valid_pub_->publish(message);
}

}  // namespace football_navigation
