// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#ifndef FOOTBALL_NAVIGATION__FOOTBALL_GOAL_ADAPTER_HPP_
#define FOOTBALL_NAVIGATION__FOOTBALL_GOAL_ADAPTER_HPP_

#include <deque>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "football_navigation/core/football_geometry.hpp"

namespace football_navigation
{

class FootballGoalAdapter : public rclcpp::Node
{
public:
  FootballGoalAdapter();

private:
  void ballPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void kickTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void strikerCallback(const std_msgs::msg::String::SharedPtr msg);
  void roleCallback(const std_msgs::msg::String::SharedPtr msg);
  void tacticalTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void goalEventCallback(const std_msgs::msg::String::SharedPtr msg);
  void matchStateCallback(const std_msgs::msg::String::SharedPtr msg);
  void odomGlobalCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void otherRobotOdomCallback(
    const std::string & robot_namespace,
    const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void safetyWatchdog();
  bool inKickoffHold(const rclcpp::Time & stamp) const;
  bool matchStateAllowsMovement() const;
  bool isSelfStriker() const;
  bool hasActiveTacticalRole() const;
  bool publishTacticalTarget();
  bool strikerAssignmentFresh() const;
  bool kickTargetFresh() const;
  bool otherRobotPosesFresh() const;
  bool findOdomAt(
    const rclcpp::Time & stamp, nav_msgs::msg::Odometry & output, double max_skew_sec) const;
  std::vector<OpponentPoint2D> collectOpponentPositions();
  bool transformToFieldFrame(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output);
  bool transformFromFieldToOutput(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output);
  bool lookupRobotPoseInFieldFrame(geometry_msgs::msg::PoseStamped & output);
  bool isBallSafelyReachable(
    const geometry_msgs::msg::PoseStamped & ball_pose,
    const geometry_msgs::msg::PoseStamped & robot_pose);
  bool processingDue() const;
  void markProcessed();
  bool validateBallPose(const geometry_msgs::msg::PoseStamped & pose);
  bool validFinitePose(const geometry_msgs::msg::Pose & pose) const;
  bool odomGlobalFresh() const;
  bool updateNoProgress(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & ball_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const std::vector<OpponentPoint2D> & opponents);
  void resetProgressWatchdog();
  void resetAlignmentWatchdog();
  bool alignmentWatchdogTriggered(double robot_yaw, double yaw_error);
  void resetPushCommitment(bool keep_alignment_position);
  double nearestOpponentDistance(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const std::vector<OpponentPoint2D> & opponents) const;
  void updateOpponentKinematics(
    const std::vector<OpponentPoint2D> & positions,
    const nav_msgs::msg::Odometry & robot_odom,
    const rclcpp::Time & stamp);
  double speedLimitForState(const std::string & state) const;
  void publishSpeedLimit(const std::string & state);
  void setControlState(const std::string & state, bool valid);
  void publishLocalizationValid(bool valid);
  void resetGoalStability();
  bool shouldPublishApproach(
    const geometry_msgs::msg::PoseStamped & field_goal,
    const std::string & state);

  std::string output_frame_;
  std::string field_frame_;
  std::string base_frame_;
  std::string ball_pose_topic_;
  std::string kick_target_topic_;
  std::string approach_pose_topic_;
  std::string team_id_;
  std::string self_namespace_;
  std::string striker_topic_;
  std::string role_topic_;
  std::string tactical_target_topic_;
  std::string goal_event_topic_;
  std::string match_state_topic_;
  std::string match_state_{"STOP"};
  std::string odom_global_topic_;
  std::string state_topic_;
  std::string control_valid_topic_;
  std::string localization_valid_topic_;
  std::string robot_namespaces_csv_;
  std::string robot_odom_topic_template_;
  std::vector<std::string> robot_namespaces_;
  std::string cmd_vel_topic_;
  std::string speed_limit_topic_;
  bool require_striker_role_{false};
  bool use_team_kick_target_{false};
  bool is_striker_{true};
  bool have_striker_assignment_{false};
  bool have_tactical_target_{false};
  std::string tactical_role_{"STOP"};
  bool require_other_robot_poses_{true};
  double path_block_radius_m_{0.50};
  LateralEntryConfig lateral_entry_config_;
  bool have_kick_target_;
  geometry_msgs::msg::PoseStamped latest_kick_target_;
  double ball_timeout_sec_;
  double max_ball_age_sec_;
  double future_tolerance_sec_;
  double max_ball_speed_mps_;
  double max_ball_jump_m_;
  double odom_timeout_sec_;
  double max_ball_odom_skew_sec_;
  double kick_target_timeout_sec_;
  double role_timeout_sec_;
  double tactical_target_timeout_sec_{0.80};
  double other_robot_timeout_sec_{0.70};
  int minimum_other_robot_count_{9};
  double approach_reached_m_;
  double approach_reached_exit_m_;
  double align_yaw_tolerance_;
  double alignment_reset_grace_sec_;
  double alignment_max_duration_sec_;
  double alignment_max_rotation_rad_;
  double alignment_progress_window_sec_;
  double alignment_min_error_improvement_rad_;
  double alignment_reacquire_lateral_m_;
  double approach_speed_limit_mps_;
  double obstacle_speed_limit_mps_;
  double ball_approach_speed_limit_mps_;
  double contact_acquire_speed_limit_mps_;
  double push_speed_limit_mps_;
  double robot_collision_length_m_{0.562};
  double robot_collision_width_m_{0.339};
  double collision_ellipse_expansion_m_{0.05};
  double obstacle_slowdown_distance_m_;
  double obstacle_slowdown_exit_distance_m_;
  double obstacle_stop_distance_m_;
  double obstacle_reaction_time_sec_;
  double obstacle_braking_deceleration_mps2_;
  double obstacle_prediction_horizon_sec_;
  double ball_approach_distance_m_;
  double ball_approach_exit_distance_m_;
  double speed_limit_update_threshold_mps_;
  double speed_limit_refresh_sec_;
  double push_target_lead_m_;
  double push_enter_lateral_error_m_;
  double push_exit_lateral_error_m_;
  double push_enter_yaw_error_rad_;
  double push_exit_yaw_error_rad_;
  double push_contact_acquire_timeout_sec_;
  double contact_acquire_min_hold_sec_;
  double push_contact_stable_hold_sec_;
  double push_contact_progress_timeout_sec_;
  double push_contact_hard_timeout_sec_;
  double push_contact_progress_epsilon_m_;
  double push_alignment_loss_sec_;
  double push_contact_loss_sec_;
  double robot_front_extent_m_;
  double robot_rear_extent_m_;
  double robot_half_width_m_;
  double ball_radius_m_;
  double contact_tolerance_m_;
  double push_goal_crossing_margin_m_;
  double ball_protection_radius_m_;
  double max_local_goal_distance_m_;
  double boundary_margin_m_;
  double ball_boundary_margin_m_;
  double approach_boundary_margin_m_;
  double maximum_ball_chase_distance_m_;
  double minimum_kick_target_distance_m_;
  double minimum_behind_alignment_m_;
  double maximum_kick_lateral_error_m_;
  double kick_alignment_hold_sec_{0.35};
  double cmd_vel_timeout_sec_;
  double goal_update_min_hold_sec_{0.25};
  double goal_update_position_threshold_m_{0.08};
  double goal_update_yaw_threshold_rad_{0.10};
  double goal_update_max_refresh_sec_{1.0};
  double field_min_x_;
  double field_max_x_;
  double field_min_y_;
  double field_max_y_;
  double update_rate_hz_;
  ApproachConfig approach_config_;
  NoProgressConfig no_progress_config_;
  bool have_last_output_time_;
  rclcpp::Time last_output_time_;
  rclcpp::Time kickoff_hold_until_;
  rclcpp::Time last_ball_stamp_;
  rclcpp::Time last_kick_target_stamp_;
  rclcpp::Time last_striker_assignment_time_;
  rclcpp::Time last_tactical_role_time_;
  rclcpp::Time last_tactical_target_stamp_;
  rclcpp::Time last_valid_ball_time_;
  rclcpp::Time latest_odom_time_;
  rclcpp::Time last_robot_pose_time_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time progress_anchor_time_;
  rclcpp::Time recovery_hold_until_;
  rclcpp::Time kick_alignment_since_;
  rclcpp::Time alignment_bad_since_;
  rclcpp::Time alignment_started_;
  rclcpp::Time alignment_progress_window_started_;
  rclcpp::Time drive_through_started_;
  rclcpp::Time contact_acquire_started_;
  rclcpp::Time front_contact_since_;
  rclcpp::Time last_push_contact_time_;
  rclcpp::Time last_push_contact_progress_time_;
  rclcpp::Time push_alignment_lost_since_;
  geometry_msgs::msg::PoseStamped last_ball_in_field_;
  geometry_msgs::msg::PoseStamped latest_tactical_target_;
  geometry_msgs::msg::PoseStamped progress_anchor_ball_;
  geometry_msgs::msg::PoseStamped progress_anchor_robot_;
  geometry_msgs::msg::PoseStamped alignment_anchor_field_;
  geometry_msgs::msg::PoseStamped last_published_approach_field_;
  nav_msgs::msg::Odometry latest_odom_global_;
  nav_msgs::msg::Odometry synchronized_odom_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::size_t odom_history_limit_{30};
  std::map<std::string, nav_msgs::msg::Odometry> latest_other_robot_odoms_;
  std::map<std::string, rclcpp::Time> latest_other_robot_times_;
  std::vector<OpponentPoint2D> previous_opponents_field_;
  rclcpp::Time previous_opponents_stamp_;
  bool have_last_ball_{false};
  bool have_odom_global_{false};
  bool control_valid_{false};
  bool last_published_control_valid_{false};
  bool have_published_control_state_{false};
  bool have_synchronized_odom_{false};
  bool have_progress_anchor_{false};
  bool have_alignment_anchor_{false};
  bool obstacle_slowdown_latched_{false};
  bool ball_approach_latched_{false};
  bool have_published_approach_{false};
  double latest_command_speed_{0.0};
  double latest_command_linear_speed_{0.0};
  double latest_command_angular_speed_{0.0};
  int stalled_window_count_{0};
  std::string control_state_{"SEARCH_BALL"};
  rclcpp::Time last_state_publish_time_;
  rclcpp::Time last_speed_limit_publish_time_;
  rclcpp::Time last_approach_publish_time_;
  std::string last_approach_state_;
  double kickoff_hold_sec_{3.0};

  bool drive_through_committed_{false};
  bool alignment_position_latched_{false};
  bool alignment_watchdog_active_{false};
  double alignment_last_yaw_{0.0};
  double alignment_accumulated_rotation_rad_{0.0};
  double alignment_window_start_error_rad_{0.0};
  double alignment_best_error_rad_{0.0};
  double drive_direction_x_{1.0};
  double drive_direction_y_{0.0};
  bool have_push_contact_{false};
  bool have_push_contact_progress_{false};
  double push_contact_best_separation_m_{0.0};
  double latest_robot_ball_distance_m_{std::numeric_limits<double>::infinity()};
  double latest_nearest_obstacle_distance_m_{std::numeric_limits<double>::infinity()};
  double latest_max_closing_speed_mps_{0.0};
  double latest_dynamic_stop_distance_m_{std::numeric_limits<double>::infinity()};
  double latest_dynamic_slowdown_distance_m_{std::numeric_limits<double>::infinity()};
  double last_published_speed_limit_mps_{std::numeric_limits<double>::quiet_NaN()};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr kick_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr role_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tactical_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_global_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> other_robot_odom_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr approach_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
};

}  // namespace football_navigation

#endif  // FOOTBALL_NAVIGATION__FOOTBALL_GOAL_ADAPTER_HPP_
