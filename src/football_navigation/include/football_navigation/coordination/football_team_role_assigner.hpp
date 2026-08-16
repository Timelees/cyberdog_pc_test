// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballTeamRoleAssigner : public rclcpp::Node
{
public:
  FootballTeamRoleAssigner();

private:

private:
  struct OdomState
  {
    nav_msgs::msg::Odometry odom;
    rclcpp::Time stamp;
    rclcpp::Time received;
  };

  struct RoleCommand
  {
    std::string role{"STOP"};
    double x{0.0};
    double y{0.0};
  };
  rclcpp::Time zeroTime() const;
  void validateTeamRosters() const;
  bool finitePose(const geometry_msgs::msg::Pose & pose) const;
  bool ballFresh(const rclcpp::Time & stamp) const;
  void ballCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odomCallback(const std::string & robot_id, const nav_msgs::msg::Odometry::SharedPtr msg);
  void authorityCallback(const std_msgs::msg::String::SharedPtr msg);
  void resetPendingChallenger(std::string & challenger, rclcpp::Time & since);
  void resetSelections();
  void startKickoff(const std::string & kickoff_state);
  void goalEventCallback(const std_msgs::msg::String::SharedPtr msg);
  void matchStateCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  static bool validMatchState(const std::string & state);
  bool matchStateAllowsMovement(const std::string & team_id) const;
  void publishMatchState();
  bool authorityConflictActive() const;
  RobotPose2D robotPose(const std::string & id) const;
  std::vector<RobotPose2D> teamPoses(const std::vector<std::string> & ids) const;
  bool poseSkewAcceptable(const std::vector<RobotPose2D> & robots) const;
  static bool currentStrikerFresh(
  const std::vector<RobotPose2D> & team, const std::string & current);
  static int countValidRobots(const std::vector<RobotPose2D> & team);
  void publishAuthority();
  void publishStrikers();
  void publishKickTargets();
  double clampTacticalX(const double x) const;
  double clampTacticalY(const double y) const;
  void publishRole(
  const std::string & robot_id, const std::string & role,
  const double x, const double y);
  void publishTeamTactics(
  const std::vector<RobotPose2D> & team, const std::string & team_id,
  const std::string & striker);
  void publishOtherRobots(const std::vector<RobotPose2D> & all);
  std::string selectNearestStriker(
  const std::vector<RobotPose2D> & robots, const std::string & current,
  rclcpp::Time & current_since, std::string & challenger,
  rclcpp::Time & challenger_since, const rclcpp::Time & stamp);
  void publishStatus(const std::string & prefix);
  void stopAllRoles(const std::string & reason);
  void clearRoles(const std::string & reason);
  void clearTeamSelection(
  std::string & current, rclcpp::Time & current_since,
  std::string & challenger, rclcpp::Time & challenger_since);
  void updateSelections(
  const std::vector<RobotPose2D> & team_a,
  const std::vector<RobotPose2D> & team_b,
  const rclcpp::Time & stamp);
  void captureKickoffReference();
  void update();

  std::string field_frame_;
  std::string ball_topic_;
  std::string odom_source_mode_;
  std::string odom_topic_template_;
  std::string authority_id_;
  std::string authority_instance_id_;
  std::string authority_topic_;
  std::string match_state_topic_;
  std::string match_state_command_topic_;
  std::string match_state_;
  std::string conflicting_authority_;
  std::vector<std::string> team_a_namespaces_;
  std::vector<std::string> team_b_namespaces_;
  std::string forced_team_a_striker_namespace_;
  double team_a_attack_goal_x_{8.0};
  double team_a_attack_goal_y_{0.0};
  double team_b_attack_goal_x_{-2.0};
  double team_b_attack_goal_y_{0.0};
  double field_min_x_{-2.0};
  double field_max_x_{8.0};
  double field_min_y_{-3.0};
  double field_max_y_{3.0};
  double tactical_boundary_margin_m_{0.30};
  double update_rate_hz_{5.0};
  double max_pose_age_sec_{0.40};
  double max_pose_skew_sec_{0.15};
  double ball_timeout_sec_{0.35};
  double future_tolerance_sec_{0.08};
  double authority_timeout_sec_{1.0};
  double striker_min_hold_sec_{1.5};
  double striker_benefit_threshold_{0.45};
  double striker_switch_confirm_sec_{0.80};
  double kickoff_hold_sec_{3.0};
  double kickoff_release_distance_m_{0.25};
  double kickoff_max_duration_sec_{10.0};
  int minimum_online_per_team_{1};
  int minimum_other_robot_count_{9};
  bool have_ball_{false};
  bool kickoff_reference_valid_{false};
  geometry_msgs::msg::PoseStamped latest_ball_;
  rclcpp::Time latest_ball_time_;
  rclcpp::Time kickoff_hold_until_;
  rclcpp::Time kickoff_started_;
  double kickoff_ball_x_{0.0};
  double kickoff_ball_y_{0.0};
  rclcpp::Time striker_since_a_;
  rclcpp::Time striker_since_b_;
  rclcpp::Time challenger_since_a_;
  rclcpp::Time challenger_since_b_;
  rclcpp::Time authority_conflict_time_;
  std::string current_striker_a_;
  std::string current_striker_b_;
  std::string challenger_a_;
  std::string challenger_b_;
  std::map<std::string, OdomState> robot_odoms_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr>
    other_robot_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> role_pubs_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
    tactical_target_pubs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ball_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr authority_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_state_command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr striker_a_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr striker_b_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kick_target_a_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kick_target_b_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr authority_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
