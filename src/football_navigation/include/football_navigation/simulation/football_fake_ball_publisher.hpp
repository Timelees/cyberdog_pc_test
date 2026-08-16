// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "football_navigation/core/football_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballFakeBallPublisher : public rclcpp::Node
{
public:
  FootballFakeBallPublisher();

private:
  bool lookupRobotInField(const std::string & ns, double & x, double & y) const;
  void getTeamGoalForStriker(const std::string & striker, double & gx, double & gy) const;
  bool inKickoffHold(const rclcpp::Time & stamp) const;
  bool scoredAtGoal(
  const double goal_x, const double goal_y,
  const double home_x, const double home_y) const;
  bool teamAScored() const;
  bool teamBScored() const;
  void publishMatchStateCommand(const std::string & state);
  void resetToKickoff(const rclcpp::Time & stamp, const char * scoring_team);
  void finishSingleShot(const char * scoring_team);

  struct ContactCandidate
  {
    std::string robot;
    double strength{0.0};
    double vx{0.0};
    double vy{0.0};
  };
  bool contactCandidate(const std::string & robot, ContactCandidate & output) const;
  std::string selectBallPossessionStriker() const;
  void publishBallPossessionStriker(const std::string & possessor);
  bool tryPushByAnyRobot(const rclcpp::Time & stamp, const double dt);
  bool controlledRobotPoseFresh(const rclcpp::Time & stamp) const;
  bool controlledRobotSeen(const rclcpp::Time & stamp) const;
  void updateControlledContactDiagnostics(const rclcpp::Time & stamp);
  void publishDiagnostics(const rclcpp::Time & stamp);
  bool outOfBounds() const;
  void performPendingOpponentKickoff(const rclcpp::Time & stamp);
  void signalOutOfBounds();
  void publishBallPose();

  std::string frame_id_;
  std::string ball_topic_;
  std::string goal_event_topic_;
  std::string ball_possession_striker_topic_;
  double ball_contest_radius_m_{3.0};
  double publish_rate_hz_{10.0};
  double ball_x_{2.0};
  double ball_y_{0.0};
  double ball_z_{0.0};
  double kickoff_x_{0.0};
  double kickoff_y_{0.0};
  double robot_front_extent_m_{0.25};
  double robot_rear_extent_m_{0.23};
  double robot_half_width_m_{0.13};
  double ball_radius_m_{0.11};
  double contact_tolerance_m_{0.03};
  double minimum_contact_speed_mps_{0.02};
  double ball_friction_mps2_{0.8};
  double max_sim_ball_speed_mps_{0.25};
  double max_ball_acceleration_mps2_{0.45};
  double velocity_transfer_gain_{0.85};
  double goal_width_m_{1.2};
  double field_min_x_{-2.0};
  double field_max_x_{8.0};
  double field_min_y_{-3.0};
  double field_max_y_{3.0};
  double kickoff_hold_sec_{3.0};
  double team_a_attack_goal_x_{8.0};
  double team_a_attack_goal_y_{0.0};
  double team_b_attack_goal_x_{-2.0};
  double team_b_attack_goal_y_{0.0};
  std::string team_a_striker_topic_;
  std::string team_b_striker_topic_;
  std::string odom_topic_template_;
  bool continuous_demo_enabled_{false};
  bool single_shot_enabled_{false};
  std::string controlled_robot_namespace_;
  std::string match_state_topic_;
  std::string control_state_topic_;
  std::string front_contact_topic_;
  std::string side_contact_topic_;
  std::string rear_contact_topic_;
  std::string contact_robot_topic_;
  std::string controlled_robot_pose_seen_topic_;
  std::string controlled_robot_pose_fresh_topic_;
  std::string controlled_robot_transform_valid_topic_;
  std::string controlled_robot_seen_topic_;
  std::string ball_velocity_topic_;
  std::string match_state_{"STOP"};
  std::string control_state_{"SAFE_STOP"};

  double sim_ball_x_{2.0};
  double sim_ball_y_{0.0};
  double ball_velocity_x_{0.0};
  double ball_velocity_y_{0.0};
  rclcpp::Time last_publish_time_;
  rclcpp::Time kickoff_hold_until_;
  bool pending_opponent_kickoff_{false};
  bool goal_completed_{false};
  bool controlled_robot_pose_seen_{false};
  bool controlled_robot_pose_fresh_{false};
  bool controlled_robot_transform_valid_{false};
  bool current_front_contact_{false};
  bool current_side_contact_{false};
  bool current_rear_contact_{false};
  std::string current_contact_robot_;

  bool have_striker_a_{false};
  bool have_striker_b_{false};
  std::string striker_a_;
  std::string striker_b_;

  std::map<std::string, nav_msgs::msg::Odometry> robot_odoms_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_state_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ball_possession_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr side_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_robot_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_pose_seen_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_pose_fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_transform_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_seen_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ball_velocity_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_a_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_b_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_state_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
