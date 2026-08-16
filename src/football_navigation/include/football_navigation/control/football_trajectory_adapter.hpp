// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballTrajectoryAdapter : public rclcpp::Node
{
public:
  FootballTrajectoryAdapter();

private:
  void clearTrackingState();
  void goalEventCallback(
  const std_msgs::msg::String::SharedPtr);
  bool inKickoffHold(
  const rclcpp::Time & stamp) const;
  static bool finitePose(
  const geometry_msgs::msg::Pose & pose);
  bool canPublish() const;
  void approachCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishPlannerUpdate();
  void publishTrackingHeartbeat();

  std::string target_frame_;
  std::string input_topic_;
  std::string output_goal_topic_;
  std::string output_tracking_topic_;
  std::string output_heartbeat_topic_;
  std::string goal_event_topic_;
  std::string control_valid_topic_;

  bool publish_tracking_pose_{true};
  bool publish_goal_pose_{false};
  bool have_control_valid_{false};
  bool control_valid_{false};
  bool separate_heartbeat_topic_{false};
  bool have_tracking_pose_{false};
  bool planner_pose_published_{false};
  bool planner_update_pending_{false};

  double max_input_age_sec_{0.50};
  double future_tolerance_sec_{0.08};
  double kickoff_hold_sec_{3.0};
  double planner_update_rate_hz_{0.0};
  double tracking_pose_heartbeat_hz_{10.0};

  geometry_msgs::msg::PoseStamped
    latest_tracking_pose_;

  rclcpp::Time kickoff_hold_until_;
  rclcpp::Time last_input_stamp_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped
  >::SharedPtr approach_sub_;

  rclcpp::Subscription<
    std_msgs::msg::String
  >::SharedPtr goal_event_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Bool
  >::SharedPtr control_valid_sub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped
  >::SharedPtr goal_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped
  >::SharedPtr tracking_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped
  >::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr
    heartbeat_timer_;

  rclcpp::TimerBase::SharedPtr
    planner_timer_;
};

}  // namespace football_navigation
