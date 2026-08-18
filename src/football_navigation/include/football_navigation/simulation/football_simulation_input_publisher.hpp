// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballSimulationInputPublisher : public rclcpp::Node
{
public:
  FootballSimulationInputPublisher();

private:
  geometry_msgs::msg::PoseStamped makePose(
    const rclcpp::Time & stamp, const double x, const double y, const double yaw) const;
  void publishScene();

  std::string robot_namespace_;
  std::string field_frame_;
  std::string odom_topic_template_;
  std::string ball_topic_;
  std::string approach_topic_;
  std::string path_topic_;
  std::string striker_topic_;
  std::string kick_target_topic_;
  std::string match_state_topic_;
  std::string cmd_vel_topic_;
  std::string control_state_topic_;
  std::string control_state_{"SEARCH_BALL"};
  double publish_rate_hz_{20.0};
  double cmd_vel_timeout_sec_{0.5};
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  double ball_x_{3.0};
  double ball_y_{0.0};
  double kick_target_x_{8.0};
  double kick_target_y_{0.0};
  bool stop_at_ball_{true};
  double ball_contact_distance_m_{0.42};
  bool simulate_ball_push_{true};
  double ball_push_transfer_gain_{1.0};
  double ball_push_lateral_gain_{0.05};
  bool have_cmd_vel_{false};
  bool ball_push_active_{false};
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time latest_cmd_vel_time_;
  rclcpp::Time last_update_time_;
  bool have_approach_{false};
  geometry_msgs::msg::PoseStamped latest_approach_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr striker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kick_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr approach_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
