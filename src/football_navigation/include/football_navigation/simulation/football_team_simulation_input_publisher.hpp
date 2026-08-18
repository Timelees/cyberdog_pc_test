// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>
#include <map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballTeamSimulationInputPublisher : public rclcpp::Node
{
public:
  FootballTeamSimulationInputPublisher();

private:
  void publishBall();
  void strikerCallback(const std_msgs::msg::String::SharedPtr msg);
  void odomCallback(const std::string & robot, const nav_msgs::msg::Odometry::SharedPtr msg);
  void stateCallback(const std::string & robot, const std_msgs::msg::String::SharedPtr msg);

  std::string field_frame_;
  std::string ball_topic_;
  std::string odom_topic_template_;
  std::string state_topic_template_;
  std::string striker_topic_;
  std::string kick_target_topic_;
  std::vector<std::string> robot_namespaces_;
  double publish_rate_hz_{20.0};
  double ball_x_{3.0};
  double ball_y_{0.0};
  bool ball_motion_enabled_{false};
  double ball_motion_x_amplitude_m_{1.0};
  double ball_motion_y_amplitude_m_{1.2};
  double ball_motion_period_sec_{12.0};
  rclcpp::Time start_time_;
  rclcpp::Time last_push_odom_time_;
  std::string striker_namespace_;
  std::string striker_state_;
  std::map<std::string, nav_msgs::msg::Odometry> odoms_;
  std::map<std::string, rclcpp::Time> odom_times_;
  std::map<std::string, std::string> states_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kick_target_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> state_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
