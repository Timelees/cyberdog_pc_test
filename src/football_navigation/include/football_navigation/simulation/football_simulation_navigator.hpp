// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballSimulationNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  FootballSimulationNavigator();

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void publishStop();
  bool isPushState() const;
  void publishCommand(const geometry_msgs::msg::Twist & command);
  void controlTick();

  std::string field_frame_;
  std::string action_name_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string push_cmd_vel_topic_;
  std::string speed_limit_topic_;
  std::string control_state_topic_;
  double control_rate_hz_{20.0};
  double linear_gain_{0.9};
  double angular_gain_{1.5};
  double push_bearing_gain_{0.6};
  double max_linear_speed_mps_{0.45};
  double max_angular_speed_rps_{0.8};
  double position_tolerance_m_{0.06};
  double yaw_tolerance_rad_{0.08};
  double odom_timeout_sec_{0.5};
  double speed_limit_mps_{0.0};
  std::string control_state_{"SEARCH_BALL"};
  std::mutex mutex_;
  bool have_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time latest_odom_time_;
  rclcpp::Time goal_started_time_;
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr push_cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_state_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
