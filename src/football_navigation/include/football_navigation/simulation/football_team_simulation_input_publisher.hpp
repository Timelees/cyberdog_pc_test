// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace football_navigation
{

class FootballTeamSimulationInputPublisher : public rclcpp::Node
{
public:
  FootballTeamSimulationInputPublisher();

private:
  void publishBall();

  std::string field_frame_;
  std::string ball_topic_;
  std::string kick_target_topic_;
  double publish_rate_hz_{20.0};
  double ball_x_{3.0};
  double ball_y_{0.0};
  bool ball_motion_enabled_{false};
  double ball_motion_x_amplitude_m_{1.0};
  double ball_motion_y_amplitude_m_{1.2};
  double ball_motion_period_sec_{12.0};
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kick_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
