// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include "football_navigation/simulation/football_team_simulation_input_publisher.hpp"

namespace football_navigation
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;

}  // namespace

FootballTeamSimulationInputPublisher::FootballTeamSimulationInputPublisher()
  : Node("football_team_simulation_input_publisher")
{
  field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
  ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
  kick_target_topic_ = declare_parameter<std::string>(
    "kick_target_topic", "/football/team_a/kick_target");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
  ball_x_ = declare_parameter<double>("ball_x", 3.0);
  ball_y_ = declare_parameter<double>("ball_y", 0.0);
  ball_motion_enabled_ = declare_parameter<bool>("ball_motion_enabled", true);
  ball_motion_x_amplitude_m_ =
    declare_parameter<double>("ball_motion_x_amplitude_m", 1.0);
  ball_motion_y_amplitude_m_ =
    declare_parameter<double>("ball_motion_y_amplitude_m", 1.2);
  ball_motion_period_sec_ = declare_parameter<double>("ball_motion_period_sec", 12.0);

  if (field_frame_.empty() || ball_topic_.empty() ||
    !std::isfinite(publish_rate_hz_) ||
    publish_rate_hz_ <= 0.0 || !std::isfinite(ball_x_) || !std::isfinite(ball_y_) ||
    !std::isfinite(ball_motion_x_amplitude_m_) || ball_motion_x_amplitude_m_ < 0.0 ||
    !std::isfinite(ball_motion_y_amplitude_m_) || ball_motion_y_amplitude_m_ < 0.0 ||
    !std::isfinite(ball_motion_period_sec_) || ball_motion_period_sec_ <= 0.0)
  {
    throw std::invalid_argument("invalid team simulation input parameters");
  }

  ball_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    ball_topic_, rclcpp::SensorDataQoS().keep_last(5));
  kick_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    kick_target_topic_, rclcpp::QoS(1).reliable().transient_local());
  start_time_ = now();
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_)),
    std::bind(&FootballTeamSimulationInputPublisher::publishBall, this));
}

void FootballTeamSimulationInputPublisher::publishBall()
{
  const auto stamp = now();
  double x = ball_x_;
  double y = ball_y_;
  if (ball_motion_enabled_) {
    const double phase = kTwoPi * (stamp - start_time_).seconds() / ball_motion_period_sec_;
    x += ball_motion_x_amplitude_m_ * std::sin(phase);
    y += ball_motion_y_amplitude_m_ * std::sin(0.5 * phase);
  }

  geometry_msgs::msg::PoseStamped ball;
  ball.header.stamp = stamp;
  ball.header.frame_id = field_frame_;
  ball.pose.position.x = x;
  ball.pose.position.y = y;
  ball.pose.orientation.w = 1.0;
  ball_pub_->publish(ball);
  geometry_msgs::msg::PoseStamped kick_target = ball;
  kick_target.pose.position.x = 8.0;
  kick_target.pose.position.y = 0.0;
  kick_target_pub_->publish(kick_target);
}

}  // namespace football_navigation
