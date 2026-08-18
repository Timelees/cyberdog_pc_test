// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <sstream>

#include "football_navigation/simulation/football_team_simulation_input_publisher.hpp"

namespace football_navigation
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;

std::vector<std::string> splitCsv(const std::string & csv)
{
  std::vector<std::string> output;
  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto first = item.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = item.find_last_not_of(" \t\r\n");
    output.push_back(item.substr(first, last - first + 1));
  }
  return output;
}

std::string expandTopic(std::string pattern, const std::string & robot)
{
  const std::string token = "{namespace}";
  const auto position = pattern.find(token);
  if (position == std::string::npos) {
    throw std::invalid_argument("team simulation topic template must contain {namespace}");
  }
  pattern.replace(position, token.size(), robot);
  return pattern;
}
}  // namespace

FootballTeamSimulationInputPublisher::FootballTeamSimulationInputPublisher()
  : Node("football_team_simulation_input_publisher")
{
  field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
  ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
  odom_topic_template_ = declare_parameter<std::string>(
    "odom_topic_template", "/global_vio/{namespace}/odom");
  state_topic_template_ = declare_parameter<std::string>(
    "state_topic_template", "/{namespace}/football/state");
  striker_topic_ = declare_parameter<std::string>(
    "striker_topic", "/football/team_a/striker");
  kick_target_topic_ = declare_parameter<std::string>(
    "kick_target_topic", "/football/team_a/kick_target");
  robot_namespaces_ = splitCsv(declare_parameter<std::string>(
    "robot_namespaces_csv", "cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5"));
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
  ball_x_ = declare_parameter<double>("ball_x", 3.0);
  ball_y_ = declare_parameter<double>("ball_y", 0.0);
  ball_motion_enabled_ = declare_parameter<bool>("ball_motion_enabled", true);
  ball_motion_x_amplitude_m_ =
    declare_parameter<double>("ball_motion_x_amplitude_m", 1.0);
  ball_motion_y_amplitude_m_ =
    declare_parameter<double>("ball_motion_y_amplitude_m", 1.2);
  ball_motion_period_sec_ = declare_parameter<double>("ball_motion_period_sec", 12.0);

  if (field_frame_.empty() || ball_topic_.empty() || robot_namespaces_.empty() ||
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
  striker_sub_ = create_subscription<std_msgs::msg::String>(
    striker_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballTeamSimulationInputPublisher::strikerCallback, this,
    std::placeholders::_1));
  for (const auto & robot : robot_namespaces_) {
    odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
      expandTopic(odom_topic_template_, robot), rclcpp::SensorDataQoS().keep_last(5),
      [this, robot](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odomCallback(robot, msg);
      }));
    state_subs_.push_back(create_subscription<std_msgs::msg::String>(
      expandTopic(state_topic_template_, robot), rclcpp::QoS(1).reliable().transient_local(),
      [this, robot](const std_msgs::msg::String::SharedPtr msg) {
        stateCallback(robot, msg);
      }));
  }
  start_time_ = now();
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_)),
    std::bind(&FootballTeamSimulationInputPublisher::publishBall, this));
}

void FootballTeamSimulationInputPublisher::strikerCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (msg && !msg->data.empty()) {
    if (striker_namespace_ != msg->data) {
      last_push_odom_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    striker_namespace_ = msg->data;
  }
}

void FootballTeamSimulationInputPublisher::odomCallback(
  const std::string & robot, const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg || msg->header.frame_id != field_frame_) {
    return;
  }
  odoms_[robot] = *msg;
  odom_times_[robot] = now();
}

void FootballTeamSimulationInputPublisher::stateCallback(
  const std::string & robot, const std_msgs::msg::String::SharedPtr msg)
{
  if (msg) {
    states_[robot] = msg->data;
  }
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

  const auto odom_it = odoms_.find(striker_namespace_);
  const auto state_it = states_.find(striker_namespace_);
  if (odom_it != odoms_.end() && state_it != states_.end() &&
    state_it->second == "PUSH_BALL")
  {
    const auto & odom = odom_it->second;
    const auto stamp = rclcpp::Time(odom.header.stamp, get_clock()->get_clock_type());
    if (last_push_odom_time_.nanoseconds() > 0 && stamp > last_push_odom_time_) {
      const double forward = odom.twist.twist.linear.x;
      if (std::isfinite(forward) && forward > 0.0) {
        x += std::min(0.04, forward * (stamp - last_push_odom_time_).seconds());
        if (!ball_motion_enabled_) {
          ball_x_ = x;
          ball_y_ = y;
        }
      }
    }
    last_push_odom_time_ = stamp;
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
