// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "football_navigation/core/football_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballBallFusion : public rclcpp::Node
{
public:
  FootballBallFusion();

private:
  void goalEventCallback(const std_msgs::msg::String::SharedPtr);
  bool finitePose(const geometry_msgs::msg::Pose & pose) const;
  bool insideField(const double x, const double y) const;
  bool motionValid(const geometry_msgs::msg::PoseStamped & candidate) const;
  void publish(geometry_msgs::msg::PoseStamped output);
  void globalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool findNearestOdom(
  const std::string & robot,
  const rclcpp::Time & detection_stamp,
  nav_msgs::msg::Odometry & output) const;
  void fuse();

  std::string mode_;
  std::string field_frame_;
  std::string output_topic_;
  double max_detection_age_sec_{0.30};
  double future_tolerance_sec_{0.08};
  double max_odom_age_sec_{0.30};
  double max_detection_odom_skew_sec_{0.12};
  double max_fusion_time_skew_sec_{0.08};
  int minimum_inlier_count_{2};
  double inlier_radius_m_{0.45};
  double max_ball_speed_mps_{6.0};
  double field_min_x_{-2.25};
  double field_max_x_{8.25};
  double field_min_y_{-3.25};
  double field_max_y_{3.25};
  bool have_previous_{false};
  geometry_msgs::msg::PoseStamped previous_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> detections_;
  std::map<std::string, std::deque<nav_msgs::msg::Odometry>> odom_histories_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr global_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_event_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> detection_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
