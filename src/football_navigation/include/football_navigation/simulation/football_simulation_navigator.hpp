// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
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
  void otherRobotOdomCallback(
    const std::string & robot_namespace,
    const nav_msgs::msg::Odometry::SharedPtr msg);
  bool buildSmoothLocalPath(
    double robot_x, double robot_y, double target_x, double target_y);
  double nearestRobotClearance(double robot_x, double robot_y, double robot_yaw);
  void publishLocalPlan(const rclcpp::Time & stamp);
  std::pair<double, double> localLookahead(double robot_x, double robot_y) const;
  void controlTick();

  std::string field_frame_;
  std::string action_name_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string push_cmd_vel_topic_;
  std::string speed_limit_topic_;
  std::string control_state_topic_;
  std::string self_namespace_;
  std::string robot_namespaces_csv_;
  std::string robot_odom_topic_template_;
  std::string local_plan_topic_;
  double control_rate_hz_{20.0};
  double linear_gain_{0.9};
  double angular_gain_{1.5};
  double push_bearing_gain_{0.6};
  double max_linear_speed_mps_{0.45};
  double max_angular_speed_rps_{0.8};
  double position_tolerance_m_{0.06};
  double yaw_tolerance_rad_{0.08};
  double odom_timeout_sec_{0.5};
  double other_robot_timeout_sec_{0.7};
  double robot_collision_length_m_{0.562};
  double robot_collision_width_m_{0.339};
  double collision_ellipse_expansion_m_{0.05};
  double collision_path_clearance_m_{0.08};
  double collision_slowdown_clearance_m_{0.35};
  double collision_hard_stop_clearance_m_{0.02};
  double detour_extra_clearance_m_{0.08};
  double local_path_lookahead_m_{0.32};
  double path_heading_gain_{2.0};
  double turn_in_place_threshold_rad_{0.75};
  int smooth_path_samples_{41};
  int multi_obstacle_lattice_stations_{17};
  double multi_obstacle_lateral_step_m_{0.18};
  double multi_obstacle_max_lateral_m_{3.0};
  double multi_obstacle_max_lane_change_m_{0.54};
  double multi_obstacle_turn_penalty_{0.25};
  double field_min_x_{-8.0};
  double field_max_x_{8.0};
  double field_min_y_{-4.0};
  double field_max_y_{4.0};
  double speed_limit_mps_{0.0};
  std::string control_state_{"SEARCH_BALL"};
  std::mutex mutex_;
  bool have_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  std::map<std::string, nav_msgs::msg::Odometry> other_robot_odoms_;
  std::map<std::string, rclcpp::Time> other_robot_times_;
  std::vector<std::string> robot_namespaces_;
  std::vector<std::pair<double, double>> local_path_points_;
  bool avoidance_active_{false};
  std::string avoidance_strategy_{"direct"};
  bool have_local_path_target_{false};
  double local_path_target_x_{0.0};
  double local_path_target_y_{0.0};
  rclcpp::Time latest_odom_time_;
  rclcpp::Time goal_started_time_;
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr push_cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_state_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
    other_robot_odom_subs_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
