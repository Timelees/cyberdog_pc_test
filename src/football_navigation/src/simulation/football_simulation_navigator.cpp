// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

// Minimal NavigateToPose server used only by the single-robot simulation.
// It closes the same action-client -> cmd_vel interface used on the robot.

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
#include "football_navigation/simulation/football_simulation_navigator.hpp"

namespace football_navigation
{

FootballSimulationNavigator::FootballSimulationNavigator()
: Node("football_simulation_navigator")
{
    field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/global_vio/cyberdog_1/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    push_cmd_vel_topic_ = declare_parameter<std::string>(
      "push_cmd_vel_topic", "football/push_cmd_vel");
    speed_limit_topic_ = declare_parameter<std::string>("speed_limit_topic", "speed_limit");
    control_state_topic_ = declare_parameter<std::string>(
      "control_state_topic", "football/state");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    linear_gain_ = declare_parameter<double>("linear_gain", 0.9);
    angular_gain_ = declare_parameter<double>("angular_gain", 1.5);
    push_bearing_gain_ = declare_parameter<double>("push_bearing_gain", 0.6);
    max_linear_speed_mps_ = declare_parameter<double>("max_linear_speed_mps", 0.45);
    max_angular_speed_rps_ = declare_parameter<double>("max_angular_speed_rps", 0.8);
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.06);
    yaw_tolerance_rad_ = declare_parameter<double>("yaw_tolerance_rad", 0.08);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);

    if (field_frame_.empty() || control_rate_hz_ <= 0.0 || linear_gain_ <= 0.0 ||
      angular_gain_ <= 0.0 || push_bearing_gain_ < 0.0 || max_linear_speed_mps_ <= 0.0 ||
      max_angular_speed_rps_ <= 0.0 || position_tolerance_m_ <= 0.0 ||
      yaw_tolerance_rad_ <= 0.0 || odom_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid simulation navigator parameters");
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    push_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(push_cmd_vel_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS().keep_last(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg && msg->header.frame_id == field_frame_) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_odom_ = *msg;
          latest_odom_time_ = now();
          have_odom_ = true;
        }
      });
    speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
      speed_limit_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
        if (!msg || msg->percentage || !std::isfinite(msg->speed_limit) ||
          msg->speed_limit < 0.0)
        {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        speed_limit_mps_ = msg->speed_limit;
      });
    control_state_sub_ = create_subscription<std_msgs::msg::String>(
      control_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const bool was_pushing = isPushState();
        control_state_ = msg->data;
        if (was_pushing && !isPushState()) {
          push_cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        }
      });

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name_,
      std::bind(&FootballSimulationNavigator::handleGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&FootballSimulationNavigator::handleCancel, this, std::placeholders::_1),
      std::bind(&FootballSimulationNavigator::handleAccepted, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FootballSimulationNavigator::controlTick, this));
  }

rclcpp_action::GoalResponse FootballSimulationNavigator::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    if (!goal || goal->pose.header.frame_id != field_frame_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    double yaw = 0.0;
    if (!yawFromQuaternion(goal->pose.pose.orientation, yaw)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

rclcpp_action::CancelResponse FootballSimulationNavigator::handleCancel(const std::shared_ptr<GoalHandle>)
{
    return rclcpp_action::CancelResponse::ACCEPT;
  }

void FootballSimulationNavigator::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ && active_goal_->is_active()) {
      active_goal_->abort(std::make_shared<NavigateToPose::Result>());
    }
    active_goal_ = goal_handle;
    goal_started_time_ = now();
    RCLCPP_INFO(
      get_logger(), "simulation navigator accepted goal (%.2f, %.2f)",
      goal_handle->get_goal()->pose.pose.position.x,
      goal_handle->get_goal()->pose.pose.position.y);
  }

void FootballSimulationNavigator::publishStop()
{
    publishCommand(geometry_msgs::msg::Twist());
  }

bool FootballSimulationNavigator::isPushState() const
{
    return control_state_ == "CONTACT_ACQUIRE" || control_state_ == "PUSH_BALL";
  }

void FootballSimulationNavigator::publishCommand(const geometry_msgs::msg::Twist & command)
{
    cmd_vel_pub_->publish(command);
    if (isPushState()) {
      push_cmd_vel_pub_->publish(command);
    }
  }

void FootballSimulationNavigator::controlTick()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_) {
      publishStop();
      return;
    }
    if (active_goal_->is_canceling()) {
      publishStop();
      active_goal_->canceled(std::make_shared<NavigateToPose::Result>());
      active_goal_.reset();
      return;
    }
    if (!active_goal_->is_active()) {
      publishStop();
      active_goal_.reset();
      return;
    }
    if (!have_odom_ || (now() - latest_odom_time_).seconds() > odom_timeout_sec_) {
      publishStop();
      return;
    }

    const auto & target = active_goal_->get_goal()->pose;
    const double dx = target.pose.position.x - latest_odom_.pose.pose.position.x;
    const double dy = target.pose.position.y - latest_odom_.pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    double robot_yaw = 0.0;
    double target_yaw = 0.0;
    if (!yawFromQuaternion(latest_odom_.pose.pose.orientation, robot_yaw) ||
      !yawFromQuaternion(target.pose.orientation, target_yaw))
    {
      publishStop();
      active_goal_->abort(std::make_shared<NavigateToPose::Result>());
      active_goal_.reset();
      return;
    }

    const double yaw_error = signedYawError(target_yaw, robot_yaw);
    if (distance <= position_tolerance_m_ && std::fabs(yaw_error) <= yaw_tolerance_rad_) {
      publishStop();
      active_goal_->succeed(std::make_shared<NavigateToPose::Result>());
      active_goal_.reset();
      return;
    }

    geometry_msgs::msg::Twist command;
    double push_bearing_error = 0.0;
    if (distance > position_tolerance_m_) {
      const double active_speed_limit = speed_limit_mps_ > 0.0 ?
        std::min(max_linear_speed_mps_, speed_limit_mps_) : max_linear_speed_mps_;
      const double world_speed = std::min(active_speed_limit, linear_gain_ * distance);
      if (isPushState()) {
        // During contact, forbid holonomic lateral translation. Move along
        // the robot heading and steer gently back toward the push corridor.
        push_bearing_error = signedYawError(std::atan2(dy, dx), robot_yaw);
        command.linear.x = world_speed * std::max(0.0, std::cos(push_bearing_error));
        command.linear.y = 0.0;
      } else {
        const double world_vx = world_speed * dx / distance;
        const double world_vy = world_speed * dy / distance;
        command.linear.x = std::cos(robot_yaw) * world_vx + std::sin(robot_yaw) * world_vy;
        command.linear.y = -std::sin(robot_yaw) * world_vx + std::cos(robot_yaw) * world_vy;
      }
    }
    if (isPushState()) {
      // Continue rotating after the translational tolerance is reached;
      // otherwise a contact goal can remain active forever with zero cmd_vel.
      command.angular.z = std::clamp(
        angular_gain_ * yaw_error + push_bearing_gain_ * push_bearing_error,
        -max_angular_speed_rps_, max_angular_speed_rps_);
    } else {
      command.angular.z = std::clamp(
        angular_gain_ * yaw_error, -max_angular_speed_rps_, max_angular_speed_rps_);
    }
    publishCommand(command);

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose.header = latest_odom_.header;
    feedback->current_pose.pose = latest_odom_.pose.pose;
    feedback->distance_remaining = static_cast<float>(distance);
    feedback->navigation_time = now() - goal_started_time_;
    active_goal_->publish_feedback(feedback);
  }


}  // namespace football_navigation
