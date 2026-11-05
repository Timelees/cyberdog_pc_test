// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

// Fixed, deterministic inputs for the first single-robot football simulation.
// The actual striker target is still computed by FootballGoalAdapter.

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "football_navigation/simulation/football_simulation_input_publisher.hpp"

namespace football_navigation
{
namespace
{

std::string expandNamespace(std::string pattern, const std::string & robot_namespace)
{
  constexpr const char * token = "{namespace}";
  const auto position = pattern.find(token);
  if (position == std::string::npos) {
    throw std::invalid_argument("odom_topic_template must contain {namespace}");
  }
  pattern.replace(position, std::string(token).size(), robot_namespace);
  return pattern;
}

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace

FootballSimulationInputPublisher::FootballSimulationInputPublisher()
  : Node("football_simulation_input_publisher")
{
    robot_namespace_ = normalizeRobotNamespace(
      declare_parameter<std::string>("robot_namespace", "cyberdog_1"));
    field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
    odom_topic_template_ = declare_parameter<std::string>(
      "odom_topic_template", "/global_vio/{namespace}/odom");
    publish_odom_ = declare_parameter<bool>("publish_odom", true);
    publish_ball_ = declare_parameter<bool>("publish_ball", true);
    publish_path_ = declare_parameter<bool>("publish_path", true);
    ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
    ball_input_topic_ = declare_parameter<std::string>("ball_input_topic", "");
    ball_override_topic_ = declare_parameter<std::string>(
      "ball_override_topic", "/football/simulation/ball_override");
    approach_topic_ = declare_parameter<std::string>(
      "approach_topic", "/cyberdog_1/football/approach_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/cyberdog_1/plan");
    striker_topic_ = declare_parameter<std::string>(
      "striker_topic", "/football/team_a/striker");
    role_topic_ = declare_parameter<std::string>("role_topic", "");
    if (role_topic_.empty()) {
      role_topic_ = "/" + robot_namespace_ + "/football/role";
    }
    kick_target_topic_ = declare_parameter<std::string>(
      "kick_target_topic", "/football/team_a/kick_target");
    match_state_topic_ = declare_parameter<std::string>(
      "match_state_topic", "/football/match_state");
    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic", "/cyberdog_1/cmd_vel");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    ball_input_timeout_sec_ = declare_parameter<double>("ball_input_timeout_sec", 0.5);
    cmd_vel_timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.5);
    robot_x_ = declare_parameter<double>("robot_x", 0.0);
    robot_y_ = declare_parameter<double>("robot_y", 0.0);
    robot_yaw_ = declare_parameter<double>("robot_yaw", 0.0);
    ball_x_ = declare_parameter<double>("ball_x", 3.0);
    ball_y_ = declare_parameter<double>("ball_y", 0.0);
    kick_target_x_ = declare_parameter<double>("kick_target_x", 8.0);
    kick_target_y_ = declare_parameter<double>("kick_target_y", 0.0);
    stop_at_ball_ = declare_parameter<bool>("stop_at_ball", true);
    ball_contact_distance_m_ = declare_parameter<double>("ball_contact_distance_m", 0.42);

    if (robot_namespace_.empty() || field_frame_.empty() || publish_rate_hz_ <= 0.0 ||
      ball_input_timeout_sec_ <= 0.0 ||
      cmd_vel_timeout_sec_ <= 0.0 || ball_contact_distance_m_ <= 0.0)
    {
      throw std::invalid_argument("invalid football simulation input parameters");
    }

    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        expandNamespace(odom_topic_template_, robot_namespace_),
        rclcpp::SensorDataQoS().keep_last(10));
    }
    if (publish_ball_) {
      ball_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        ball_topic_, rclcpp::SensorDataQoS().keep_last(5));
    }
    ball_override_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ball_override_topic_, rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!msg || msg->header.frame_id != field_frame_ ||
          !std::isfinite(msg->pose.position.x) ||
          !std::isfinite(msg->pose.position.y))
        {
          return;
        }
        ball_x_ = msg->pose.position.x;
        ball_y_ = msg->pose.position.y;
        RCLCPP_INFO(
          get_logger(), "simulation ball overridden to (%.2f, %.2f)", ball_x_, ball_y_);
      });
    if (!ball_input_topic_.empty()) {
      ball_input_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        ball_input_topic_, rclcpp::SensorDataQoS().keep_last(5),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          if (!msg || msg->header.frame_id != field_frame_ ||
            !std::isfinite(msg->point.x) || !std::isfinite(msg->point.y))
          {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000,
              "ignoring live ball input with invalid frame or coordinates (expected frame '%s')",
              field_frame_.c_str());
            return;
          }
          ball_x_ = msg->point.x;
          ball_y_ = msg->point.y;
          last_ball_input_time_ = now();
          if (!have_live_ball_input_ || live_ball_input_stale_) {
            RCLCPP_INFO(
              get_logger(), "live ball input active at (%.2f, %.2f)", ball_x_, ball_y_);
          }
          have_live_ball_input_ = true;
          live_ball_input_stale_ = false;
        });
    }
    if (publish_path_) {
      path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    }
    striker_pub_ = create_publisher<std_msgs::msg::String>(striker_topic_, latchedQos());
    role_pub_ = create_publisher<std_msgs::msg::String>(role_topic_, latchedQos());
    kick_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      kick_target_topic_, latchedQos());
    match_state_pub_ = create_publisher<std_msgs::msg::String>(match_state_topic_, latchedQos());
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          latest_cmd_vel_ = *msg;
          latest_cmd_vel_time_ = now();
          have_cmd_vel_ = true;
        }
      });

    approach_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      approach_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg && msg->header.frame_id == field_frame_) {
          latest_approach_ = *msg;
          have_approach_ = true;
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FootballSimulationInputPublisher::publishScene, this));
    last_update_time_ = now();

    RCLCPP_INFO(
      get_logger(), "simulation: robot=%s pose=(%.2f, %.2f, %.2f), ball=(%.2f, %.2f)",
      robot_namespace_.c_str(), robot_x_, robot_y_, robot_yaw_, ball_x_, ball_y_);
  }

geometry_msgs::msg::PoseStamped FootballSimulationInputPublisher::makePose(
    const rclcpp::Time & stamp, const double x, const double y, const double yaw) const
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = field_frame_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = quaternionFromYaw(yaw);
    return pose;
  }

void FootballSimulationInputPublisher::publishScene()
{
    const auto stamp = now();
    double dt = (stamp - last_update_time_).seconds();
    last_update_time_ = stamp;
    if (dt <= 0.0 || dt > 0.25) {
      dt = 1.0 / publish_rate_hz_;
    }

    const bool command_fresh = have_cmd_vel_ &&
      (stamp - latest_cmd_vel_time_).seconds() <= cmd_vel_timeout_sec_;
    geometry_msgs::msg::Twist applied_cmd;
    if (command_fresh) {
      applied_cmd = latest_cmd_vel_;
      const double c = std::cos(robot_yaw_);
      const double s = std::sin(robot_yaw_);
      const double next_x = robot_x_ +
        (c * applied_cmd.linear.x - s * applied_cmd.linear.y) * dt;
      const double next_y = robot_y_ +
        (s * applied_cmd.linear.x + c * applied_cmd.linear.y) * dt;
      const double robot_dx = next_x - robot_x_;
      const double robot_dy = next_y - robot_y_;
      const double current_ball_distance = std::hypot(ball_x_ - robot_x_, ball_y_ - robot_y_);
      const double next_ball_distance = std::hypot(ball_x_ - next_x, ball_y_ - next_y);

      if (!stop_at_ball_ || next_ball_distance >= ball_contact_distance_m_) {
        robot_x_ = next_x;
        robot_y_ = next_y;
      } else {
        applied_cmd.linear.x = 0.0;
        applied_cmd.linear.y = 0.0;
      }
      robot_yaw_ = normalizeAngle(robot_yaw_ + applied_cmd.angular.z * dt);
    }
    const auto robot_pose = makePose(stamp, robot_x_, robot_y_, robot_yaw_);

    nav_msgs::msg::Odometry odom;
    odom.header = robot_pose.header;
    odom.child_frame_id = robot_namespace_ + "/base_link";
    odom.pose.pose = robot_pose.pose;
    odom.twist.twist = applied_cmd;
    if (odom_pub_) {
      odom_pub_->publish(odom);
    }

    if (ball_pub_) {
      const bool live_ball_fresh = ball_input_topic_.empty() ||
        (have_live_ball_input_ &&
        (stamp - last_ball_input_time_).seconds() <= ball_input_timeout_sec_);
      if (live_ball_fresh) {
        ball_pub_->publish(makePose(stamp, ball_x_, ball_y_, 0.0));
      } else if (!live_ball_input_stale_) {
        live_ball_input_stale_ = true;
        RCLCPP_WARN(
          get_logger(), "live ball input stale; stopping ball pose publication after %.2f s",
          ball_input_timeout_sec_);
      }
    }
    kick_target_pub_->publish(makePose(stamp, kick_target_x_, kick_target_y_, 0.0));

    std_msgs::msg::String striker;
    striker.data = robot_namespace_;
    striker_pub_->publish(striker);
    std_msgs::msg::String role;
    role.data = "STRIKER";
    role_pub_->publish(role);
    std_msgs::msg::String match_state;
    match_state.data = "PLAY";
    match_state_pub_->publish(match_state);

    if (path_pub_ && have_approach_) {
      nav_msgs::msg::Path path;
      path.header.stamp = stamp;
      path.header.frame_id = field_frame_;
      path.poses.push_back(robot_pose);
      latest_approach_.header.stamp = stamp;
      path.poses.push_back(latest_approach_);
      path_pub_->publish(path);
    }
  }


}  // namespace football_navigation
