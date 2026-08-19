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
    publish_path_ = declare_parameter<bool>("publish_path", true);
    ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
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
    control_state_topic_ = declare_parameter<std::string>(
      "control_state_topic", "/cyberdog_1/football/state");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
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
    simulate_ball_push_ = declare_parameter<bool>("simulate_ball_push", true);
    ball_push_transfer_gain_ = declare_parameter<double>("ball_push_transfer_gain", 1.0);
    ball_push_lateral_gain_ = declare_parameter<double>("ball_push_lateral_gain", 0.05);

    if (robot_namespace_.empty() || field_frame_.empty() || publish_rate_hz_ <= 0.0 ||
      cmd_vel_timeout_sec_ <= 0.0 || ball_contact_distance_m_ <= 0.0 ||
      ball_push_transfer_gain_ < 0.0 || ball_push_transfer_gain_ > 1.0 ||
      ball_push_lateral_gain_ < 0.0 || ball_push_lateral_gain_ > 1.0)
    {
      throw std::invalid_argument("invalid football simulation input parameters");
    }

    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        expandNamespace(odom_topic_template_, robot_namespace_),
        rclcpp::SensorDataQoS().keep_last(10));
    }
    ball_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      ball_topic_, rclcpp::SensorDataQoS().keep_last(5));
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
        ball_push_active_ = false;
        RCLCPP_INFO(
          get_logger(), "simulation ball overridden to (%.2f, %.2f)", ball_x_, ball_y_);
      });
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
    control_state_sub_ = create_subscription<std_msgs::msg::String>(
      control_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg) {
          control_state_ = msg->data;
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

      const double kick_dx = kick_target_x_ - ball_x_;
      const double kick_dy = kick_target_y_ - ball_y_;
      const double kick_distance = std::hypot(kick_dx, kick_dy);
      const double kick_x = kick_distance > 1e-6 ? kick_dx / kick_distance : 1.0;
      const double kick_y = kick_distance > 1e-6 ? kick_dy / kick_distance : 0.0;
      const double forward_motion = robot_dx * kick_x + robot_dy * kick_y;
      const double lateral_motion = -robot_dx * kick_y + robot_dy * kick_x;
      // Use the swept step as well as the current pose for contact acquisition.
      // Otherwise the anti-penetration branch can stop the robot just outside
      // the threshold while the push branch waits for a current-pose contact,
      // leaving a narrow floating-point dead zone around 0.425 m.
      const bool reaches_ball_contact =
        std::min(current_ball_distance, next_ball_distance) <=
        ball_contact_distance_m_ + 0.005;
      const bool pushing_ball = simulate_ball_push_ &&
        (control_state_ == "CONTACT_ACQUIRE" || control_state_ == "PUSH_BALL") &&
        reaches_ball_contact &&
        forward_motion > 0.0;

      if (pushing_ball != ball_push_active_) {
        RCLCPP_INFO(
          get_logger(), "simulation ball push %s state=%s separation=%.3f",
          pushing_ball ? "ACTIVE" : "INACTIVE", control_state_.c_str(),
          current_ball_distance);
        ball_push_active_ = pushing_ball;
      }

      if (pushing_ball) {
        // A low-speed dribble transfers forward motion to the ball while
        // heavily attenuating lateral slip. This keeps the ball in the
        // configured ball -> kick-target corridor during simulation.
        ball_x_ += ball_push_transfer_gain_ * forward_motion * kick_x -
          ball_push_lateral_gain_ * lateral_motion * kick_y;
        ball_y_ += ball_push_transfer_gain_ * forward_motion * kick_y +
          ball_push_lateral_gain_ * lateral_motion * kick_x;
        robot_x_ = next_x;
        robot_y_ = next_y;
      } else if (!stop_at_ball_ || control_state_ == "PUSH_REALIGN" ||
        next_ball_distance >= ball_contact_distance_m_)
      {
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

    ball_pub_->publish(makePose(stamp, ball_x_, ball_y_, 0.0));
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
