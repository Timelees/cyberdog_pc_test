// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

// Injects occasional, bounded ball impulses while a front leg is plausibly
// sweeping through the ball.  Gazebo already models rigid-body leg contacts,
// but a perfectly centred navigation target otherwise makes every trial put
// the ball in the gap between the front legs.  This node represents the
// localization, foot-placement and timing error present on the real robot.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "gazebo_msgs/srv/apply_link_wrench.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class FootballGazeboLegContactSimulator : public rclcpp::Node
{
public:
  FootballGazeboLegContactSimulator()
  : Node("football_gazebo_leg_contact_simulator")
  {
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/global_vio/cyberdog_1/odom");
    ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
    state_topic_ = declare_parameter<std::string>(
      "control_state_topic", "/cyberdog_1/football/state");
    service_name_ = declare_parameter<std::string>(
      "apply_wrench_service", "/gazebo/apply_link_wrench");
    ball_link_ = declare_parameter<std::string>("ball_link", "football::ball");
    reference_frame_ = declare_parameter<std::string>("reference_frame", "world");
    trigger_probability_ = declare_parameter<double>("trigger_probability", 0.28);
    cooldown_sec_ = declare_parameter<double>("cooldown_sec", 0.90);
    contact_min_x_ = declare_parameter<double>("contact_min_x", 0.20);
    contact_max_x_ = declare_parameter<double>("contact_max_x", 0.52);
    contact_half_width_ = declare_parameter<double>("contact_half_width", 0.24);
    forward_force_n_ = declare_parameter<double>("forward_force_n", 1.25);
    lateral_force_n_ = declare_parameter<double>("lateral_force_n", 0.85);
    force_duration_sec_ = declare_parameter<double>("force_duration_sec", 0.07);
    rng_.seed(static_cast<std::mt19937::result_type>(
      declare_parameter<int>("random_seed", 2026)));

    if (trigger_probability_ < 0.0 || trigger_probability_ > 1.0 ||
      cooldown_sec_ <= 0.0 ||
      contact_max_x_ <= contact_min_x_ || contact_half_width_ <= 0.0 ||
      forward_force_n_ < 0.0 || lateral_force_n_ < 0.0 ||
      force_duration_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid Gazebo leg-contact simulation parameters");
    }

    wrench_client_ = create_client<gazebo_msgs::srv::ApplyLinkWrench>(service_name_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS().keep_last(5),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg) {
          odom_ = *msg;
          have_odom_ = true;
        }
      });
    ball_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ball_topic_, rclcpp::SensorDataQoS().keep_last(5),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg) {
          ball_ = *msg;
          have_ball_ = true;
        }
      });
    state_sub_ = create_subscription<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg) {
          state_ = msg->data;
        }
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FootballGazeboLegContactSimulator::tick, this));
  }

private:
  void tick()
  {
    if (!have_odom_ || !have_ball_ ||
      (state_ != "CONTACT_ACQUIRE" && state_ != "PUSH_BALL"))
    {
      return;
    }
    const auto stamp = now();
    if (last_impulse_.nanoseconds() > 0 &&
      (stamp - last_impulse_).seconds() < cooldown_sec_)
    {
      return;
    }

    tf2::Quaternion quaternion(
      odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
      odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    if (!std::isfinite(yaw)) {
      return;
    }
    const double dx = ball_.pose.position.x - odom_.pose.pose.position.x;
    const double dy = ball_.pose.position.y - odom_.pose.pose.position.y;
    const double forward = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (forward < contact_min_x_ || forward > contact_max_x_ ||
      std::fabs(lateral) > contact_half_width_ || probability_(rng_) > trigger_probability_)
    {
      return;
    }

    // Randomly select the nominal striking foot and vary the magnitude, so
    // repeated contacts exercise both left and right ball deflections.
    const double side = probability_(rng_) < 0.5 ? -1.0 : 1.0;
    const double magnitude = 0.75 + 0.50 * probability_(rng_);
    const double body_fx = forward_force_n_ * magnitude;
    const double body_fy = side * lateral_force_n_ * magnitude;

    auto request = std::make_shared<gazebo_msgs::srv::ApplyLinkWrench::Request>();
    request->link_name = ball_link_;
    request->reference_frame = reference_frame_;
    request->wrench.force.x = std::cos(yaw) * body_fx - std::sin(yaw) * body_fy;
    request->wrench.force.y = std::sin(yaw) * body_fx + std::cos(yaw) * body_fy;
    request->duration.sec = static_cast<int32_t>(std::floor(force_duration_sec_));
    request->duration.nanosec = static_cast<uint32_t>(
      (force_duration_sec_ - static_cast<double>(request->duration.sec)) * 1e9);

    if (!wrench_client_->service_is_ready()) {
      return;
    }
    last_impulse_ = stamp;
    wrench_client_->async_send_request(request);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "applying bounded forward ball-contact assistance");
  }

  std::string odom_topic_;
  std::string ball_topic_;
  std::string state_topic_;
  std::string service_name_;
  std::string ball_link_;
  std::string reference_frame_;
  std::string state_{"SEARCH_BALL"};
  double trigger_probability_{0.28};
  double cooldown_sec_{0.90};
  double contact_min_x_{0.20};
  double contact_max_x_{0.52};
  double contact_half_width_{0.24};
  double forward_force_n_{1.25};
  double lateral_force_n_{0.85};
  double force_duration_sec_{0.07};
  bool have_odom_{false};
  bool have_ball_{false};
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::PoseStamped ball_;
  rclcpp::Time last_impulse_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> probability_{0.0, 1.0};
  rclcpp::Client<gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr wrench_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ball_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootballGazeboLegContactSimulator>());
  rclcpp::shutdown();
  return 0;
}
