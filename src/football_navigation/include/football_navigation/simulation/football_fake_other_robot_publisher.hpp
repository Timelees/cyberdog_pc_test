// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace football_navigation
{

class FootballFakeOtherRobotPublisher : public rclcpp::Node
  {
public:
  FootballFakeOtherRobotPublisher();

private:

  private:
    struct Robot
    {
      std::string id;
      std::size_t index{0};
      double initial_global_x{0.0};
      double initial_global_y{0.0};
      double initial_global_yaw{0.0};
      double local_x{0.0};
      double local_y{0.0};
      double local_yaw{0.0};
      double local_vx{0.0};
      double local_vy{0.0};
      double wz{0.0};
      bool have_cmd_vel{false};

      geometry_msgs::msg::Twist latest_cmd_vel;
      rclcpp::Time latest_cmd_vel_time;

      rclcpp::Publisher<
          nav_msgs::msg::Odometry>::SharedPtr
          odom_out_pub;
      rclcpp::Publisher<
          nav_msgs::msg::Odometry>::SharedPtr
          odom_slam_pub;
      rclcpp::Publisher<
          tf2_msgs::msg::TFMessage>::SharedPtr
          namespaced_tf_pub;
      rclcpp::Publisher<
          nav_msgs::msg::Odometry>::SharedPtr
          acceptance_global_odom_pub;
      rclcpp::Publisher<
          geometry_msgs::msg::TwistStamped>::SharedPtr
          integrated_cmd_vel_pub;
      rclcpp::Subscription<
          geometry_msgs::msg::Twist>::SharedPtr
          cmd_vel_sub;
    };
  void setContinuousDemoPeerMotion(
    Robot &robot,
    const double elapsed_time);
  void integrateRobot(
    Robot &robot,
    const double dt,
    const double elapsed_time,
    const rclcpp::Time &stamp);
  nav_msgs::msg::Odometry makeOdom(
    const Robot &robot,
    const rclcpp::Time &stamp,
    const std::string &frame,
    const std::string &child_frame) const;
  nav_msgs::msg::Odometry makeAcceptanceGlobalOdom(
    const Robot &robot,
    const rclcpp::Time &stamp) const;
  geometry_msgs::msg::TransformStamped makeTransform(
    const rclcpp::Time &stamp,
    const std::string &parent,
    const std::string &child,
    const double x,
    const double y,
    const double yaw) const;
  geometry_msgs::msg::TransformStamped makeRawTagObservation(
    const Robot &robot,
    const rclcpp::Time &stamp) const;
  void publishRawTagObservations();
  tf2_msgs::msg::TFMessage makeTfMessage(
    const Robot &robot,
    const rclcpp::Time &stamp) const;
  void update();

    std::string field_frame_;
    std::vector<std::string> robot_namespaces_;
    std::string selected_namespace_;

    bool simulate_selected_robot_{true};
    bool scripted_peer_motion_enabled_{true};
    bool randomize_initial_poses_{false};
    int random_seed_{2026};
    double minimum_initial_separation_m_{0.90};

    double scripted_peer_linear_speed_mps_{0.08};
    double scripted_peer_lateral_speed_mps_{0.04};
    double scripted_peer_yaw_rate_rps_{0.12};

    bool continuous_demo_enabled_{false};

    double demo_peer_motion_start_delay_sec_{3.5};
    double demo_peer_motion_period_sec_{12.0};
    double demo_peer_crossing_y_amplitude_m_{0.75};
    double demo_peer_crossing_x_min_{-1.80};
    double demo_peer_crossing_x_max_{7.80};

    std::size_t selected_robot_index_{0};
    std::size_t demo_blocker_index_{1};

    double selected_initial_global_x_{0.0};
    double selected_initial_global_y_{0.0};

    std::string cmd_vel_topic_template_;
    std::string odom_out_topic_template_;
    std::string odom_out_frame_id_;
    std::string odom_out_child_frame_id_;
    std::string odom_slam_topic_template_;

    bool publish_odom_slam_{true};

    std::string tf_topic_template_;
    std::string tag_frame_template_;

    bool publish_acceptance_global_odom_{false};

    std::string acceptance_global_odom_topic_template_;
    bool publish_integrated_cmd_vel_{false};
    std::string integrated_cmd_vel_topic_template_;

    double cmd_vel_timeout_sec_{0.30};
    double max_linear_speed_mps_{0.25};
    double max_angular_speed_rps_{0.55};
    double publish_rate_hz_{10.0};

    double field_min_x_{-2.0};
    double field_max_x_{8.0};
    double field_min_y_{-3.0};
    double field_max_y_{3.0};
    double robot_collision_length_m_{0.562};
    double robot_collision_width_m_{0.339};
    double collision_ellipse_expansion_m_{0.05};
    double boundary_margin_m_{0.10};

    rclcpp::Time simulation_start_time_;
    rclcpp::Time last_update_;

    std::vector<Robot> robots_;

    rclcpp::Publisher<
        tf2_msgs::msg::TFMessage>::SharedPtr
        global_tf_static_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
  };

}  // namespace football_navigation
