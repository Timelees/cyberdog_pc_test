// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

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
#include "football_navigation/simulation/football_fake_other_robot_publisher.hpp"

namespace football_navigation
{
  namespace
  {

    constexpr double kPi = 3.14159265358979323846;

    std::vector<std::string> splitCsv(const std::string &csv)
    {
      std::vector<std::string> output;
      std::stringstream stream(csv);
      std::string value;

      while (std::getline(stream, value, ','))
      {
        const auto first =
            value.find_first_not_of(" \t\n\r");

        if (first == std::string::npos)
        {
          continue;
        }

        const auto last =
            value.find_last_not_of(" \t\n\r");

        output.push_back(
            normalizeRobotNamespace(
                value.substr(first, last - first + 1)));
      }

      return output;
    }

    std::string expandTopic(
        const std::string &pattern,
        const std::string &robot)
    {
      std::string output = pattern;
      const std::string token = "{namespace}";
      const auto position = output.find(token);

      if (position == std::string::npos)
      {
        throw std::invalid_argument(
            "topic/frame template must contain {namespace}");
      }

      output.replace(position, token.size(), robot);
      return output;
    }

  } // namespace

  FootballFakeOtherRobotPublisher::FootballFakeOtherRobotPublisher()
        : Node("football_fake_other_robot_publisher")
{
      field_frame_ =
          declare_parameter<std::string>(
              "frame_id",
              "tag_global");

      robot_namespaces_ =
          splitCsv(
              declare_parameter<std::string>(
                  "robot_namespaces_csv",
                  "cyberdog_1,cyberdog_2,cyberdog_3,"
                  "cyberdog_4,cyberdog_5,"
                  "cyberdog_6,cyberdog_7,cyberdog_8,"
                  "cyberdog_9,cyberdog_10"));

      selected_namespace_ =
          normalizeRobotNamespace(
              declare_parameter<std::string>(
                  "selected_namespace",
                  "cyberdog_2"));

      simulate_selected_robot_ =
          declare_parameter<bool>(
              "simulate_selected_robot",
              true);

      scripted_peer_motion_enabled_ =
          declare_parameter<bool>(
              "scripted_peer_motion_enabled",
              true);

      scripted_peer_linear_speed_mps_ =
          declare_parameter<double>(
              "scripted_peer_linear_speed_mps",
              0.08);

      scripted_peer_lateral_speed_mps_ =
          declare_parameter<double>(
              "scripted_peer_lateral_speed_mps",
              0.04);

      scripted_peer_yaw_rate_rps_ =
          declare_parameter<double>(
              "scripted_peer_yaw_rate_rps",
              0.12);

      continuous_demo_enabled_ =
          declare_parameter<bool>(
              "continuous_demo_enabled",
              false);

      demo_peer_motion_start_delay_sec_ =
          declare_parameter<double>(
              "demo_peer_motion_start_delay_sec",
              3.5);

      demo_peer_motion_period_sec_ =
          declare_parameter<double>(
              "demo_peer_motion_period_sec",
              12.0);

      demo_peer_crossing_y_amplitude_m_ =
          declare_parameter<double>(
              "demo_peer_crossing_y_amplitude_m",
              0.75);

      demo_peer_crossing_x_min_ =
          declare_parameter<double>(
              "demo_peer_crossing_x_min",
              -1.80);

      demo_peer_crossing_x_max_ =
          declare_parameter<double>(
              "demo_peer_crossing_x_max",
              7.80);

      cmd_vel_topic_template_ =
          declare_parameter<std::string>(
              "cmd_vel_topic_template",
              "/{namespace}/cmd_vel");

      odom_out_topic_template_ =
          declare_parameter<std::string>(
              "odom_out_topic_template",
              "/{namespace}/odom_out");

      odom_out_frame_id_ =
          declare_parameter<std::string>(
              "odom_out_frame_id", "odom");

      odom_out_child_frame_id_ =
          declare_parameter<std::string>(
              "odom_out_child_frame_id", "base_link_leg");

      odom_slam_topic_template_ =
          declare_parameter<std::string>(
              "odom_slam_topic_template",
              "/{namespace}/odom_slam");

      publish_odom_slam_ =
          declare_parameter<bool>(
              "publish_odom_slam",
              true);

      tf_topic_template_ =
          declare_parameter<std::string>(
              "tf_topic_template",
              "/{namespace}/tf");

      tag_frame_template_ =
          declare_parameter<std::string>(
              "tag_frame_template",
              "{namespace}/tag_0_observation");

      publish_acceptance_global_odom_ =
          declare_parameter<bool>(
              "publish_acceptance_global_odom",
              false);

      acceptance_global_odom_topic_template_ =
          declare_parameter<std::string>(
              "acceptance_global_odom_topic_template",
              "/{namespace}/odom_global");

      publish_integrated_cmd_vel_ =
          declare_parameter<bool>(
              "publish_integrated_cmd_vel",
              false);

      integrated_cmd_vel_topic_template_ =
          declare_parameter<std::string>(
              "integrated_cmd_vel_topic_template",
              "/{namespace}/football/fake_integrated_cmd_vel");

      cmd_vel_timeout_sec_ =
          declare_parameter<double>(
              "cmd_vel_timeout_sec",
              0.30);

      max_linear_speed_mps_ =
          declare_parameter<double>(
              "max_linear_speed_mps",
              0.25);

      max_angular_speed_rps_ =
          declare_parameter<double>(
              "max_angular_speed_rps",
              0.55);

      publish_rate_hz_ =
          declare_parameter<double>(
              "publish_rate_hz",
              10.0);

      field_min_x_ =
          declare_parameter<double>(
              "field_min_x",
              -2.0);

      field_max_x_ =
          declare_parameter<double>(
              "field_max_x",
              8.0);

      field_min_y_ =
          declare_parameter<double>(
              "field_min_y",
              -3.0);

      field_max_y_ =
          declare_parameter<double>(
              "field_max_y",
              3.0);

      robot_collision_radius_m_ =
          declare_parameter<double>(
              "robot_collision_radius_m",
              0.38);

      boundary_margin_m_ =
          declare_parameter<double>(
              "boundary_margin_m",
              0.10);

      if (field_min_x_ >= field_max_x_ ||
          field_min_y_ >= field_max_y_ ||
          robot_collision_radius_m_ <= 0.0 ||
          boundary_margin_m_ < 0.0)
      {
        throw std::invalid_argument(
            "invalid fake world field geometry");
      }

      if (field_frame_ != "tag_global")
      {
        throw std::invalid_argument(
            "fake world frame_id must be tag_global");
      }

      if (robot_namespaces_.empty())
      {
        throw std::invalid_argument(
            "robot_namespaces_csv must not be empty");
      }

      std::vector<std::string> unique =
          robot_namespaces_;

      std::sort(unique.begin(), unique.end());

      if (std::adjacent_find(
              unique.begin(),
              unique.end()) != unique.end())
      {
        throw std::invalid_argument(
            "robot_namespaces_csv must contain unique names");
      }

      const auto selected =
          std::find(
              robot_namespaces_.begin(),
              robot_namespaces_.end(),
              selected_namespace_);

      if (selected == robot_namespaces_.end())
      {
        throw std::invalid_argument(
            "selected_namespace must be in robot_namespaces_csv");
      }

      selected_robot_index_ =
          static_cast<std::size_t>(
              std::distance(
                  robot_namespaces_.begin(),
                  selected));

      const std::size_t selected_row_start =
          (selected_robot_index_ / 5) * 5;

      const std::size_t selected_column =
          selected_robot_index_ % 5;

      demo_blocker_index_ =
          selected_row_start +
          (selected_column < 4 ? selected_column + 1 : selected_column - 1);

      selected_initial_global_x_ =
          0.5 +
          1.2 *
              static_cast<double>(selected_column);

      selected_initial_global_y_ =
          selected_robot_index_ < 5 ? -1.2 : 1.2;

      if (!std::isfinite(
              scripted_peer_linear_speed_mps_) ||
          !std::isfinite(
              scripted_peer_lateral_speed_mps_) ||
          !std::isfinite(
              scripted_peer_yaw_rate_rps_) ||
          scripted_peer_linear_speed_mps_ < 0.0 ||
          scripted_peer_lateral_speed_mps_ < 0.0 ||
          scripted_peer_yaw_rate_rps_ < 0.0)
      {
        throw std::invalid_argument(
            "scripted peer speeds must be finite and non-negative");
      }

      if (!std::isfinite(
              demo_peer_motion_start_delay_sec_) ||
          !std::isfinite(
              demo_peer_motion_period_sec_) ||
          !std::isfinite(
              demo_peer_crossing_y_amplitude_m_) ||
          !std::isfinite(
              demo_peer_crossing_x_min_) ||
          !std::isfinite(
              demo_peer_crossing_x_max_) ||
          demo_peer_motion_start_delay_sec_ < 0.0 ||
          demo_peer_motion_period_sec_ <= 4.0 ||
          demo_peer_crossing_y_amplitude_m_ <= 0.20 ||
          demo_peer_crossing_x_min_ >=
              demo_peer_crossing_x_max_ ||
          demo_peer_crossing_x_min_ <=
              field_min_x_ + boundary_margin_m_ ||
          demo_peer_crossing_x_max_ >=
              field_max_x_ - boundary_margin_m_ ||
          demo_peer_crossing_y_amplitude_m_ >=
              std::min(-field_min_y_, field_max_y_) -
                  boundary_margin_m_)
      {
        throw std::invalid_argument(
            "invalid continuous demo peer trajectory parameters");
      }

      robots_.reserve(robot_namespaces_.size());

      for (std::size_t index = 0;
           index < robot_namespaces_.size();
           ++index)
      {
        const auto &robot_namespace =
            robot_namespaces_[index];

        if (!simulate_selected_robot_ &&
            robot_namespace == selected_namespace_)
        {
          continue;
        }

        Robot robot;
        robot.id = robot_namespace;
        robot.index = index;
        robot.initial_global_x =
            0.5 +
            1.2 *
                static_cast<double>(index % 5);
        robot.initial_global_y =
            index < 5 ? -1.2 : 1.2;
        robot.initial_global_yaw =
            index < 5 ? 0.0 : kPi;
        robot.latest_cmd_vel_time =
            rclcpp::Time(
                0,
                0,
                get_clock()->get_clock_type());

        robots_.push_back(robot);
      }

      for (std::size_t index = 0;
           index < robots_.size();
           ++index)
      {
        auto &robot = robots_[index];

        robot.odom_out_pub =
            create_publisher<nav_msgs::msg::Odometry>(
                expandTopic(
                    odom_out_topic_template_,
                    robot.id),
                rclcpp::QoS(10).reliable());

        if (publish_odom_slam_)
        {
          robot.odom_slam_pub =
              create_publisher<nav_msgs::msg::Odometry>(
                  expandTopic(
                      odom_slam_topic_template_,
                      robot.id),
                  rclcpp::QoS(10).reliable());
        }

        robot.namespaced_tf_pub =
            create_publisher<tf2_msgs::msg::TFMessage>(
                expandTopic(
                    tf_topic_template_,
                    robot.id),
                rclcpp::QoS(100).reliable());

        if (publish_acceptance_global_odom_)
        {
          robot.acceptance_global_odom_pub =
              create_publisher<nav_msgs::msg::Odometry>(
                  expandTopic(
                      acceptance_global_odom_topic_template_,
                      robot.id),
                  rclcpp::SensorDataQoS().keep_last(5));
        }

        if (publish_integrated_cmd_vel_)
        {
          robot.integrated_cmd_vel_pub =
              create_publisher<geometry_msgs::msg::TwistStamped>(
                  expandTopic(
                      integrated_cmd_vel_topic_template_,
                      robot.id),
                  rclcpp::QoS(20).reliable());
        }

        robot.cmd_vel_sub =
            create_subscription<geometry_msgs::msg::Twist>(
                expandTopic(
                    cmd_vel_topic_template_,
                    robot.id),
                rclcpp::QoS(1).reliable(),
                [this, index](
                    const geometry_msgs::msg::Twist::SharedPtr msg)
                {
                  if (!msg ||
                      !std::isfinite(msg->linear.x) ||
                      !std::isfinite(msg->linear.y) ||
                      !std::isfinite(msg->angular.z))
                  {
                    return;
                  }

                  auto &target = robots_.at(index);
                  target.latest_cmd_vel = *msg;
                  target.latest_cmd_vel_time = now();
                  target.have_cmd_vel = true;
                });
      }

      global_tf_static_pub_ =
          create_publisher<tf2_msgs::msg::TFMessage>(
              "/tf_static",
              rclcpp::QoS(
                  rclcpp::KeepLast(1))
                  .transient_local()
                  .reliable());

      publishRawTagObservations();

      simulation_start_time_ = now();
      last_update_ = simulation_start_time_;

      const double period =
          publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.1;

      timer_ =
          create_wall_timer(
              std::chrono::duration_cast<
                  std::chrono::nanoseconds>(
                  std::chrono::duration<double>(period)),
              std::bind(
                  &FootballFakeOtherRobotPublisher::update,
                  this));
    }

void FootballFakeOtherRobotPublisher::setContinuousDemoPeerMotion(
    Robot &robot,
    const double elapsed_time)
{
      const double omega =
          2.0 * kPi /
          demo_peer_motion_period_sec_;

      const double peer_order =
          static_cast<double>(robot.index);

      const double motion_time =
          std::max(
              0.0,
              elapsed_time -
                  peer_order * 0.35);

      const double phase =
          omega * motion_time;

      const double blend =
          0.5 *
          (1.0 - std::cos(phase));

      const double blend_rate =
          0.5 *
          omega *
          std::sin(phase);

      const bool is_demo_blocker =
          robot.index == demo_blocker_index_;

      const double horizontal_direction =
          robot.index % 2 == 0 ? 1.0 : -1.0;

      const double vertical_excursion =
          robot.initial_global_y < 0.0 ? demo_peer_crossing_y_amplitude_m_ : -demo_peer_crossing_y_amplitude_m_;

      double target_x =
          std::clamp(
              robot.initial_global_x +
                  horizontal_direction * 0.30,
              field_min_x_ + boundary_margin_m_,
              field_max_x_ - boundary_margin_m_);

      double target_y =
          robot.initial_global_y +
          vertical_excursion;

      if (is_demo_blocker)
      {
        const double side =
            robot.initial_global_x >
                    selected_initial_global_x_
                ? 1.0
                : -1.0;

        const double blocker_vertical_excursion =
            selected_initial_global_y_ < 0.0 ? 0.74 : -0.74;

        target_x =
            std::clamp(
                selected_initial_global_x_ +
                    side * 0.20,
                demo_peer_crossing_x_min_,
                demo_peer_crossing_x_max_);

        target_y =
            selected_initial_global_y_ +
            blocker_vertical_excursion;
      }

      const double global_x =
          robot.initial_global_x +
          (target_x -
           robot.initial_global_x) *
              blend;

      const double global_y =
          robot.initial_global_y +
          (target_y -
           robot.initial_global_y) *
              blend;

      double world_vx =
          (target_x -
           robot.initial_global_x) *
          blend_rate;

      double world_vy =
          (target_y -
           robot.initial_global_y) *
          blend_rate;

      const double speed =
          std::hypot(world_vx, world_vy);

      if (speed > max_linear_speed_mps_ &&
          speed > 1e-9)
      {
        const double scale =
            max_linear_speed_mps_ / speed;
        world_vx *= scale;
        world_vy *= scale;
      }

      const double global_yaw =
          robot.initial_global_yaw;

      const double dx =
          global_x -
          robot.initial_global_x;

      const double dy =
          global_y -
          robot.initial_global_y;

      const double c0 =
          std::cos(robot.initial_global_yaw);

      const double s0 =
          std::sin(robot.initial_global_yaw);

      robot.local_x =
          c0 * dx +
          s0 * dy;

      robot.local_y =
          -s0 * dx +
          c0 * dy;

      robot.local_yaw =
          football_navigation::normalizeAngle(
              global_yaw -
              robot.initial_global_yaw);

      robot.local_vx =
          std::cos(global_yaw) * world_vx +
          std::sin(global_yaw) * world_vy;

      robot.local_vy =
          -std::sin(global_yaw) * world_vx +
          std::cos(global_yaw) * world_vy;

      robot.wz = 0.0;
    }

void FootballFakeOtherRobotPublisher::integrateRobot(
    Robot &robot,
    const double dt,
    const double elapsed_time,
    const rclcpp::Time &stamp)
{
      robot.local_vx = 0.0;
      robot.local_vy = 0.0;
      robot.wz = 0.0;

      if (continuous_demo_enabled_ &&
          robot.id != selected_namespace_)
      {
        if (elapsed_time <
            demo_peer_motion_start_delay_sec_)
        {
          return;
        }

        setContinuousDemoPeerMotion(
            robot,
            elapsed_time -
                demo_peer_motion_start_delay_sec_);

        return;
      }

      const bool cmd_vel_is_fresh =
          robot.have_cmd_vel &&
          robot.latest_cmd_vel_time.nanoseconds() > 0 &&
          (stamp -
           robot.latest_cmd_vel_time)
                  .seconds() <=
              cmd_vel_timeout_sec_;

      double vx = 0.0;
      double vy = 0.0;
      double wz = 0.0;

      if (cmd_vel_is_fresh)
      {
        vx = robot.latest_cmd_vel.linear.x;
        vy = robot.latest_cmd_vel.linear.y;
        wz = robot.latest_cmd_vel.angular.z;
      }
      else if (
          robot.id != selected_namespace_ &&
          scripted_peer_motion_enabled_)
      {
        const double phase =
            elapsed_time * 0.35 +
            static_cast<double>(robot.index) *
                0.73;

        vx =
            scripted_peer_linear_speed_mps_ *
            (0.65 +
             0.35 * std::sin(phase));

        vy =
            scripted_peer_lateral_speed_mps_ *
            std::cos(phase * 0.83);

        wz =
            scripted_peer_yaw_rate_rps_ *
            std::sin(phase * 0.57);
      }
      else
      {
        return;
      }

      const double speed =
          std::hypot(vx, vy);

      if (speed > max_linear_speed_mps_)
      {
        const double scale =
            max_linear_speed_mps_ / speed;
        vx *= scale;
        vy *= scale;
      }

      wz =
          std::clamp(
              wz,
              -max_angular_speed_rps_,
              max_angular_speed_rps_);

      const double world_vx =
          std::cos(robot.local_yaw) * vx -
          std::sin(robot.local_yaw) * vy;

      const double world_vy =
          std::sin(robot.local_yaw) * vx +
          std::cos(robot.local_yaw) * vy;

      robot.local_x += world_vx * dt;
      robot.local_y += world_vy * dt;
      robot.local_yaw =
          football_navigation::normalizeAngle(
              robot.local_yaw +
              wz * dt);
      robot.local_vx = vx;
      robot.local_vy = vy;
      robot.wz = wz;
    }

nav_msgs::msg::Odometry FootballFakeOtherRobotPublisher::makeOdom(
    const Robot &robot,
    const rclcpp::Time &stamp,
    const std::string &frame,
    const std::string &child_frame) const
{
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = frame;
      odom.child_frame_id = child_frame;
      odom.pose.pose.position.x = robot.local_x;
      odom.pose.pose.position.y = robot.local_y;
      odom.pose.pose.orientation =
          quaternionFromYaw(robot.local_yaw);
      odom.twist.twist.linear.x = robot.local_vx;
      odom.twist.twist.linear.y = robot.local_vy;
      odom.twist.twist.angular.z = robot.wz;
      return odom;
    }

nav_msgs::msg::Odometry FootballFakeOtherRobotPublisher::makeAcceptanceGlobalOdom(
    const Robot &robot,
    const rclcpp::Time &stamp) const
{
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = field_frame_;
      odom.child_frame_id = "base_link";

      const double c =
          std::cos(robot.initial_global_yaw);

      const double s =
          std::sin(robot.initial_global_yaw);

      odom.pose.pose.position.x =
          robot.initial_global_x +
          c * robot.local_x -
          s * robot.local_y;

      odom.pose.pose.position.y =
          robot.initial_global_y +
          s * robot.local_x +
          c * robot.local_y;

      odom.pose.pose.orientation =
          quaternionFromYaw(
              football_navigation::normalizeAngle(
                  robot.initial_global_yaw +
                  robot.local_yaw));

      odom.twist.twist.linear.x =
          robot.local_vx;
      odom.twist.twist.linear.y =
          robot.local_vy;
      odom.twist.twist.angular.z =
          robot.wz;

      return odom;
    }

geometry_msgs::msg::TransformStamped FootballFakeOtherRobotPublisher::makeTransform(
    const rclcpp::Time &stamp,
    const std::string &parent,
    const std::string &child,
    const double x,
    const double y,
    const double yaw) const
{
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = parent;
      transform.child_frame_id = child;
      transform.transform.translation.x = x;
      transform.transform.translation.y = y;
      transform.transform.rotation =
          quaternionFromYaw(yaw);
      return transform;
    }

geometry_msgs::msg::TransformStamped FootballFakeOtherRobotPublisher::makeRawTagObservation(
    const Robot &robot,
    const rclcpp::Time &stamp) const
{
      const auto base_to_raw_tag =
          makeSimulatedRawTagObservation(
              robot.initial_global_x,
              robot.initial_global_y,
              robot.initial_global_yaw);

      geometry_msgs::msg::TransformStamped result;
      result.header.stamp = stamp;
      result.header.frame_id = "base_link";
      result.child_frame_id =
          expandTopic(
              tag_frame_template_,
              robot.id);
      result.transform = base_to_raw_tag;
      return result;
    }

void FootballFakeOtherRobotPublisher::publishRawTagObservations()
{
      tf2_msgs::msg::TFMessage message;
      const auto stamp = now();

      for (const auto &robot : robots_)
      {
        message.transforms.push_back(
            makeRawTagObservation(
                robot,
                stamp));
      }

      global_tf_static_pub_->publish(message);
    }

tf2_msgs::msg::TFMessage FootballFakeOtherRobotPublisher::makeTfMessage(
    const Robot &robot,
    const rclcpp::Time &stamp) const
{
      tf2_msgs::msg::TFMessage message;

      if (publish_acceptance_global_odom_)
      {
        message.transforms.push_back(
            makeTransform(
                stamp,
                field_frame_,
                "vodom",
                robot.initial_global_x,
                robot.initial_global_y,
                robot.initial_global_yaw));
      }
      else
      {
        message.transforms.push_back(
            makeTransform(
                stamp,
                "map",
                "vodom",
                0.0,
                0.0,
                0.0));
      }

      message.transforms.push_back(
          makeTransform(
              stamp,
              "vodom",
              "base_link",
              robot.local_x,
              robot.local_y,
              robot.local_yaw));

      return message;
    }

void FootballFakeOtherRobotPublisher::update()
{
      const auto stamp = now();

      double dt =
          (stamp - last_update_).seconds();

      if (dt <= 0.0 || dt > 0.5)
      {
        dt =
            publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.1;
      }

      last_update_ = stamp;

      const double elapsed_time =
          std::max(
              0.0,
              (stamp -
               simulation_start_time_)
                  .seconds());

      std::vector<Robot> proposed = robots_;

      for (auto &robot : proposed)
      {
        integrateRobot(
            robot,
            dt,
            elapsed_time,
            stamp);

        robot.local_x =
            std::clamp(
                robot.local_x,
                field_min_x_ +
                    boundary_margin_m_ -
                    robot.initial_global_x,
                field_max_x_ -
                    boundary_margin_m_ -
                    robot.initial_global_x);

        robot.local_y =
            std::clamp(
                robot.local_y,
                field_min_y_ +
                    boundary_margin_m_ -
                    robot.initial_global_y,
                field_max_y_ -
                    boundary_margin_m_ -
                    robot.initial_global_y);
      }

      const double minimum_distance =
          2.0 * robot_collision_radius_m_;

      std::vector<double> correction_x(
          proposed.size(),
          0.0);

      std::vector<double> correction_y(
          proposed.size(),
          0.0);

      for (std::size_t i = 0;
           i < proposed.size();
           ++i)
      {
        for (std::size_t j = i + 1;
             j < proposed.size();
             ++j)
        {
          const double dx =
              (proposed[j].initial_global_x +
               proposed[j].local_x) -
              (proposed[i].initial_global_x +
               proposed[i].local_x);

          const double dy =
              (proposed[j].initial_global_y +
               proposed[j].local_y) -
              (proposed[i].initial_global_y +
               proposed[i].local_y);

          const double distance =
              std::hypot(dx, dy);

          if (distance >= minimum_distance)
          {
            continue;
          }

          const double ux =
              distance > 1e-9 ? dx / distance : (proposed[i].id < proposed[j].id ? 1.0 : -1.0);

          const double uy =
              distance > 1e-9 ? dy / distance : 0.0;

          const double separation =
              minimum_distance - distance;

          const bool fixed_i =
              proposed[i].id ==
              selected_namespace_;

          const bool fixed_j =
              proposed[j].id ==
              selected_namespace_;

          if (fixed_i)
          {
            correction_x[j] +=
                separation * ux;
            correction_y[j] +=
                separation * uy;
          }
          else if (fixed_j)
          {
            correction_x[i] -=
                separation * ux;
            correction_y[i] -=
                separation * uy;
          }
          else
          {
            const double retreat =
                0.5 * separation;

            correction_x[i] -=
                retreat * ux;
            correction_y[i] -=
                retreat * uy;
            correction_x[j] +=
                retreat * ux;
            correction_y[j] +=
                retreat * uy;
          }
        }
      }

      for (std::size_t i = 0;
           i < proposed.size();
           ++i)
      {
        proposed[i].local_x =
            std::clamp(
                proposed[i].local_x +
                    correction_x[i],
                field_min_x_ +
                    boundary_margin_m_ -
                    proposed[i].initial_global_x,
                field_max_x_ -
                    boundary_margin_m_ -
                    proposed[i].initial_global_x);

        proposed[i].local_y =
            std::clamp(
                proposed[i].local_y +
                    correction_y[i],
                field_min_y_ +
                    boundary_margin_m_ -
                    proposed[i].initial_global_y,
                field_max_y_ -
                    boundary_margin_m_ -
                    proposed[i].initial_global_y);
      }

      robots_ = std::move(proposed);

      for (const auto &robot : robots_)
      {
        robot.odom_out_pub->publish(
            makeOdom(
                robot,
                stamp,
                odom_out_frame_id_,
                odom_out_child_frame_id_));

        if (robot.odom_slam_pub)
        {
          robot.odom_slam_pub->publish(
              makeOdom(
                  robot,
                  stamp,
                  "map",
                  "base_link"));
        }

        robot.namespaced_tf_pub->publish(
            makeTfMessage(
                robot,
                stamp));

        if (robot.acceptance_global_odom_pub)
        {
          robot.acceptance_global_odom_pub->publish(
              makeAcceptanceGlobalOdom(
                  robot,
                  stamp));
        }

        if (robot.integrated_cmd_vel_pub)
        {
          geometry_msgs::msg::TwistStamped integrated;
          integrated.header.stamp = stamp;
          integrated.header.frame_id = "base_link";
          integrated.twist.linear.x = robot.local_vx;
          integrated.twist.linear.y = robot.local_vy;
          integrated.twist.angular.z = robot.wz;
          robot.integrated_cmd_vel_pub->publish(integrated);
        }
      }
    }


} // namespace football_navigation
