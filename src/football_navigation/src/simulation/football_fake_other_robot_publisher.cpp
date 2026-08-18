// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>
#include <map>
#include <random>
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

    struct InitialPose2D
    {
      double x;
      double y;
      double yaw;
    };

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

      keyboard_controlled_namespace_ =
          normalizeRobotNamespace(
              declare_parameter<std::string>(
                  "keyboard_controlled_namespace",
                  ""));

      simulate_selected_robot_ =
          declare_parameter<bool>(
              "simulate_selected_robot",
              true);

      scripted_peer_motion_enabled_ =
          declare_parameter<bool>(
              "scripted_peer_motion_enabled",
              true);

      randomize_initial_poses_ = declare_parameter<bool>(
        "randomize_initial_poses", false);
      random_seed_ = declare_parameter<int>("random_seed", 2026);
      random_engine_.seed(static_cast<std::mt19937::result_type>(random_seed_));
      minimum_initial_separation_m_ = declare_parameter<double>(
        "minimum_initial_separation_m", 0.90);
      random_peer_motion_enabled_ = declare_parameter<bool>(
        "random_peer_motion_enabled", false);
      random_peer_motion_start_delay_sec_ = declare_parameter<double>(
        "random_peer_motion_start_delay_sec", 0.5);
      random_peer_min_speed_mps_ = declare_parameter<double>(
        "random_peer_min_speed_mps", 0.08);
      random_peer_max_speed_mps_ = declare_parameter<double>(
        "random_peer_max_speed_mps", 0.18);
      random_peer_min_target_duration_sec_ = declare_parameter<double>(
        "random_peer_min_target_duration_sec", 4.0);
      random_peer_max_target_duration_sec_ = declare_parameter<double>(
        "random_peer_max_target_duration_sec", 9.0);
      random_peer_goal_tolerance_m_ = declare_parameter<double>(
        "random_peer_goal_tolerance_m", 0.25);
      random_peer_min_travel_distance_m_ = declare_parameter<double>(
        "random_peer_min_travel_distance_m", 1.0);
      random_peer_turn_gain_ = declare_parameter<double>(
        "random_peer_turn_gain", 1.8);
      random_peer_min_clearance_m_ = declare_parameter<double>(
        "random_peer_min_clearance_m", 0.03);

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

      selected_global_odom_topic_ = declare_parameter<std::string>(
        "selected_global_odom_topic",
        expandTopic("/global_vio/{namespace}/odom", selected_namespace_));

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

      robot_collision_length_m_ = declare_parameter<double>(
        "robot_collision_length_m", 0.562);
      robot_collision_width_m_ = declare_parameter<double>(
        "robot_collision_width_m", 0.339);
      collision_ellipse_expansion_m_ = declare_parameter<double>(
        "collision_ellipse_expansion_m", 0.05);
      const auto collision_ellipse = makeCircumscribedCollisionEllipse(
        robot_collision_length_m_, robot_collision_width_m_, collision_ellipse_expansion_m_);
      const double maximum_collision_extent = std::max(
        collision_ellipse.semi_major_m, collision_ellipse.semi_minor_m);

      boundary_margin_m_ =
          declare_parameter<double>(
              "boundary_margin_m",
              0.10);

      if (field_min_x_ >= field_max_x_ ||
          field_min_y_ >= field_max_y_ ||
          robot_collision_length_m_ <= 0.0 ||
          robot_collision_width_m_ <= 0.0 ||
          collision_ellipse_expansion_m_ < 0.0 ||
          boundary_margin_m_ < 0.0 ||
          minimum_initial_separation_m_ < 2.0 * maximum_collision_extent)
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

      if (!keyboard_controlled_namespace_.empty() &&
          std::find(
              robot_namespaces_.begin(),
              robot_namespaces_.end(),
              keyboard_controlled_namespace_) == robot_namespaces_.end())
      {
        throw std::invalid_argument(
            "keyboard_controlled_namespace must be in robot_namespaces_csv");
      }

      if (!keyboard_controlled_namespace_.empty() &&
          keyboard_controlled_namespace_ == selected_namespace_)
      {
        throw std::invalid_argument(
            "keyboard_controlled_namespace must not be selected_namespace");
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

      selected_initial_global_x_ = declare_parameter<double>(
        "selected_initial_global_x", 0.0);
      selected_initial_global_y_ = declare_parameter<double>(
        "selected_initial_global_y", 0.0);

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

      if (random_peer_motion_enabled_ && (
          !std::isfinite(random_peer_motion_start_delay_sec_) ||
          !std::isfinite(random_peer_min_speed_mps_) ||
          !std::isfinite(random_peer_max_speed_mps_) ||
          !std::isfinite(random_peer_min_target_duration_sec_) ||
          !std::isfinite(random_peer_max_target_duration_sec_) ||
          !std::isfinite(random_peer_goal_tolerance_m_) ||
          !std::isfinite(random_peer_min_travel_distance_m_) ||
          !std::isfinite(random_peer_turn_gain_) ||
          !std::isfinite(random_peer_min_clearance_m_) ||
          random_peer_motion_start_delay_sec_ < 0.0 ||
          random_peer_min_speed_mps_ <= 0.0 ||
          random_peer_max_speed_mps_ < random_peer_min_speed_mps_ ||
          random_peer_max_speed_mps_ > max_linear_speed_mps_ ||
          random_peer_min_target_duration_sec_ <= 0.0 ||
          random_peer_max_target_duration_sec_ < random_peer_min_target_duration_sec_ ||
          random_peer_goal_tolerance_m_ <= 0.0 ||
          random_peer_min_travel_distance_m_ <= random_peer_goal_tolerance_m_ ||
          random_peer_turn_gain_ <= 0.0 ||
          random_peer_min_clearance_m_ < 0.0))
      {
        throw std::invalid_argument("invalid random peer motion parameters");
      }

      const int enabled_peer_motion_modes =
        (random_peer_motion_enabled_ ? 1 : 0) +
        (scripted_peer_motion_enabled_ ? 1 : 0) +
        (continuous_demo_enabled_ ? 1 : 0);
      if (enabled_peer_motion_modes > 1) {
        throw std::invalid_argument(
          "enable only one of random, scripted, or continuous peer motion");
      }

      if (continuous_demo_enabled_ && (!std::isfinite(
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
                  boundary_margin_m_))
      {
        throw std::invalid_argument(
            "invalid continuous demo peer trajectory parameters");
      }

      robots_.reserve(robot_namespaces_.size());

      std::map<std::string, std::vector<double>> configured_initial_poses;
      for (const auto & robot_namespace : robot_namespaces_) {
        auto pose = declare_parameter<std::vector<double>>(
          "initial_pose_" + robot_namespace, std::vector<double>{});
        if (!pose.empty() && (pose.size() != 3 ||
          !std::all_of(pose.begin(), pose.end(), [](const double value) {
            return std::isfinite(value);
          })))
        {
          throw std::invalid_argument(
            "initial_pose_<namespace> must be empty or [x, y, yaw]");
        }
        if (pose.size() == 3) {
          configured_initial_poses.emplace(robot_namespace, std::move(pose));
        }
      }

      const auto selected_configured = configured_initial_poses.find(selected_namespace_);
      if (simulate_selected_robot_ && selected_configured != configured_initial_poses.end()) {
        selected_initial_global_x_ = selected_configured->second[0];
        selected_initial_global_y_ = selected_configured->second[1];
      }

      std::uniform_real_distribution<double> random_x(
        field_min_x_ + boundary_margin_m_ + maximum_collision_extent,
        field_max_x_ - boundary_margin_m_ - maximum_collision_extent);
      std::uniform_real_distribution<double> random_y(
        field_min_y_ + boundary_margin_m_ + maximum_collision_extent,
        field_max_y_ - boundary_margin_m_ - maximum_collision_extent);
      std::uniform_real_distribution<double> random_yaw(-kPi, kPi);
      std::vector<InitialPose2D> occupied_poses{
        {selected_initial_global_x_, selected_initial_global_y_, 0.0}};
      for (const auto & configured : configured_initial_poses) {
        if (configured.first == selected_namespace_) {
          continue;
        }
        const double configured_x = configured.second[0];
        const double configured_y = configured.second[1];
        const double configured_yaw = configured.second[2];
        if (configured_x <= field_min_x_ + boundary_margin_m_ + maximum_collision_extent ||
          configured_x >= field_max_x_ - boundary_margin_m_ - maximum_collision_extent ||
          configured_y <= field_min_y_ + boundary_margin_m_ + maximum_collision_extent ||
          configured_y >= field_max_y_ - boundary_margin_m_ - maximum_collision_extent ||
          std::any_of(occupied_poses.begin(), occupied_poses.end(),
          [&collision_ellipse, configured_x, configured_y, configured_yaw](const auto & occupied) {
            return orientedEllipseClearance(
              configured_x, configured_y, configured_yaw, collision_ellipse,
              occupied.x, occupied.y, occupied.yaw, collision_ellipse) < 0.0;
          }))
        {
          throw std::invalid_argument(
            "configured initial pose for " + configured.first +
            " must stay inside the field and not overlap");
        }
        occupied_poses.push_back({configured_x, configured_y, configured_yaw});
      }

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
        const auto configured_pose = configured_initial_poses.find(robot_namespace);
        if (configured_pose != configured_initial_poses.end()) {
          robot.initial_global_x = configured_pose->second[0];
          robot.initial_global_y = configured_pose->second[1];
          robot.initial_global_yaw = configured_pose->second[2];
        } else if (randomize_initial_poses_) {
          bool placed = false;
          for (int attempt = 0; attempt < 1000 && !placed; ++attempt) {
            const double candidate_x = random_x(random_engine_);
            const double candidate_y = random_y(random_engine_);
            const double candidate_yaw = random_yaw(random_engine_);
            placed = std::all_of(
              occupied_poses.begin(), occupied_poses.end(),
              [this, &collision_ellipse, candidate_x, candidate_y, candidate_yaw](
                const auto & occupied) {
                return std::hypot(candidate_x - occupied.x, candidate_y - occupied.y) >=
                       minimum_initial_separation_m_ &&
                       orientedEllipseClearance(
                         candidate_x, candidate_y, candidate_yaw, collision_ellipse,
                         occupied.x, occupied.y, occupied.yaw, collision_ellipse) >= 0.0;
              });
            if (placed) {
              robot.initial_global_x = candidate_x;
              robot.initial_global_y = candidate_y;
              robot.initial_global_yaw = candidate_yaw;
              occupied_poses.push_back({candidate_x, candidate_y, candidate_yaw});
            }
          }
          if (!placed) {
            throw std::runtime_error(
              "unable to place all simulated robots with requested separation");
          }
        } else {
          robot.initial_global_x = 0.5 + 1.2 * static_cast<double>(index % 5);
          robot.initial_global_y = index < 5 ? -1.2 : 1.2;
          robot.initial_global_yaw = index < 5 ? 0.0 : kPi;
        }
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

      if (!simulate_selected_robot_) {
        selected_global_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          selected_global_odom_topic_, rclcpp::SensorDataQoS().keep_last(5),
          [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            if (!msg || !std::isfinite(msg->pose.pose.position.x) ||
              !std::isfinite(msg->pose.pose.position.y))
            {
              return;
            }
            selected_global_x_ = msg->pose.pose.position.x;
            selected_global_y_ = msg->pose.pose.position.y;
            have_selected_global_pose_ = yawFromQuaternion(
              msg->pose.pose.orientation, selected_global_yaw_);
          });
      }

      publishRawTagObservations();

      simulation_start_time_ = now();
      last_update_ = simulation_start_time_;

      RCLCPP_INFO(
        get_logger(),
        "fake multi-robot motion: selected=%s externally_driven=%d peers=%zu "
        "keyboard=%s mode=%s random_speed=[%.2f,%.2f] seed=%d",
        selected_namespace_.c_str(), !simulate_selected_robot_, robots_.size(),
        keyboard_controlled_namespace_.empty() ? "none" : keyboard_controlled_namespace_.c_str(),
        random_peer_motion_enabled_ ? "random_waypoint" :
        (continuous_demo_enabled_ ? "continuous_demo" :
        (scripted_peer_motion_enabled_ ? "scripted" : "stationary")),
        random_peer_min_speed_mps_, random_peer_max_speed_mps_, random_seed_);

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

void FootballFakeOtherRobotPublisher::assignRandomPeerTarget(
    Robot &robot,
    const double elapsed_time)
{
      const auto collision_ellipse = makeCircumscribedCollisionEllipse(
        robot_collision_length_m_, robot_collision_width_m_, collision_ellipse_expansion_m_);
      const double boundary_extent = std::max(
        collision_ellipse.semi_major_m, collision_ellipse.semi_minor_m);
      std::uniform_real_distribution<double> target_x_distribution(
        field_min_x_ + boundary_margin_m_ + boundary_extent,
        field_max_x_ - boundary_margin_m_ - boundary_extent);
      std::uniform_real_distribution<double> target_y_distribution(
        field_min_y_ + boundary_margin_m_ + boundary_extent,
        field_max_y_ - boundary_margin_m_ - boundary_extent);
      std::uniform_real_distribution<double> speed_distribution(
        random_peer_min_speed_mps_, random_peer_max_speed_mps_);
      std::uniform_real_distribution<double> duration_distribution(
        random_peer_min_target_duration_sec_, random_peer_max_target_duration_sec_);

      const double cosine = std::cos(robot.initial_global_yaw);
      const double sine = std::sin(robot.initial_global_yaw);
      const double current_x = robot.initial_global_x +
        cosine * robot.local_x - sine * robot.local_y;
      const double current_y = robot.initial_global_y +
        sine * robot.local_x + cosine * robot.local_y;

      for (int attempt = 0; attempt < 64; ++attempt) {
        const double target_x = target_x_distribution(random_engine_);
        const double target_y = target_y_distribution(random_engine_);
        if (std::hypot(target_x - current_x, target_y - current_y) <
          random_peer_min_travel_distance_m_)
        {
          continue;
        }
        bool separated = std::hypot(
          target_x - selected_initial_global_x_,
          target_y - selected_initial_global_y_) >= minimum_initial_separation_m_;
        for (const auto &other : robots_) {
          if (!separated || other.id == robot.id) {
            continue;
          }
          const double other_cosine = std::cos(other.initial_global_yaw);
          const double other_sine = std::sin(other.initial_global_yaw);
          const double other_x = other.initial_global_x +
            other_cosine * other.local_x - other_sine * other.local_y;
          const double other_y = other.initial_global_y +
            other_sine * other.local_x + other_cosine * other.local_y;
          separated = std::hypot(target_x - other_x, target_y - other_y) >=
            minimum_initial_separation_m_;
        }
        if (!separated) {
          continue;
        }
        robot.random_target_global_x = target_x;
        robot.random_target_global_y = target_y;
        robot.random_cruise_speed_mps = speed_distribution(random_engine_);
        robot.random_target_expiry_sec = elapsed_time + duration_distribution(random_engine_);
        robot.have_random_target = true;
        return;
      }

      robot.have_random_target = false;
      robot.random_target_expiry_sec = elapsed_time + 1.0;
    }

void FootballFakeOtherRobotPublisher::setRandomPeerMotion(
    Robot &robot,
    const double dt,
    const double elapsed_time)
{
      const double initial_cosine = std::cos(robot.initial_global_yaw);
      const double initial_sine = std::sin(robot.initial_global_yaw);
      const double current_x = robot.initial_global_x +
        initial_cosine * robot.local_x - initial_sine * robot.local_y;
      const double current_y = robot.initial_global_y +
        initial_sine * robot.local_x + initial_cosine * robot.local_y;
      const double target_distance = robot.have_random_target ? std::hypot(
        robot.random_target_global_x - current_x,
        robot.random_target_global_y - current_y) : 0.0;
      if (!robot.have_random_target ||
        target_distance <= random_peer_goal_tolerance_m_ ||
        elapsed_time >= robot.random_target_expiry_sec)
      {
        assignRandomPeerTarget(robot, elapsed_time);
      }
      if (!robot.have_random_target) {
        return;
      }

      const double desired_global_yaw = std::atan2(
        robot.random_target_global_y - current_y,
        robot.random_target_global_x - current_x);
      const double current_global_yaw = normalizeAngle(
        robot.initial_global_yaw + robot.local_yaw);
      const double yaw_error = signedYawError(desired_global_yaw, current_global_yaw);
      const double heading_scale = std::fabs(yaw_error) >= 0.5 * kPi ?
        0.0 : std::max(0.0, std::cos(yaw_error));
      const double vx = robot.random_cruise_speed_mps * heading_scale;
      const double wz = std::clamp(
        random_peer_turn_gain_ * yaw_error,
        -max_angular_speed_rps_, max_angular_speed_rps_);

      robot.local_x += std::cos(robot.local_yaw) * vx * dt;
      robot.local_y += std::sin(robot.local_yaw) * vx * dt;
      robot.local_yaw = normalizeAngle(robot.local_yaw + wz * dt);
      robot.local_vx = vx;
      robot.local_vy = 0.0;
      robot.wz = wz;
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

      if (random_peer_motion_enabled_ && robot.id != selected_namespace_ &&
          robot.id != keyboard_controlled_namespace_)
      {
        if (elapsed_time < random_peer_motion_start_delay_sec_) {
          return;
        }
        setRandomPeerMotion(
          robot, dt, elapsed_time - random_peer_motion_start_delay_sec_);
        return;
      }

      if (continuous_demo_enabled_ &&
          robot.id != selected_namespace_ &&
          robot.id != keyboard_controlled_namespace_)
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
          robot.id != keyboard_controlled_namespace_ &&
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

std::pair<double, double> FootballFakeOtherRobotPublisher::robotGlobalPosition(
  const Robot & robot) const
{
  const double cosine = std::cos(robot.initial_global_yaw);
  const double sine = std::sin(robot.initial_global_yaw);
  return {
    robot.initial_global_x + cosine * robot.local_x - sine * robot.local_y,
    robot.initial_global_y + sine * robot.local_x + cosine * robot.local_y};
}

void FootballFakeOtherRobotPublisher::setRobotGlobalPosition(
  Robot & robot, const double global_x, const double global_y) const
{
  const double dx = global_x - robot.initial_global_x;
  const double dy = global_y - robot.initial_global_y;
  const double cosine = std::cos(robot.initial_global_yaw);
  const double sine = std::sin(robot.initial_global_yaw);
  robot.local_x = cosine * dx + sine * dy;
  robot.local_y = -sine * dx + cosine * dy;
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
      std::vector<bool> random_motion_interrupted(proposed.size(), false);
      const auto collision_ellipse = makeCircumscribedCollisionEllipse(
        robot_collision_length_m_, robot_collision_width_m_, collision_ellipse_expansion_m_);

      for (std::size_t index = 0; index < proposed.size(); ++index)
      {
        auto &robot = proposed[index];
        integrateRobot(
            robot,
            dt,
            elapsed_time,
            stamp);

        const double robot_yaw = robot.initial_global_yaw + robot.local_yaw;
        const double x_extent = ellipseSupportRadius(collision_ellipse, robot_yaw, 1.0, 0.0);
        const double y_extent = ellipseSupportRadius(collision_ellipse, robot_yaw, 0.0, 1.0);

        const auto [unclamped_x, unclamped_y] = robotGlobalPosition(robot);
        const double clamped_x = std::clamp(
          unclamped_x, field_min_x_ + boundary_margin_m_ + x_extent,
          field_max_x_ - boundary_margin_m_ - x_extent);
        const double clamped_y = std::clamp(
          unclamped_y, field_min_y_ + boundary_margin_m_ + y_extent,
          field_max_y_ - boundary_margin_m_ - y_extent);
        setRobotGlobalPosition(robot, clamped_x, clamped_y);
        random_motion_interrupted[index] =
          std::fabs(clamped_x - unclamped_x) > 1e-9 ||
          std::fabs(clamped_y - unclamped_y) > 1e-9;
      }

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
          const auto [first_x, first_y] = robotGlobalPosition(proposed[i]);
          const auto [second_x, second_y] = robotGlobalPosition(proposed[j]);
          const double dx = second_x - first_x;
          const double dy = second_y - first_y;

          const double distance = std::hypot(dx, dy);
          const double clearance = orientedEllipseClearance(
            first_x,
            first_y,
            proposed[i].initial_global_yaw + proposed[i].local_yaw,
            collision_ellipse,
            second_x,
            second_y,
            proposed[j].initial_global_yaw + proposed[j].local_yaw,
            collision_ellipse);

          if (clearance >= random_peer_min_clearance_m_)
          {
            continue;
          }

          const double ux =
              distance > 1e-9 ? dx / distance : (proposed[i].id < proposed[j].id ? 1.0 : -1.0);

          const double uy =
              distance > 1e-9 ? dy / distance : 0.0;

          const double separation = random_peer_min_clearance_m_ - clearance;
          random_motion_interrupted[i] = true;
          random_motion_interrupted[j] = true;

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

      // The striker is integrated by the separate simulation input node.  Make
      // random peers yield to its latest global pose so both simulators share
      // the same no-overlap invariant.
      if (!simulate_selected_robot_ && have_selected_global_pose_) {
        for (std::size_t i = 0; i < proposed.size(); ++i) {
          const auto [peer_x, peer_y] = robotGlobalPosition(proposed[i]);
          const double dx = peer_x - selected_global_x_;
          const double dy = peer_y - selected_global_y_;
          const double distance = std::hypot(dx, dy);
          const double clearance = orientedEllipseClearance(
            selected_global_x_, selected_global_y_, selected_global_yaw_, collision_ellipse,
            peer_x, peer_y,
            proposed[i].initial_global_yaw + proposed[i].local_yaw, collision_ellipse);
          if (clearance < random_peer_min_clearance_m_) {
            const double ux = distance > 1e-9 ? dx / distance : 1.0;
            const double uy = distance > 1e-9 ? dy / distance : 0.0;
            const double separation = random_peer_min_clearance_m_ - clearance;
            correction_x[i] += separation * ux;
            correction_y[i] += separation * uy;
            random_motion_interrupted[i] = true;
          }
        }
      }

      for (std::size_t i = 0;
           i < proposed.size();
           ++i)
      {
        const double robot_yaw = proposed[i].initial_global_yaw + proposed[i].local_yaw;
        const double x_extent = ellipseSupportRadius(collision_ellipse, robot_yaw, 1.0, 0.0);
        const double y_extent = ellipseSupportRadius(collision_ellipse, robot_yaw, 0.0, 1.0);
        const auto [global_x, global_y] = robotGlobalPosition(proposed[i]);
        setRobotGlobalPosition(
          proposed[i],
          std::clamp(global_x + correction_x[i],
            field_min_x_ + boundary_margin_m_ + x_extent,
            field_max_x_ - boundary_margin_m_ - x_extent),
          std::clamp(global_y + correction_y[i],
            field_min_y_ + boundary_margin_m_ + y_extent,
            field_max_y_ - boundary_margin_m_ - y_extent));
        if (random_motion_interrupted[i] && random_peer_motion_enabled_) {
          proposed[i].have_random_target = false;
          proposed[i].local_vx = 0.0;
          proposed[i].local_vy = 0.0;
          proposed[i].wz = 0.0;
        }
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
