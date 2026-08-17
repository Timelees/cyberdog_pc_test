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
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "football_navigation/simulation/football_simulation_navigator.hpp"

namespace football_navigation
{
namespace
{
std::vector<std::string> splitCsv(const std::string & csv)
{
  std::vector<std::string> output;
  std::stringstream stream(csv);
  std::string value;
  while (std::getline(stream, value, ',')) {
    const auto first = value.find_first_not_of(" \t\r\n/");
    const auto last = value.find_last_not_of(" \t\r\n/");
    if (first != std::string::npos) {
      output.push_back(value.substr(first, last - first + 1));
    }
  }
  return output;
}

std::string expandTopic(std::string pattern, const std::string & robot_namespace)
{
  const std::string token = "{namespace}";
  const auto position = pattern.find(token);
  if (position == std::string::npos) {
    throw std::invalid_argument("robot_odom_topic_template must contain {namespace}");
  }
  pattern.replace(position, token.size(), robot_namespace);
  return pattern;
}

double pointToSegmentDistance(
  const double px, const double py,
  const double ax, const double ay,
  const double bx, const double by,
  double & projection)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_squared = dx * dx + dy * dy;
  projection = length_squared > 1e-9 ?
    std::clamp(((px - ax) * dx + (py - ay) * dy) / length_squared, 0.0, 1.0) : 0.0;
  return std::hypot(px - (ax + projection * dx), py - (ay + projection * dy));
}
}  // namespace

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
    self_namespace_ = declare_parameter<std::string>("self_namespace", "cyberdog_1");
    robot_namespaces_csv_ = declare_parameter<std::string>(
      "robot_namespaces_csv",
      "cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5,"
      "cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10");
    robot_odom_topic_template_ = declare_parameter<std::string>(
      "robot_odom_topic_template", "/global_vio/{namespace}/odom");
    local_plan_topic_ = declare_parameter<std::string>("local_plan_topic", "local_plan");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
    linear_gain_ = declare_parameter<double>("linear_gain", 0.9);
    angular_gain_ = declare_parameter<double>("angular_gain", 1.5);
    push_bearing_gain_ = declare_parameter<double>("push_bearing_gain", 0.6);
    max_linear_speed_mps_ = declare_parameter<double>("max_linear_speed_mps", 0.45);
    max_angular_speed_rps_ = declare_parameter<double>("max_angular_speed_rps", 0.8);
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.06);
    yaw_tolerance_rad_ = declare_parameter<double>("yaw_tolerance_rad", 0.08);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);
    other_robot_timeout_sec_ = declare_parameter<double>("other_robot_timeout_sec", 0.7);
    robot_collision_length_m_ = declare_parameter<double>(
      "robot_collision_length_m", 0.562);
    robot_collision_width_m_ = declare_parameter<double>(
      "robot_collision_width_m", 0.339);
    collision_ellipse_expansion_m_ = declare_parameter<double>(
      "collision_ellipse_expansion_m", 0.05);
    collision_path_clearance_m_ = declare_parameter<double>(
      "collision_path_clearance_m", 0.08);
    collision_slowdown_clearance_m_ = declare_parameter<double>(
      "collision_slowdown_clearance_m", 0.35);
    collision_hard_stop_clearance_m_ = declare_parameter<double>(
      "collision_hard_stop_clearance_m", 0.02);
    clearance_recovery_trigger_clearance_m_ = declare_parameter<double>(
      "clearance_recovery_trigger_clearance_m", 0.16);
    clearance_recovery_exit_clearance_m_ = declare_parameter<double>(
      "clearance_recovery_exit_clearance_m", 0.24);
    clearance_recovery_distance_m_ = declare_parameter<double>(
      "clearance_recovery_distance_m", 1.0);
    clearance_recovery_speed_mps_ = declare_parameter<double>(
      "clearance_recovery_speed_mps", 0.20);
    detour_extra_clearance_m_ = declare_parameter<double>(
      "detour_extra_clearance_m", 0.08);
    local_path_lookahead_m_ = declare_parameter<double>("local_path_lookahead_m", 0.32);
    path_heading_gain_ = declare_parameter<double>("path_heading_gain", 2.0);
    turn_in_place_threshold_rad_ = declare_parameter<double>(
      "turn_in_place_threshold_rad", 0.75);
    smooth_path_samples_ = declare_parameter<int>("smooth_path_samples", 41);
    multi_obstacle_lattice_stations_ = declare_parameter<int>(
      "multi_obstacle_lattice_stations", 17);
    multi_obstacle_lateral_step_m_ = declare_parameter<double>(
      "multi_obstacle_lateral_step_m", 0.18);
    multi_obstacle_max_lateral_m_ = declare_parameter<double>(
      "multi_obstacle_max_lateral_m", 3.0);
    multi_obstacle_max_lane_change_m_ = declare_parameter<double>(
      "multi_obstacle_max_lane_change_m", 0.54);
    multi_obstacle_turn_penalty_ = declare_parameter<double>(
      "multi_obstacle_turn_penalty", 0.25);
    dynamic_replan_min_period_sec_ = declare_parameter<double>(
      "dynamic_replan_min_period_sec", 0.30);
    dynamic_replan_translation_m_ = declare_parameter<double>(
      "dynamic_replan_translation_m", 0.08);
    dynamic_replan_yaw_rad_ = declare_parameter<double>(
      "dynamic_replan_yaw_rad", 0.18);
    field_min_x_ = declare_parameter<double>("field_min_x", -8.0);
    field_max_x_ = declare_parameter<double>("field_max_x", 8.0);
    field_min_y_ = declare_parameter<double>("field_min_y", -4.0);
    field_max_y_ = declare_parameter<double>("field_max_y", 4.0);

    if (field_frame_.empty() || control_rate_hz_ <= 0.0 || linear_gain_ <= 0.0 ||
      angular_gain_ <= 0.0 || push_bearing_gain_ < 0.0 || max_linear_speed_mps_ <= 0.0 ||
      max_angular_speed_rps_ <= 0.0 || position_tolerance_m_ <= 0.0 ||
      yaw_tolerance_rad_ <= 0.0 || odom_timeout_sec_ <= 0.0 ||
      other_robot_timeout_sec_ <= 0.0 || robot_collision_length_m_ <= 0.0 ||
      robot_collision_width_m_ <= 0.0 || collision_ellipse_expansion_m_ < 0.0 ||
      collision_path_clearance_m_ <= collision_hard_stop_clearance_m_ ||
      collision_hard_stop_clearance_m_ < 0.0 ||
      collision_slowdown_clearance_m_ <= collision_hard_stop_clearance_m_ ||
      clearance_recovery_trigger_clearance_m_ <= collision_hard_stop_clearance_m_ ||
      clearance_recovery_trigger_clearance_m_ >= clearance_recovery_exit_clearance_m_ ||
      clearance_recovery_exit_clearance_m_ <= collision_hard_stop_clearance_m_ ||
      clearance_recovery_exit_clearance_m_ > collision_slowdown_clearance_m_ ||
      clearance_recovery_distance_m_ <= 0.0 ||
      clearance_recovery_speed_mps_ <= 0.0 ||
      clearance_recovery_speed_mps_ > max_linear_speed_mps_ ||
      detour_extra_clearance_m_ <= 0.0 || local_path_lookahead_m_ <= 0.0 ||
      path_heading_gain_ <= 0.0 || turn_in_place_threshold_rad_ <= 0.0 ||
      smooth_path_samples_ < 15 || smooth_path_samples_ > 201 ||
      multi_obstacle_lattice_stations_ < 7 || multi_obstacle_lattice_stations_ > 61 ||
      multi_obstacle_lateral_step_m_ <= 0.0 || multi_obstacle_max_lateral_m_ <= 0.0 ||
      multi_obstacle_max_lane_change_m_ < multi_obstacle_lateral_step_m_ ||
      multi_obstacle_turn_penalty_ < 0.0 ||
      dynamic_replan_min_period_sec_ <= 0.0 || dynamic_replan_translation_m_ <= 0.0 ||
      dynamic_replan_yaw_rad_ <= 0.0 ||
      field_min_x_ >= field_max_x_ || field_min_y_ >= field_max_y_)
    {
      throw std::invalid_argument("invalid simulation navigator parameters");
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    push_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(push_cmd_vel_topic_, 10);
    local_plan_pub_ = create_publisher<nav_msgs::msg::Path>(local_plan_topic_, 10);
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
    robot_namespaces_ = splitCsv(robot_namespaces_csv_);
    for (const auto & robot_namespace : robot_namespaces_) {
      if (robot_namespace == self_namespace_) {
        continue;
      }
      other_robot_odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        expandTopic(robot_odom_topic_template_, robot_namespace),
        rclcpp::SensorDataQoS().keep_last(5),
        [this, robot_namespace](const nav_msgs::msg::Odometry::SharedPtr msg) {
          otherRobotOdomCallback(robot_namespace, msg);
        }));
    }
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

void FootballSimulationNavigator::otherRobotOdomCallback(
  const std::string & robot_namespace,
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg || msg->header.frame_id != field_frame_) {
    return;
  }
  double yaw = 0.0;
  if (!std::isfinite(msg->pose.pose.position.x) ||
    !std::isfinite(msg->pose.pose.position.y) ||
    !yawFromQuaternion(msg->pose.pose.orientation, yaw))
  {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  other_robot_odoms_[robot_namespace] = *msg;
  other_robot_times_.insert_or_assign(robot_namespace, now());
}

double FootballSimulationNavigator::nearestRobotClearance(
  const double robot_x, const double robot_y, const double robot_yaw)
{
  double nearest = std::numeric_limits<double>::infinity();
  const auto stamp = now();
  const auto collision_ellipse = makeCircumscribedCollisionEllipse(
    robot_collision_length_m_, robot_collision_width_m_, collision_ellipse_expansion_m_);
  for (const auto & entry : other_robot_odoms_) {
    const auto received = other_robot_times_.find(entry.first);
    if (received == other_robot_times_.end() ||
      (stamp - received->second).seconds() > other_robot_timeout_sec_)
    {
      continue;
    }
    double obstacle_yaw = 0.0;
    if (!yawFromQuaternion(entry.second.pose.pose.orientation, obstacle_yaw)) {
      continue;
    }
    nearest = std::min(nearest, orientedEllipseClearance(
      robot_x, robot_y, robot_yaw, collision_ellipse,
      entry.second.pose.pose.position.x, entry.second.pose.pose.position.y,
      obstacle_yaw, collision_ellipse));
  }
  return nearest;
}

bool FootballSimulationNavigator::buildClearanceRecoveryPath(
  const double robot_x, const double robot_y, const double robot_yaw)
{
  const auto stamp = now();
  const auto collision_ellipse = makeCircumscribedCollisionEllipse(
    robot_collision_length_m_, robot_collision_width_m_, collision_ellipse_expansion_m_);
  struct Obstacle
  {
    double x;
    double y;
    double yaw;
  };
  std::vector<Obstacle> obstacles;
  double initial_clearance = std::numeric_limits<double>::infinity();
  double nearest_x = robot_x;
  double nearest_y = robot_y;
  for (const auto & entry : other_robot_odoms_) {
    const auto received = other_robot_times_.find(entry.first);
    double obstacle_yaw = 0.0;
    if (received == other_robot_times_.end() ||
      (stamp - received->second).seconds() > other_robot_timeout_sec_ ||
      !yawFromQuaternion(entry.second.pose.pose.orientation, obstacle_yaw))
    {
      continue;
    }
    const auto & position = entry.second.pose.pose.position;
    const double clearance = orientedEllipseClearance(
      robot_x, robot_y, robot_yaw, collision_ellipse,
      position.x, position.y, obstacle_yaw, collision_ellipse);
    if (clearance < initial_clearance) {
      initial_clearance = clearance;
      nearest_x = position.x;
      nearest_y = position.y;
    }
    obstacles.push_back({position.x, position.y, obstacle_yaw});
  }
  if (obstacles.empty() || initial_clearance >= clearance_recovery_exit_clearance_m_) {
    clearance_recovery_active_ = false;
    return false;
  }

  const double away_heading = std::atan2(robot_y - nearest_y, robot_x - nearest_x);
  constexpr int kHeadingSamples = 24;
  constexpr int kPathSamples = 21;
  double best_score = -std::numeric_limits<double>::infinity();
  std::vector<std::pair<double, double>> best_path;
  for (int heading_index = 0; heading_index < kHeadingSamples; ++heading_index) {
    const double heading = away_heading + 2.0 * M_PI *
      static_cast<double>(heading_index) / static_cast<double>(kHeadingSamples);
    const double direction_x = std::cos(heading);
    const double direction_y = std::sin(heading);
    double previous_clearance = initial_clearance;
    double endpoint_clearance = std::numeric_limits<double>::infinity();
    bool valid = true;
    std::vector<std::pair<double, double>> candidate;
    candidate.reserve(kPathSamples);
    candidate.emplace_back(robot_x, robot_y);
    for (int sample = 1; sample < kPathSamples; ++sample) {
      const double distance = clearance_recovery_distance_m_ *
        static_cast<double>(sample) / static_cast<double>(kPathSamples - 1);
      const double x = robot_x + direction_x * distance;
      const double y = robot_y + direction_y * distance;
      const double x_extent = ellipseSupportRadius(collision_ellipse, heading, 1.0, 0.0);
      const double y_extent = ellipseSupportRadius(collision_ellipse, heading, 0.0, 1.0);
      if (x <= field_min_x_ + x_extent || x >= field_max_x_ - x_extent ||
        y <= field_min_y_ + y_extent || y >= field_max_y_ - y_extent)
      {
        valid = false;
        break;
      }
      double sample_clearance = std::numeric_limits<double>::infinity();
      for (const auto & obstacle : obstacles) {
        sample_clearance = std::min(sample_clearance, orientedEllipseClearance(
          x, y, heading, collision_ellipse,
          obstacle.x, obstacle.y, obstacle.yaw, collision_ellipse));
      }
      // A recovery route may begin inside the hard-stop domain, but every
      // translational sample must move toward greater clearance.
      if (sample_clearance + 0.005 < previous_clearance) {
        valid = false;
        break;
      }
      previous_clearance = sample_clearance;
      endpoint_clearance = sample_clearance;
      candidate.emplace_back(x, y);
    }
    if (!valid || endpoint_clearance < clearance_recovery_exit_clearance_m_) {
      continue;
    }
    const double away_alignment = std::cos(signedYawError(heading, away_heading));
    const double heading_change = std::fabs(signedYawError(heading, robot_yaw));
    const double score = endpoint_clearance + 0.06 * away_alignment -
      0.01 * heading_change;
    if (score > best_score) {
      best_score = score;
      best_path = std::move(candidate);
    }
  }
  if (best_path.empty()) {
    clearance_recovery_active_ = false;
    return false;
  }

  local_path_points_ = std::move(best_path);
  avoidance_active_ = true;
  clearance_recovery_active_ = true;
  avoidance_strategy_ = "clearance_recovery";
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "close obstacle clearance=%.3f: following active recovery path to %.3f m",
    initial_clearance, clearance_recovery_exit_clearance_m_);
  return true;
}

bool FootballSimulationNavigator::buildSmoothLocalPath(
  const double robot_x, const double robot_y,
  const double target_x, const double target_y)
{
  const auto stamp = now();
  const bool same_target = have_local_path_target_ &&
    std::hypot(target_x - local_path_target_x_, target_y - local_path_target_y_) < 0.08;
  if (!isPushState() && same_target && !local_path_points_.empty()) {
    if (last_local_path_plan_time_.nanoseconds() > 0 &&
      (stamp - last_local_path_plan_time_).seconds() < dynamic_replan_min_period_sec_)
    {
      return avoidance_active_;
    }
    bool obstacle_changed = false;
    std::size_t fresh_obstacle_count = 0;
    for (const auto & entry : other_robot_odoms_) {
      const auto received = other_robot_times_.find(entry.first);
      if (received == other_robot_times_.end() ||
        (stamp - received->second).seconds() > other_robot_timeout_sec_)
      {
        continue;
      }
      ++fresh_obstacle_count;
      double yaw = 0.0;
      const auto planned = planned_obstacle_poses_.find(entry.first);
      if (!yawFromQuaternion(entry.second.pose.pose.orientation, yaw) ||
        planned == planned_obstacle_poses_.end() ||
        std::hypot(
          entry.second.pose.pose.position.x - planned->second[0],
          entry.second.pose.pose.position.y - planned->second[1]) >=
        dynamic_replan_translation_m_ ||
        std::fabs(signedYawError(yaw, planned->second[2])) >= dynamic_replan_yaw_rad_)
      {
        obstacle_changed = true;
        break;
      }
    }
    if (!obstacle_changed && fresh_obstacle_count == planned_obstacle_poses_.size()) {
      return avoidance_active_;
    }
  }

  last_local_path_plan_time_ = stamp;
  planned_obstacle_poses_.clear();
  for (const auto & entry : other_robot_odoms_) {
    const auto received = other_robot_times_.find(entry.first);
    double yaw = 0.0;
    if (received != other_robot_times_.end() &&
      (stamp - received->second).seconds() <= other_robot_timeout_sec_ &&
      yawFromQuaternion(entry.second.pose.pose.orientation, yaw))
    {
      planned_obstacle_poses_[entry.first] = {
        entry.second.pose.pose.position.x, entry.second.pose.pose.position.y, yaw};
    }
  }
  const double path_dx = target_x - robot_x;
  const double path_dy = target_y - robot_y;
  const double path_length = std::hypot(path_dx, path_dy);
  const std::string previous_strategy = avoidance_strategy_;
  avoidance_strategy_ = "planning";
  local_path_points_.clear();
  local_path_target_x_ = target_x;
  local_path_target_y_ = target_y;
  have_local_path_target_ = true;
  if (path_length <= position_tolerance_m_) {
    local_path_points_.emplace_back(robot_x, robot_y);
    return false;
  }

  const auto collision_ellipse = makeCircumscribedCollisionEllipse(
    robot_collision_length_m_, robot_collision_width_m_, collision_ellipse_expansion_m_);
  const double path_yaw = std::atan2(path_dy, path_dx);
  const nav_msgs::msg::Odometry * blocker = nullptr;
  double blocker_projection = 0.0;
  double closest_along_path = std::numeric_limits<double>::infinity();
  std::size_t direct_blocker_count = 0;
  for (const auto & entry : other_robot_odoms_) {
    const auto received = other_robot_times_.find(entry.first);
    if (received == other_robot_times_.end() ||
      (stamp - received->second).seconds() > other_robot_timeout_sec_)
    {
      continue;
    }
    double projection = 0.0;
    const auto & position = entry.second.pose.pose.position;
    pointToSegmentDistance(
      position.x, position.y, robot_x, robot_y, target_x, target_y, projection);
    double obstacle_yaw = 0.0;
    if (!yawFromQuaternion(entry.second.pose.pose.orientation, obstacle_yaw)) {
      continue;
    }
    const double projected_x = robot_x + projection * path_dx;
    const double projected_y = robot_y + projection * path_dy;
    const double clearance = orientedEllipseClearance(
      projected_x, projected_y, path_yaw, collision_ellipse,
      position.x, position.y, obstacle_yaw, collision_ellipse);
    if (projection > 0.08 && projection < 0.94 &&
      clearance < collision_path_clearance_m_)
    {
      ++direct_blocker_count;
      if (projection < closest_along_path) {
        blocker = &entry.second;
        blocker_projection = projection;
        closest_along_path = projection;
      }
    }
  }
  const double unit_x = path_dx / path_length;
  const double unit_y = path_dy / path_length;
  const double perpendicular_x = -unit_y;
  const double perpendicular_y = unit_x;
  const auto make_curve = [&](const double side, const double amplitude) {
      std::vector<std::pair<double, double>> curve;
      curve.reserve(static_cast<std::size_t>(smooth_path_samples_));
      for (int index = 0; index < smooth_path_samples_; ++index) {
        const double progress = static_cast<double>(index) /
          static_cast<double>(smooth_path_samples_ - 1);
        const double sine = std::sin(3.14159265358979323846 * progress);
        const double lateral = side * amplitude * sine * sine;
        curve.emplace_back(
          robot_x + progress * path_dx + perpendicular_x * lateral,
          robot_y + progress * path_dy + perpendicular_y * lateral);
      }
      return curve;
    };

  if (!blocker || isPushState()) {
    local_path_points_ = make_curve(0.0, 0.0);
    if (avoidance_active_) {
      RCLCPP_INFO(get_logger(), "smooth local avoidance cleared; direct path restored");
    }
    avoidance_active_ = false;
    avoidance_strategy_ = "direct";
    return false;
  }

  const auto curveClearance = [&](const std::vector<std::pair<double, double>> & curve) {
      double minimum = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < curve.size(); ++index) {
        const auto & point = curve[index];
        const std::size_t previous = index > 0 ? index - 1 : 0;
        const std::size_t next = std::min(index + 1, curve.size() - 1);
        const double point_yaw = std::atan2(
          curve[next].second - curve[previous].second,
          curve[next].first - curve[previous].first);
        const double x_radius = ellipseSupportRadius(collision_ellipse, point_yaw, 1.0, 0.0);
        const double y_radius = ellipseSupportRadius(collision_ellipse, point_yaw, 0.0, 1.0);
        if (point.first <= field_min_x_ + x_radius ||
          point.first >= field_max_x_ - x_radius ||
          point.second <= field_min_y_ + y_radius ||
          point.second >= field_max_y_ - y_radius)
        {
          return -1.0;
        }
        for (const auto & entry : other_robot_odoms_) {
          const auto received = other_robot_times_.find(entry.first);
          if (received == other_robot_times_.end() ||
            (stamp - received->second).seconds() > other_robot_timeout_sec_)
          {
            continue;
          }
          double obstacle_yaw = 0.0;
          if (!yawFromQuaternion(entry.second.pose.pose.orientation, obstacle_yaw)) {
            continue;
          }
          minimum = std::min(minimum, orientedEllipseClearance(
            point.first, point.second, point_yaw, collision_ellipse,
            entry.second.pose.pose.position.x, entry.second.pose.pose.position.y,
            obstacle_yaw, collision_ellipse));
        }
      }
      return minimum;
    };

  const double sine_at_blocker = std::sin(
    3.14159265358979323846 * blocker_projection);
  double blocker_yaw = 0.0;
  yawFromQuaternion(blocker->pose.pose.orientation, blocker_yaw);
  const auto & blocker_position = blocker->pose.pose.position;
  const double blocker_lateral =
    (blocker_position.x - robot_x) * perpendicular_x +
    (blocker_position.y - robot_y) * perpendicular_y;
  const double required_center_separation =
    ellipseSupportRadius(collision_ellipse, path_yaw, perpendicular_x, perpendicular_y) +
    ellipseSupportRadius(collision_ellipse, blocker_yaw, perpendicular_x, perpendicular_y) +
    collision_path_clearance_m_ + detour_extra_clearance_m_;
  double best_length = std::numeric_limits<double>::infinity();
  double best_clearance = 0.0;
  std::vector<std::pair<double, double>> best_curve;
  for (const double side : {-1.0, 1.0}) {
    const double base_amplitude = std::max(
      0.05, required_center_separation + side * blocker_lateral) /
      std::max(0.08, sine_at_blocker * sine_at_blocker);
    for (int scale_index = 0; scale_index < 5; ++scale_index) {
      const double amplitude = base_amplitude * (1.0 + 0.20 * scale_index);
      auto curve = make_curve(side, amplitude);
      const double clearance = curveClearance(curve);
      if (clearance < collision_path_clearance_m_) {
        continue;
      }
      double length = 0.0;
      for (std::size_t index = 1; index < curve.size(); ++index) {
        length += std::hypot(
          curve[index].first - curve[index - 1].first,
          curve[index].second - curve[index - 1].second);
      }
      if (length < best_length) {
        best_length = length;
        best_clearance = clearance;
        best_curve = std::move(curve);
      }
      break;
    }
  }
  if (best_curve.empty()) {
    const int station_count = multi_obstacle_lattice_stations_;
    const int lane_radius = std::max(
      1, static_cast<int>(std::floor(
        multi_obstacle_max_lateral_m_ / multi_obstacle_lateral_step_m_)));
    const int lane_count = 2 * lane_radius + 1;
    const int center_lane = lane_radius;
    const auto laneOffset = [&](const int lane) {
        return static_cast<double>(lane - center_lane) * multi_obstacle_lateral_step_m_;
      };
    const double search_clearance =
      collision_path_clearance_m_ + detour_extra_clearance_m_;
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> costs(
      static_cast<std::size_t>(station_count),
      std::vector<double>(static_cast<std::size_t>(lane_count), infinity));
    std::vector<std::vector<int>> parents(
      static_cast<std::size_t>(station_count),
      std::vector<int>(static_cast<std::size_t>(lane_count), -1));
    costs[0][center_lane] = 0.0;

    const auto stationPoint = [&](const int station, const int lane) {
        const double progress = static_cast<double>(station) /
          static_cast<double>(station_count - 1);
        const double lateral = laneOffset(lane);
        return std::pair<double, double>{
          robot_x + progress * path_dx + perpendicular_x * lateral,
          robot_y + progress * path_dy + perpendicular_y * lateral};
      };
    const auto transitionClearance = [&curveClearance](
      const std::pair<double, double> & from,
      const std::pair<double, double> & to) {
        constexpr int transition_samples = 7;
        std::vector<std::pair<double, double>> segment;
        segment.reserve(transition_samples);
        for (int sample = 0; sample < transition_samples; ++sample) {
          const double ratio = static_cast<double>(sample) /
            static_cast<double>(transition_samples - 1);
          segment.emplace_back(
            from.first + ratio * (to.first - from.first),
            from.second + ratio * (to.second - from.second));
        }
        return curveClearance(segment);
      };

    for (int station = 1; station < station_count; ++station) {
      const bool final_station = station == station_count - 1;
      const int first_lane = final_station ? center_lane : 0;
      const int last_lane = final_station ? center_lane : lane_count - 1;
      for (int lane = first_lane; lane <= last_lane; ++lane) {
        const auto current_point = stationPoint(station, lane);
        for (int previous_lane = 0; previous_lane < lane_count; ++previous_lane) {
          const double previous_cost = costs[station - 1][previous_lane];
          if (!std::isfinite(previous_cost) ||
            std::fabs(laneOffset(lane) - laneOffset(previous_lane)) >
            multi_obstacle_max_lane_change_m_ + 1e-9)
          {
            continue;
          }
          const auto previous_point = stationPoint(station - 1, previous_lane);
          const double clearance = transitionClearance(previous_point, current_point);
          if (clearance < search_clearance) {
            continue;
          }
          const double segment_length = std::hypot(
            current_point.first - previous_point.first,
            current_point.second - previous_point.second);
          double turn_cost = 0.0;
          const int earlier_lane = parents[station - 1][previous_lane];
          if (station > 1 && earlier_lane >= 0) {
            const auto earlier_point = stationPoint(station - 2, earlier_lane);
            const double previous_heading = std::atan2(
              previous_point.second - earlier_point.second,
              previous_point.first - earlier_point.first);
            const double current_heading = std::atan2(
              current_point.second - previous_point.second,
              current_point.first - previous_point.first);
            turn_cost = multi_obstacle_turn_penalty_ *
              std::fabs(signedYawError(current_heading, previous_heading));
          }
          const double center_bias = 0.004 * std::fabs(laneOffset(lane));
          const double candidate_cost = previous_cost + segment_length + turn_cost + center_bias;
          if (candidate_cost < costs[station][lane]) {
            costs[station][lane] = candidate_cost;
            parents[station][lane] = previous_lane;
          }
        }
      }
    }

    if (std::isfinite(costs[station_count - 1][center_lane])) {
      std::vector<double> lateral_profile(static_cast<std::size_t>(station_count), 0.0);
      int lane = center_lane;
      bool reconstruction_valid = true;
      for (int station = station_count - 1; station >= 0; --station) {
        lateral_profile[static_cast<std::size_t>(station)] = laneOffset(lane);
        if (station > 0) {
          lane = parents[station][lane];
          if (lane < 0) {
            reconstruction_valid = false;
            break;
          }
        }
      }

      if (reconstruction_valid) {
        const int output_samples = std::max(
          smooth_path_samples_, 4 * (station_count - 1) + 1);
        for (const double tangent_scale : {1.0, 0.5, 0.0}) {
          std::vector<std::pair<double, double>> candidate;
          candidate.reserve(static_cast<std::size_t>(output_samples));
          for (int sample = 0; sample < output_samples; ++sample) {
            const double progress = static_cast<double>(sample) /
              static_cast<double>(output_samples - 1);
            const double station_position = progress * static_cast<double>(station_count - 1);
            const int segment = std::min(
              station_count - 2, static_cast<int>(std::floor(station_position)));
            const double u = station_position - static_cast<double>(segment);
            const double y0 = lateral_profile[static_cast<std::size_t>(segment)];
            const double y1 = lateral_profile[static_cast<std::size_t>(segment + 1)];
            const double m0 = segment == 0 ? 0.0 :
              0.5 * tangent_scale *
              (lateral_profile[static_cast<std::size_t>(segment + 1)] -
              lateral_profile[static_cast<std::size_t>(segment - 1)]);
            const double m1 = segment + 1 == station_count - 1 ? 0.0 :
              0.5 * tangent_scale *
              (lateral_profile[static_cast<std::size_t>(segment + 2)] -
              lateral_profile[static_cast<std::size_t>(segment)]);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double lateral =
              (2.0 * u3 - 3.0 * u2 + 1.0) * y0 +
              (u3 - 2.0 * u2 + u) * m0 +
              (-2.0 * u3 + 3.0 * u2) * y1 +
              (u3 - u2) * m1;
            candidate.emplace_back(
              robot_x + progress * path_dx + perpendicular_x * lateral,
              robot_y + progress * path_dy + perpendicular_y * lateral);
          }
          const double clearance = curveClearance(candidate);
          if (clearance >= collision_path_clearance_m_) {
            best_clearance = clearance;
            best_curve = std::move(candidate);
            avoidance_strategy_ = "multi_obstacle_lattice";
            break;
          }
        }
      }
    }
  }
  if (best_curve.empty()) {
    local_path_points_ = make_curve(0.0, 0.0);
    if (!avoidance_active_) {
      RCLCPP_WARN(
        get_logger(),
        "no collision-free multi-obstacle route; hard-stop protection active blockers=%zu",
        direct_blocker_count);
    }
    avoidance_active_ = true;
    avoidance_strategy_ = "hard_stop";
    return false;
  }

  local_path_points_ = std::move(best_curve);
  if (avoidance_strategy_ != "multi_obstacle_lattice") {
    avoidance_strategy_ = "single_smooth_curve";
  }
  if (!avoidance_active_ || avoidance_strategy_ != previous_strategy) {
    const auto & obstacle = blocker->pose.pose.position;
    RCLCPP_INFO(
      get_logger(),
      "elliptical avoidance active strategy=%s blockers=%zu first=(%.2f, %.2f) "
      "points=%zu edge_clearance=%.2f",
      avoidance_strategy_.c_str(), direct_blocker_count, obstacle.x, obstacle.y,
      local_path_points_.size(), best_clearance);
  }
  avoidance_active_ = true;
  return true;
}

void FootballSimulationNavigator::publishLocalPlan(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = field_frame_;
  path.header.stamp = stamp;
  for (std::size_t index = 0; index < local_path_points_.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = field_frame_;
      pose.header.stamp = stamp;
      pose.pose.position.x = local_path_points_[index].first;
      pose.pose.position.y = local_path_points_[index].second;
      const std::size_t next = std::min(index + 1, local_path_points_.size() - 1);
      const std::size_t previous = index > 0 ? index - 1 : 0;
      const double tangent_x = local_path_points_[next].first -
        local_path_points_[previous].first;
      const double tangent_y = local_path_points_[next].second -
        local_path_points_[previous].second;
      pose.pose.orientation = quaternionFromYaw(std::atan2(tangent_y, tangent_x));
      path.poses.push_back(pose);
  }
  local_plan_pub_->publish(path);
}

std::pair<double, double> FootballSimulationNavigator::localLookahead(
  const double robot_x, const double robot_y) const
{
  if (local_path_points_.empty()) {
    return {robot_x, robot_y};
  }
  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < local_path_points_.size(); ++index) {
    const double distance = std::hypot(
      local_path_points_[index].first - robot_x,
      local_path_points_[index].second - robot_y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = index;
    }
  }
  double accumulated = 0.0;
  for (std::size_t index = closest_index + 1; index < local_path_points_.size(); ++index) {
    accumulated += std::hypot(
      local_path_points_[index].first - local_path_points_[index - 1].first,
      local_path_points_[index].second - local_path_points_[index - 1].second);
    if (accumulated >= local_path_lookahead_m_) {
      return local_path_points_[index];
    }
  }
  return local_path_points_.back();
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
      local_path_points_.clear();
      have_local_path_target_ = false;
      return;
    }
    if (!active_goal_->is_active()) {
      publishStop();
      active_goal_.reset();
      local_path_points_.clear();
      have_local_path_target_ = false;
      return;
    }
    if (!have_odom_ || (now() - latest_odom_time_).seconds() > odom_timeout_sec_) {
      publishStop();
      return;
    }

    const auto & target = active_goal_->get_goal()->pose;
    const double robot_x = latest_odom_.pose.pose.position.x;
    const double robot_y = latest_odom_.pose.pose.position.y;
    const double final_dx = target.pose.position.x - robot_x;
    const double final_dy = target.pose.position.y - robot_y;
    const double final_distance = std::hypot(final_dx, final_dy);
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
    if (final_distance <= position_tolerance_m_ && std::fabs(yaw_error) <= yaw_tolerance_rad_) {
      publishStop();
      active_goal_->succeed(std::make_shared<NavigateToPose::Result>());
      active_goal_.reset();
      local_path_points_.clear();
      have_local_path_target_ = false;
      return;
    }

    const double nearest_robot_clearance = nearestRobotClearance(
      robot_x, robot_y, robot_yaw);
    const bool clearance_recovery = !isPushState() &&
      nearest_robot_clearance <= clearance_recovery_trigger_clearance_m_ &&
      buildClearanceRecoveryPath(robot_x, robot_y, robot_yaw);
    if (!clearance_recovery) {
      clearance_recovery_active_ = false;
      buildSmoothLocalPath(
        robot_x, robot_y, target.pose.position.x, target.pose.position.y);
    }
    publishLocalPlan(now());
    const auto lookahead = localLookahead(robot_x, robot_y);
    const double navigation_x = lookahead.first;
    const double navigation_y = lookahead.second;
    const double dx = navigation_x - robot_x;
    const double dy = navigation_y - robot_y;
    const double distance = std::hypot(dx, dy);

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
        const double path_bearing_error = signedYawError(std::atan2(dy, dx), robot_yaw);
        const double heading_scale = std::fabs(path_bearing_error) >= turn_in_place_threshold_rad_ ?
          0.0 : std::max(0.0, std::cos(path_bearing_error));
        command.linear.x = world_speed * heading_scale;
        command.linear.y = 0.0;
      }
    }
    if (isPushState()) {
      // Continue rotating after the translational tolerance is reached;
      // otherwise a contact goal can remain active forever with zero cmd_vel.
      command.angular.z = std::clamp(
        angular_gain_ * yaw_error + push_bearing_gain_ * push_bearing_error,
        -max_angular_speed_rps_, max_angular_speed_rps_);
    } else {
      const double heading_target = distance > position_tolerance_m_ ?
        std::atan2(dy, dx) : target_yaw;
      command.angular.z = std::clamp(
        path_heading_gain_ * signedYawError(heading_target, robot_yaw),
        -max_angular_speed_rps_, max_angular_speed_rps_);
    }
    if (nearest_robot_clearance <= collision_hard_stop_clearance_m_) {
      if (clearance_recovery_active_ && !isPushState()) {
        command.linear.x = std::min(
          command.linear.x, clearance_recovery_speed_mps_);
        command.linear.y = 0.0;
      } else {
        command.linear.x = 0.0;
        command.linear.y = 0.0;
      }
    } else if (nearest_robot_clearance < collision_slowdown_clearance_m_) {
      const double scale = std::clamp(
        (nearest_robot_clearance - collision_hard_stop_clearance_m_) /
        (collision_slowdown_clearance_m_ - collision_hard_stop_clearance_m_),
        0.0, 1.0);
      command.linear.x *= scale;
      command.linear.y *= scale;
    }
    publishCommand(command);

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose.header = latest_odom_.header;
    feedback->current_pose.pose = latest_odom_.pose.pose;
    feedback->distance_remaining = static_cast<float>(final_distance);
    feedback->navigation_time = now() - goal_started_time_;
    active_goal_->publish_feedback(feedback);
  }


}  // namespace football_navigation
