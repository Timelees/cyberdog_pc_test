// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

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
#include "football_navigation/coordination/football_ball_fusion.hpp"

namespace football_navigation
{
namespace
{

std::vector<std::string> splitCsv(const std::string & csv)
{
  std::vector<std::string> result;
  std::stringstream stream(csv);
  std::string value;
  while (std::getline(stream, value, ',')) {
    value.erase(0, value.find_first_not_of(" \t\n\r"));
    const auto last = value.find_last_not_of(" \t\n\r");
    if (!value.empty() && last != std::string::npos) {
      result.push_back(value.substr(0, last + 1));
    }
  }
  return result;
}

std::string expand(const std::string & pattern, const std::string & robot)
{
  std::string output = pattern;
  const auto position = output.find("{namespace}");
  if (position == std::string::npos) {
    throw std::invalid_argument("topic template must contain {namespace}");
  }
  output.replace(position, 11, robot);
  return output;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto middle = values.size() / 2;
  return values.size() % 2 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
}

}  // namespace

FootballBallFusion::FootballBallFusion()
  : Node("football_ball_fusion")
{
    mode_ = declare_parameter<std::string>("mode", "global_external");
    if (mode_ != "global_external" && mode_ != "local_multi_robot") {
      throw std::invalid_argument("mode must be global_external or local_multi_robot");
    }
    field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
    // 感知节点或假世界节点向上游发布原始观测；足球控制模块只使用这里输出的、
    // 已通过校验且位于 field_frame 下的球位姿。
    output_topic_ = declare_parameter<std::string>("output_topic", "/football/ball_pose");
    max_detection_age_sec_ = declare_parameter<double>("max_detection_age_sec", 0.30);
    future_tolerance_sec_ = declare_parameter<double>("future_tolerance_sec", 0.08);
    max_odom_age_sec_ = declare_parameter<double>("max_odom_age_sec", 0.30);
    max_detection_odom_skew_sec_ =
      declare_parameter<double>("max_detection_odom_skew_sec", 0.12);
    max_fusion_time_skew_sec_ = declare_parameter<double>("max_fusion_time_skew_sec", 0.08);
    minimum_inlier_count_ = declare_parameter<int>("minimum_inlier_count", 2);
    inlier_radius_m_ = declare_parameter<double>("inlier_radius_m", 0.45);
    max_ball_speed_mps_ = declare_parameter<double>("max_ball_speed_mps", 6.0);
    field_min_x_ = declare_parameter<double>("field_min_x", -2.25);
    field_max_x_ = declare_parameter<double>("field_max_x", 8.25);
    field_min_y_ = declare_parameter<double>("field_min_y", -3.25);
    field_max_y_ = declare_parameter<double>("field_max_y", 3.25);
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic_, rclcpp::SensorDataQoS().keep_last(5));
    goal_event_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("goal_event_topic", "/football/goal_scored"),
      rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        goalEventCallback(msg);
      });

    if (mode_ == "global_external") {
      const auto input = declare_parameter<std::string>(
        "global_input_topic", "/football/ball_pose_raw");
      global_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        input, rclcpp::SensorDataQoS().keep_last(5),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          globalCallback(msg);
        });
      RCLCPP_INFO(
        get_logger(),
        "football ball fusion ready mode=%s field=%s input=%s output=%s",
        mode_.c_str(), field_frame_.c_str(), input.c_str(), output_topic_.c_str());
      return;
    }

    const auto robots = splitCsv(declare_parameter<std::string>(
      "robot_namespaces_csv",
      "cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5,"
      "cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10"));
    const auto detection_template = declare_parameter<std::string>(
      "local_ball_topic_template", "/{namespace}/football/ball_pose_local");
    const auto odom_template = declare_parameter<std::string>(
      "odom_topic_template", "/global_vio/{namespace}/odom");
    for (const auto & robot : robots) {
      detection_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
        expand(detection_template, robot), rclcpp::SensorDataQoS().keep_last(5),
        [this, robot](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          if (!msg || (msg->header.frame_id != "base_link" &&
              msg->header.frame_id != robot + "/base_link") || !finitePose(msg->pose)) {
            return;
          }
          const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
          const double age = stamp.nanoseconds() > 0 ?
            (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
          const auto previous = detections_.find(robot);
          if (stamp.nanoseconds() <= 0 || age > max_detection_age_sec_ ||
            age < -future_tolerance_sec_ ||
            (previous != detections_.end() && stamp <= rclcpp::Time(
              previous->second.header.stamp, get_clock()->get_clock_type())))
          {
            return;
          }
          detections_[robot] = *msg;
        }));
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        expand(odom_template, robot), rclcpp::SensorDataQoS().keep_last(5),
        [this, robot](const nav_msgs::msg::Odometry::SharedPtr msg) {
          if (!msg || msg->header.frame_id != field_frame_ || !finitePose(msg->pose.pose)) {
            return;
          }
          const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
          const double age = stamp.nanoseconds() > 0 ?
            (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
          auto & history = odom_histories_[robot];
          if (stamp.nanoseconds() <= 0 || age > max_odom_age_sec_ ||
            age < -future_tolerance_sec_ ||
            (!history.empty() && stamp <= rclcpp::Time(
              history.back().header.stamp, get_clock()->get_clock_type())))
          {
            return;
          }
          history.push_back(*msg);
          while (history.size() > 20) {
            history.pop_front();
          }
        }));
    }
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        fuse();
      });

    RCLCPP_INFO(
      get_logger(),
      "football ball fusion ready mode=%s field=%s robots=%zu output=%s",
      mode_.c_str(), field_frame_.c_str(), robots.size(), output_topic_.c_str());
  }

void FootballBallFusion::goalEventCallback(const std_msgs::msg::String::SharedPtr)
{
    have_previous_ = false;
    previous_ = geometry_msgs::msg::PoseStamped();
    detections_.clear();
    odom_histories_.clear();
  }

bool FootballBallFusion::finitePose(const geometry_msgs::msg::Pose & pose) const
{
    double yaw = 0.0;
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           yawFromQuaternion(pose.orientation, yaw);
  }

bool FootballBallFusion::insideField(const double x, const double y) const
{
    return x >= field_min_x_ && x <= field_max_x_ && y >= field_min_y_ && y <= field_max_y_;
  }

bool FootballBallFusion::motionValid(const geometry_msgs::msg::PoseStamped & candidate) const
{
    if (!have_previous_) {
      return true;
    }
    const rclcpp::Time stamp(candidate.header.stamp, get_clock()->get_clock_type());
    const rclcpp::Time previous_stamp(previous_.header.stamp, get_clock()->get_clock_type());
    const double dt = (stamp - previous_stamp).seconds();
    if (dt <= 0.0) {
      return false;
    }
    return std::hypot(
      candidate.pose.position.x - previous_.pose.position.x,
      candidate.pose.position.y - previous_.pose.position.y) / dt <= max_ball_speed_mps_;
  }

void FootballBallFusion::publish(geometry_msgs::msg::PoseStamped output)
{
    if (!finitePose(output.pose) || !insideField(
        output.pose.position.x, output.pose.position.y) || !motionValid(output))
    {
      return;
    }
    output.header.frame_id = field_frame_;
    output.pose.position.z = 0.0;
    output.pose.orientation.x = 0.0;
    output.pose.orientation.y = 0.0;
    output.pose.orientation.z = 0.0;
    output.pose.orientation.w = 1.0;
    publisher_->publish(output);
    previous_ = output;
    have_previous_ = true;
  }

void FootballBallFusion::globalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!msg || msg->header.frame_id != field_frame_ || !finitePose(msg->pose)) {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const double age = stamp.nanoseconds() > 0 ?
      (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
    if (stamp.nanoseconds() <= 0 || age > max_detection_age_sec_ ||
      age < -future_tolerance_sec_)
    {
      return;
    }
    publish(*msg);
  }

bool FootballBallFusion::findNearestOdom(
  const std::string & robot,
  const rclcpp::Time & detection_stamp,
  nav_msgs::msg::Odometry & output) const
{
    const auto found = odom_histories_.find(robot);
    if (found == odom_histories_.end() || found->second.empty()) {
      return false;
    }
    double best_skew = std::numeric_limits<double>::infinity();
    for (const auto & odom : found->second) {
      const rclcpp::Time stamp(odom.header.stamp, get_clock()->get_clock_type());
      const double skew = std::fabs((detection_stamp - stamp).seconds());
      if (skew < best_skew) {
        best_skew = skew;
        output = odom;
      }
    }
    return best_skew <= max_detection_odom_skew_sec_;
  }

void FootballBallFusion::fuse()
{
    struct Candidate {double x; double y; rclcpp::Time stamp;};
    std::vector<Candidate> candidates;
    for (const auto & item : detections_) {
      const rclcpp::Time detection_stamp(
        item.second.header.stamp, get_clock()->get_clock_type());
      nav_msgs::msg::Odometry odom;
      if (!findNearestOdom(item.first, detection_stamp, odom) ||
        !finitePose(item.second.pose) || !finitePose(odom.pose.pose))
      {
        continue;
      }
      const rclcpp::Time odom_stamp(odom.header.stamp, get_clock()->get_clock_type());
      const double detection_age = (now() - detection_stamp).seconds();
      const double odom_age = (now() - odom_stamp).seconds();
      if (detection_age > max_detection_age_sec_ || detection_age < -future_tolerance_sec_ ||
        odom_age > max_odom_age_sec_ || odom_age < -future_tolerance_sec_)
      {
        continue;
      }
      const auto & local = item.second.pose.position;
      const auto & robot = odom.pose.pose;
      double yaw = 0.0;
      if (!yawFromQuaternion(robot.orientation, yaw)) {
        continue;
      }
      const double x = robot.position.x + std::cos(yaw) * local.x - std::sin(yaw) * local.y;
      const double y = robot.position.y + std::sin(yaw) * local.x + std::cos(yaw) * local.y;
      if (std::isfinite(x) && std::isfinite(y) && insideField(x, y)) {
        candidates.push_back({x, y, detection_stamp});
      }
    }
    if (candidates.empty()) {
      return;
    }
    const auto newest_it = std::max_element(
      candidates.begin(), candidates.end(),
      [](const Candidate & lhs, const Candidate & rhs) {return lhs.stamp < rhs.stamp;});
    const rclcpp::Time newest = newest_it->stamp;
    std::vector<Candidate> synchronized;
    synchronized.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      if (std::fabs((newest - candidate.stamp).seconds()) <= max_fusion_time_skew_sec_) {
        synchronized.push_back(candidate);
      }
    }
    if (static_cast<int>(synchronized.size()) < minimum_inlier_count_) {
      return;
    }
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto & candidate : synchronized) {
      xs.push_back(candidate.x);
      ys.push_back(candidate.y);
    }
    const double median_x = median(xs);
    const double median_y = median(ys);
    double sum_x = 0.0;
    double sum_y = 0.0;
    std::size_t count = 0;
    for (const auto & candidate : synchronized) {
      if (std::hypot(candidate.x - median_x, candidate.y - median_y) <= inlier_radius_m_) {
        sum_x += candidate.x;
        sum_y += candidate.y;
        ++count;
      }
    }
    if (static_cast<int>(count) < minimum_inlier_count_) {
      return;
    }
    geometry_msgs::msg::PoseStamped output;
    output.header.stamp = newest;
    output.header.frame_id = field_frame_;
    output.pose.position.x = sum_x / count;
    output.pose.position.y = sum_y / count;
    output.pose.orientation.w = 1.0;
    publish(output);
  }


}  // namespace football_navigation
