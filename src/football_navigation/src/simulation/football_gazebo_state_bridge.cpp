// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class FootballGazeboStateBridge : public rclcpp::Node
{
public:
  FootballGazeboStateBridge()
  : Node("football_gazebo_state_bridge")
  {
    robot_model_ = declare_parameter<std::string>("robot_model", "actors");
    ball_model_ = declare_parameter<std::string>("ball_model", "football");
    field_frame_ = declare_parameter<std::string>("field_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/global_vio/cyberdog_1/odom");
    robot_models_csv_ = declare_parameter<std::string>("robot_models_csv", "");
    robot_namespaces_csv_ = declare_parameter<std::string>("robot_namespaces_csv", "");
    odom_topic_template_ = declare_parameter<std::string>(
      "odom_topic_template", "/global_vio/{namespace}/odom");
    ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
    if (robot_model_.empty() || ball_model_.empty() || field_frame_.empty() ||
      base_frame_.empty() || odom_topic_.empty() || ball_topic_.empty())
    {
      throw std::invalid_argument("Gazebo state bridge parameters must not be empty");
    }

    robot_models_ = splitCsv(robot_models_csv_);
    robot_namespaces_ = splitCsv(robot_namespaces_csv_);
    if (robot_models_.empty() && robot_namespaces_.empty()) {
      robot_models_.push_back(robot_model_);
      robot_namespaces_.push_back("");
    }
    if (robot_models_.size() != robot_namespaces_.size()) {
      throw std::invalid_argument(
              "robot_models_csv and robot_namespaces_csv must have equal lengths");
    }
    if (robot_models_.size() > 1 &&
      odom_topic_template_.find("{namespace}") == std::string::npos)
    {
      throw std::invalid_argument("odom_topic_template must contain {namespace}");
    }

    // Nav2's odometry consumer requests reliable delivery. Gazebo publishes
    // model states best-effort, but the derived topics are application state
    // and must be republished with a QoS that is compatible with Nav2 and the
    // football goal adapter.
    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    for (const auto & robot_namespace : robot_namespaces_) {
      std::string topic = odom_topic_;
      if (!robot_namespace.empty()) {
        topic = odom_topic_template_;
        const auto token = topic.find("{namespace}");
        if (token == std::string::npos) {
          throw std::invalid_argument("odom_topic_template must contain {namespace}");
        }
        topic.replace(token, std::string("{namespace}").size(), robot_namespace);
      }
      odom_pubs_.push_back(create_publisher<nav_msgs::msg::Odometry>(topic, odom_qos));
    }
    ball_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      ball_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
      "/gazebo/model_states", rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&FootballGazeboStateBridge::onStates, this, std::placeholders::_1));
  }

private:
  std::vector<std::string> splitCsv(const std::string & csv) const
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

  int findModel(const gazebo_msgs::msg::ModelStates & states, const std::string & name) const
  {
    const auto it = std::find(states.name.begin(), states.name.end(), name);
    return it == states.name.end() ? -1 : static_cast<int>(it - states.name.begin());
  }

  void onStates(const gazebo_msgs::msg::ModelStates::SharedPtr states)
  {
    if (!states) {
      return;
    }
    const auto stamp = now();
    for (std::size_t index = 0; index < robot_models_.size(); ++index) {
      const int robot = findModel(*states, robot_models_[index]);
      if (robot >= 0 && static_cast<std::size_t>(robot) < states->pose.size() &&
        static_cast<std::size_t>(robot) < states->twist.size())
      {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = field_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose = states->pose[robot];
        odom.twist.twist = states->twist[robot];
        odom_pubs_[index]->publish(odom);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Gazebo robot model '%s' not found",
          robot_models_[index].c_str());
      }
    }

    const int ball = findModel(*states, ball_model_);
    if (ball >= 0 && static_cast<std::size_t>(ball) < states->pose.size()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = field_frame_;
      pose.pose = states->pose[ball];
      ball_pub_->publish(pose);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Gazebo ball model '%s' not found",
        ball_model_.c_str());
    }
  }

  std::string robot_model_;
  std::string ball_model_;
  std::string field_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  std::string robot_models_csv_;
  std::string robot_namespaces_csv_;
  std::string odom_topic_template_;
  std::string ball_topic_;
  std::vector<std::string> robot_models_;
  std::vector<std::string> robot_namespaces_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ball_pub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr states_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootballGazeboStateBridge>());
  rclcpp::shutdown();
  return 0;
}
