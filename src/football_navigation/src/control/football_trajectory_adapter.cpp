// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "football_navigation/control/football_trajectory_adapter.hpp"

namespace football_navigation
{

FootballTrajectoryAdapter::FootballTrajectoryAdapter()
  : Node("football_trajectory_adapter")
{
    target_frame_ = declare_parameter<std::string>(
      "target_frame",
      "base_link");

    input_topic_ = declare_parameter<std::string>(
      "input_approach_pose_topic",
      "football/approach_pose");

    output_goal_topic_ = declare_parameter<std::string>(
      "output_goal_pose_topic",
      "goal_pose");

    output_tracking_topic_ = declare_parameter<std::string>(
      "output_tracking_pose_topic",
      "tracking_pose");

    output_heartbeat_topic_ = declare_parameter<std::string>(
      "output_tracking_heartbeat_topic",
      "");

    publish_tracking_pose_ = declare_parameter<bool>(
      "publish_tracking_pose",
      true);

    publish_goal_pose_ = declare_parameter<bool>(
      "publish_goal_pose",
      false);

    max_input_age_sec_ = declare_parameter<double>(
      "max_input_age_sec",
      0.50);

    future_tolerance_sec_ = declare_parameter<double>(
      "future_tolerance_sec",
      0.08);

    goal_event_topic_ = declare_parameter<std::string>(
      "goal_event_topic",
      "/football/goal_scored");

    control_valid_topic_ = declare_parameter<std::string>(
      "control_valid_topic",
      "football/control_valid");

    kickoff_hold_sec_ = declare_parameter<double>(
      "kickoff_hold_sec",
      3.0);

    planner_update_rate_hz_ = declare_parameter<double>(
      "planner_update_rate_hz",
      0.0);

    tracking_pose_heartbeat_hz_ =
      declare_parameter<double>(
      "tracking_pose_heartbeat_hz",
      10.0);

    if (
      !std::isfinite(planner_update_rate_hz_) ||
      planner_update_rate_hz_ < 0.0)
    {
      throw std::invalid_argument(
              "planner_update_rate_hz must be finite and non-negative");
    }

    if (
      !std::isfinite(tracking_pose_heartbeat_hz_) ||
      tracking_pose_heartbeat_hz_ <= 0.0)
    {
      throw std::invalid_argument(
              "tracking_pose_heartbeat_hz must be finite and positive");
    }

    separate_heartbeat_topic_ =
      !output_heartbeat_topic_.empty() &&
      output_heartbeat_topic_ != output_tracking_topic_;

    approach_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_,
      10,
      std::bind(
        &FootballTrajectoryAdapter::approachCallback,
        this,
        std::placeholders::_1));

    goal_event_sub_ =
      create_subscription<std_msgs::msg::String>(
      goal_event_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(
        &FootballTrajectoryAdapter::goalEventCallback,
        this,
        std::placeholders::_1));

    control_valid_sub_ =
      create_subscription<std_msgs::msg::Bool>(
      control_valid_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        have_control_valid_ = true;
        control_valid_ = msg && msg->data;

        if (!control_valid_) {
          clearTrackingState();
        }
      });

    goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      output_goal_topic_,
      10);

    // A planner target is a geometric command. Keep only the newest sample
    // and require reliable delivery; freshness is carried separately by the
    // heartbeat topic and must not replay cached base_link geometry.
    tracking_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      output_tracking_topic_,
      rclcpp::QoS(1).reliable());

    if (separate_heartbeat_topic_) {
      heartbeat_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(
        output_heartbeat_topic_,
        10);
    }

    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          1.0 / tracking_pose_heartbeat_hz_)),
      std::bind(
        &FootballTrajectoryAdapter::publishTrackingHeartbeat,
        this));

    if (planner_update_rate_hz_ > 0.0) {
      planner_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
            1.0 / planner_update_rate_hz_)),
        std::bind(
          &FootballTrajectoryAdapter::publishPlannerUpdate,
          this));
    }
  }

void FootballTrajectoryAdapter::clearTrackingState()
{
    have_tracking_pose_ = false;
    planner_pose_published_ = false;
    planner_update_pending_ = false;
  }

void FootballTrajectoryAdapter::goalEventCallback(
  const std_msgs::msg::String::SharedPtr)
{
    kickoff_hold_until_ =
      now() +
      rclcpp::Duration::from_seconds(
      kickoff_hold_sec_);

    clearTrackingState();
  }

bool FootballTrajectoryAdapter::inKickoffHold(
  const rclcpp::Time & stamp) const
{
    return
      kickoff_hold_until_.nanoseconds() > 0 &&
      stamp < kickoff_hold_until_;
  }

bool FootballTrajectoryAdapter::finitePose(
  const geometry_msgs::msg::Pose & pose)
{
    const auto & q = pose.orientation;

    const double norm =
      q.x * q.x +
      q.y * q.y +
      q.z * q.z +
      q.w * q.w;

    return
      std::isfinite(pose.position.x) &&
      std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z) &&
      std::isfinite(q.x) &&
      std::isfinite(q.y) &&
      std::isfinite(q.z) &&
      std::isfinite(q.w) &&
      norm > 1e-8;
  }

bool FootballTrajectoryAdapter::canPublish() const
{
    return
      publish_tracking_pose_ &&
      have_tracking_pose_ &&
      have_control_valid_ &&
      control_valid_ &&
      !inKickoffHold(now());
  }

void FootballTrajectoryAdapter::approachCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (
      !have_control_valid_ ||
      !control_valid_ ||
      inKickoffHold(now()) ||
      !msg ||
      msg->header.frame_id != target_frame_ ||
      !finitePose(msg->pose))
    {
      return;
    }

    const rclcpp::Time stamp(
      msg->header.stamp,
      get_clock()->get_clock_type());

    const double age =
      (now() - stamp).seconds();

    if (
      stamp.nanoseconds() <= 0 ||
      age > max_input_age_sec_ ||
      age < -future_tolerance_sec_ ||
      (
        last_input_stamp_.nanoseconds() > 0 &&
        stamp <= last_input_stamp_
      ))
    {
      return;
    }

    last_input_stamp_ = stamp;

    if (publish_goal_pose_) {
      goal_pub_->publish(*msg);
    }

    if (!publish_tracking_pose_) {
      return;
    }

    latest_tracking_pose_ = *msg;
    have_tracking_pose_ = true;
    planner_update_pending_ = true;

    /*
     * Preserve production behavior when a separate heartbeat topic
     * and a bounded planner update rate are not configured.
     */
    if (
      !separate_heartbeat_topic_ ||
      planner_update_rate_hz_ <= 0.0)
    {
      tracking_pub_->publish(
        latest_tracking_pose_);

      planner_pose_published_ = true;
      planner_update_pending_ = false;
      return;
    }

    /*
     * Publish the first planner target immediately. Later planner updates
     * are serialized by planner_timer_, but only when a new approach
     * message has arrived. Replaying cached base_link coordinates after
     * the robot moves would create a different global goal, so the 10 Hz
     * freshness heartbeat must never become a geometric target update.
     */
    if (!planner_pose_published_) {
      publishPlannerUpdate();
    }

    publishTrackingHeartbeat();
  }

void FootballTrajectoryAdapter::publishPlannerUpdate()
{
    if (!canPublish() || !planner_update_pending_) {
      return;
    }

    auto output = latest_tracking_pose_;

    tracking_pub_->publish(output);
    planner_pose_published_ = true;
    planner_update_pending_ = false;
  }

void FootballTrajectoryAdapter::publishTrackingHeartbeat()
{
    if (!canPublish()) {
      return;
    }

    auto output = latest_tracking_pose_;
    output.header.stamp = now();

    if (separate_heartbeat_topic_) {
      heartbeat_pub_->publish(output);
    } else {
      tracking_pub_->publish(output);
    }
  }


}  // namespace football_navigation
