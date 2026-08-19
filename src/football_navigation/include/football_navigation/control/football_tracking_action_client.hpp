// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "protocol/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace football_navigation
{

class FootballTrackingActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  FootballTrackingActionClient();

private:
  void strikerCallback(const std_msgs::msg::String::SharedPtr msg);
  void controlValidCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void motionStatusCallback(const protocol::msg::MotionStatus::SharedPtr msg);
  void resetCostmapReadinessLocked();
  void costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
    bool local_costmap);
  bool costmapsReadyLocked(const rclcpp::Time & stamp) const;
  void roleCallback(const std_msgs::msg::String::SharedPtr msg);
  void cancelActiveGoal(const char * reason);
  void scheduleRetry(const char * reason);
  bool isSelfStriker() const;
  bool inKickoffHold(const rclcpp::Time & stamp) const;
  bool matchStateAllowsMovement() const;
  void matchStateCallback(const std_msgs::msg::String::SharedPtr msg);
  void goalEventCallback(const std_msgs::msg::String::SharedPtr msg);
  void trackingPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void maybeSendGoal();
  void goalResponseCallback(uint64_t generation, GoalHandle::SharedPtr goal_handle);
  void resultCallback(uint64_t generation, const GoalHandle::WrappedResult & result);

  bool enabled_;
  std::string tracking_pose_topic_;
  std::string action_name_;
  double tick_rate_hz_;
  double send_period_sec_;
  double pose_timeout_sec_;
  double future_tolerance_sec_;
  double completed_pose_xy_tolerance_;
  double completed_pose_yaw_tolerance_;
  std::string expected_tracking_frame_;
  double action_retry_initial_sec_;
  double action_retry_max_sec_;
  double retry_delay_sec_;
  std::string behavior_tree_;
  bool require_striker_role_{false};
  double role_timeout_sec_{1.0};
  std::string team_id_;
  std::string self_namespace_;
  std::string striker_topic_;
  std::string role_topic_;
  std::string goal_event_topic_;
  std::string match_state_topic_;
  std::string match_state_{"STOP"};
  std::string control_valid_topic_;
  bool require_costmap_ready_{false};
  std::string local_costmap_topic_;
  std::string planner_costmap_topic_;
  std::string expected_costmap_frame_;
  double costmap_timeout_sec_{1.50};
  int minimum_costmap_marked_cells_{1};
  bool is_striker_{true};
  bool have_striker_assignment_{false};
  bool control_valid_{false};
  bool require_slow_walk_{true};
  bool slow_walk_ready_{false};
  std::string motion_status_topic_;
  std::string tactical_role_{"STOP"};
  double kickoff_hold_sec_{3.0};

  std::mutex mutex_;
  bool have_tracking_pose_{false};
  bool goal_pending_{false};
  bool goal_active_{false};
  bool cancel_pending_{false};
  bool have_completed_tracking_pose_{false};
  bool force_retry_{false};
  uint64_t goal_generation_{0};
  uint64_t active_generation_{0};
  uint64_t sent_goal_generation_{0};
  GoalHandle::SharedPtr active_goal_handle_;
  geometry_msgs::msg::PoseStamped latest_tracking_pose_;
  geometry_msgs::msg::PoseStamped sent_goal_pose_;
  geometry_msgs::msg::PoseStamped last_completed_tracking_pose_;
  rclcpp::Time latest_pose_time_;
  rclcpp::Time last_pose_stamp_;
  rclcpp::Time kickoff_hold_until_;
  rclcpp::Time last_striker_assignment_time_;
  rclcpp::Time last_role_time_;
  rclcpp::Time next_retry_time_;
  rclcpp::Time last_goal_send_time_;
  rclcpp::Time local_costmap_ready_time_;
  rclcpp::Time planner_costmap_ready_time_;
  bool local_costmap_ready_{false};
  bool planner_costmap_ready_{false};

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr role_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_valid_sub_;
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr planner_costmap_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
