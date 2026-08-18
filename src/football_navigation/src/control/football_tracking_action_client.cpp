// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "football_navigation/core/football_geometry.hpp"
#include "football_navigation/control/football_tracking_action_client.hpp"

namespace football_navigation
{

FootballTrackingActionClient::FootballTrackingActionClient()
: Node("football_tracking_action_client")
{
  enabled_ = declare_parameter<bool>("enabled", false);
  tracking_pose_topic_ =
    declare_parameter<std::string>("tracking_pose_topic", "tracking_pose");
  action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
  tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 2.0);
  send_period_sec_ = declare_parameter<double>("send_period_sec", 0.5);
  pose_timeout_sec_ = declare_parameter<double>("tracking_pose_timeout_sec", 0.60);
  future_tolerance_sec_ = declare_parameter<double>("future_tolerance_sec", 0.08);
  completed_pose_xy_tolerance_ = declare_parameter<double>(
    "completed_pose_xy_tolerance", 0.05);
  completed_pose_yaw_tolerance_ = declare_parameter<double>(
    "completed_pose_yaw_tolerance", 0.08);
  expected_tracking_frame_ = declare_parameter<std::string>(
    "expected_tracking_frame", "vodom");
  if (!std::isfinite(tick_rate_hz_) || tick_rate_hz_ <= 0.0 ||
    !std::isfinite(send_period_sec_) || send_period_sec_ <= 0.0 ||
    !std::isfinite(pose_timeout_sec_) || pose_timeout_sec_ <= 0.0 ||
    !std::isfinite(future_tolerance_sec_) || future_tolerance_sec_ < 0.0 ||
    !std::isfinite(completed_pose_xy_tolerance_) ||
    completed_pose_xy_tolerance_ < 0.0 ||
    !std::isfinite(completed_pose_yaw_tolerance_) ||
    completed_pose_yaw_tolerance_ < 0.0)
  {
    throw std::invalid_argument("invalid tracking action parameters");
  }
  action_retry_initial_sec_ = declare_parameter<double>("action_retry_initial_sec", 0.5);
  action_retry_max_sec_ = declare_parameter<double>("action_retry_max_sec", 3.0);
  if (!std::isfinite(action_retry_initial_sec_) || action_retry_initial_sec_ <= 0.0 ||
    !std::isfinite(action_retry_max_sec_) ||
    action_retry_max_sec_ < action_retry_initial_sec_)
  {
    throw std::invalid_argument("invalid tracking action retry parameters");
  }
  retry_delay_sec_ = action_retry_initial_sec_;
  behavior_tree_ = declare_parameter<std::string>("behavior_tree", "");
  require_striker_role_ = declare_parameter<bool>("require_striker_role", false);
  role_timeout_sec_ = declare_parameter<double>("role_timeout_sec", 0.80);
  if (!std::isfinite(role_timeout_sec_) || role_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("role_timeout_sec must be positive and finite");
  }
  team_id_ = declare_parameter<std::string>("team_id", "");
  if (!team_id_.empty()) {
    std::transform(
      team_id_.begin(), team_id_.end(), team_id_.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  }
  self_namespace_ = normalizeRobotNamespace(
    declare_parameter<std::string>("self_namespace", ""));
  if (self_namespace_.empty()) {
    self_namespace_ = normalizeRobotNamespace(get_namespace());
  }
  if (team_id_.empty() && !self_namespace_.empty()) {
    team_id_ = inferTeamIdFromNamespace(self_namespace_);
  }
  if (team_id_ != "a" && team_id_ != "b") {
    throw std::invalid_argument(
            "team_id must be a or b, or self_namespace must match cyberdog_<1..10>");
  }
  striker_topic_ = declare_parameter<std::string>("striker_topic", "");
  if (require_striker_role_ && striker_topic_.empty() && !team_id_.empty()) {
    striker_topic_ = "/football/team_" + team_id_ + "/striker";
  }
  role_topic_ = declare_parameter<std::string>("role_topic", "football/role");
  goal_event_topic_ =
    declare_parameter<std::string>("goal_event_topic", "/football/goal_scored");
  match_state_topic_ = declare_parameter<std::string>(
    "match_state_topic", "/football/match_state");
  kickoff_hold_sec_ = declare_parameter<double>("kickoff_hold_sec", 3.0);
  if (!std::isfinite(kickoff_hold_sec_) || kickoff_hold_sec_ < 0.0) {
    throw std::invalid_argument("kickoff_hold_sec must be finite and non-negative");
  }
  control_valid_topic_ =
    declare_parameter<std::string>("control_valid_topic", "football/control_valid");
  require_costmap_ready_ = declare_parameter<bool>(
    "require_costmap_ready", false);
  local_costmap_topic_ = declare_parameter<std::string>(
    "local_costmap_topic", "local_costmap_tracking/costmap");
  planner_costmap_topic_ = declare_parameter<std::string>(
    "planner_costmap_topic", "rolling_window_costmap/costmap");
  expected_costmap_frame_ = declare_parameter<std::string>(
    "expected_costmap_frame", "vodom");
  costmap_timeout_sec_ = declare_parameter<double>(
    "costmap_timeout_sec", 1.50);
  minimum_costmap_marked_cells_ = declare_parameter<int>(
    "minimum_costmap_marked_cells", 1);
  if (!std::isfinite(costmap_timeout_sec_) || costmap_timeout_sec_ <= 0.0 ||
    minimum_costmap_marked_cells_ < 0 || expected_costmap_frame_.empty())
  {
    throw std::invalid_argument("invalid costmap readiness parameters");
  }

  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
  tracking_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    tracking_pose_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FootballTrackingActionClient::trackingPoseCallback, this, std::placeholders::_1));
  if (require_striker_role_ && !striker_topic_.empty()) {
    striker_sub_ = create_subscription<std_msgs::msg::String>(
      striker_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&FootballTrackingActionClient::strikerCallback, this, std::placeholders::_1));
    is_striker_ = false;
    have_striker_assignment_ = false;
  }
  role_sub_ = create_subscription<std_msgs::msg::String>(
    role_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballTrackingActionClient::roleCallback, this, std::placeholders::_1));
  goal_event_sub_ = create_subscription<std_msgs::msg::String>(
    goal_event_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&FootballTrackingActionClient::goalEventCallback, this, std::placeholders::_1));
  match_state_sub_ = create_subscription<std_msgs::msg::String>(
    match_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballTrackingActionClient::matchStateCallback, this, std::placeholders::_1));
  control_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    control_valid_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&FootballTrackingActionClient::controlValidCallback, this, std::placeholders::_1));
  if (require_costmap_ready_) {
    const auto costmap_qos = rclcpp::QoS(1).reliable().transient_local();
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, costmap_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmapCallback(msg, true);
      });
    planner_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      planner_costmap_topic_, costmap_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmapCallback(msg, false);
      });
  }

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / tick_rate_hz_)),
    std::bind(&FootballTrackingActionClient::maybeSendGoal, this));

  RCLCPP_INFO(
    get_logger(),
    "football_tracking_action_client enabled=%d action=%s tracking_pose_topic=%s "
    "expected_tracking_frame=%s require_striker_role=%d team_id=%s "
    "tick_rate_hz=%.2f send_period_sec=%.2f "
    "completed_pose_xy_tolerance=%.3f completed_pose_yaw_tolerance=%.3f "
    "require_costmap_ready=%d local_costmap=%s planner_costmap=%s "
    "behavior_tree=%s",
    enabled_, action_name_.c_str(), tracking_pose_topic_.c_str(), expected_tracking_frame_.c_str(),
    require_striker_role_, team_id_.c_str(),
    tick_rate_hz_, send_period_sec_,
    completed_pose_xy_tolerance_, completed_pose_yaw_tolerance_,
    require_costmap_ready_, local_costmap_topic_.c_str(),
    planner_costmap_topic_.c_str(),
    behavior_tree_.empty() ? "(navigator default)" : behavior_tree_.c_str());
}

void FootballTrackingActionClient::strikerCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  const std::string assigned = normalizeRobotNamespace(msg->data);
  const bool was_striker = is_striker_;
  is_striker_ = !self_namespace_.empty() && assigned == self_namespace_;
  have_striker_assignment_ = !assigned.empty();
  last_striker_assignment_time_ = now();

  if (was_striker != is_striker_) {
    RCLCPP_INFO(
      get_logger(),
      "football_tracking_action_client: striker role %s (assigned=%s self=%s)",
      is_striker_ ? "ACTIVE" : "INACTIVE", assigned.c_str(), self_namespace_.c_str());
    if (!is_striker_) {
      cancelActiveGoal("striker role lost");
    }
  }
}

void FootballTrackingActionClient::controlValidCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool was_valid = control_valid_;
  control_valid_ = msg && msg->data;
  if (!control_valid_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      resetCostmapReadinessLocked();
    }
    cancelActiveGoal("football control invalid");
    return;
  }

  if (!was_valid && require_costmap_ready_) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Callbacks received while control is invalid are ignored, so clearing
    // both flags requires one subsequent publication from each costmap.
    local_costmap_ready_ = false;
    planner_costmap_ready_ = false;
    local_costmap_ready_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    planner_costmap_ready_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

void FootballTrackingActionClient::resetCostmapReadinessLocked()
{
  local_costmap_ready_ = false;
  planner_costmap_ready_ = false;
  local_costmap_ready_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  planner_costmap_ready_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

void FootballTrackingActionClient::costmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
  const bool local_costmap)
{
  if (!msg || msg->header.frame_id != expected_costmap_frame_ ||
    msg->info.resolution <= 0.0 || msg->info.width == 0 || msg->info.height == 0 ||
    msg->data.empty())
  {
    return;
  }

  const int marked_cells = static_cast<int>(std::count_if(
    msg->data.begin(), msg->data.end(),
    [](const int8_t value) {return value >= 90;}));
  const rclcpp::Time receipt = now();

  std::lock_guard<std::mutex> lock(mutex_);
  // Galactic's Costmap2DPublisher sends OccupancyGrid with a zero header
  // stamp. Receipt time is therefore the only valid freshness clock here.
  if (!control_valid_) {
    return;
  }

  const bool ready = marked_cells >= minimum_costmap_marked_cells_;
  if (local_costmap) {
    local_costmap_ready_ = ready;
    local_costmap_ready_time_ = ready ? receipt :
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
  } else {
    planner_costmap_ready_ = ready;
    planner_costmap_ready_time_ = ready ? receipt :
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }
}

bool FootballTrackingActionClient::costmapsReadyLocked(const rclcpp::Time & stamp) const
{
  if (!require_costmap_ready_) {
    return true;
  }
  if (!local_costmap_ready_ || !planner_costmap_ready_ ||
    local_costmap_ready_time_.nanoseconds() <= 0 ||
    planner_costmap_ready_time_.nanoseconds() <= 0)
  {
    return false;
  }
  return (stamp - local_costmap_ready_time_).seconds() <= costmap_timeout_sec_ &&
         (stamp - planner_costmap_ready_time_).seconds() <= costmap_timeout_sec_;
}

void FootballTrackingActionClient::roleCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || (msg->data != "STRIKER" && msg->data != "SUPPORT" &&
    msg->data != "DEFENDER_LEFT" && msg->data != "DEFENDER_RIGHT" &&
    msg->data != "GOALKEEPER" && msg->data != "STOP"))
  {
    return;
  }
  tactical_role_ = msg->data;
  last_role_time_ = now();
  if (tactical_role_ == "STOP") {
    cancelActiveGoal("tactical role stopped");
  }
}

void FootballTrackingActionClient::cancelActiveGoal(const char * reason)
{
  GoalHandle::SharedPtr handle;
  bool state_changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool had_goal = goal_active_ || goal_pending_ || cancel_pending_;
    const bool had_pose = have_tracking_pose_;
    if (!had_goal && !had_pose) {
      return;
    }
    if (had_goal) {
      ++goal_generation_;
    }
    have_tracking_pose_ = false;
    goal_pending_ = false;
    if (goal_active_ && active_goal_handle_ && !cancel_pending_) {
      handle = active_goal_handle_;
      cancel_pending_ = true;
    } else if (!cancel_pending_) {
      goal_active_ = false;
      active_goal_handle_.reset();
    }
    retry_delay_sec_ = action_retry_initial_sec_;
    next_retry_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    force_retry_ = true;
    state_changed = true;
  }
  if (handle) {
    action_client_->async_cancel_goal(
      handle,
      [this](auto) {
        std::lock_guard<std::mutex> lock(mutex_);
        cancel_pending_ = false;
        goal_active_ = false;
        active_goal_handle_.reset();
      });
    RCLCPP_WARN(get_logger(), "football tracking goal cancelled: %s", reason);
  } else if (state_changed) {
    RCLCPP_DEBUG(
      get_logger(),
      "football tracking input invalidated without active goal: %s",
      reason);
  }
}

void FootballTrackingActionClient::scheduleRetry(const char * reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  next_retry_time_ = now() + rclcpp::Duration::from_seconds(retry_delay_sec_);
  force_retry_ = true;
  RCLCPP_WARN(
    get_logger(), "football tracking retry in %.2fs: %s", retry_delay_sec_, reason);
  retry_delay_sec_ = std::min(action_retry_max_sec_, retry_delay_sec_ * 2.0);
}

bool FootballTrackingActionClient::isSelfStriker() const
{
  if (last_role_time_.nanoseconds() > 0 &&
    (now() - last_role_time_).seconds() <= role_timeout_sec_)
  {
    return tactical_role_ != "STOP";
  }
  if (!require_striker_role_) {
    return true;
  }
  return have_striker_assignment_ && is_striker_ &&
         last_striker_assignment_time_.nanoseconds() > 0 &&
         (now() - last_striker_assignment_time_).seconds() <= role_timeout_sec_;
}

bool FootballTrackingActionClient::inKickoffHold(const rclcpp::Time & stamp) const
{
  return kickoff_hold_until_.nanoseconds() > 0 && stamp < kickoff_hold_until_;
}

bool FootballTrackingActionClient::matchStateAllowsMovement() const
{
  return match_state_ == "PLAY" ||
    (match_state_ == "KICKOFF_A" && team_id_ == "a") ||
    (match_state_ == "KICKOFF_B" && team_id_ == "b");
}

void FootballTrackingActionClient::matchStateCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || (msg->data != "STOP" && msg->data != "READY" && msg->data != "PLAY" &&
      msg->data != "KICKOFF_A" && msg->data != "KICKOFF_B" && msg->data != "FINISHED"))
  {
    return;
  }
  match_state_ = msg->data;
  if (!matchStateAllowsMovement()) {
    cancelActiveGoal("match state disallows movement");
  }
}

void FootballTrackingActionClient::goalEventCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    kickoff_hold_until_ = now() + rclcpp::Duration::from_seconds(kickoff_hold_sec_);
  }
  cancelActiveGoal("kickoff hold");
  RCLCPP_INFO(
    get_logger(),
    "football_tracking_action_client: kickoff hold %.1fs (%s)",
    kickoff_hold_sec_, msg->data.c_str());
}

void FootballTrackingActionClient::trackingPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const double q_norm = msg ?
    msg->pose.orientation.x * msg->pose.orientation.x +
    msg->pose.orientation.y * msg->pose.orientation.y +
    msg->pose.orientation.z * msg->pose.orientation.z +
    msg->pose.orientation.w * msg->pose.orientation.w : 0.0;
  if (!msg || msg->header.frame_id != expected_tracking_frame_ ||
    !std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) ||
    !std::isfinite(msg->pose.orientation.x) || !std::isfinite(msg->pose.orientation.y) ||
    !std::isfinite(msg->pose.orientation.z) || !std::isfinite(msg->pose.orientation.w) ||
    q_norm <= 1e-8)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "football_tracking_action_client: tracking pose invalid: expected frame %s, "
      "finite pose, and valid quaternion",
      expected_tracking_frame_.c_str());
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double age = stamp.nanoseconds() > 0 ?
    (now() - stamp).seconds() : pose_timeout_sec_ + 1.0;
  if (stamp.nanoseconds() <= 0 || age > pose_timeout_sec_ ||
    age < -future_tolerance_sec_ ||
    (last_pose_stamp_.nanoseconds() > 0 && stamp <= last_pose_stamp_))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_tracking_pose_ = *msg;
  latest_pose_time_ = now();
  last_pose_stamp_ = stamp;
  have_tracking_pose_ = true;
}

void FootballTrackingActionClient::maybeSendGoal()
{
  if (!enabled_) {
    return;
  }

  if (next_retry_time_.nanoseconds() > 0 && now() < next_retry_time_) {
    return;
  }

  if (!control_valid_ || !matchStateAllowsMovement()) {
    cancelActiveGoal("football control invalid");
    return;
  }

  if (inKickoffHold(now())) {
    cancelActiveGoal("kickoff hold");
    return;
  }

  if (!isSelfStriker()) {
    cancelActiveGoal("not striker");
    return;
  }

  bool pose_stale = false;
  bool completed_pose_unchanged = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cancel_pending_) {
      return;
    }
    if (!have_tracking_pose_) {
      return;
    }
    if ((now() - latest_pose_time_).seconds() > pose_timeout_sec_) {
      pose_stale = true;
    }
    if (goal_pending_ && !pose_stale) {
      return;
    }
    if (goal_active_ && !pose_stale &&
      isPoseNear(
        latest_tracking_pose_, sent_goal_pose_,
        completed_pose_xy_tolerance_, completed_pose_yaw_tolerance_))
    {
      return;
    }
    completed_pose_unchanged =
      !force_retry_ &&
      have_completed_tracking_pose_ &&
      isPoseNear(
        latest_tracking_pose_,
        last_completed_tracking_pose_,
        completed_pose_xy_tolerance_,
        completed_pose_yaw_tolerance_);
  }
  if (pose_stale) {
    cancelActiveGoal("tracking pose timeout");
    return;
  }
  if (completed_pose_unchanged) {
    return;
  }

  bool costmaps_ready = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    costmaps_ready = costmapsReadyLocked(now());
  }
  if (!costmaps_ready) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "football_tracking_action_client: waiting for fresh marked data on %s and %s",
      local_costmap_topic_.c_str(), planner_costmap_topic_.c_str());
    return;
  }

  if (!action_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "football_tracking_action_client: action server %s is not ready",
      action_name_.c_str());
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = latest_tracking_pose_;
  goal.behavior_tree = behavior_tree_;

  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cancel_pending_) {
      return;
    }
    const rclcpp::Time send_time = now();
    if (last_goal_send_time_.nanoseconds() > 0 &&
      (send_time - last_goal_send_time_).seconds() < send_period_sec_)
    {
      return;
    }
    goal_pending_ = true;
    force_retry_ = false;
    last_goal_send_time_ = send_time;
    generation = ++goal_generation_;
    sent_goal_pose_ = goal.pose;
    sent_goal_generation_ = generation;
  }

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  options.goal_response_callback =
    [this, generation](GoalHandle::SharedPtr goal_handle) {
      goalResponseCallback(generation, goal_handle);
    };

  options.feedback_callback =
    [](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback>)
    {
      // Galactic requires a non-empty callback before dispatching feedback.
    };

  options.result_callback =
    [this, generation](const GoalHandle::WrappedResult & result) {
      resultCallback(generation, result);
    };

  action_client_->async_send_goal(goal, options);

  RCLCPP_INFO(
    get_logger(),
    "football_tracking_action_client: sent NavigateToPose goal to %s",
    action_name_.c_str());
}

void FootballTrackingActionClient::goalResponseCallback(
  const uint64_t generation,
  GoalHandle::SharedPtr goal_handle)
{
  bool cancel = false;
  bool current_generation = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_generation = generation == goal_generation_;
    if (!current_generation) {
      cancel = static_cast<bool>(goal_handle);
    } else {
      goal_pending_ = false;
      goal_active_ = static_cast<bool>(goal_handle);
      active_goal_handle_ = goal_handle;
      active_generation_ = generation;
      if (goal_handle) {
        force_retry_ = false;
      }
      cancel = goal_handle &&
        (!control_valid_ || !matchStateAllowsMovement() || !have_tracking_pose_ ||
        !isSelfStriker() || inKickoffHold(now()) || !costmapsReadyLocked(now()));
    }
  }

  if (!goal_handle) {
    RCLCPP_ERROR(
      get_logger(),
      "football_tracking_action_client: NavigateToPose goal rejected");
    if (current_generation) {
      scheduleRetry("goal rejected");
    }
    return;
  }

  if (cancel) {
    if (current_generation) {
      cancelActiveGoal("control invalid while goal pending");
    } else {
      action_client_->async_cancel_goal(goal_handle);
    }
    return;
  }

  retry_delay_sec_ = action_retry_initial_sec_;
  next_retry_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  RCLCPP_INFO(
    get_logger(),
    "football_tracking_action_client: NavigateToPose goal accepted");
}

void FootballTrackingActionClient::resultCallback(
  const uint64_t generation,
  const GoalHandle::WrappedResult & result)
{
  geometry_msgs::msg::PoseStamped completed_goal;
  bool have_completed_goal = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // A newer tracking goal may already be pending while Nav2 reports the
    // preempted goal's result. Never let that stale result clear new state.
    if (generation != active_generation_ || generation != goal_generation_) {
      return;
    }

    goal_pending_ = false;
    goal_active_ = false;
    active_goal_handle_.reset();
    cancel_pending_ = false;
    if (generation == sent_goal_generation_) {
      completed_goal = sent_goal_pose_;
      have_completed_goal = true;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      // The tracking target may change while an action is executing (for
      // example BALL_APPROACH -> ALIGN_TO_GOAL). Record the pose that this
      // generation actually completed, never the newest queued target.
      if (generation == sent_goal_generation_) {
        last_completed_tracking_pose_ = sent_goal_pose_;
        have_completed_tracking_pose_ = true;
      }
      force_retry_ = false;
    }
  }

  if (result.code == rclcpp_action::ResultCode::ABORTED) {
    scheduleRetry("action aborted");
  }

  const char * result_name = "UNKNOWN";
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_name = "SUCCEEDED";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      result_name = "CANCELED";
      break;
    case rclcpp_action::ResultCode::ABORTED:
      result_name = "ABORTED";
      break;
    default:
      break;
  }
  RCLCPP_INFO(
    get_logger(),
    "football_tracking_action_client: NavigateToPose result=%s generation=%lu goal=(%.2f, %.2f)%s",
    result_name, static_cast<unsigned long>(generation),
    have_completed_goal ? completed_goal.pose.position.x : 0.0,
    have_completed_goal ? completed_goal.pose.position.y : 0.0,
    have_completed_goal ? "" : " (superseded generation)");
}

}  // namespace football_navigation
