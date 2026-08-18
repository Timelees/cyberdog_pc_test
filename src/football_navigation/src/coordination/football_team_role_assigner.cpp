// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "football_navigation/coordination/football_team_role_assigner.hpp"

namespace football_navigation
{
namespace
{

std::vector<std::string> splitCsv(const std::string & csv)
{
  std::vector<std::string> values;
  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto first = item.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = item.find_last_not_of(" \t\n\r");
    values.push_back(normalizeRobotNamespace(item.substr(first, last - first + 1)));
  }
  return values;
}

std::string expandTopicTemplate(const std::string & pattern, const std::string & robot_id)
{
  const std::string token = "{namespace}";
  const auto position = pattern.find(token);
  if (position == std::string::npos) {
    throw std::invalid_argument("odom_topic_template must contain {namespace}");
  }
  std::string output = pattern;
  output.replace(position, token.size(), normalizeRobotNamespace(robot_id));
  return output;
}

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace

FootballTeamRoleAssigner::FootballTeamRoleAssigner()
  : Node("football_team_role_assigner")
{
    const auto zero = zeroTime();
    latest_ball_time_ = zero;
    kickoff_hold_until_ = zero;
    kickoff_started_ = zero;
    striker_since_a_ = zero;
    striker_since_b_ = zero;
    challenger_since_a_ = zero;
    challenger_since_b_ = zero;
    authority_conflict_time_ = zero;

    field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
    ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
    odom_source_mode_ =
      declare_parameter<std::string>("odom_source_mode", "pc_forwarded");
    if (odom_source_mode_ != "pc_forwarded") {
      throw std::invalid_argument("odom_source_mode must be pc_forwarded");
    }
    odom_topic_template_ = declare_parameter<std::string>(
      "odom_topic_template", "/global_vio/{namespace}/odom");
    authority_id_ =
      declare_parameter<std::string>("authority_id", "football_pc_primary");
    std::ostringstream instance;
    instance << authority_id_ << '@' << now().nanoseconds() << '@' <<
      reinterpret_cast<std::uintptr_t>(this);
    authority_instance_id_ = instance.str();
    authority_topic_ =
      declare_parameter<std::string>("authority_topic", "/football/role_authority");
    match_state_topic_ =
      declare_parameter<std::string>("match_state_topic", "/football/match_state");
    match_state_command_topic_ = declare_parameter<std::string>(
      "match_state_command_topic", "/football/match_state_command");
    match_state_ = declare_parameter<std::string>("initial_match_state", "STOP");
    if (!validMatchState(match_state_)) {
      throw std::invalid_argument("initial_match_state is invalid");
    }

    team_a_namespaces_ = splitCsv(declare_parameter<std::string>(
      "team_a_namespaces_csv", "cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5"));
    team_b_namespaces_ = splitCsv(declare_parameter<std::string>(
      "team_b_namespaces_csv", "cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10"));
    validateTeamRosters();
    forced_team_a_striker_namespace_ = normalizeRobotNamespace(
      declare_parameter<std::string>("forced_team_a_striker_namespace", ""));
    if (!forced_team_a_striker_namespace_.empty() &&
      std::find(
        team_a_namespaces_.begin(), team_a_namespaces_.end(),
        forced_team_a_striker_namespace_) == team_a_namespaces_.end())
    {
      throw std::invalid_argument(
              "forced_team_a_striker_namespace must be empty or a team-a robot");
    }

    team_a_attack_goal_x_ = declare_parameter<double>("team_a_attack_goal_x", 8.0);
    team_a_attack_goal_y_ = declare_parameter<double>("team_a_attack_goal_y", 0.0);
    team_b_attack_goal_x_ = declare_parameter<double>("team_b_attack_goal_x", -2.0);
    team_b_attack_goal_y_ = declare_parameter<double>("team_b_attack_goal_y", 0.0);
    field_min_x_ = declare_parameter<double>("field_min_x", -2.0);
    field_max_x_ = declare_parameter<double>("field_max_x", 8.0);
    field_min_y_ = declare_parameter<double>("field_min_y", -3.0);
    field_max_y_ = declare_parameter<double>("field_max_y", 3.0);
    tactical_boundary_margin_m_ =
      declare_parameter<double>("tactical_boundary_margin_m", 0.30);

    const double goal_separation = std::hypot(
      team_a_attack_goal_x_ - team_b_attack_goal_x_,
      team_a_attack_goal_y_ - team_b_attack_goal_y_);
    if (!std::isfinite(goal_separation) || goal_separation < 1.0) {
      throw std::invalid_argument("team attack goals must be finite and physically separated");
    }
    if (!std::isfinite(field_min_x_) || !std::isfinite(field_max_x_) ||
      !std::isfinite(field_min_y_) || !std::isfinite(field_max_y_) ||
      field_min_x_ >= field_max_x_ || field_min_y_ >= field_max_y_ ||
      tactical_boundary_margin_m_ < 0.0 ||
      2.0 * tactical_boundary_margin_m_ >= field_max_x_ - field_min_x_ ||
      2.0 * tactical_boundary_margin_m_ >= field_max_y_ - field_min_y_)
    {
      throw std::invalid_argument("invalid football field bounds or tactical boundary margin");
    }

    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 5.0);
    max_pose_age_sec_ = declare_parameter<double>("max_pose_age_sec", 0.40);
    max_pose_skew_sec_ = declare_parameter<double>("max_pose_skew_sec", 0.15);
    ball_timeout_sec_ = declare_parameter<double>("ball_timeout_sec", 0.35);
    future_tolerance_sec_ = declare_parameter<double>("future_tolerance_sec", 0.08);
    authority_timeout_sec_ = declare_parameter<double>("authority_timeout_sec", 1.0);
    striker_min_hold_sec_ = declare_parameter<double>("striker_min_hold_sec", 1.5);
    striker_benefit_threshold_ =
      declare_parameter<double>("striker_benefit_threshold", 0.45);
    striker_switch_confirm_sec_ =
      declare_parameter<double>("striker_switch_confirm_sec", 0.80);
    kickoff_hold_sec_ = declare_parameter<double>("kickoff_hold_sec", 3.0);
    kickoff_release_distance_m_ =
      declare_parameter<double>("kickoff_release_distance_m", 0.25);
    kickoff_max_duration_sec_ =
      declare_parameter<double>("kickoff_max_duration_sec", 10.0);
    minimum_online_per_team_ = declare_parameter<int>("minimum_online_per_team", 1);
    minimum_other_robot_count_ = declare_parameter<int>("minimum_other_robot_count", 9);

    if (update_rate_hz_ <= 0.0 || max_pose_age_sec_ <= 0.0 ||
      max_pose_skew_sec_ < 0.0 || ball_timeout_sec_ <= 0.0 ||
      future_tolerance_sec_ < 0.0 || authority_timeout_sec_ <= 0.0 ||
      striker_min_hold_sec_ < 0.0 || striker_benefit_threshold_ < 0.0 ||
      striker_switch_confirm_sec_ < 0.0 || kickoff_hold_sec_ < 0.0 ||
      kickoff_release_distance_m_ <= 0.0 ||
      kickoff_max_duration_sec_ <= kickoff_hold_sec_ ||
      minimum_online_per_team_ < 1 || minimum_online_per_team_ > 5 ||
      minimum_other_robot_count_ < 1 || minimum_other_robot_count_ > 9)
    {
      throw std::invalid_argument("invalid football competition safety parameters");
    }

    striker_a_pub_ = create_publisher<std_msgs::msg::String>(
      "/football/team_a/striker", latchedQos());
    striker_b_pub_ = create_publisher<std_msgs::msg::String>(
      "/football/team_b/striker", latchedQos());
    // 这些 field_frame 位姿表示球的目的地。GoalAdapter 根据它们计算射门方向，
    // 并在后续流程中另行生成机器人的导航目标。
    kick_target_a_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/football/team_a/kick_target", latchedQos());
    kick_target_b_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/football/team_b/kick_target", latchedQos());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/football/team_role_status", latchedQos());
    authority_pub_ = create_publisher<std_msgs::msg::String>(authority_topic_, latchedQos());
    match_state_pub_ = create_publisher<std_msgs::msg::String>(match_state_topic_, latchedQos());
    publishMatchState();

    ball_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ball_topic_, rclcpp::SensorDataQoS().keep_last(5),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        ballCallback(msg);
      });
    authority_sub_ = create_subscription<std_msgs::msg::String>(
      authority_topic_, latchedQos(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        authorityCallback(msg);
      });
    goal_event_sub_ = create_subscription<std_msgs::msg::String>(
      "/football/goal_scored", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        goalEventCallback(msg);
      });
    match_state_command_sub_ = create_subscription<std_msgs::msg::String>(
      match_state_command_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        matchStateCommandCallback(msg);
      });

    std::vector<std::string> all = team_a_namespaces_;
    all.insert(all.end(), team_b_namespaces_.begin(), team_b_namespaces_.end());
    for (const auto & robot_id : all) {
      const std::string topic = expandTopicTemplate(odom_topic_template_, robot_id);
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS().keep_last(10),
        [this, robot_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odomCallback(robot_id, msg);
        }));
      role_pubs_[robot_id] = create_publisher<std_msgs::msg::String>(
        "/" + robot_id + "/football/role", latchedQos());
      tactical_target_pubs_[robot_id] = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + robot_id + "/football/tactical_target", latchedQos());
    }

    const double period = 1.0 / update_rate_hz_;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      [this]() {
        update();
      });

    RCLCPP_INFO(
      get_logger(),
      "football role authority=%s field=%s odom_mode=%s template=%s update_rate=%.2fHz",
      authority_id_.c_str(), field_frame_.c_str(), odom_source_mode_.c_str(),
      odom_topic_template_.c_str(), update_rate_hz_);
  }

rclcpp::Time FootballTeamRoleAssigner::zeroTime() const
{
    return rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

void FootballTeamRoleAssigner::validateTeamRosters() const
{
    if (team_a_namespaces_.size() != 5 || team_b_namespaces_.size() != 5) {
      throw std::invalid_argument("each football team must contain exactly five robots");
    }

    std::set<std::string> all_ids;
    const auto validate = [&all_ids](
      const std::vector<std::string> & team, const int first_id, const int last_id,
      const std::string & label)
      {
        for (const auto & robot_id : team) {
          const int numeric_id = numericRobotId(robot_id);
          const std::string expected = "cyberdog_" + std::to_string(numeric_id);
          if (numeric_id < first_id || numeric_id > last_id || robot_id != expected) {
            throw std::invalid_argument(
                    label + " contains invalid robot namespace: " + robot_id);
          }
          if (!all_ids.insert(robot_id).second) {
            throw std::invalid_argument("duplicate robot namespace: " + robot_id);
          }
        }
      };

    validate(team_a_namespaces_, 1, 5, "team_a");
    validate(team_b_namespaces_, 6, 10, "team_b");
  }

bool FootballTeamRoleAssigner::finitePose(const geometry_msgs::msg::Pose & pose) const
{
    const auto & q = pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && std::isfinite(q.x) &&
           std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) && norm > 1e-8;
  }

bool FootballTeamRoleAssigner::ballFresh(const rclcpp::Time & stamp) const
{
    if (!have_ball_) {
      return false;
    }
    const double age = (stamp - latest_ball_time_).seconds();
    return age >= 0.0 && age <= ball_timeout_sec_;
  }

void FootballTeamRoleAssigner::ballCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!msg || msg->header.frame_id != field_frame_ || !finitePose(msg->pose)) {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const double age = stamp.nanoseconds() > 0 ? (now() - stamp).seconds() : 1e9;
    if (age < -future_tolerance_sec_ || age > ball_timeout_sec_ ||
      (have_ball_ && stamp <= rclcpp::Time(
        latest_ball_.header.stamp, get_clock()->get_clock_type())))
    {
      return;
    }
    latest_ball_ = *msg;
    latest_ball_time_ = now();
    have_ball_ = true;
  }

void FootballTeamRoleAssigner::odomCallback(const std::string & robot_id, const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg || msg->header.frame_id != field_frame_ || !finitePose(msg->pose.pose)) {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const double age = stamp.nanoseconds() > 0 ? (now() - stamp).seconds() : 1e9;
    const auto previous = robot_odoms_.find(robot_id);
    if (age < -future_tolerance_sec_ || age > max_pose_age_sec_ ||
      (previous != robot_odoms_.end() && stamp <= previous->second.stamp))
    {
      return;
    }
    robot_odoms_[robot_id] = OdomState{*msg, stamp, now()};
  }

void FootballTeamRoleAssigner::authorityCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
      return;
    }
    std::stringstream stream(msg->data);
    std::string seen_authority;
    std::string seen_instance;
    std::string seen_time;
    if (!std::getline(stream, seen_authority, '|') ||
      !std::getline(stream, seen_instance, '|') ||
      !std::getline(stream, seen_time, '|') ||
      seen_instance == authority_instance_id_)
    {
      return;
    }
    double remote_time = 0.0;
    try {
      remote_time = std::stod(seen_time);
    } catch (const std::exception &) {
      return;
    }
    if (seen_authority.empty() || seen_instance.empty() || !std::isfinite(remote_time) ||
      std::fabs(now().seconds() - remote_time) > authority_timeout_sec_)
    {
      return;
    }
    authority_conflict_time_ = now();
    conflicting_authority_ = seen_authority + "/" + seen_instance;
  }

void FootballTeamRoleAssigner::resetPendingChallenger(std::string & challenger, rclcpp::Time & since)
{
    challenger.clear();
    since = zeroTime();
  }

void FootballTeamRoleAssigner::resetSelections()
{
    current_striker_a_.clear();
    current_striker_b_.clear();
    striker_since_a_ = zeroTime();
    striker_since_b_ = zeroTime();
    resetPendingChallenger(challenger_a_, challenger_since_a_);
    resetPendingChallenger(challenger_b_, challenger_since_b_);
  }

void FootballTeamRoleAssigner::startKickoff(const std::string & kickoff_state)
{
    match_state_ = kickoff_state;
    const auto stamp = now();
    kickoff_hold_until_ = stamp + rclcpp::Duration::from_seconds(kickoff_hold_sec_);
    kickoff_started_ = stamp;
    kickoff_reference_valid_ = false;
    resetSelections();
    publishMatchState();
    stopAllRoles("kickoff_start");
  }

void FootballTeamRoleAssigner::goalEventCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg) {
      return;
    }
    if (msg->data == "goal team_a kickoff") {
      startKickoff("KICKOFF_B");
    } else if (msg->data == "goal team_b kickoff") {
      startKickoff("KICKOFF_A");
    }
  }

void FootballTeamRoleAssigner::matchStateCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg || !validMatchState(msg->data)) {
      return;
    }
    if (msg->data == "KICKOFF_A" || msg->data == "KICKOFF_B") {
      startKickoff(msg->data);
      return;
    }

    const bool entering_new_play = msg->data == "PLAY" && match_state_ != "PLAY";
    match_state_ = msg->data;
    kickoff_hold_until_ = zeroTime();
    kickoff_started_ = zeroTime();
    kickoff_reference_valid_ = false;
    if (entering_new_play ||
      (!matchStateAllowsMovement("a") && !matchStateAllowsMovement("b")))
    {
      resetSelections();
    }
    publishMatchState();
    if (!matchStateAllowsMovement("a") && !matchStateAllowsMovement("b")) {
      stopAllRoles("match_command_" + match_state_);
    }
  }

bool FootballTeamRoleAssigner::validMatchState(const std::string & state)
{
    return state == "STOP" || state == "READY" || state == "PLAY" ||
           state == "KICKOFF_A" || state == "KICKOFF_B" || state == "FINISHED";
  }

bool FootballTeamRoleAssigner::matchStateAllowsMovement(const std::string & team_id) const
{
    return match_state_ == "PLAY" ||
           (match_state_ == "KICKOFF_A" && team_id == "a") ||
           (match_state_ == "KICKOFF_B" && team_id == "b");
  }

void FootballTeamRoleAssigner::publishMatchState()
{
    std_msgs::msg::String state;
    state.data = match_state_;
    match_state_pub_->publish(state);
  }

bool FootballTeamRoleAssigner::authorityConflictActive() const
{
    return authority_conflict_time_.nanoseconds() > 0 &&
           (now() - authority_conflict_time_).seconds() <= authority_timeout_sec_;
  }

RobotPose2D FootballTeamRoleAssigner::robotPose(const std::string & id) const
{
    RobotPose2D robot;
    robot.id = id;
    const auto found = robot_odoms_.find(id);
    if (found == robot_odoms_.end()) {
      return robot;
    }
    const double age = (now() - found->second.stamp).seconds();
    if (age < 0.0 || age > max_pose_age_sec_) {
      return robot;
    }
    const auto & odom = found->second.odom;
    robot.x = odom.pose.pose.position.x;
    robot.y = odom.pose.pose.position.y;
    if (!yawFromQuaternion(odom.pose.pose.orientation, robot.yaw)) {
      return robot;
    }
    const double local_vx = odom.twist.twist.linear.x;
    const double local_vy = odom.twist.twist.linear.y;
    robot.vx = std::cos(robot.yaw) * local_vx - std::sin(robot.yaw) * local_vy;
    robot.vy = std::sin(robot.yaw) * local_vx + std::cos(robot.yaw) * local_vy;
    robot.data_age = age;
    robot.stamp_sec = found->second.stamp.seconds();
    robot.valid = std::isfinite(robot.x) && std::isfinite(robot.y) &&
      std::isfinite(robot.yaw) && std::isfinite(robot.vx) && std::isfinite(robot.vy);
    return robot;
  }

std::vector<RobotPose2D> FootballTeamRoleAssigner::teamPoses(const std::vector<std::string> & ids) const
{
    std::vector<RobotPose2D> robots;
    robots.reserve(ids.size());
    for (const auto & id : ids) {
      robots.push_back(robotPose(id));
    }
    return robots;
  }

bool FootballTeamRoleAssigner::poseSkewAcceptable(const std::vector<RobotPose2D> & robots) const
{
    double oldest = std::numeric_limits<double>::infinity();
    double newest = -std::numeric_limits<double>::infinity();
    for (const auto & robot : robots) {
      if (!robot.valid) {
        continue;
      }
      oldest = std::min(oldest, robot.stamp_sec);
      newest = std::max(newest, robot.stamp_sec);
    }
    return !std::isfinite(oldest) || newest - oldest <= max_pose_skew_sec_;
  }

bool FootballTeamRoleAssigner::currentStrikerFresh(
  const std::vector<RobotPose2D> & team, const std::string & current)
{
    return std::any_of(
      team.begin(), team.end(), [&current](const RobotPose2D & robot) {
        return robot.valid && robot.id == current;
      });
  }

int FootballTeamRoleAssigner::countValidRobots(const std::vector<RobotPose2D> & team)
{
    return static_cast<int>(std::count_if(
      team.begin(), team.end(), [](const RobotPose2D & robot) {
        return robot.valid;
      }));
  }

void FootballTeamRoleAssigner::publishAuthority()
{
    std_msgs::msg::String heartbeat;
    heartbeat.data = authority_id_ + "|" + authority_instance_id_ + "|" +
      std::to_string(now().seconds());
    authority_pub_->publish(heartbeat);
  }

void FootballTeamRoleAssigner::publishStrikers()
{
    std_msgs::msg::String team_a;
    team_a.data = current_striker_a_;
    striker_a_pub_->publish(team_a);
    std_msgs::msg::String team_b;
    team_b.data = current_striker_b_;
    striker_b_pub_->publish(team_b);
  }

void FootballTeamRoleAssigner::publishKickTargets()
{
    geometry_msgs::msg::PoseStamped team_a;
    team_a.header.stamp = now();
    team_a.header.frame_id = field_frame_;
    team_a.pose.position.x = team_a_attack_goal_x_;
    team_a.pose.position.y = team_a_attack_goal_y_;
    team_a.pose.orientation.w = 1.0;
    kick_target_a_pub_->publish(team_a);

    auto team_b = team_a;
    team_b.pose.position.x = team_b_attack_goal_x_;
    team_b.pose.position.y = team_b_attack_goal_y_;
    kick_target_b_pub_->publish(team_b);
  }

double FootballTeamRoleAssigner::clampTacticalX(const double x) const
{
    return std::clamp(
      x, field_min_x_ + tactical_boundary_margin_m_,
      field_max_x_ - tactical_boundary_margin_m_);
  }

double FootballTeamRoleAssigner::clampTacticalY(const double y) const
{
    return std::clamp(
      y, field_min_y_ + tactical_boundary_margin_m_,
      field_max_y_ - tactical_boundary_margin_m_);
  }

void FootballTeamRoleAssigner::publishRole(
  const std::string & robot_id, const std::string & role,
  const double x, const double y)
{
    std_msgs::msg::String role_msg;
    role_msg.data = role;
    role_pubs_.at(robot_id)->publish(role_msg);
    if (role == "STOP") {
      return;
    }
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = field_frame_;
    target.pose.position.x = clampTacticalX(x);
    target.pose.position.y = clampTacticalY(y);
    target.pose.orientation.w = 1.0;
    tactical_target_pubs_.at(robot_id)->publish(target);
  }

void FootballTeamRoleAssigner::publishTeamTactics(
  const std::vector<RobotPose2D> & team, const std::string & team_id,
  const std::string & striker)
{
    std::map<std::string, RoleDecision> commands;
    for (const auto & robot : team) {
      commands.emplace(robot.id, RoleDecision{});
    }

    if (matchStateAllowsMovement(team_id) && !striker.empty()) {
      const auto context = makeStrategyContext(team_id);
      auto striker_command = commands.find(striker);
      if (striker_command != commands.end()) {
        striker_command->second = striker_strategy_.decide(context);
      }

      std::vector<RobotPose2D> remaining;
      for (const auto & robot : team) {
        if (robot.valid && robot.id != striker) {
          remaining.push_back(robot);
        }
      }
      std::sort(
        remaining.begin(), remaining.end(),
        [](const RobotPose2D & lhs, const RobotPose2D & rhs) {
          return numericRobotId(lhs.id) < numericRobotId(rhs.id);
        });

      const RoleStrategy * strategies[] = {
        &support_strategy_, &defender_left_strategy_,
        &defender_right_strategy_, &goalkeeper_strategy_};
      for (std::size_t i = 0; i < remaining.size() && i < 4; ++i) {
        commands.at(remaining[i].id) = strategies[i]->decide(context);
      }
    }

    // Publish exactly one role command per robot per update. Publishing STOP
    // first and the active role second would make the tracking client cancel
    // a healthy action goal at every role-authority tick.
    for (const auto & robot : team) {
      const auto & command = commands.at(robot.id);
      publishRole(robot.id, command.role, command.target_x, command.target_y);
    }
  }

RoleStrategyContext FootballTeamRoleAssigner::makeStrategyContext(
  const std::string & team_id) const
{
    RoleStrategyContext context;
    context.ball_x = latest_ball_.pose.position.x;
    context.ball_y = latest_ball_.pose.position.y;
    if (team_id == "a") {
      context.attack_goal_x = team_a_attack_goal_x_;
      context.attack_goal_y = team_a_attack_goal_y_;
      context.home_goal_x = team_b_attack_goal_x_;
      context.home_goal_y = team_b_attack_goal_y_;
    } else {
      context.attack_goal_x = team_b_attack_goal_x_;
      context.attack_goal_y = team_b_attack_goal_y_;
      context.home_goal_x = team_a_attack_goal_x_;
      context.home_goal_y = team_a_attack_goal_y_;
    }
    return context;
  }

std::string FootballTeamRoleAssigner::selectNearestStriker(
  const std::vector<RobotPose2D> & robots, const std::string & current,
  rclcpp::Time & current_since, std::string & challenger,
  rclcpp::Time & challenger_since, const rclcpp::Time & stamp)
{
    const auto best_index = indexOfClosestRobot(
      robots, latest_ball_.pose.position.x, latest_ball_.pose.position.y);
    if (best_index >= robots.size()) {
      current_since = zeroTime();
      resetPendingChallenger(challenger, challenger_since);
      return "";
    }

    const std::string best_id = robots[best_index].id;
    const auto current_it = std::find_if(
      robots.begin(), robots.end(), [&current](const RobotPose2D & robot) {
        return robot.valid && robot.id == current;
      });

    if (current.empty() || current_it == robots.end()) {
      current_since = stamp;
      resetPendingChallenger(challenger, challenger_since);
      return best_id;
    }
    if (best_id == current) {
      resetPendingChallenger(challenger, challenger_since);
      return current;
    }
    if ((stamp - current_since).seconds() < striker_min_hold_sec_) {
      resetPendingChallenger(challenger, challenger_since);
      return current;
    }

    const auto context = makeStrategyContext("a");
    const double best_distance = StrikerRoleStrategy::distanceToBall(
      robots[best_index].x, robots[best_index].y, context);
    const double current_distance = StrikerRoleStrategy::distanceToBall(
      current_it->x, current_it->y, context);
    if (!(best_distance + striker_benefit_threshold_ < current_distance)) {
      resetPendingChallenger(challenger, challenger_since);
      return current;
    }

    if (challenger != best_id) {
      challenger = best_id;
      challenger_since = stamp;
      return current;
    }
    if (striker_switch_confirm_sec_ > 0.0 &&
      (stamp - challenger_since).seconds() < striker_switch_confirm_sec_)
    {
      return current;
    }

    current_since = stamp;
    resetPendingChallenger(challenger, challenger_since);
    return best_id;
  }

void FootballTeamRoleAssigner::publishStatus(const std::string & prefix)
{
    std_msgs::msg::String status;
    status.data = prefix + " team_a_striker=" + current_striker_a_ +
      " team_b_striker=" + current_striker_b_;
    status_pub_->publish(status);
  }

void FootballTeamRoleAssigner::stopAllRoles(const std::string & reason)
{
    for (const auto & item : role_pubs_) {
      publishRole(item.first, "STOP", 0.0, 0.0);
    }
    publishStrikers();
    publishStatus(reason);
  }

void FootballTeamRoleAssigner::clearRoles(const std::string & reason)
{
    resetSelections();
    stopAllRoles(reason);
  }

void FootballTeamRoleAssigner::clearTeamSelection(
  std::string & current, rclcpp::Time & current_since,
  std::string & challenger, rclcpp::Time & challenger_since)
{
    current.clear();
    current_since = zeroTime();
    resetPendingChallenger(challenger, challenger_since);
  }

void FootballTeamRoleAssigner::updateSelections(
  const std::vector<RobotPose2D> & team_a,
  const std::vector<RobotPose2D> & team_b,
  const rclcpp::Time & stamp)
{
    const bool team_a_eligible = matchStateAllowsMovement("a") &&
      countValidRobots(team_a) >= minimum_online_per_team_ &&
      poseSkewAcceptable(team_a);
    if (team_a_eligible) {
      if (!forced_team_a_striker_namespace_.empty()) {
        const auto forced = std::find_if(
          team_a.begin(), team_a.end(), [this](const RobotPose2D & robot) {
            return robot.valid && robot.id == forced_team_a_striker_namespace_;
          });
        if (forced == team_a.end()) {
          clearTeamSelection(
            current_striker_a_, striker_since_a_,
            challenger_a_, challenger_since_a_);
        } else {
          if (current_striker_a_ != forced_team_a_striker_namespace_) {
            striker_since_a_ = stamp;
          }
          current_striker_a_ = forced_team_a_striker_namespace_;
          resetPendingChallenger(challenger_a_, challenger_since_a_);
        }
      } else {
        current_striker_a_ = selectNearestStriker(
          team_a, current_striker_a_, striker_since_a_,
          challenger_a_, challenger_since_a_, stamp);
      }
    } else {
      clearTeamSelection(
        current_striker_a_, striker_since_a_,
        challenger_a_, challenger_since_a_);
    }

    const bool team_b_eligible = matchStateAllowsMovement("b") &&
      countValidRobots(team_b) >= minimum_online_per_team_ &&
      poseSkewAcceptable(team_b);
    if (team_b_eligible) {
      current_striker_b_ = selectNearestStriker(
        team_b, current_striker_b_, striker_since_b_,
        challenger_b_, challenger_since_b_, stamp);
    } else {
      clearTeamSelection(
        current_striker_b_, striker_since_b_,
        challenger_b_, challenger_since_b_);
    }
    publishStrikers();
  }

void FootballTeamRoleAssigner::captureKickoffReference()
{
    kickoff_ball_x_ = latest_ball_.pose.position.x;
    kickoff_ball_y_ = latest_ball_.pose.position.y;
    kickoff_reference_valid_ = true;
  }

void FootballTeamRoleAssigner::update()
{
    const auto stamp = now();
    publishAuthority();

    auto team_a = teamPoses(team_a_namespaces_);
    auto team_b = teamPoses(team_b_namespaces_);
    std::vector<RobotPose2D> all = team_a;
    all.insert(all.end(), team_b.begin(), team_b.end());
    publishKickTargets();

    if (authorityConflictActive()) {
      clearRoles("SAFE_STOP conflicting_authority=" + conflicting_authority_);
      return;
    }

    const bool kickoff_state = match_state_ == "KICKOFF_A" || match_state_ == "KICKOFF_B";
    const bool fresh_ball = ballFresh(stamp);
    if (kickoff_state && kickoff_hold_until_.nanoseconds() > 0 && stamp < kickoff_hold_until_) {
      if (fresh_ball) {
        captureKickoffReference();
      }
      publishMatchState();
      clearRoles("kickoff_hold");
      return;
    }

    if (!matchStateAllowsMovement("a") && !matchStateAllowsMovement("b")) {
      clearRoles("match_" + match_state_);
      return;
    }
    if (!fresh_ball) {
      clearRoles("waiting_ball");
      return;
    }

    if (kickoff_state) {
      if (!kickoff_reference_valid_) {
        captureKickoffReference();
        clearRoles("kickoff_reference_acquired");
        return;
      }
      const bool ball_released = std::hypot(
        latest_ball_.pose.position.x - kickoff_ball_x_,
        latest_ball_.pose.position.y - kickoff_ball_y_) >= kickoff_release_distance_m_;
      const bool kickoff_expired = kickoff_started_.nanoseconds() > 0 &&
        (stamp - kickoff_started_).seconds() >= kickoff_max_duration_sec_;
      if (ball_released || kickoff_expired) {
        match_state_ = "PLAY";
        kickoff_hold_until_ = zeroTime();
        kickoff_started_ = zeroTime();
        kickoff_reference_valid_ = false;
        resetSelections();
        publishMatchState();
      }
    }

    updateSelections(team_a, team_b, stamp);

    // Role selection and motion permission are separate contracts. Each team
    // may immediately re-elect from its remaining fresh members, but motion is
    // fail-closed until every robot is available for the nine-peer obstacle
    // feed required by each selected robot.
    if (countValidRobots(all) < minimum_other_robot_count_ + 1) {
      stopAllRoles("SAFE_STOP team_data_stale");
      return;
    }
    if (!poseSkewAcceptable(all)) {
      stopAllRoles("SAFE_STOP team_pose_skew");
      return;
    }

    if (matchStateAllowsMovement("a") && current_striker_a_.empty()) {
      clearRoles("SAFE_STOP team_a_no_valid_striker");
      return;
    }
    if (matchStateAllowsMovement("b") && current_striker_b_.empty()) {
      clearRoles("SAFE_STOP team_b_no_valid_striker");
      return;
    }

    publishTeamTactics(team_a, "a", current_striker_a_);
    publishTeamTactics(team_b, "b", current_striker_b_);
    publishStatus("active");
  }


}  // namespace football_navigation
