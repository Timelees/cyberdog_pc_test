// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.
//
// 假球：在配置位置发布静态足球位姿，供导航和踢球动作流程验证。
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "football_navigation/core/football_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "football_navigation/simulation/football_fake_ball_publisher.hpp"

namespace football_navigation
{
namespace
{

std::string normalizeNamespace(const std::string & ns)
{
  if (ns.empty()) {
    return "";
  }
  if (ns.front() == '/') {
    return ns.substr(1);
  }
  return ns;
}

}  // namespace

FootballFakeBallPublisher::FootballFakeBallPublisher()
  : Node("football_fake_ball_publisher")
{
    frame_id_ = declare_parameter<std::string>("frame_id", "tag_global");
    // 假球模式仍发布到原始输入话题；之后必须由融合节点生成
    // /football/ball_pose，从而保留与真实数据相同的校验链路。
    ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose_raw");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    ball_x_ = declare_parameter<double>("ball_x", 2.0);
    ball_y_ = declare_parameter<double>("ball_y", 0.0);
    ball_z_ = declare_parameter<double>("ball_z", 0.0);
    kickoff_x_ = declare_parameter<double>("kickoff_x", 0.0);
    kickoff_y_ = declare_parameter<double>("kickoff_y", 0.0);
    robot_front_extent_m_ = declare_parameter<double>("robot_front_extent_m", 0.25);
    robot_rear_extent_m_ = declare_parameter<double>("robot_rear_extent_m", 0.23);
    robot_half_width_m_ = declare_parameter<double>("robot_half_width_m", 0.13);
    ball_radius_m_ = declare_parameter<double>("ball_radius_m", 0.11);
    contact_tolerance_m_ = declare_parameter<double>("contact_tolerance_m", 0.035);
    goal_width_m_ = declare_parameter<double>("goal_width_m", 1.2);
    field_min_x_ = declare_parameter<double>("field_min_x", -2.0);
    field_max_x_ = declare_parameter<double>("field_max_x", 8.0);
    field_min_y_ = declare_parameter<double>("field_min_y", -3.0);
    field_max_y_ = declare_parameter<double>("field_max_y", 3.0);
    kickoff_hold_sec_ = declare_parameter<double>("kickoff_hold_sec", 3.0);
    team_a_attack_goal_x_ = declare_parameter<double>("team_a_attack_goal_x", 8.0);
    team_a_attack_goal_y_ = declare_parameter<double>("team_a_attack_goal_y", 0.0);
    team_b_attack_goal_x_ = declare_parameter<double>("team_b_attack_goal_x", -2.0);
    team_b_attack_goal_y_ = declare_parameter<double>("team_b_attack_goal_y", 0.0);
    team_a_striker_topic_ =
      declare_parameter<std::string>("team_a_striker_topic", "/football/team_a/striker");
    team_b_striker_topic_ =
      declare_parameter<std::string>("team_b_striker_topic", "/football/team_b/striker");
    goal_event_topic_ =
      declare_parameter<std::string>("goal_event_topic", "/football/goal_scored");
    ball_possession_striker_topic_ = declare_parameter<std::string>(
      "ball_possession_striker_topic", "/football/ball_possession_striker");
    ball_contest_radius_m_ = declare_parameter<double>("ball_contest_radius_m", 3.0);
    odom_topic_template_ = declare_parameter<std::string>(
      "odom_topic_template", "/{namespace}/odom_global");
    continuous_demo_enabled_ =
      declare_parameter<bool>("continuous_demo_enabled", false);
    single_shot_enabled_ =
      declare_parameter<bool>("single_shot_enabled", false);
    controlled_robot_namespace_ = normalizeNamespace(
      declare_parameter<std::string>("controlled_robot_namespace", ""));
    match_state_topic_ = declare_parameter<std::string>(
      "match_state_topic", "/football/match_state");
    front_contact_topic_ = declare_parameter<std::string>(
      "front_contact_topic", "/football/fake_ball/front_contact");
    side_contact_topic_ = declare_parameter<std::string>(
      "side_contact_topic", "/football/fake_ball/side_contact");
    rear_contact_topic_ = declare_parameter<std::string>(
      "rear_contact_topic", "/football/fake_ball/rear_contact");
    contact_robot_topic_ = declare_parameter<std::string>(
      "contact_robot_topic", "/football/fake_ball/contact_robot");
    controlled_robot_pose_seen_topic_ = declare_parameter<std::string>(
      "controlled_robot_pose_seen_topic",
      "/football/fake_ball/controlled_robot_pose_seen");
    controlled_robot_pose_fresh_topic_ = declare_parameter<std::string>(
      "controlled_robot_pose_fresh_topic",
      "/football/fake_ball/controlled_robot_pose_fresh");
    controlled_robot_transform_valid_topic_ = declare_parameter<std::string>(
      "controlled_robot_transform_valid_topic",
      "/football/fake_ball/controlled_robot_transform_valid");
    controlled_robot_seen_topic_ = declare_parameter<std::string>(
      "controlled_robot_seen_topic", "/football/fake_ball/controlled_robot_seen");
    ball_velocity_topic_ = declare_parameter<std::string>(
      "ball_velocity_topic", "/football/fake_ball/velocity");

    if (publish_rate_hz_ <= 0.0 || robot_front_extent_m_ <= 0.0 ||
      robot_rear_extent_m_ <= 0.0 || robot_half_width_m_ <= 0.0 ||
      ball_radius_m_ <= 0.0 ||
      contact_tolerance_m_ < 0.0 ||
      field_min_x_ >= field_max_x_ || field_min_y_ >= field_max_y_ ||
      kickoff_hold_sec_ < 0.0 ||
      ((continuous_demo_enabled_ || single_shot_enabled_) &&
      controlled_robot_namespace_.empty()))
    {
      throw std::invalid_argument("invalid fake ball parameters");
    }

    sim_ball_x_ = ball_x_;
    sim_ball_y_ = ball_y_;
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      ball_topic_, rclcpp::SensorDataQoS().keep_last(5));
    goal_event_pub_ = create_publisher<std_msgs::msg::String>(goal_event_topic_, 10);
    match_state_command_pub_ = create_publisher<std_msgs::msg::String>(
      "/football/match_state_command", rclcpp::QoS(10).reliable());
    ball_possession_pub_ = create_publisher<std_msgs::msg::String>(
      ball_possession_striker_topic_, 10);
    front_contact_pub_ = create_publisher<std_msgs::msg::Bool>(
      front_contact_topic_, rclcpp::QoS(10).reliable());
    side_contact_pub_ = create_publisher<std_msgs::msg::Bool>(
      side_contact_topic_, rclcpp::QoS(10).reliable());
    rear_contact_pub_ = create_publisher<std_msgs::msg::Bool>(
      rear_contact_topic_, rclcpp::QoS(10).reliable());
    contact_robot_pub_ = create_publisher<std_msgs::msg::String>(
      contact_robot_topic_, rclcpp::QoS(10).reliable());
    controlled_robot_pose_seen_pub_ = create_publisher<std_msgs::msg::Bool>(
      controlled_robot_pose_seen_topic_, rclcpp::QoS(1).reliable().transient_local());
    controlled_robot_pose_fresh_pub_ = create_publisher<std_msgs::msg::Bool>(
      controlled_robot_pose_fresh_topic_, rclcpp::QoS(1).reliable().transient_local());
    controlled_robot_transform_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      controlled_robot_transform_valid_topic_, rclcpp::QoS(1).reliable().transient_local());
    controlled_robot_seen_pub_ = create_publisher<std_msgs::msg::Bool>(
      controlled_robot_seen_topic_, rclcpp::QoS(1).reliable().transient_local());
    ball_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      ball_velocity_topic_, rclcpp::QoS(10).reliable());

    striker_a_sub_ = create_subscription<std_msgs::msg::String>(
      team_a_striker_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        striker_a_ = normalizeNamespace(msg->data);
        have_striker_a_ = !striker_a_.empty();
      });
    striker_b_sub_ = create_subscription<std_msgs::msg::String>(
      team_b_striker_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        striker_b_ = normalizeNamespace(msg->data);
        have_striker_b_ = !striker_b_.empty();
      });
    match_state_sub_ = create_subscription<std_msgs::msg::String>(
      match_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg && (msg->data == "STOP" || msg->data == "READY" ||
          msg->data == "KICKOFF_A" || msg->data == "KICKOFF_B" ||
          msg->data == "PLAY" || msg->data == "FINISHED"))
        {
          match_state_ = msg->data;
        }
      });
    for (int robot_index = 1; robot_index <= 10; ++robot_index) {
      const std::string robot = "cyberdog_" + std::to_string(robot_index);
      std::string topic = odom_topic_template_;
      const auto token = topic.find("{namespace}");
      if (token == std::string::npos) {
        throw std::invalid_argument("odom_topic_template must contain {namespace}");
      }
      topic.replace(token, 11, robot);
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS().keep_last(5),
        [this, robot](const nav_msgs::msg::Odometry::SharedPtr msg) {
          if (robot == controlled_robot_namespace_ && msg) {
            controlled_robot_pose_seen_ = true;
          }

          double yaw = 0.0;
          const bool transform_valid =
            msg &&
            msg->header.frame_id == frame_id_ &&
            std::isfinite(msg->pose.pose.position.x) &&
            std::isfinite(msg->pose.pose.position.y) &&
            std::isfinite(msg->twist.twist.linear.x) &&
            std::isfinite(msg->twist.twist.linear.y) &&
            std::isfinite(msg->twist.twist.angular.z) &&
            yawFromQuaternion(msg->pose.pose.orientation, yaw);
          if (robot == controlled_robot_namespace_) {
            controlled_robot_transform_valid_ = transform_valid;
          }
          if (!transform_valid) {
            return;
          }

          const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
          const double age = stamp.nanoseconds() > 0 ?
            (now() - stamp).seconds() : std::numeric_limits<double>::infinity();
          const auto previous = robot_odoms_.find(robot);
          if (stamp.nanoseconds() <= 0 || age > 0.5 || age < -0.08 ||
            (previous != robot_odoms_.end() && stamp <= rclcpp::Time(
              previous->second.header.stamp, get_clock()->get_clock_type())))
          {
            return;
          }
          robot_odoms_[robot] = *msg;
        }));
    }

    const double period_sec = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.1;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_sec)),
      std::bind(&FootballFakeBallPublisher::publishBallPose, this));

    RCLCPP_INFO(
      get_logger(),
      "football_fake_ball_publisher topic=%s frame=%s init=(%.2f,%.2f) kickoff=(%.2f,%.2f) "
      "robot_footprint=(%.2f,%.2f) ball_radius=%.2f "
      "goal_w=%.2f continuous_demo=%s single_shot=%s controlled_robot=%s",
      ball_topic_.c_str(), frame_id_.c_str(), ball_x_, ball_y_, kickoff_x_, kickoff_y_,
      robot_front_extent_m_ + robot_rear_extent_m_,
      2.0 * robot_half_width_m_, ball_radius_m_,
      goal_width_m_,
      continuous_demo_enabled_ ? "true" : "false",
      single_shot_enabled_ ? "true" : "false",
      controlled_robot_namespace_.c_str());
  }

bool FootballFakeBallPublisher::lookupRobotInField(const std::string & ns, double & x, double & y) const
{
    if (frame_id_.empty() || ns.empty()) {
      return false;
    }
    const auto found = robot_odoms_.find(normalizeNamespace(ns));
    if (found == robot_odoms_.end()) {
      return false;
    }
    const rclcpp::Time stamp(found->second.header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() <= 0 || (now() - stamp).seconds() > 0.5) {
      return false;
    }
    x = found->second.pose.pose.position.x;
    y = found->second.pose.pose.position.y;
    return true;
  }

void FootballFakeBallPublisher::getTeamGoalForStriker(const std::string & striker, double & gx, double & gy) const
{
    if (have_striker_a_ && striker == striker_a_) {
      gx = team_a_attack_goal_x_;
      gy = team_a_attack_goal_y_;
      return;
    }
    if (have_striker_b_ && striker == striker_b_) {
      gx = team_b_attack_goal_x_;
      gy = team_b_attack_goal_y_;
      return;
    }
    gx = team_a_attack_goal_x_;
    gy = team_a_attack_goal_y_;
  }

bool FootballFakeBallPublisher::inKickoffHold(const rclcpp::Time & stamp) const
{
    return kickoff_hold_until_.nanoseconds() > 0 && stamp < kickoff_hold_until_;
  }

bool FootballFakeBallPublisher::scoredAtGoal(
  const double goal_x, const double goal_y,
  const double home_x, const double home_y) const
{
    const double dx = goal_x - home_x;
    const double dy = goal_y - home_y;
    const double norm = std::hypot(dx, dy);
    if (norm < 1e-6) {
      return false;
    }
    const double ux = dx / norm;
    const double uy = dy / norm;
    const double to_ball_x = sim_ball_x_ - goal_x;
    const double to_ball_y = sim_ball_y_ - goal_y;
    const double center_plane = to_ball_x * ux + to_ball_y * uy;
    const double lateral = std::abs(-to_ball_x * uy + to_ball_y * ux);
    return center_plane >= 0.0 && lateral + ball_radius_m_ <= goal_width_m_ * 0.5;
  }

bool FootballFakeBallPublisher::teamAScored() const
{
    return scoredAtGoal(
      team_a_attack_goal_x_, team_a_attack_goal_y_,
      team_b_attack_goal_x_, team_b_attack_goal_y_);
  }

bool FootballFakeBallPublisher::teamBScored() const
{
    return scoredAtGoal(
      team_b_attack_goal_x_, team_b_attack_goal_y_,
      team_a_attack_goal_x_, team_a_attack_goal_y_);
  }

void FootballFakeBallPublisher::publishMatchStateCommand(const std::string & state)
{
    std_msgs::msg::String command;
    command.data = state;
    match_state_command_pub_->publish(command);
    match_state_ = state;
  }

void FootballFakeBallPublisher::resetToKickoff(const rclcpp::Time & stamp, const char * scoring_team)
{
    sim_ball_x_ = kickoff_x_;
    sim_ball_y_ = kickoff_y_;
    kickoff_hold_until_ = stamp + rclcpp::Duration::from_seconds(kickoff_hold_sec_);
    const std::string kickoff_state = std::string(scoring_team) == "a" ?
      "KICKOFF_B" : "KICKOFF_A";
    publishMatchStateCommand(kickoff_state);
    pending_opponent_kickoff_ = continuous_demo_enabled_ &&
      inferTeamIdFromNamespace(controlled_robot_namespace_) !=
      (kickoff_state == "KICKOFF_A" ? "a" : "b");
    std_msgs::msg::String event;
    event.data = std::string("goal team_") + scoring_team + " kickoff";
    goal_event_pub_->publish(event);
    RCLCPP_INFO(
      get_logger(),
      "GOAL team_%s! ball kickoff at (%.2f, %.2f), hold %.1fs before play resumes",
      scoring_team, kickoff_x_, kickoff_y_, kickoff_hold_sec_);
  }

void FootballFakeBallPublisher::finishSingleShot(const char * scoring_team)
{
    goal_completed_ = true;

    std_msgs::msg::String event;
    event.data = std::string("goal team_") + scoring_team + " single_shot";
    goal_event_pub_->publish(event);
    publishMatchStateCommand("FINISHED");

    RCLCPP_INFO(
      get_logger(),
      "GOAL team_%s! single-shot ball remains beyond the goal line at (%.2f, %.2f)",
      scoring_team, sim_ball_x_, sim_ball_y_);
  }

std::string FootballFakeBallPublisher::selectBallPossessionStriker() const
{
    struct Candidate
    {
      std::string ns;
      double dist_goal;
      double dist_ball;
    };
    std::vector<Candidate> in_contest;
    std::vector<Candidate> all;
    const std::string strikers[2] = {striker_a_, striker_b_};
    const bool have_striker[2] = {have_striker_a_, have_striker_b_};
    for (int i = 0; i < 2; ++i) {
      if (!have_striker[i] || strikers[i].empty()) {
        continue;
      }
      double rx = 0.0;
      double ry = 0.0;
      if (!lookupRobotInField(strikers[i], rx, ry)) {
        continue;
      }
      double gx = 0.0;
      double gy = 0.0;
      getTeamGoalForStriker(strikers[i], gx, gy);
      const double dist_ball = std::hypot(sim_ball_x_ - rx, sim_ball_y_ - ry);
      const double dist_goal = std::hypot(gx - rx, gy - ry);
      all.push_back({strikers[i], dist_goal, dist_ball});
      if (dist_ball <= ball_contest_radius_m_) {
        in_contest.push_back({strikers[i], dist_goal, dist_ball});
      }
    }
    const auto pick = [](const std::vector<Candidate> & pool) -> std::string {
      if (pool.empty()) {
        return "";
      }
      const Candidate * best = &pool.front();
      for (const auto & cand : pool) {
        if (cand.dist_ball < best->dist_ball ||
          (std::abs(cand.dist_ball - best->dist_ball) < 0.05 && cand.dist_goal < best->dist_goal))
        {
          best = &cand;
        }
      }
      return best->ns;
    };
    if (!in_contest.empty()) {
      return pick(in_contest);
    }
    return pick(all);
  }

void FootballFakeBallPublisher::publishBallPossessionStriker(const std::string & possessor)
{
    std_msgs::msg::String msg;
    msg.data = possessor;
    ball_possession_pub_->publish(msg);
  }

bool FootballFakeBallPublisher::controlledRobotPoseFresh(const rclcpp::Time & stamp) const
{
    if (controlled_robot_namespace_.empty()) {
      return false;
    }

    const auto found = robot_odoms_.find(controlled_robot_namespace_);
    if (found == robot_odoms_.end()) {
      return false;
    }

    const rclcpp::Time odom_stamp(
      found->second.header.stamp, get_clock()->get_clock_type());
    if (odom_stamp.nanoseconds() <= 0) {
      return false;
    }

    const double age = (stamp - odom_stamp).seconds();
    return age >= -0.08 && age <= 0.5;
  }

bool FootballFakeBallPublisher::controlledRobotSeen(const rclcpp::Time & stamp) const
{
    return controlled_robot_pose_seen_ &&
      controlled_robot_transform_valid_ &&
      controlledRobotPoseFresh(stamp);
  }

void FootballFakeBallPublisher::updateControlledContactDiagnostics(const rclcpp::Time & stamp)
{
    current_front_contact_ = false;
    current_side_contact_ = false;
    current_rear_contact_ = false;
    current_contact_robot_.clear();
    controlled_robot_pose_fresh_ = controlledRobotPoseFresh(stamp);

    if (!controlled_robot_pose_fresh_ ||
      !controlled_robot_transform_valid_ ||
      controlled_robot_namespace_.empty())
    {
      return;
    }

    const auto found = robot_odoms_.find(controlled_robot_namespace_);
    if (found == robot_odoms_.end()) {
      return;
    }

    double robot_yaw = 0.0;
    if (!yawFromQuaternion(found->second.pose.pose.orientation, robot_yaw)) {
      controlled_robot_transform_valid_ = false;
      return;
    }

    const auto contact = computeBallContactMetrics(
      found->second.pose.pose.position.x,
      found->second.pose.pose.position.y,
      robot_yaw,
      robot_front_extent_m_,
      robot_rear_extent_m_,
      robot_half_width_m_,
      sim_ball_x_,
      sim_ball_y_,
      ball_radius_m_,
      contact_tolerance_m_);
    current_front_contact_ = contact.front_contact;
    current_side_contact_ = contact.side_contact;
    current_rear_contact_ = contact.rear_contact;
    if (contact.intersects) {
      current_contact_robot_ = controlled_robot_namespace_;
    }
  }

void FootballFakeBallPublisher::publishDiagnostics(const rclcpp::Time & stamp)
{
    updateControlledContactDiagnostics(stamp);

    std_msgs::msg::Bool contact;
    contact.data = current_front_contact_;
    front_contact_pub_->publish(contact);

    std_msgs::msg::Bool side;
    side.data = current_side_contact_;
    side_contact_pub_->publish(side);

    std_msgs::msg::Bool rear;
    rear.data = current_rear_contact_;
    rear_contact_pub_->publish(rear);

    std_msgs::msg::String robot;
    robot.data = current_contact_robot_;
    contact_robot_pub_->publish(robot);

    std_msgs::msg::Bool pose_seen;
    pose_seen.data = controlled_robot_pose_seen_;
    controlled_robot_pose_seen_pub_->publish(pose_seen);

    std_msgs::msg::Bool pose_fresh;
    pose_fresh.data = controlled_robot_pose_fresh_;
    controlled_robot_pose_fresh_pub_->publish(pose_fresh);

    std_msgs::msg::Bool transform_valid;
    transform_valid.data = controlled_robot_transform_valid_;
    controlled_robot_transform_valid_pub_->publish(transform_valid);

    std_msgs::msg::Bool controlled_seen;
    controlled_seen.data = controlledRobotSeen(stamp);
    controlled_robot_seen_pub_->publish(controlled_seen);

    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = frame_id_;
    velocity.twist.linear.x = 0.0;
    velocity.twist.linear.y = 0.0;
    ball_velocity_pub_->publish(velocity);
  }

bool FootballFakeBallPublisher::outOfBounds() const
{
    return sim_ball_x_ < field_min_x_ ||
      sim_ball_x_ > field_max_x_ ||
      sim_ball_y_ < field_min_y_ ||
      sim_ball_y_ > field_max_y_;
  }

void FootballFakeBallPublisher::performPendingOpponentKickoff(const rclcpp::Time & stamp)
{
    if (!pending_opponent_kickoff_ || inKickoffHold(stamp)) {
      return;
    }

    // The fake ball remains static; only match state changes after kickoff.
    pending_opponent_kickoff_ = false;
    publishMatchStateCommand("PLAY");
  }

void FootballFakeBallPublisher::signalOutOfBounds()
{
    goal_completed_ = true;
    std_msgs::msg::String event;
    event.data = "OUT_OF_BOUNDS";
    goal_event_pub_->publish(event);
    publishMatchStateCommand("FINISHED");
  }

void FootballFakeBallPublisher::publishBallPose()
{
    const auto stamp = now();
    if (goal_completed_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = sim_ball_x_;
      pose.pose.position.y = sim_ball_y_;
      pose.pose.position.z = ball_z_;
      pose.pose.orientation.w = 1.0;
      publisher_->publish(pose);
      publishDiagnostics(stamp);
      return;
    }

    performPendingOpponentKickoff(stamp);
    publishBallPossessionStriker(inKickoffHold(stamp) ? "" : selectBallPossessionStriker());
    if (teamAScored()) {
      if (single_shot_enabled_) {
        finishSingleShot("a");
      } else {
        resetToKickoff(stamp, "a");
      }
    } else if (teamBScored()) {
      if (single_shot_enabled_) {
        finishSingleShot("b");
      } else {
        resetToKickoff(stamp, "b");
      }
    } else if (outOfBounds()) {
      signalOutOfBounds();
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = sim_ball_x_;
    pose.pose.position.y = sim_ball_y_;
    pose.pose.position.z = ball_z_;
    pose.pose.orientation.w = 1.0;
    publisher_->publish(pose);
    publishDiagnostics(stamp);
  }


}  // namespace football_navigation
