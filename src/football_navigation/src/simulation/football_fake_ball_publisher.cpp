// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.
//
// 假球：开局静止在 ball_x/ball_y；所有机器人按真实接触方向推球；进球后中场开球。
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
#include "football_navigation/football_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

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

class FootballFakeBallPublisher : public rclcpp::Node
{
public:
  FootballFakeBallPublisher()
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
    minimum_contact_speed_mps_ =
      declare_parameter<double>("minimum_contact_speed_mps", 0.02);
    ball_friction_mps2_ = declare_parameter<double>("ball_friction_mps2", 0.8);
    max_sim_ball_speed_mps_ = declare_parameter<double>("max_sim_ball_speed_mps", 0.25);
    max_ball_acceleration_mps2_ = declare_parameter<double>(
      "max_ball_acceleration_mps2", 0.45);
    velocity_transfer_gain_ = declare_parameter<double>("velocity_transfer_gain", 0.85);
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
    control_state_topic_ = declare_parameter<std::string>(
      "control_state_topic", "football/state");
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
      contact_tolerance_m_ < 0.0 || minimum_contact_speed_mps_ < 0.0 ||
      ball_friction_mps2_ < 0.0 || max_sim_ball_speed_mps_ <= 0.0 ||
      max_sim_ball_speed_mps_ > 0.35 || max_ball_acceleration_mps2_ <= 0.0 ||
      velocity_transfer_gain_ <= 0.0 ||
      field_min_x_ >= field_max_x_ || field_min_y_ >= field_max_y_ ||
      kickoff_hold_sec_ < 0.0 ||
      ((continuous_demo_enabled_ || single_shot_enabled_) &&
      controlled_robot_namespace_.empty()))
    {
      throw std::invalid_argument("invalid continuous low-speed fake ball parameters");
    }

    sim_ball_x_ = ball_x_;
    sim_ball_y_ = ball_y_;
    last_publish_time_ = now();

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
    control_state_sub_ = create_subscription<std_msgs::msg::String>(
      control_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (msg) {
          control_state_ = msg->data;
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
      "robot_footprint=(%.2f,%.2f) ball_radius=%.2f max_ball_speed=%.2f "
      "goal_w=%.2f continuous_demo=%s single_shot=%s controlled_robot=%s",
      ball_topic_.c_str(), frame_id_.c_str(), ball_x_, ball_y_, kickoff_x_, kickoff_y_,
      robot_front_extent_m_ + robot_rear_extent_m_,
      2.0 * robot_half_width_m_, ball_radius_m_,
      max_sim_ball_speed_mps_, goal_width_m_,
      continuous_demo_enabled_ ? "true" : "false",
      single_shot_enabled_ ? "true" : "false",
      controlled_robot_namespace_.c_str());
  }

private:
  bool lookupRobotInField(const std::string & ns, double & x, double & y) const
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

  void getTeamGoalForStriker(const std::string & striker, double & gx, double & gy) const
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

  bool inKickoffHold(const rclcpp::Time & stamp) const
  {
    return kickoff_hold_until_.nanoseconds() > 0 && stamp < kickoff_hold_until_;
  }

  bool scoredAtGoal(
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

  bool teamAScored() const
  {
    return scoredAtGoal(
      team_a_attack_goal_x_, team_a_attack_goal_y_,
      team_b_attack_goal_x_, team_b_attack_goal_y_);
  }

  bool teamBScored() const
  {
    return scoredAtGoal(
      team_b_attack_goal_x_, team_b_attack_goal_y_,
      team_a_attack_goal_x_, team_a_attack_goal_y_);
  }

  void publishMatchStateCommand(const std::string & state)
  {
    std_msgs::msg::String command;
    command.data = state;
    match_state_command_pub_->publish(command);
    match_state_ = state;
  }

  void resetToKickoff(const rclcpp::Time & stamp, const char * scoring_team)
  {
    sim_ball_x_ = kickoff_x_;
    sim_ball_y_ = kickoff_y_;
    ball_velocity_x_ = 0.0;
    ball_velocity_y_ = 0.0;
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

  void finishSingleShot(const char * scoring_team)
  {
    goal_completed_ = true;
    ball_velocity_x_ = 0.0;
    ball_velocity_y_ = 0.0;

    std_msgs::msg::String event;
    event.data = std::string("goal team_") + scoring_team + " single_shot";
    goal_event_pub_->publish(event);
    publishMatchStateCommand("FINISHED");

    RCLCPP_INFO(
      get_logger(),
      "GOAL team_%s! single-shot ball remains beyond the goal line at (%.2f, %.2f)",
      scoring_team, sim_ball_x_, sim_ball_y_);
  }

  struct ContactCandidate
  {
    std::string robot;
    double strength{0.0};
    double vx{0.0};
    double vy{0.0};
  };

  bool contactCandidate(const std::string & robot, ContactCandidate & output) const
  {
    const std::string normalized_robot = normalizeNamespace(robot);
    if (!controlled_robot_namespace_.empty() &&
      normalized_robot != controlled_robot_namespace_)
    {
      return false;
    }

    const std::string team = inferTeamIdFromNamespace(normalized_robot);
    if (match_state_ != "PLAY" &&
      !((match_state_ == "KICKOFF_A" && team == "a") ||
      (match_state_ == "KICKOFF_B" && team == "b")))
    {
      return false;
    }

    if (normalized_robot == controlled_robot_namespace_ &&
      control_state_ != "CONTACT_ACQUIRE" &&
      control_state_ != "PUSH_BALL" &&
      control_state_ != "DRIVE_THROUGH")
    {
      return false;
    }

    if ((team == "a" && (!have_striker_a_ || striker_a_ != normalized_robot)) ||
      (team == "b" && (!have_striker_b_ || striker_b_ != normalized_robot)))
    {
      return false;
    }

    const auto found = robot_odoms_.find(normalized_robot);
    if (found == robot_odoms_.end()) {
      return false;
    }

    const auto & odom = found->second;
    const rclcpp::Time odom_stamp(
      odom.header.stamp, get_clock()->get_clock_type());
    if (odom_stamp.nanoseconds() <= 0 ||
      (now() - odom_stamp).seconds() > 0.5)
    {
      return false;
    }

    double robot_yaw = 0.0;
    if (!yawFromQuaternion(odom.pose.pose.orientation, robot_yaw)) {
      return false;
    }

    const auto contact = computeBallContactMetrics(
      odom.pose.pose.position.x, odom.pose.pose.position.y, robot_yaw,
      robot_front_extent_m_, robot_rear_extent_m_, robot_half_width_m_,
      sim_ball_x_, sim_ball_y_,
      ball_radius_m_, contact_tolerance_m_);
    if (!contact.front_contact) {
      return false;
    }

    const double forward_speed = odom.twist.twist.linear.x;
    if (!std::isfinite(forward_speed) ||
      forward_speed < minimum_contact_speed_mps_)
    {
      return false;
    }

    const double ball_speed = std::clamp(
      forward_speed * velocity_transfer_gain_, 0.0, max_sim_ball_speed_mps_);
    output.robot = normalized_robot;
    output.strength = forward_speed;
    output.vx = ball_speed * std::cos(robot_yaw);
    output.vy = ball_speed * std::sin(robot_yaw);
    return true;
  }

  std::string selectBallPossessionStriker() const
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

  void publishBallPossessionStriker(const std::string & possessor)
  {
    std_msgs::msg::String msg;
    msg.data = possessor;
    ball_possession_pub_->publish(msg);
  }

  bool tryPushByAnyRobot(const rclcpp::Time & stamp, const double dt)
  {
    if (inKickoffHold(stamp)) {
      publishBallPossessionStriker("");
      return false;
    }

    publishBallPossessionStriker(selectBallPossessionStriker());

    ContactCandidate best;
    bool pushed = false;
    for (const auto & item : robot_odoms_) {
      ContactCandidate candidate;
      if (contactCandidate(item.first, candidate) &&
        (!pushed || candidate.strength > best.strength))
      {
        best = candidate;
        pushed = true;
      }
    }

    if (pushed) {
      current_contact_robot_ = best.robot;

      const double delta_x = best.vx - ball_velocity_x_;
      const double delta_y = best.vy - ball_velocity_y_;
      const double delta_speed = std::hypot(delta_x, delta_y);
      const double max_delta = max_ball_acceleration_mps2_ * std::max(0.0, dt);
      const double scale = delta_speed > max_delta && delta_speed > 1e-9 ?
        max_delta / delta_speed : 1.0;
      ball_velocity_x_ += delta_x * scale;
      ball_velocity_y_ += delta_y * scale;
    }

    return pushed;
  }

  bool controlledRobotPoseFresh(const rclcpp::Time & stamp) const
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

  bool controlledRobotSeen(const rclcpp::Time & stamp) const
  {
    return controlled_robot_pose_seen_ &&
      controlled_robot_transform_valid_ &&
      controlledRobotPoseFresh(stamp);
  }

  void updateControlledContactDiagnostics(const rclcpp::Time & stamp)
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

  void publishDiagnostics(const rclcpp::Time & stamp)
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
    velocity.twist.linear.x = ball_velocity_x_;
    velocity.twist.linear.y = ball_velocity_y_;
    ball_velocity_pub_->publish(velocity);
  }

  bool outOfBounds() const
  {
    return sim_ball_x_ < field_min_x_ ||
      sim_ball_x_ > field_max_x_ ||
      sim_ball_y_ < field_min_y_ ||
      sim_ball_y_ > field_max_y_;
  }

  void performPendingOpponentKickoff(const rclcpp::Time & stamp)
  {
    if (!pending_opponent_kickoff_ || inKickoffHold(stamp)) {
      return;
    }

    // The fake ball may not acquire momentum without a real robot-footprint
    // contact. Resume play, but leave the ball stationary until a robot
    // reaches it through the normal navigation and contact chain.
    pending_opponent_kickoff_ = false;
    ball_velocity_x_ = 0.0;
    ball_velocity_y_ = 0.0;
    publishMatchStateCommand("PLAY");
  }

  void signalOutOfBounds()
  {
    goal_completed_ = true;
    ball_velocity_x_ = ball_velocity_y_ = 0.0;
    std_msgs::msg::String event;
    event.data = "OUT_OF_BOUNDS";
    goal_event_pub_->publish(event);
    publishMatchStateCommand("FINISHED");
  }

  void publishBallPose()
  {
    const auto stamp = now();
    double dt = (stamp - last_publish_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.1;
    }
    last_publish_time_ = stamp;

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
    const bool pushed = tryPushByAnyRobot(stamp, dt);
    sim_ball_x_ += ball_velocity_x_ * dt;
    sim_ball_y_ += ball_velocity_y_ * dt;
    const double speed = std::hypot(ball_velocity_x_, ball_velocity_y_);
    if (!pushed && speed > 1e-6) {
      const double new_speed = std::max(0.0, speed - ball_friction_mps2_ * dt);
      const double scale = new_speed / speed;
      ball_velocity_x_ *= scale;
      ball_velocity_y_ *= scale;
    }

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

  std::string frame_id_;
  std::string ball_topic_;
  std::string goal_event_topic_;
  std::string ball_possession_striker_topic_;
  double ball_contest_radius_m_{3.0};
  double publish_rate_hz_{10.0};
  double ball_x_{2.0};
  double ball_y_{0.0};
  double ball_z_{0.0};
  double kickoff_x_{0.0};
  double kickoff_y_{0.0};
  double robot_front_extent_m_{0.25};
  double robot_rear_extent_m_{0.23};
  double robot_half_width_m_{0.13};
  double ball_radius_m_{0.11};
  double contact_tolerance_m_{0.03};
  double minimum_contact_speed_mps_{0.02};
  double ball_friction_mps2_{0.8};
  double max_sim_ball_speed_mps_{0.25};
  double max_ball_acceleration_mps2_{0.45};
  double velocity_transfer_gain_{0.85};
  double goal_width_m_{1.2};
  double field_min_x_{-2.0};
  double field_max_x_{8.0};
  double field_min_y_{-3.0};
  double field_max_y_{3.0};
  double kickoff_hold_sec_{3.0};
  double team_a_attack_goal_x_{8.0};
  double team_a_attack_goal_y_{0.0};
  double team_b_attack_goal_x_{-2.0};
  double team_b_attack_goal_y_{0.0};
  std::string team_a_striker_topic_;
  std::string team_b_striker_topic_;
  std::string odom_topic_template_;
  bool continuous_demo_enabled_{false};
  bool single_shot_enabled_{false};
  std::string controlled_robot_namespace_;
  std::string match_state_topic_;
  std::string control_state_topic_;
  std::string front_contact_topic_;
  std::string side_contact_topic_;
  std::string rear_contact_topic_;
  std::string contact_robot_topic_;
  std::string controlled_robot_pose_seen_topic_;
  std::string controlled_robot_pose_fresh_topic_;
  std::string controlled_robot_transform_valid_topic_;
  std::string controlled_robot_seen_topic_;
  std::string ball_velocity_topic_;
  std::string match_state_{"STOP"};
  std::string control_state_{"SAFE_STOP"};

  double sim_ball_x_{2.0};
  double sim_ball_y_{0.0};
  double ball_velocity_x_{0.0};
  double ball_velocity_y_{0.0};
  rclcpp::Time last_publish_time_;
  rclcpp::Time kickoff_hold_until_;
  bool pending_opponent_kickoff_{false};
  bool goal_completed_{false};
  bool controlled_robot_pose_seen_{false};
  bool controlled_robot_pose_fresh_{false};
  bool controlled_robot_transform_valid_{false};
  bool current_front_contact_{false};
  bool current_side_contact_{false};
  bool current_rear_contact_{false};
  std::string current_contact_robot_;

  bool have_striker_a_{false};
  bool have_striker_b_{false};
  std::string striker_a_;
  std::string striker_b_;

  std::map<std::string, nav_msgs::msg::Odometry> robot_odoms_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr match_state_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ball_possession_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr side_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_robot_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_pose_seen_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_pose_fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_transform_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr controlled_robot_seen_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ball_velocity_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_a_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_b_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_state_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<football_navigation::FootballFakeBallPublisher>());
  rclcpp::shutdown();
  return 0;
}
