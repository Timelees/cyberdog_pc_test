// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#ifndef FOOTBALL_NAVIGATION__FOOTBALL_GEOMETRY_HPP_
#define FOOTBALL_NAVIGATION__FOOTBALL_GEOMETRY_HPP_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"

namespace football_navigation
{

struct RobotPose2D
{
  std::string id;
  double x{0.0};
  double y{0.0};
  bool valid{false};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double stamp_sec{0.0};
  double data_age{0.0};
  bool path_blocked{false};
  bool ball_visible{true};
  double blocked_time{0.0};
  bool current_role{false};
};

std::string normalizeRobotNamespace(const std::string & ns);

std::string inferTeamIdFromNamespace(const std::string & ns);

double planarDistance(double ax, double ay, double bx, double by);

std::pair<double, double> worldVelocityToBody(
  double world_vx, double world_vy, double yaw);

size_t indexOfClosestRobot(
  const std::vector<RobotPose2D> & robots,
  double ball_x, double ball_y);

int numericRobotId(const std::string & id);

double attackerScore(
  const RobotPose2D & robot, double ball_x, double ball_y,
  double nominal_speed_mps, double max_data_age_sec);

size_t indexOfBestAttacker(
  const std::vector<RobotPose2D> & robots,
  double ball_x, double ball_y,
  double nominal_speed_mps, double max_data_age_sec);

std::string selectStrikerWithHysteresis(
  const std::vector<RobotPose2D> & robots,
  double ball_x, double ball_y,
  const std::string & current_striker,
  double hysteresis_m);

struct ApproachConfig
{
  double approach_distance{0.8};
  double default_kick_yaw{0.0};
  double min_direction_norm{1e-4};
};

struct LateralEntryConfig
{
  double entry_lateral_m{0.7};
  double retreat_clearance_m{0.35};
  double ball_protection_radius_m{0.28};
  double opponent_endpoint_clearance_m{0.0};
  double field_min_x{-2.0};
  double field_max_x{8.0};
  double field_min_y{-3.0};
  double field_max_y{3.0};
  double boundary_margin_m{0.25};
  double min_direction_norm{1e-4};
  int preferred_side{0};
};

int deterministicYieldSide(
  const std::string & team_id, const std::string & robot_id, double relative_y);

struct NoProgressMetrics
{
  double elapsed_sec{0.0};
  double ball_distance_improvement{0.0};
  double goal_distance_improvement{0.0};
  double path_progress{0.0};
  double actual_speed{0.0};
  double command_speed{0.0};
  double obstacle_distance{0.0};
  double ball_motion{0.0};
  double ball_distance{0.0};
};

struct NoProgressConfig
{
  double window_sec{1.5};
  double min_distance_improvement{0.05};
  double min_path_progress{0.08};
  double max_actual_speed{0.03};
  double obstacle_near_m{0.75};
  double max_ball_motion{0.05};
  double ball_contact_m{0.25};
};

bool isNoProgress(
  const NoProgressMetrics & metrics, const NoProgressConfig & config);

bool approachTargetPassed(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & target_pose,
  double margin_m,
  double lateral_tolerance_m = 0.25);

// 几何辅助函数使用的最小二维障碍物表示。
// pair 的含义是 field_frame（通常为 tag_global）下的 (x, y)。虽然类型名中
// 使用了 Opponent，但该列表实际上既可能包含对手，也可能包含队友。
using OpponentPoint2D = std::pair<double, double>;

struct BallContactMetrics
{
  bool intersects{false};
  bool front_contact{false};
  bool side_contact{false};
  bool rear_contact{false};
  double forward_offset_m{0.0};
  double lateral_offset_m{0.0};
  double separation_m{0.0};
};

BallContactMetrics computeBallContactMetrics(
  double robot_x, double robot_y, double robot_yaw,
  double robot_front_extent_m, double robot_rear_extent_m,
  double robot_half_width_m,
  double ball_x, double ball_y, double ball_radius_m,
  double contact_tolerance_m = 0.0);

double distancePointToSegment(
  double px, double py,
  double ax, double ay, double bx, double by);

bool isOpponentBlockingPathToPoint(
  double robot_x, double robot_y,
  double target_x, double target_y,
  const std::vector<OpponentPoint2D> & opponents,
  double block_radius_m);

bool isRobotBehindAndAlignedForKick(
  double robot_x, double robot_y,
  double ball_x, double ball_y,
  double target_x, double target_y,
  double minimum_behind_m,
  double maximum_lateral_m);

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

bool yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & input,
  double & yaw);

geometry_msgs::msg::Transform makeSimulatedRawTagObservation(
  double tag_global_x, double tag_global_y, double tag_global_yaw);

geometry_msgs::msg::Transform recoverTagGlobalBasePose(
  const geometry_msgs::msg::Transform & raw_observation);

geometry_msgs::msg::PoseStamped computeApproachPose(
  const geometry_msgs::msg::PoseStamped & ball_pose,
  const geometry_msgs::msg::PoseStamped & kick_target_pose,
  const ApproachConfig & config);

geometry_msgs::msg::PoseStamped computeApproachPoseFromRobot(
  const geometry_msgs::msg::PoseStamped & ball_pose,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const ApproachConfig & config);

struct ApproachPlan
{
  geometry_msgs::msg::PoseStamped pose;
  bool feasible{false};
  bool used_detour{false};
};

ApproachPlan computeApproachPoseAvoidingBlocker(
  const geometry_msgs::msg::PoseStamped & ball_pose,
  const geometry_msgs::msg::PoseStamped & kick_target_pose,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<OpponentPoint2D> & opponents,
  const ApproachConfig & approach_config,
  const LateralEntryConfig & lateral_config,
  double block_radius_m);

double normalizeAngle(double angle);

double signedYawError(double target_yaw, double current_yaw);

double yawDistance(double a, double b);

bool isPoseNear(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs,
  double xy_tolerance,
  double yaw_tolerance);

}  // namespace football_navigation

#endif  // FOOTBALL_NAVIGATION__FOOTBALL_GEOMETRY_HPP_
