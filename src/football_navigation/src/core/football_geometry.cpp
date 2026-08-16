// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#include "football_navigation/core/football_geometry.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"

namespace football_navigation
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}


geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion msg;
  msg.x = quat.x();
  msg.y = quat.y();
  msg.z = quat.z();
  msg.w = quat.w();
  return msg;
}

bool yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & input,
  double & yaw)
{
  const double norm_sq = input.x * input.x + input.y * input.y +
    input.z * input.z + input.w * input.w;
  if (!std::isfinite(norm_sq) || norm_sq <= 1e-12) {
    return false;
  }
  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  const double x = input.x * inv_norm;
  const double y = input.y * inv_norm;
  const double z = input.z * inv_norm;
  const double w = input.w * inv_norm;
  yaw = std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
  return std::isfinite(yaw);
}

geometry_msgs::msg::Transform makeSimulatedRawTagObservation(
  const double tag_global_x, const double tag_global_y, const double tag_global_yaw)
{
  // 使用真实检测器的墙面 Tag/光学坐标轴约定，将期望的 tag_global -> base
  // 位姿编码成原始观测。该模拟观测经 recoverTagGlobalBasePose() 恢复后应
  // 得到原始全局位姿，使假世界测试不会绕过实际使用的定位转换链路。
  tf2::Transform output_to_base;
  output_to_base.setIdentity();
  output_to_base.setOrigin(tf2::Vector3(tag_global_x, tag_global_y, 0.0));
  tf2::Quaternion base_rotation;
  base_rotation.setRPY(0.0, 0.0, tag_global_yaw);
  output_to_base.setRotation(base_rotation);

  tf2::Matrix3x3 wall_alignment(
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0);
  tf2::Transform tag_to_global;
  tag_to_global.setIdentity();
  tag_to_global.setBasis(wall_alignment);
  tf2::Quaternion rotate_x_pi;
  rotate_x_pi.setRPY(kPi, 0.0, 0.0);
  tf2::Transform x_rotation;
  x_rotation.setIdentity();
  x_rotation.setRotation(rotate_x_pi);
  const tf2::Transform base_to_raw = output_to_base.inverse() *
    (x_rotation * tag_to_global.inverse());

  geometry_msgs::msg::Transform output;
  output.translation.x = base_to_raw.getOrigin().x();
  output.translation.y = base_to_raw.getOrigin().y();
  output.translation.z = base_to_raw.getOrigin().z();
  output.rotation.x = base_to_raw.getRotation().x();
  output.rotation.y = base_to_raw.getRotation().y();
  output.rotation.z = base_to_raw.getRotation().z();
  output.rotation.w = base_to_raw.getRotation().w();
  return output;
}

geometry_msgs::msg::Transform recoverTagGlobalBasePose(
  const geometry_msgs::msg::Transform & raw_observation)
{
  // 将检测器输出的原始相对观测转换为 tag_global -> base 位姿。固定旋转用于
  // 统一光学坐标轴、墙面 Tag 坐标轴和 ROS 机体坐标轴；由于原始观测与目标
  // 位姿的变换方向相反，因此组合变换时需要求逆。
  tf2::Transform raw_to_tag;
  raw_to_tag.setOrigin(tf2::Vector3(
    raw_observation.translation.x, raw_observation.translation.y, raw_observation.translation.z));
  raw_to_tag.setRotation(tf2::Quaternion(
    raw_observation.rotation.x, raw_observation.rotation.y,
    raw_observation.rotation.z, raw_observation.rotation.w));
  tf2::Matrix3x3 wall_alignment(
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0);
  tf2::Transform tag_to_global;
  tag_to_global.setIdentity();
  tag_to_global.setBasis(wall_alignment);
  tf2::Quaternion rotate_x_pi;
  rotate_x_pi.setRPY(kPi, 0.0, 0.0);
  tf2::Transform x_rotation;
  x_rotation.setIdentity();
  x_rotation.setRotation(rotate_x_pi);
  const tf2::Transform output_to_base =
    (x_rotation * tag_to_global.inverse()) * raw_to_tag.inverse();

  geometry_msgs::msg::Transform output;
  output.translation.x = output_to_base.getOrigin().x();
  output.translation.y = output_to_base.getOrigin().y();
  output.translation.z = output_to_base.getOrigin().z();
  output.rotation.x = output_to_base.getRotation().x();
  output.rotation.y = output_to_base.getRotation().y();
  output.rotation.z = output_to_base.getRotation().z();
  output.rotation.w = output_to_base.getRotation().w();
  return output;
}

geometry_msgs::msg::PoseStamped computeApproachPose(
  const geometry_msgs::msg::PoseStamped & ball_pose,
  const geometry_msgs::msg::PoseStamped & kick_target_pose,
  const ApproachConfig & config)
{
  // u 指向“球 -> 射门目标”。接近点位于球沿 -u 方向 approach_distance 处，
  // 朝向沿 +u，从而形成“机器人 -> 球 -> 射门目标”的射门排列。
  const double dx = kick_target_pose.pose.position.x - ball_pose.pose.position.x;
  const double dy = kick_target_pose.pose.position.y - ball_pose.pose.position.y;
  const double norm = std::hypot(dx, dy);

  double ux = std::cos(config.default_kick_yaw);
  double uy = std::sin(config.default_kick_yaw);
  if (norm > config.min_direction_norm) {
    ux = dx / norm;
    uy = dy / norm;
  }

  geometry_msgs::msg::PoseStamped approach_pose;
  approach_pose.header = ball_pose.header;
  approach_pose.pose.position.x = ball_pose.pose.position.x - ux * config.approach_distance;
  approach_pose.pose.position.y = ball_pose.pose.position.y - uy * config.approach_distance;
  approach_pose.pose.position.z = ball_pose.pose.position.z;
  approach_pose.pose.orientation = quaternionFromYaw(std::atan2(uy, ux));
  return approach_pose;
}

BallContactMetrics computeBallContactMetrics(
  const double robot_x,
  const double robot_y,
  const double robot_yaw,
  const double robot_front_extent_m,
  const double robot_rear_extent_m,
  const double robot_half_width_m,
  const double ball_x,
  const double ball_y,
  const double ball_radius_m,
  const double contact_tolerance_m)
{
  BallContactMetrics metrics;

  if (!std::isfinite(robot_x) ||
    !std::isfinite(robot_y) ||
    !std::isfinite(robot_yaw) ||
    !std::isfinite(robot_front_extent_m) ||
    !std::isfinite(robot_rear_extent_m) ||
    !std::isfinite(robot_half_width_m) ||
    !std::isfinite(ball_x) ||
    !std::isfinite(ball_y) ||
    !std::isfinite(ball_radius_m) ||
    !std::isfinite(contact_tolerance_m) ||
    robot_front_extent_m <= 0.0 ||
    robot_rear_extent_m <= 0.0 ||
    robot_half_width_m <= 0.0 ||
    ball_radius_m <= 0.0 ||
    contact_tolerance_m < 0.0)
  {
    return metrics;
  }

  const double dx = ball_x - robot_x;
  const double dy = ball_y - robot_y;
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);

  metrics.forward_offset_m = c * dx + s * dy;
  metrics.lateral_offset_m = -s * dx + c * dy;

  // CyberDog base_link is not assumed to be the exact geometric centre.
  // The footprint therefore uses independent forward and rear extents.
  const double closest_forward = std::clamp(
    metrics.forward_offset_m,
    -robot_rear_extent_m,
    robot_front_extent_m);
  const double closest_lateral = std::clamp(
    metrics.lateral_offset_m,
    -robot_half_width_m,
    robot_half_width_m);

  const double distance_to_rectangle = std::hypot(
    metrics.forward_offset_m - closest_forward,
    metrics.lateral_offset_m - closest_lateral);

  metrics.separation_m = distance_to_rectangle - ball_radius_m;
  metrics.intersects = metrics.separation_m <= contact_tolerance_m;

  const double lateral_excess = std::max(
    0.0,
    std::fabs(metrics.lateral_offset_m) - robot_half_width_m);
  const double distance_to_front_face = std::hypot(
    metrics.forward_offset_m - robot_front_extent_m,
    lateral_excess);
  const bool ball_is_in_front =
    metrics.forward_offset_m >= robot_front_extent_m - contact_tolerance_m;
  const bool laterally_reaches_front_face =
      std::fabs(metrics.lateral_offset_m) <=
    robot_half_width_m + ball_radius_m + contact_tolerance_m;

  metrics.front_contact =
    metrics.intersects &&
    ball_is_in_front &&
    laterally_reaches_front_face &&
    distance_to_front_face <= ball_radius_m + contact_tolerance_m;

  const double distance_to_rear_face = std::hypot(
    metrics.forward_offset_m + robot_rear_extent_m,
    lateral_excess);
  const bool ball_is_behind =
    metrics.forward_offset_m <= -robot_rear_extent_m + contact_tolerance_m;
  const bool laterally_reaches_rear_face =
    std::fabs(metrics.lateral_offset_m) <=
    robot_half_width_m + ball_radius_m + contact_tolerance_m;

  metrics.rear_contact =
    metrics.intersects &&
    !metrics.front_contact &&
    ball_is_behind &&
    laterally_reaches_rear_face &&
    distance_to_rear_face <= ball_radius_m + contact_tolerance_m;

  metrics.side_contact =
    metrics.intersects &&
    !metrics.front_contact &&
    !metrics.rear_contact;

  return metrics;
}

double distancePointToSegment(
  const double px, const double py,
  const double ax, const double ay, const double bx, const double by)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double len_sq = abx * abx + aby * aby;
  if (len_sq < 1e-12) {
    return std::hypot(px - ax, py - ay);
  }
  double t = ((px - ax) * abx + (py - ay) * aby) / len_sq;
  t = std::max(0.0, std::min(1.0, t));
  const double proj_x = ax + t * abx;
  const double proj_y = ay + t * aby;
  return std::hypot(px - proj_x, py - proj_y);
}

bool isOpponentBlockingPathToPoint(
  const double robot_x, const double robot_y,
  const double target_x, const double target_y,
  const std::vector<OpponentPoint2D> & opponents,
  const double block_radius_m)
{
  for (const auto & opponent : opponents) {
    const double dist = distancePointToSegment(
      opponent.first, opponent.second,
      robot_x, robot_y, target_x, target_y);
    if (dist < block_radius_m) {
      return true;
    }
  }
  return false;
}

bool isRobotBehindAndAlignedForKick(
  const double robot_x, const double robot_y,
  const double ball_x, const double ball_y,
  const double target_x, const double target_y,
  const double minimum_behind_m,
  const double maximum_lateral_m)
{
  const double dx = target_x - ball_x;
  const double dy = target_y - ball_y;
  const double norm = std::hypot(dx, dy);
  if (!std::isfinite(norm) || norm <= 1e-9) {
    return false;
  }
  const double ux = dx / norm;
  const double uy = dy / norm;
  const double rx = robot_x - ball_x;
  const double ry = robot_y - ball_y;
  return rx * ux + ry * uy <= -minimum_behind_m &&
         std::fabs(rx * (-uy) + ry * ux) <= maximum_lateral_m;
}

ApproachPlan computeApproachPoseAvoidingBlocker(
  const geometry_msgs::msg::PoseStamped & ball_pose,
  const geometry_msgs::msg::PoseStamped & kick_target_pose,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<OpponentPoint2D> & opponents,
  const ApproachConfig & approach_config,
  const LateralEntryConfig & lateral_config,
  const double block_radius_m)
{
  // 此处返回的绕行点只是临时 staging/入口点。机器人到达后，后续更新仍应
  // 继续前往标准球后点，再完成朝向对准和触球；绕行点不是新的射门目标。
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double ball_x = ball_pose.pose.position.x;
  const double ball_y = ball_pose.pose.position.y;
  auto behind = computeApproachPose(ball_pose, kick_target_pose, approach_config);
  behind.pose.position.x = std::clamp(
    behind.pose.position.x,
    lateral_config.field_min_x + lateral_config.boundary_margin_m,
    lateral_config.field_max_x - lateral_config.boundary_margin_m);
  behind.pose.position.y = std::clamp(
    behind.pose.position.y,
    lateral_config.field_min_y + lateral_config.boundary_margin_m,
    lateral_config.field_max_y - lateral_config.boundary_margin_m);
  const double behind_x = behind.pose.position.x;
  const double behind_y = behind.pose.position.y;
  const auto endpoint_clear =
    [&](const double x, const double y)
    {
      return std::all_of(
        opponents.begin(), opponents.end(),
        [&](const OpponentPoint2D & opponent)
        {
          return planarDistance(
            x, y, opponent.first, opponent.second) >=
                 lateral_config.opponent_endpoint_clearance_m;
        });
    };

  // Moving robots on the transit segment belong to the NavFn/DWB costmaps.
  // Changing the football target as the same obstacle moves makes the target
  // generator and planner fight each other, and can leave a reached detour as
  // a permanent one-pose goal. Only target occupancy and the ball corridor
  // decide whether a staging target is needed here.
  if (endpoint_clear(behind_x, behind_y) &&
    distancePointToSegment(
      ball_x, ball_y, robot_x, robot_y, behind_x, behind_y) >=
    lateral_config.ball_protection_radius_m)
  {
    return ApproachPlan{behind, true, false};
  }

  const double dx = kick_target_pose.pose.position.x - ball_x;
  const double dy = kick_target_pose.pose.position.y - ball_y;
  const double norm = std::hypot(dx, dy);
  double ux = 1.0;
  double uy = 0.0;
  if (norm > lateral_config.min_direction_norm) {
    ux = dx / norm;
    uy = dy / norm;
  }

  struct Candidate {double x; double y; int side;};
  // 尝试射门线两侧的垂直偏移点，以及沿射门反方向进一步后退的候选点。
  const std::vector<Candidate> candidates = {
    {behind_x - uy * lateral_config.entry_lateral_m,
      behind_y + ux * lateral_config.entry_lateral_m, 1},
    {behind_x + uy * lateral_config.entry_lateral_m,
      behind_y - ux * lateral_config.entry_lateral_m, -1},
    {behind_x - ux * lateral_config.retreat_clearance_m,
      behind_y - uy * lateral_config.retreat_clearance_m, 0},
  };
  geometry_msgs::msg::PoseStamped entry_pose = behind;
  bool found = false;
  std::pair<bool, double> best_rank{
    true, std::numeric_limits<double>::infinity()};
  for (const auto & candidate : candidates) {
    const bool in_field =
      candidate.x >= lateral_config.field_min_x + lateral_config.boundary_margin_m &&
      candidate.x <= lateral_config.field_max_x - lateral_config.boundary_margin_m &&
      candidate.y >= lateral_config.field_min_y + lateral_config.boundary_margin_m &&
      candidate.y <= lateral_config.field_max_y - lateral_config.boundary_margin_m;
    const bool protects_ball = distancePointToSegment(
      ball_x, ball_y, robot_x, robot_y, candidate.x, candidate.y) >=
      lateral_config.ball_protection_radius_m;
    const bool path_clear = !isOpponentBlockingPathToPoint(
      robot_x, robot_y, candidate.x, candidate.y, opponents, block_radius_m);
    // A staging point only needs to be reachable now. Requiring the next leg
    // to an occupied behind-ball pose rejects the very detour needed while
    // that pose is blocked; the next update will advance after it clears.
    if (!in_field || !protects_ball || !path_clear ||
      !endpoint_clear(candidate.x, candidate.y))
    {
      continue;
    }

    const double boundary_clearance = std::min({
      candidate.x - lateral_config.field_min_x,
      lateral_config.field_max_x - candidate.x,
      candidate.y - lateral_config.field_min_y,
      lateral_config.field_max_y - candidate.y});
    const double cost = std::hypot(candidate.x - robot_x, candidate.y - robot_y) +
      std::hypot(behind_x - candidate.x, behind_y - candidate.y) +
      2.0 / std::max(0.05, boundary_clearance);
    const std::pair<bool, double> rank{
      lateral_config.preferred_side != 0 &&
      candidate.side != lateral_config.preferred_side,
      cost};
    if (!found || rank < best_rank) {
      best_rank = rank;
      entry_pose.pose.position.x = candidate.x;
      entry_pose.pose.position.y = candidate.y;
      found = true;
    }
  }

  entry_pose.pose.orientation = quaternionFromYaw(std::atan2(uy, ux));
  return ApproachPlan{entry_pose, found, found};
}

int deterministicYieldSide(
  const std::string & team_id, const std::string & robot_id, const double relative_y)
{
  const int team_bit = team_id == "b" ? 1 : 0;
  const int robot = numericRobotId(robot_id);
  const int robot_bit = robot == std::numeric_limits<int>::max() ? 0 : robot & 1;
  const int relative_bit = relative_y < 0.0 ? 1 : 0;
  return ((team_bit + robot_bit + relative_bit) & 1) == 0 ? 1 : -1;
}

bool isNoProgress(const NoProgressMetrics & metrics, const NoProgressConfig & config)
{
  const bool constrained = metrics.obstacle_distance <= config.obstacle_near_m ||
    metrics.ball_distance <= config.ball_contact_m;
  return metrics.elapsed_sec >= config.window_sec &&
         metrics.ball_distance_improvement < config.min_distance_improvement &&
         metrics.goal_distance_improvement < config.min_distance_improvement &&
         metrics.path_progress < config.min_path_progress &&
         metrics.actual_speed <= config.max_actual_speed &&
         metrics.ball_motion <= config.max_ball_motion && constrained;
}

bool approachTargetPassed(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const double margin_m,
  const double lateral_tolerance_m)
{
  if (!std::isfinite(margin_m) || margin_m < 0.0 ||
    !std::isfinite(lateral_tolerance_m) || lateral_tolerance_m <= 0.0 ||
    robot_pose.header.frame_id.empty() ||
    robot_pose.header.frame_id != target_pose.header.frame_id)
  {
    return false;
  }

  double target_yaw = 0.0;
  if (!yawFromQuaternion(target_pose.pose.orientation, target_yaw)) {
    return false;
  }
  const double forward_delta =
    (robot_pose.pose.position.x - target_pose.pose.position.x) * std::cos(target_yaw) +
    (robot_pose.pose.position.y - target_pose.pose.position.y) * std::sin(target_yaw);
  const double lateral_delta = std::fabs(
    -(robot_pose.pose.position.x - target_pose.pose.position.x) * std::sin(target_yaw) +
    (robot_pose.pose.position.y - target_pose.pose.position.y) * std::cos(target_yaw));
  return std::isfinite(forward_delta) && std::isfinite(lateral_delta) &&
         forward_delta > margin_m && lateral_delta <= lateral_tolerance_m;
}

geometry_msgs::msg::PoseStamped computeApproachPoseFromRobot(
  const geometry_msgs::msg::PoseStamped & ball_pose,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const ApproachConfig & config)
{
  // 根据“机器人 -> 球”而非“球 -> 射门目标”计算接近方向。该点适合在射门
  // 方向不可靠时作为自然的第一阶段接近点；当前前锋主流程并不以它为主目标。
  const double dx = ball_pose.pose.position.x - robot_pose.pose.position.x;
  const double dy = ball_pose.pose.position.y - robot_pose.pose.position.y;
  const double norm = std::hypot(dx, dy);

  double ux = std::cos(config.default_kick_yaw);
  double uy = std::sin(config.default_kick_yaw);
  if (norm > config.min_direction_norm) {
    ux = dx / norm;
        uy = dy / norm;
  }

  geometry_msgs::msg::PoseStamped approach_pose;
  approach_pose.header = ball_pose.header;
  approach_pose.pose.position.x = ball_pose.pose.position.x - ux * config.approach_distance;
  approach_pose.pose.position.y = ball_pose.pose.position.y - uy * config.approach_distance;
  approach_pose.pose.position.z = ball_pose.pose.position.z;
  approach_pose.pose.orientation = quaternionFromYaw(std::atan2(uy, ux));
  return approach_pose;
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double signedYawError(const double target_yaw, const double current_yaw)
{
  return normalizeAngle(target_yaw - current_yaw);
}

double yawDistance(const double a, const double b)
{
  return std::fabs(signedYawError(a, b));
}

bool isPoseNear(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs,
  double xy_tolerance,
  double yaw_tolerance)
{
  const double dx = lhs.pose.position.x - rhs.pose.position.x;
  const double dy = lhs.pose.position.y - rhs.pose.position.y;
  if (std::hypot(dx, dy) > xy_tolerance) {
    return false;
  }

  double lhs_yaw = 0.0;
  double rhs_yaw = 0.0;
  if (!yawFromQuaternion(lhs.pose.orientation, lhs_yaw) ||
    !yawFromQuaternion(rhs.pose.orientation, rhs_yaw))
  {
    return false;
  }
  return yawDistance(lhs_yaw, rhs_yaw) <= yaw_tolerance;
}

std::string normalizeRobotNamespace(const std::string & ns)
{
  const auto first = ns.find_first_not_of('/');
  if (first == std::string::npos) {
    return "";
  }
  const auto last = ns.find_last_not_of('/');
  return ns.substr(first, last - first + 1);
}

std::string inferTeamIdFromNamespace(const std::string & ns)
{
  const std::string normalized = normalizeRobotNamespace(ns);
  const std::string prefix = "cyberdog_";
  if (normalized.rfind(prefix, 0) != 0) {
    return "";
  }

  const std::string suffix = normalized.substr(prefix.size());
  if (suffix.empty() ||
    !std::all_of(suffix.begin(), suffix.end(), [](const unsigned char value) {
      return std::isdigit(value) != 0;
    }))
  {
    return "";
  }

  try {
    std::size_t parsed = 0;
    const int robot_index = std::stoi(suffix, &parsed);
    if (parsed != suffix.size() ||
      normalized != prefix + std::to_string(robot_index) ||
      robot_index < 1 || robot_index > 10)
    {
      return "";
    }
    return robot_index <= 5 ? "a" : "b";
  } catch (const std::exception &) {
    return "";
  }
}

double planarDistance(const double ax, const double ay, const double bx, const double by)
{
  return std::hypot(ax - bx, ay - by);
}

CollisionEllipse makeCircumscribedCollisionEllipse(
  const double robot_length_m, const double robot_width_m,
  const double expansion_m)
{
  constexpr double inverse_sqrt_two = 0.70710678118654752440;
  return {
    robot_length_m * inverse_sqrt_two + expansion_m,
    robot_width_m * inverse_sqrt_two + expansion_m};
}

double ellipseSupportRadius(
  const CollisionEllipse & ellipse, const double ellipse_yaw,
  const double direction_x, const double direction_y)
{
  const double direction_norm = std::hypot(direction_x, direction_y);
  if (direction_norm <= 1e-12) {
    return std::max(ellipse.semi_major_m, ellipse.semi_minor_m);
  }
  const double unit_x = direction_x / direction_norm;
  const double unit_y = direction_y / direction_norm;
  const double cosine = std::cos(ellipse_yaw);
  const double sine = std::sin(ellipse_yaw);
  const double local_x = cosine * unit_x + sine * unit_y;
  const double local_y = -sine * unit_x + cosine * unit_y;
  return std::hypot(
    ellipse.semi_major_m * local_x,
    ellipse.semi_minor_m * local_y);
}

double orientedEllipseClearance(
  const double first_x, const double first_y, const double first_yaw,
  const CollisionEllipse & first,
  const double second_x, const double second_y, const double second_yaw,
  const CollisionEllipse & second)
{
  const double direction_x = second_x - first_x;
  const double direction_y = second_y - first_y;
  const double center_distance = std::hypot(direction_x, direction_y);
  const double occupied_distance =
    ellipseSupportRadius(first, first_yaw, direction_x, direction_y) +
    ellipseSupportRadius(second, second_yaw, -direction_x, -direction_y);
  return center_distance - occupied_distance;
}

std::pair<double, double> worldVelocityToBody(
  const double world_vx, const double world_vy, const double yaw)
{
  return {
    std::cos(yaw) * world_vx + std::sin(yaw) * world_vy,
    -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy};
}

int numericRobotId(const std::string & id)
{
  const std::string normalized = normalizeRobotNamespace(id);
  const auto separator = normalized.find_last_of('_');
  if (separator == std::string::npos || separator + 1 >= normalized.size()) {
    return std::numeric_limits<int>::max();
  }
  try {
    return std::stoi(normalized.substr(separator + 1));
  } catch (const std::exception &) {
    return std::numeric_limits<int>::max();
  }
}

double attackerScore(
  const RobotPose2D & robot, const double ball_x, const double ball_y,
  const double nominal_speed_mps, const double max_data_age_sec)
{
  if (!robot.valid || !robot.ball_visible || !std::isfinite(robot.x) ||
    !std::isfinite(robot.y) || !std::isfinite(robot.yaw) ||
    robot.data_age < 0.0 || robot.data_age > max_data_age_sec)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double distance = planarDistance(robot.x, robot.y, ball_x, ball_y);
  if (distance < 1e-6) {
    return 0.0;
  }
  const double ux = (ball_x - robot.x) / distance;
  const double uy = (ball_y - robot.y) / distance;
  const double velocity_toward_ball = robot.vx * ux + robot.vy * uy;
  const double heading = std::atan2(ball_y - robot.y, ball_x - robot.x);
  const double heading_error = yawDistance(robot.yaw, heading);
  const double effective_speed = std::clamp(
    0.12 + velocity_toward_ball, 0.08, std::max(0.08, nominal_speed_mps));
  const double eta = distance / effective_speed;
  return eta + 0.30 * heading_error + 2.0 * robot.data_age +
         (robot.path_blocked ? 1.5 : 0.0);
}

size_t indexOfBestAttacker(
  const std::vector<RobotPose2D> & robots,
  const double ball_x, const double ball_y,
  const double nominal_speed_mps, const double max_data_age_sec)
{
  size_t best_index = robots.size();
  double best_score = std::numeric_limits<double>::infinity();
  int best_robot_id = std::numeric_limits<int>::max();
  for (size_t i = 0; i < robots.size(); ++i) {
    const double score = attackerScore(
      robots[i], ball_x, ball_y, nominal_speed_mps, max_data_age_sec);
    const int robot_id = numericRobotId(robots[i].id);
    if (score + 1e-6 < best_score ||
      (std::fabs(score - best_score) <= 1e-6 && robot_id < best_robot_id))
    {
      best_score = score;
      best_robot_id = robot_id;
      best_index = i;
    }
  }
  return best_index;
}

size_t indexOfClosestRobot(
  const std::vector<RobotPose2D> & robots,
  const double ball_x, const double ball_y)
{
  size_t best_index = robots.size();
  double best_distance = std::numeric_limits<double>::infinity();
  std::string best_id;

  for (size_t i = 0; i < robots.size(); ++i) {
    if (!robots[i].valid) {
      continue;
    }

    const double distance = planarDistance(robots[i].x, robots[i].y, ball_x, ball_y);
    const bool closer = distance + 1e-6 < best_distance;
    const bool tie_break = std::fabs(distance - best_distance) <= 1e-6 &&
      numericRobotId(robots[i].id) < numericRobotId(best_id);
    if (closer || tie_break) {
      best_distance = distance;
      best_index = i;
      best_id = robots[i].id;
    }
  }

  return best_index;
}

std::string selectStrikerWithHysteresis(
  const std::vector<RobotPose2D> & robots,
  const double ball_x, const double ball_y,
  const std::string & current_striker,
  const double hysteresis_m)
{
  const auto closest_index = indexOfClosestRobot(robots, ball_x, ball_y);
  if (closest_index >= robots.size()) {
    return "";
  }

  const std::string closest_id = robots[closest_index].id;
  const std::string normalized_current = normalizeRobotNamespace(current_striker);
  if (normalized_current.empty()) {
    return closest_id;
  }

  if (normalizeRobotNamespace(closest_id) == normalized_current) {
    return closest_id;
  }

  double current_distance = std::numeric_limits<double>::infinity();
  for (const auto & robot : robots) {
    if (!robot.valid) {
      continue;
    }
    if (normalizeRobotNamespace(robot.id) == normalized_current) {
      current_distance = planarDistance(robot.x, robot.y, ball_x, ball_y);
      break;
    }
  }

  const double closest_distance = planarDistance(
    robots[closest_index].x, robots[closest_index].y, ball_x, ball_y);
  if (closest_distance + hysteresis_m < current_distance) {
    return closest_id;
  }

  return normalized_current;
}

}  // namespace football_navigation
