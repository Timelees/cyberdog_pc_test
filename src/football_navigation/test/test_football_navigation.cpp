#include <gtest/gtest.h>

#include <cmath>

#include "football_navigation/core/football_geometry.hpp"

namespace football_navigation
{
namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.w = 1.0;
  return result;
}

TEST(FootballApproach, KeepsTransitPathOutsideConfiguredBallClearance)
{
  const auto ball = pose(0.0, 0.0);
  const auto target = pose(1.0, 0.0);
  const auto robot = pose(0.5, 1.5);
  ApproachConfig approach;
  approach.approach_distance = 0.4;
  LateralEntryConfig lateral;
  lateral.ball_protection_radius_m = 0.45;
  lateral.robot_footprint_radius_m = 0.35;
  lateral.entry_lateral_m = 1.0;
  lateral.field_min_x = -3.0;
  lateral.field_max_x = 3.0;
  lateral.field_min_y = -3.0;
  lateral.field_max_y = 3.0;

  const auto plan = computeApproachPoseAvoidingBlocker(
    ball, target, robot, {}, approach, lateral, 0.5);

  ASSERT_TRUE(plan.feasible);
  EXPECT_TRUE(plan.used_detour);
  EXPECT_GE(
    distancePointToSegment(
      ball.pose.position.x, ball.pose.position.y,
      robot.pose.position.x, robot.pose.position.y,
      plan.pose.pose.position.x, plan.pose.pose.position.y),
    lateral.ball_protection_radius_m + lateral.robot_footprint_radius_m);
}

TEST(FootballApproach, FootprintRadiusContainsFrontAndSideExtents)
{
  EXPECT_NEAR(
    circumscribedRobotFootprintRadius(0.302, 0.170, 0.562, 0.339),
    std::hypot(0.302, 0.170), 1e-9);
}

TEST(FootballApproach, ContactPoseLeavesBallRadiusAndMarginAheadOfRobot)
{
  const auto ball = pose(0.0, 0.0);
  const auto target = pose(1.0, 0.0);
  ApproachConfig approach;
  approach.approach_distance = 0.6;
  approach.kick_ball_lateral_offset_m = -0.136;

  const auto contact = computeContactApproachPose(
    ball, target, approach, 0.302, 0.11, 0.08);

  EXPECT_NEAR(contact.pose.position.x, -0.492, 1e-9);
  EXPECT_NEAR(contact.pose.position.y, 0.136, 1e-9);
  EXPECT_NEAR(
    std::hypot(contact.pose.position.x, contact.pose.position.y),
    std::hypot(0.492, 0.136), 1e-9);
}

TEST(FootballApproach, KickContactPoseTargetsConfiguredGaitFootEndpoint)
{
  const auto ball = pose(0.0, 0.0);
  const auto target = pose(1.0, 0.0);
  ApproachConfig approach;

  const auto contact = computeKickContactPose(
    ball, target, approach, 0.394, -0.056);

  EXPECT_NEAR(contact.pose.position.x, -0.394, 1e-9);
  EXPECT_NEAR(contact.pose.position.y, 0.056, 1e-9);
  EXPECT_NEAR(
    distanceBallToKickFoot(
      contact.pose.position.x, contact.pose.position.y, 0.0,
      ball.pose.position.x, ball.pose.position.y, 0.394, -0.056, 0.02),
    0.02, 1e-9);
}

TEST(FootballApproach, FrontClearanceStanceRemainsInsideKickSweepReach)
{
  const auto ball = pose(0.0, 0.0);
  const auto target = pose(1.0, 0.0);
  ApproachConfig approach;
  approach.kick_ball_lateral_offset_m = -0.056;

  const auto stance = computeContactApproachPose(
    ball, target, approach, 0.302, 0.11, 0.04);

  EXPECT_NEAR(stance.pose.position.x, -0.452, 1e-9);
  EXPECT_NEAR(stance.pose.position.y, 0.056, 1e-9);
  EXPECT_NEAR(-stance.pose.position.x - 0.302 - 0.11, 0.04, 1e-9);
  EXPECT_LT(
    distanceBallToKickTrajectory(
      stance.pose.position.x, stance.pose.position.y, 0.0,
      ball.pose.position.x, ball.pose.position.y,
      0.074, -0.056, 0.01, 0.394, -0.056, 0.02),
    0.11 - 0.02);
}

TEST(FootballApproach, KickFootDistanceRejectsBallOutsideStrikeRadius)
{
  EXPECT_GT(
    distanceBallToKickFoot(0.0, 0.0, 0.0, 0.472, -0.169, 0.374, -0.056, 0.02),
    0.11);
  EXPECT_LT(
    distanceBallToKickFoot(0.0, 0.0, 0.0, 0.374, -0.056, 0.374, -0.056, 0.02),
    0.11);
}

TEST(FootballApproach, KickTrajectoryDistanceFindsClosestPointOnSweep)
{
  const double distance = distanceBallToKickTrajectory(
    0.0, 0.0, 0.0,
    0.20, -0.056,
    0.054, -0.056, 0.05,
    0.374, -0.056, 0.06);

  EXPECT_NEAR(distance, 0.0545358776515, 1e-9);
}

TEST(FootballApproach, RejectsContactAlignmentFromObservedMissedKickPose)
{
  const double distance = distanceBallToKickTrajectory(
    0.72004, -0.92370, 1.19296,
    1.05119, -0.54688,
    0.074, -0.056, 0.01,
    0.394, -0.056, 0.02);

  EXPECT_NEAR(distance, 0.13880445247384, 1e-9);
  EXPECT_GT(distance, 0.11 - 0.02);
}

}  // namespace
}  // namespace football_navigation
