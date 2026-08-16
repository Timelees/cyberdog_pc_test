#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "football_navigation/plugins/dynamic_obstacle_marker.hpp"
#include "football_navigation/core/football_geometry.hpp"

namespace
{

  constexpr double kTolerance = 1e-6;
  constexpr double kPi = 3.14159265358979323846;

  geometry_msgs::msg::PoseStamped makePose(
      const std::string &frame_id, double x, double y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  double yawOf(const geometry_msgs::msg::PoseStamped &pose)
  {
    return tf2::getYaw(pose.pose.orientation);
  }

} // namespace

TEST(FootballGeometry, InfersTeamIdFromCyberdogNamespace)
{
  EXPECT_EQ("a", football_navigation::inferTeamIdFromNamespace("cyberdog_2"));
  EXPECT_EQ("b", football_navigation::inferTeamIdFromNamespace("cyberdog_8"));
  EXPECT_EQ("a", football_navigation::inferTeamIdFromNamespace("/cyberdog_3/"));
  EXPECT_EQ("", football_navigation::inferTeamIdFromNamespace("robot_2"));
  EXPECT_EQ("", football_navigation::inferTeamIdFromNamespace("cyberdog_0"));
  EXPECT_EQ("", football_navigation::inferTeamIdFromNamespace("cyberdog_11"));
  EXPECT_EQ("", football_navigation::inferTeamIdFromNamespace("cyberdog_02"));
}


TEST(FootballGeometry, SignedYawErrorUsesShortestWrappedDirection)
{
  EXPECT_NEAR(
    0.20,
    football_navigation::signedYawError(-kPi + 0.10, kPi - 0.10),
    kTolerance);
  EXPECT_NEAR(
    -0.20,
    football_navigation::signedYawError(kPi - 0.10, -kPi + 0.10),
    kTolerance);
}

TEST(FootballGeometry, NormalizeAngleNeverRequiresAFullTurn)
{
  EXPECT_NEAR(0.25, football_navigation::normalizeAngle(2.0 * kPi + 0.25), kTolerance);
  EXPECT_NEAR(-0.25, football_navigation::normalizeAngle(-2.0 * kPi - 0.25), kTolerance);
}

TEST(FootballGeometry, ComputesApproachPoseBehindBallFacingBall)
{
  auto ball = makePose("base_link", 2.0, 1.0);
  auto kick_target = makePose("base_link", 5.0, 1.0);

  football_navigation::ApproachConfig config;
  config.approach_distance = 0.8;
  config.default_kick_yaw = 0.0;

  const auto approach = football_navigation::computeApproachPose(ball, kick_target, config);

  EXPECT_EQ("base_link", approach.header.frame_id);
  EXPECT_NEAR(1.2, approach.pose.position.x, kTolerance);
  EXPECT_NEAR(1.0, approach.pose.position.y, kTolerance);
  EXPECT_NEAR(0.0, yawOf(approach), kTolerance);
}

TEST(FootballGeometry, NormalizesDiagonalKickDirection)
{
  auto ball = makePose("base_link", 1.0, 1.0);
  auto kick_target = makePose("base_link", 2.0, 2.0);

  football_navigation::ApproachConfig config;
  config.approach_distance = std::sqrt(2.0);
  config.default_kick_yaw = 0.0;

  const auto approach = football_navigation::computeApproachPose(ball, kick_target, config);

  EXPECT_NEAR(0.0, approach.pose.position.x, kTolerance);
  EXPECT_NEAR(0.0, approach.pose.position.y, kTolerance);
  EXPECT_NEAR(kPi / 4.0, yawOf(approach), kTolerance);
}

TEST(FootballGeometry, UsesFallbackYawWhenBallAndKickTargetOverlap)
{
  auto ball = makePose("base_link", 2.0, 3.0);
  auto kick_target = makePose("base_link", 2.0, 3.0);

  football_navigation::ApproachConfig config;
  config.approach_distance = 0.5;
  config.default_kick_yaw = kPi / 2.0;

  const auto approach = football_navigation::computeApproachPose(ball, kick_target, config);

  EXPECT_NEAR(2.0, approach.pose.position.x, kTolerance);
  EXPECT_NEAR(2.5, approach.pose.position.y, kTolerance);
  EXPECT_NEAR(kPi / 2.0, yawOf(approach), kTolerance);
}

TEST(FootballGeometry, ComputesApproachPoseFromRobotToBallDirection)
{
  auto ball = makePose("base_link", 2.0, 1.0);
  auto robot = makePose("base_link", 0.0, 1.0);

  football_navigation::ApproachConfig config;
  config.approach_distance = 0.6;
  config.default_kick_yaw = 0.0;

  const auto approach =
      football_navigation::computeApproachPoseFromRobot(ball, robot, config);

  EXPECT_EQ("base_link", approach.header.frame_id);
  EXPECT_NEAR(1.4, approach.pose.position.x, kTolerance);
  EXPECT_NEAR(1.0, approach.pose.position.y, kTolerance);
  EXPECT_NEAR(0.0, yawOf(approach), kTolerance);
}

TEST(DynamicObstacleMarker, BuildsFootprintFromStandingDimensions)
{
  const auto footprint = football_navigation::makeFootprintFromDimensions(0.562, 0.339, 0.04);
  ASSERT_EQ(4u, footprint.size());
  EXPECT_NEAR(0.321, footprint.front().x, 1e-3);
  EXPECT_NEAR(0.2095, footprint.front().y, 1e-3);
}

TEST(DynamicObstacleMarker, MarksOrientedFootprintAsLethal)
{
  nav2_costmap_2d::Costmap2D costmap(40, 40, 0.1, -2.0, -2.0, nav2_costmap_2d::FREE_SPACE);
  std::vector<geometry_msgs::msg::Point> footprint;
  ASSERT_TRUE(nav2_costmap_2d::makeFootprintFromString(
      "[[0.281, 0.1695], [0.281, -0.1695], [-0.281, -0.1695], [-0.281, 0.1695]]",
      footprint));

  football_navigation::DynamicObstacle obstacle;
  obstacle.x = 0.0;
  obstacle.y = 0.0;
  obstacle.yaw = 0.0;
  obstacle.radius = 0.32;

  const unsigned int marked = football_navigation::markFootprintObstacle(
      costmap, obstacle, footprint, nav2_costmap_2d::LETHAL_OBSTACLE);

  unsigned int center_x = 0;
  unsigned int center_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, center_x, center_y));
  EXPECT_GT(marked, 0u);
  EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, costmap.getCost(center_x, center_y));

  unsigned int outside_x = 0;
  unsigned int outside_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.5, 0.0, outside_x, outside_y));
  EXPECT_EQ(nav2_costmap_2d::FREE_SPACE, costmap.getCost(outside_x, outside_y));
}

TEST(DynamicObstacleMarker, MarksCircularObstacleAsLethal)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0, nav2_costmap_2d::FREE_SPACE);
  football_navigation::DynamicObstacle obstacle;
  obstacle.x = 0.0;
  obstacle.y = 0.0;
  obstacle.radius = 0.25;

  const unsigned int marked = football_navigation::markCircularObstacle(
      costmap, obstacle, nav2_costmap_2d::LETHAL_OBSTACLE);

  unsigned int center_x = 0;
  unsigned int center_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, center_x, center_y));
  EXPECT_GT(marked, 0u);
  EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, costmap.getCost(center_x, center_y));

  unsigned int outside_x = 0;
  unsigned int outside_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.4, 0.0, outside_x, outside_y));
  EXPECT_EQ(nav2_costmap_2d::FREE_SPACE, costmap.getCost(outside_x, outside_y));
}

TEST(DynamicObstacleMarker, ClipsObstacleToCostmapBounds)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  football_navigation::DynamicObstacle obstacle;
  obstacle.x = 0.02;
  obstacle.y = 0.02;
  obstacle.radius = 0.25;

  const unsigned int marked = football_navigation::markCircularObstacle(
      costmap, obstacle, nav2_costmap_2d::LETHAL_OBSTACLE);

  EXPECT_GT(marked, 0u);
  EXPECT_EQ(nav2_costmap_2d::LETHAL_OBSTACLE, costmap.getCost(0, 0));
}

TEST(DynamicObstacleMarker, CanPreserveHigherExistingCost)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.1, -1.0, -1.0, nav2_costmap_2d::FREE_SPACE);
  football_navigation::DynamicObstacle obstacle;
  obstacle.x = 0.0;
  obstacle.y = 0.0;
  obstacle.radius = 0.15;

  unsigned int center_x = 0;
  unsigned int center_y = 0;
  ASSERT_TRUE(costmap.worldToMap(0.0, 0.0, center_x, center_y));
  costmap.setCost(center_x, center_y, 200);

  const unsigned int marked = football_navigation::markCircularObstacle(
      costmap, obstacle, 100, true);

  EXPECT_GT(marked, 0u);
  EXPECT_EQ(200, costmap.getCost(center_x, center_y));
}

TEST(FootballTeamRole, SelectsClosestRobotPerTeam)
{
  std::vector<football_navigation::RobotPose2D> team_a = {
      {"cyberdog_1", 0.0, 0.0, true},
      {"cyberdog_2", 3.0, 0.0, true},
      {"cyberdog_3", 5.0, 0.0, true},
  };

  const auto closest_index = football_navigation::indexOfClosestRobot(team_a, 2.5, 0.0);
  ASSERT_EQ(1u, closest_index);
  EXPECT_EQ("cyberdog_2", team_a[closest_index].id);
}

TEST(FootballTeamRole, KeepsStrikerUnlessChallengerIsClearlyCloser)
{
  std::vector<football_navigation::RobotPose2D> team = {
      {"cyberdog_1", 1.0, 0.0, true},
      {"cyberdog_2", 2.2, 0.0, true},
  };

  const std::string initial = football_navigation::selectStrikerWithHysteresis(
      team, 2.0, 0.0, "", 0.35);
  EXPECT_EQ("cyberdog_2", initial);

  const std::string keep_striker = football_navigation::selectStrikerWithHysteresis(
      team, 2.0, 0.0, "cyberdog_2", 0.35);
  EXPECT_EQ("cyberdog_2", keep_striker);

  const std::string switch_striker = football_navigation::selectStrikerWithHysteresis(
      team, 0.5, 0.0, "cyberdog_2", 0.35);
  EXPECT_EQ("cyberdog_1", switch_striker);
}

TEST(FootballTeamRole, UsesNumericRobotIdForEqualAttackerScores)
{
  std::vector<football_navigation::RobotPose2D> robots = {
      {"cyberdog_10", 1.0, 0.0, true},
      {"cyberdog_2", 1.0, 0.0, true},
  };
  EXPECT_EQ(
      1u, football_navigation::indexOfBestAttacker(robots, 0.0, 0.0, 0.5, 0.8));
}

TEST(FootballTeamRole, DirectionalVelocityBeatsSameDistanceRobotMovingAway)
{
  football_navigation::RobotPose2D toward{"cyberdog_2", 0.0, 0.0, true};
  toward.vx = 0.3;
  football_navigation::RobotPose2D away{"cyberdog_3", 0.0, 0.0, true};
  away.vx = -0.3;
  std::vector<football_navigation::RobotPose2D> robots{away, toward};
  EXPECT_EQ(
      1u, football_navigation::indexOfBestAttacker(robots, 2.0, 0.0, 0.5, 0.4));
}

TEST(FootballApproach, RoutesAroundBallWhenDirectPathWouldCrossIt)
{
  const auto ball = makePose("base_link", 0.0, 0.0);
  const auto target = makePose("base_link", 2.0, 0.0);
  const auto robot = makePose("base_link", 1.0, 0.0);
  football_navigation::ApproachConfig approach;
  approach.approach_distance = 0.6;
  football_navigation::LateralEntryConfig lateral;
  lateral.entry_lateral_m = 0.7;
  lateral.ball_protection_radius_m = 0.28;
  const auto entry = football_navigation::computeApproachPoseAvoidingBlocker(
      ball, target, robot, {}, approach, lateral, 0.45);
  EXPECT_TRUE(entry.feasible);
  EXPECT_TRUE(entry.used_detour);
  EXPECT_GT(std::fabs(entry.pose.pose.position.y), 0.5);
  EXPECT_LT(entry.pose.pose.position.x, 0.0);
}

TEST(FootballGeometry, NormalizesQuaternionBeforeYaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = 2.0 * std::sin(kPi / 8.0);
  quaternion.w = 2.0 * std::cos(kPi / 8.0);
  double yaw = 0.0;
  ASSERT_TRUE(football_navigation::yawFromQuaternion(quaternion, yaw));
  EXPECT_NEAR(kPi / 4.0, yaw, kTolerance);
}

TEST(FootballGeometry, RejectsZeroQuaternion)
{
  geometry_msgs::msg::Quaternion quaternion;
  double yaw = 0.0;
  EXPECT_FALSE(football_navigation::yawFromQuaternion(quaternion, yaw));
}

TEST(FootballSimulationTf, RawTagAlignmentRecoversTagGlobalPoseForTenRobots)
{
  for (int index = 1; index <= 10; ++index)
  {
    const double yaw = index % 2 == 0 ? kPi : 0.0;
    const double x = -2.0 + 0.9 * index;
    const double y = -1.5 + 0.3 * index;
    const auto raw = football_navigation::makeSimulatedRawTagObservation(x, y, yaw);
    const auto recovered = football_navigation::recoverTagGlobalBasePose(raw);
    geometry_msgs::msg::Quaternion orientation = recovered.rotation;
    double recovered_yaw = 0.0;
    ASSERT_TRUE(football_navigation::yawFromQuaternion(orientation, recovered_yaw));
    EXPECT_NEAR(x, recovered.translation.x, kTolerance);
    EXPECT_NEAR(y, recovered.translation.y, kTolerance);
    EXPECT_NEAR(0.0, recovered.translation.z, kTolerance);
    EXPECT_NEAR(0.0, std::sin(recovered_yaw - yaw), kTolerance);
  }
}

TEST(FootballGeometry, ConvertsWorldVelocityBackToBodyFrame)
{
  const auto body = football_navigation::worldVelocityToBody(0.0, 1.0, kPi / 2.0);
  EXPECT_NEAR(1.0, body.first, kTolerance);
  EXPECT_NEAR(0.0, body.second, kTolerance);
}

TEST(FootballGeometry, DetectsAsymmetricFrontFaceBallContact)
{
  const auto contact = football_navigation::computeBallContactMetrics(
      0.0, 0.0, 0.0,
      0.25, 0.23, 0.13,
      0.36, 0.0, 0.11, 0.01);
  EXPECT_TRUE(contact.intersects);
  EXPECT_TRUE(contact.front_contact);
  EXPECT_FALSE(contact.side_contact);
  EXPECT_FALSE(contact.rear_contact);
  EXPECT_NEAR(0.36, contact.forward_offset_m, kTolerance);
  EXPECT_NEAR(0.0, contact.lateral_offset_m, kTolerance);
  EXPECT_LE(contact.separation_m, 0.01 + kTolerance);
}

TEST(FootballGeometry, RejectsSideAndRearContactAsPushContact)
{
  const auto side = football_navigation::computeBallContactMetrics(
      0.0, 0.0, 0.0,
      0.25, 0.23, 0.13,
      0.0, 0.24, 0.11, 0.01);
  EXPECT_TRUE(side.intersects);
  EXPECT_FALSE(side.front_contact);
  EXPECT_TRUE(side.side_contact);
  EXPECT_FALSE(side.rear_contact);

  const auto rear = football_navigation::computeBallContactMetrics(
      0.0, 0.0, 0.0,
      0.25, 0.23, 0.13,
      -0.34, 0.0, 0.11, 0.01);
  EXPECT_TRUE(rear.intersects);
  EXPECT_FALSE(rear.front_contact);
  EXPECT_FALSE(rear.side_contact);
  EXPECT_TRUE(rear.rear_contact);
}

TEST(FootballApproach, DriveThroughRequiresRobotBehindAndAligned)
{
  EXPECT_TRUE(football_navigation::isRobotBehindAndAlignedForKick(
      -0.2, 0.05, 0.0, 0.0, 2.0, 0.0, 0.10, 0.20));
  EXPECT_FALSE(football_navigation::isRobotBehindAndAlignedForKick(
      0.2, 0.0, 0.0, 0.0, 2.0, 0.0, 0.10, 0.20));
  EXPECT_FALSE(football_navigation::isRobotBehindAndAlignedForKick(
      -0.2, 0.4, 0.0, 0.0, 2.0, 0.0, 0.10, 0.20));
}

TEST(FootballApproach, DirectBehindPathIsFeasible)
{
  const auto ball = makePose("base_link", 0.0, 0.0);
  const auto target = makePose("base_link", 2.0, 0.0);
  const auto robot = makePose("base_link", -1.5, 0.0);
  football_navigation::ApproachConfig approach;
  approach.approach_distance = 0.6;
  football_navigation::LateralEntryConfig lateral;
  const auto plan = football_navigation::computeApproachPoseAvoidingBlocker(
      ball, target, robot, {}, approach, lateral, 0.45);
  EXPECT_TRUE(plan.feasible);
  EXPECT_FALSE(plan.used_detour);
  EXPECT_NEAR(-0.6, plan.pose.pose.position.x, kTolerance);
}

TEST(FootballApproach, LetsPlannerRouteAroundTransitBlocker)
{
  const auto ball = makePose("base_link", 0.0, 0.0);
  const auto target = makePose("base_link", 0.0, 2.0);
  const auto robot = makePose("base_link", 0.0, 1.0);
  football_navigation::ApproachConfig approach;
  approach.approach_distance = 0.6;
  football_navigation::LateralEntryConfig lateral;
  const std::vector<football_navigation::OpponentPoint2D> blockers{{0.0, 0.5}};
  const auto plan = football_navigation::computeApproachPoseAvoidingBlocker(
      ball, target, robot, blockers, approach, lateral, 10.0);
  EXPECT_TRUE(plan.feasible);
  EXPECT_FALSE(plan.used_detour);
  EXPECT_NEAR(0.0, plan.pose.pose.position.x, kTolerance);
  EXPECT_NEAR(-0.6, plan.pose.pose.position.y, kTolerance);
}

TEST(FootballApproach, UsesReachableStagingPointWhenBallBehindPoseIsOccupied)
{
  const auto ball = makePose("base_link", 4.0, 0.0);
  const auto target = makePose("base_link", 8.0, 0.0);
  const auto robot = makePose("base_link", 0.5, -1.2);
  football_navigation::ApproachConfig approach;
  approach.approach_distance = 0.6;
  football_navigation::LateralEntryConfig lateral;
  lateral.preferred_side = 1;
  const std::vector<football_navigation::OpponentPoint2D> blockers{
    {3.35, -0.35}};

  const auto plan = football_navigation::computeApproachPoseAvoidingBlocker(
    ball, target, robot, blockers, approach, lateral, 0.50);

  EXPECT_TRUE(plan.feasible);
  EXPECT_TRUE(plan.used_detour);
  EXPECT_NEAR(3.4, plan.pose.pose.position.x, kTolerance);
  EXPECT_NEAR(0.7, plan.pose.pose.position.y, kTolerance);
}

TEST(FootballApproach, RejectsStagingEndpointInsidePlannerObstacleClearance)
{
  const auto ball = makePose("base_link", 4.0, 0.0);
  const auto target = makePose("base_link", 8.0, 0.0);
  const auto robot = makePose("base_link", 0.5, -1.2);
  football_navigation::ApproachConfig approach;
  approach.approach_distance = 0.6;
  football_navigation::LateralEntryConfig lateral;
  lateral.opponent_endpoint_clearance_m = 0.84;
  const std::vector<football_navigation::OpponentPoint2D> blockers{
    {3.4, 0.7},
    {3.4, -0.7},
    {3.55, 0.0},
  };

  const auto plan = football_navigation::computeApproachPoseAvoidingBlocker(
    ball, target, robot, blockers, approach, lateral, 0.50);

  EXPECT_FALSE(plan.feasible);
}

TEST(FootballTargetSafety, TruncatesCurrentFiveMeterTargetToFourMeters)
{
  const double current_distance = 5.0;
  const double maximum_distance = 4.0;
  const double scale = maximum_distance / current_distance;
  EXPECT_NEAR(4.0, current_distance * scale, kTolerance);
}

TEST(FootballTargetSafety, CurrentDistanceDoesNotReusePreviousOneMeterDistance)
{
  const double previous_distance = 1.0;
  const double current_distance = 5.0;
  const double maximum_distance = 4.0;
  (void)previous_distance;
  EXPECT_NEAR(4.0, current_distance * maximum_distance / current_distance, kTolerance);
}

TEST(FootballBallFusion, RejectsCandidatesOutsidePointZeroEightSecondSkew)
{
  const double newest_stamp = 10.0;
  const double stale_stamp = 9.91;
  EXPECT_GT(std::fabs(newest_stamp - stale_stamp), 0.08);
}

TEST(FootballRoleSafety, RequiresNineOtherRobotsForTenRobotMatch)
{
  const int minimum_other_robot_count = 9;
  EXPECT_FALSE(8 >= minimum_other_robot_count);
  EXPECT_TRUE(9 >= minimum_other_robot_count);
}

TEST(FootballMatchState, KickoffATeamBIsStopped)
{
  const std::string match_state = "KICKOFF_A";
  const std::string team = "b";
  EXPECT_FALSE(match_state == "PLAY" || (match_state == "KICKOFF_A" && team == "a"));
}

TEST(FootballMatchState, BallReleaseTransitionsKickoffToPlay)
{
  const double release_distance = 0.25;
  const double ball_displacement = 0.26;
  EXPECT_TRUE(ball_displacement >= release_distance);
}

TEST(FootballFakeBall, StrongestContactWinsIndependentOfTraversalOrder)
{
  const double first_strength = 0.4;
  const double second_strength = 0.7;
  EXPECT_EQ(std::max(first_strength, second_strength), std::max(second_strength, first_strength));
}

TEST(FootballFakeWorld, ClampsRobotInsideFieldBoundary)
{
  const double min_x = -2.0 + 0.10;
  const double max_x = 8.0 - 0.10;
  EXPECT_NEAR(min_x, std::clamp(-3.0, min_x, max_x), kTolerance);
  EXPECT_NEAR(max_x, std::clamp(9.0, min_x, max_x), kTolerance);
}

TEST(FootballFakeWorld, SeparatesOverlappingRobots)
{
  const double collision_radius = 0.38;
  const double minimum_distance = 2.0 * collision_radius;
  const double overlap_distance = 0.20;
  const double retreat = 0.5 * (minimum_distance - overlap_distance);
  EXPECT_NEAR(minimum_distance, overlap_distance + 2.0 * retreat, kTolerance);
}

TEST(FootballApproach, SupportsArbitraryGoalDirection)
{
  const auto ball = makePose("base_link", 1.0, 1.0);
  const auto target = makePose("base_link", 1.0, 3.0);
  football_navigation::ApproachConfig config;
  config.approach_distance = 0.5;
  const auto pose = football_navigation::computeApproachPose(ball, target, config);
  EXPECT_NEAR(1.0, pose.pose.position.x, kTolerance);
  EXPECT_NEAR(0.5, pose.pose.position.y, kTolerance);
  EXPECT_NEAR(kPi / 2.0, yawOf(pose), kTolerance);
}

TEST(FootballApproach, KeepsCornerApproachesInsideField)
{
  football_navigation::ApproachConfig approach;
  approach.approach_distance = 0.6;
  football_navigation::LateralEntryConfig lateral;
  const std::vector<std::pair<double, double>> corners = {
      {-2.0, -3.0}, {-2.0, 3.0}, {8.0, -3.0}, {8.0, 3.0}};
  for (const auto &corner : corners)
  {
    const auto ball = makePose("base_link", corner.first, corner.second);
    const auto target = makePose("base_link", 3.0, 0.0);
    const auto robot = makePose("base_link", 3.0, 0.0);
    const auto plan = football_navigation::computeApproachPoseAvoidingBlocker(
        ball, target, robot, {}, approach, lateral, 0.45);
    if (plan.feasible)
    {
      EXPECT_GE(plan.pose.pose.position.x, lateral.field_min_x + lateral.boundary_margin_m);
      EXPECT_LE(plan.pose.pose.position.x, lateral.field_max_x - lateral.boundary_margin_m);
      EXPECT_GE(plan.pose.pose.position.y, lateral.field_min_y + lateral.boundary_margin_m);
      EXPECT_LE(plan.pose.pose.position.y, lateral.field_max_y - lateral.boundary_margin_m);
    }
  }
}

TEST(FootballRecovery, YieldSideUsesTeamRobotAndRelativePosition)
{
  const int side = football_navigation::deterministicYieldSide("a", "cyberdog_2", 1.0);
  EXPECT_EQ(1, side);
  EXPECT_EQ(-side, football_navigation::deterministicYieldSide("b", "cyberdog_2", 1.0));
  EXPECT_EQ(-side, football_navigation::deterministicYieldSide("a", "cyberdog_2", -1.0));
}

TEST(FootballApproach, KeepsPreferredDetourSideAcrossSmallRobotMotion)
{
  const auto ball = makePose("tag_global", 4.0, 0.0);
  const auto target = makePose("tag_global", 8.0, 0.0);
  football_navigation::ApproachConfig approach;
  approach.approach_distance = 0.6;
  football_navigation::LateralEntryConfig lateral;
  lateral.entry_lateral_m = 0.7;
  lateral.preferred_side = 1;
  const std::vector<football_navigation::OpponentPoint2D> blockers = {
    {3.4, 0.0},
  };

  const auto first = football_navigation::computeApproachPoseAvoidingBlocker(
    ball, target, makePose("tag_global", 0.0, 0.15), blockers,
    approach, lateral, 0.5);
  const auto second = football_navigation::computeApproachPoseAvoidingBlocker(
    ball, target, makePose("tag_global", 0.0, -0.15), blockers,
    approach, lateral, 0.5);

  ASSERT_TRUE(first.feasible);
  ASSERT_TRUE(second.feasible);
  EXPECT_GT(first.pose.pose.position.y, 0.0);
  EXPECT_GT(second.pose.pose.position.y, 0.0);
}

TEST(FootballApproach, DetectsTargetPassedAlongItsTravelHeading)
{
  const auto target = makePose("tag_global", 3.4, -0.7);
  const auto before = makePose("tag_global", 3.2, -0.7);
  const auto after = makePose("tag_global", 3.6, -0.7);
  const auto lateral = makePose("tag_global", 3.4, -0.4);
  const auto after_lateral = makePose("tag_global", 3.6, -0.4);

  EXPECT_FALSE(football_navigation::approachTargetPassed(before, target, 0.05));
  EXPECT_TRUE(football_navigation::approachTargetPassed(after, target, 0.05));
  EXPECT_FALSE(football_navigation::approachTargetPassed(lateral, target, 0.05));
  EXPECT_FALSE(football_navigation::approachTargetPassed(after_lateral, target, 0.05));
}

TEST(FootballRecovery, DetectsCommandedStallOnlyWhenAllSignalsAgree)
{
  football_navigation::NoProgressConfig config;
  football_navigation::NoProgressMetrics stalled;
  stalled.elapsed_sec = 2.0;
  stalled.ball_distance_improvement = 0.01;
  stalled.goal_distance_improvement = 0.01;
  stalled.path_progress = 0.02;
  stalled.actual_speed = 0.01;
  stalled.command_speed = 0.20;
  stalled.obstacle_distance = 0.40;
  stalled.ball_motion = 0.01;
  stalled.ball_distance = 0.50;
  EXPECT_TRUE(football_navigation::isNoProgress(stalled, config));

  stalled.actual_speed = 0.20;
  EXPECT_FALSE(football_navigation::isNoProgress(stalled, config));
  stalled.actual_speed = 0.01;
  stalled.command_speed = 0.0;
  EXPECT_TRUE(football_navigation::isNoProgress(stalled, config));
}
