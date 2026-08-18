// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include "football_navigation/coordination/defender_left_role_strategy.hpp"
#include "football_navigation/coordination/defender_right_role_strategy.hpp"
#include "football_navigation/coordination/goalkeeper_role_strategy.hpp"
#include "football_navigation/coordination/striker_role_strategy.hpp"
#include "football_navigation/coordination/support_role_strategy.hpp"

namespace football_navigation
{
namespace
{

RoleStrategyContext teamAContext()
{
  RoleStrategyContext context;
  context.ball_x = 3.0;
  context.ball_y = 1.0;
  context.attack_goal_x = 8.0;
  context.home_goal_x = -2.0;
  return context;
}

TEST(RoleStrategies, PreserveTeamATacticalTargets)
{
  const auto context = teamAContext();
  const auto striker = StrikerRoleStrategy().decide(context);
  const auto support = SupportRoleStrategy().decide(context);
  const auto left = DefenderLeftRoleStrategy().decide(context);
  const auto right = DefenderRightRoleStrategy().decide(context);
  const auto goalkeeper = GoalkeeperRoleStrategy().decide(context);

  EXPECT_EQ(striker.role, "STRIKER");
  EXPECT_DOUBLE_EQ(striker.target_x, 3.0);
  EXPECT_DOUBLE_EQ(striker.target_y, 1.0);
  EXPECT_EQ(support.role, "SUPPORT");
  EXPECT_DOUBLE_EQ(support.target_x, 2.0);
  EXPECT_DOUBLE_EQ(support.target_y, -0.4);
  EXPECT_DOUBLE_EQ(left.target_x, -0.8);
  EXPECT_DOUBLE_EQ(left.target_y, -1.1);
  EXPECT_DOUBLE_EQ(right.target_x, -0.8);
  EXPECT_DOUBLE_EQ(right.target_y, 1.1);
  EXPECT_DOUBLE_EQ(goalkeeper.target_x, -1.65);
  EXPECT_DOUBLE_EQ(goalkeeper.target_y, 0.7);
}

TEST(RoleStrategies, MirrorTargetsForTeamB)
{
  auto context = teamAContext();
  context.attack_goal_x = -2.0;
  context.home_goal_x = 8.0;

  const auto support = SupportRoleStrategy().decide(context);
  const auto left = DefenderLeftRoleStrategy().decide(context);
  const auto right = DefenderRightRoleStrategy().decide(context);
  const auto goalkeeper = GoalkeeperRoleStrategy().decide(context);

  EXPECT_DOUBLE_EQ(support.target_x, 4.0);
  EXPECT_DOUBLE_EQ(left.target_x, 6.8);
  EXPECT_DOUBLE_EQ(right.target_x, 6.8);
  EXPECT_DOUBLE_EQ(goalkeeper.target_x, 7.65);
  EXPECT_DOUBLE_EQ(goalkeeper.target_y, 0.7);
}

TEST(RoleStrategies, StrikerDistanceUsesBallPosition)
{
  const auto context = teamAContext();
  EXPECT_DOUBLE_EQ(StrikerRoleStrategy::distanceToBall(0.0, 5.0, context), 5.0);
}

}  // namespace
}  // namespace football_navigation
