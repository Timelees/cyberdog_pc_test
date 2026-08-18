// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <cmath>

#include "football_navigation/coordination/striker_role_strategy.hpp"

namespace football_navigation
{

RoleDecision StrikerRoleStrategy::decide(const RoleStrategyContext & context) const
{
  return RoleDecision{"STRIKER", context.ball_x, context.ball_y};
}

double StrikerRoleStrategy::distanceToBall(
  const double robot_x, const double robot_y, const RoleStrategyContext & context)
{
  return std::hypot(robot_x - context.ball_x, robot_y - context.ball_y);
}

}  // namespace football_navigation
