// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <algorithm>

#include "football_navigation/coordination/goalkeeper_role_strategy.hpp"

namespace football_navigation
{

RoleDecision GoalkeeperRoleStrategy::decide(const RoleStrategyContext & context) const
{
  const double direction = context.attack_goal_x >= context.home_goal_x ? 1.0 : -1.0;
  return RoleDecision{
    "GOALKEEPER", context.home_goal_x + direction * 0.35,
    std::clamp(context.ball_y, -0.7, 0.7)};
}

}  // namespace football_navigation
