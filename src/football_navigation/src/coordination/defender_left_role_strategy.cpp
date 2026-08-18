// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include "football_navigation/coordination/defender_left_role_strategy.hpp"

namespace football_navigation
{

RoleDecision DefenderLeftRoleStrategy::decide(const RoleStrategyContext & context) const
{
  const double direction = context.attack_goal_x >= context.home_goal_x ? 1.0 : -1.0;
  return RoleDecision{"DEFENDER_LEFT", context.home_goal_x + direction * 1.2, -1.1};
}

}  // namespace football_navigation
