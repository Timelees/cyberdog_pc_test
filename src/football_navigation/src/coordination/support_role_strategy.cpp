// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include "football_navigation/coordination/support_role_strategy.hpp"

namespace football_navigation
{

RoleDecision SupportRoleStrategy::decide(const RoleStrategyContext & context) const
{
  const double direction = context.attack_goal_x >= context.home_goal_x ? 1.0 : -1.0;
  // Prefer the side with more field clearance. A full implementation can
  // score opponent and passing-line clearance; this deterministic fallback
  // already avoids pinning support against the upper boundary.
  const double lateral = context.ball_y >= 0.0 ? -1.4 : 1.4;
  // Keep the support robot outside the striker's ball-to-approach corridor.
  // The larger lateral offset also leaves room for the striker's collision
  // ellipse and for local planner recovery around the ball.
  return RoleDecision{
    "SUPPORT", context.ball_x - direction * 1.0, context.ball_y + lateral};
}

}  // namespace football_navigation
