// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>

namespace football_navigation
{

struct RoleStrategyContext
{
  double ball_x{0.0};
  double ball_y{0.0};
  double attack_goal_x{0.0};
  double attack_goal_y{0.0};
  double home_goal_x{0.0};
  double home_goal_y{0.0};
};

struct RoleDecision
{
  std::string role{"STOP"};
  double target_x{0.0};
  double target_y{0.0};
};

class RoleStrategy
{
public:
  virtual ~RoleStrategy() = default;
  virtual RoleDecision decide(const RoleStrategyContext & context) const = 0;
};

}  // namespace football_navigation
