// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "football_navigation/coordination/role_strategy.hpp"

namespace football_navigation
{

class StrikerRoleStrategy final : public RoleStrategy
{
public:
  RoleDecision decide(const RoleStrategyContext & context) const override;
  static double distanceToBall(
    double robot_x, double robot_y, const RoleStrategyContext & context);
};

}  // namespace football_navigation
