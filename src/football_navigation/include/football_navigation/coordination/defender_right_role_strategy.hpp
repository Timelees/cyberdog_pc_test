// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "football_navigation/coordination/role_strategy.hpp"

namespace football_navigation
{

class DefenderRightRoleStrategy final : public RoleStrategy
{
public:
  RoleDecision decide(const RoleStrategyContext & context) const override;
};

}  // namespace football_navigation
