// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "football_navigation/football_goal_adapter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<football_navigation::FootballGoalAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
