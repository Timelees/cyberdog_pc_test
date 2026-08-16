// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <memory>

#include "football_navigation/coordination/football_team_role_assigner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    const auto node =
      std::make_shared<football_navigation::FootballTeamRoleAssigner>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("football_team_role_assigner"),
      "fatal exception: %s", exception.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("football_team_role_assigner"),
      "fatal unknown exception");
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
