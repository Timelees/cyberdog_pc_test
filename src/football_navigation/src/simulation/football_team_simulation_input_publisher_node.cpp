// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <memory>

#include "football_navigation/simulation/football_team_simulation_input_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(
      std::make_shared<football_navigation::FootballTeamSimulationInputPublisher>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("football_team_simulation_input_publisher"),
      "fatal exception: %s", exception.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
