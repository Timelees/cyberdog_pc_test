// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <memory>

#include "football_navigation/control/football_trajectory_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      football_navigation::FootballTrajectoryAdapter>());

  rclcpp::shutdown();

  return 0;
}
