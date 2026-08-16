// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#ifndef FOOTBALL_NAVIGATION__DYNAMIC_OBSTACLE_MARKER_HPP_
#define FOOTBALL_NAVIGATION__DYNAMIC_OBSTACLE_MARKER_HPP_

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace football_navigation
{

struct DynamicObstacle
{
  std::size_t source_index{0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  /** Bounding radius estimate for updateBounds. */
  double radius{0.55};
};

/** Build axis-aligned footprint from body length/width (centered at robot origin). */
std::vector<geometry_msgs::msg::Point> makeFootprintFromDimensions(
  double length_m, double width_m, double padding_m);

unsigned int markCircularObstacle(
  nav2_costmap_2d::Costmap2D & costmap,
  const DynamicObstacle & obstacle,
  unsigned char cost,
  bool use_maximum = false);

unsigned int markFootprintObstacle(
  nav2_costmap_2d::Costmap2D & costmap,
  const DynamicObstacle & obstacle,
  const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  unsigned char cost,
  bool use_maximum = false);

}  // namespace football_navigation

#endif  // FOOTBALL_NAVIGATION__DYNAMIC_OBSTACLE_MARKER_HPP_
