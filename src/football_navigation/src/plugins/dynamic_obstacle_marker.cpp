// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.

#include "football_navigation/plugins/dynamic_obstacle_marker.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/footprint.hpp"

namespace football_navigation
{

namespace
{

geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

}  // namespace

std::vector<geometry_msgs::msg::Point> makeFootprintFromDimensions(
  double length_m, double width_m, double padding_m)
{
  const double half_length = std::max(0.0, length_m * 0.5);
  const double half_width = std::max(0.0, width_m * 0.5);
  std::vector<geometry_msgs::msg::Point> footprint = {
    makePoint(half_length, half_width),
    makePoint(half_length, -half_width),
    makePoint(-half_length, -half_width),
    makePoint(-half_length, half_width),
  };
  if (padding_m > 0.0) {
    nav2_costmap_2d::padFootprint(footprint, padding_m);
  }
  return footprint;
}

namespace
{

unsigned int markPolygonCells(
  nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<geometry_msgs::msg::Point32> & polygon,
  unsigned char cost,
  bool use_maximum)
{
  if (polygon.size() < 3) {
    return 0;
  }

  std::vector<nav2_costmap_2d::MapLocation> map_polygon;
  map_polygon.reserve(polygon.size());
  for (const auto & point : polygon) {
    nav2_costmap_2d::MapLocation loc;
    if (!costmap.worldToMap(static_cast<double>(point.x), static_cast<double>(point.y), loc.x, loc.y)) {
      continue;
    }
    map_polygon.push_back(loc);
  }
  if (map_polygon.size() < 3) {
    return 0;
  }

  std::vector<nav2_costmap_2d::MapLocation> polygon_cells;
  costmap.convexFillCells(map_polygon, polygon_cells);

  unsigned int marked = 0;
  for (const auto & cell : polygon_cells) {
    const unsigned int index = costmap.getIndex(cell.x, cell.y);
    if (use_maximum) {
      costmap.getCharMap()[index] = std::max(costmap.getCharMap()[index], cost);
    } else {
      costmap.getCharMap()[index] = cost;
    }
    ++marked;
  }
  return marked;
}

}  // namespace

unsigned int markCircularObstacle(
  nav2_costmap_2d::Costmap2D & costmap,
  const DynamicObstacle & obstacle,
  unsigned char cost,
  bool use_maximum)
{
  if (obstacle.radius <= 0.0) {
    return 0;
  }

  int min_mx = 0;
  int min_my = 0;
  int max_mx = 0;
  int max_my = 0;
  costmap.worldToMapEnforceBounds(
    obstacle.x - obstacle.radius, obstacle.y - obstacle.radius, min_mx, min_my);
  costmap.worldToMapEnforceBounds(
    obstacle.x + obstacle.radius, obstacle.y + obstacle.radius, max_mx, max_my);

  const int size_x = static_cast<int>(costmap.getSizeInCellsX());
  const int size_y = static_cast<int>(costmap.getSizeInCellsY());
  min_mx = std::max(0, std::min(min_mx, size_x - 1));
  max_mx = std::max(0, std::min(max_mx, size_x - 1));
  min_my = std::max(0, std::min(min_my, size_y - 1));
  max_my = std::max(0, std::min(max_my, size_y - 1));

  const double radius_sq = obstacle.radius * obstacle.radius;
  unsigned int marked = 0;
  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      const double dx = wx - obstacle.x;
      const double dy = wy - obstacle.y;
      if (dx * dx + dy * dy <= radius_sq) {
        const auto map_x = static_cast<unsigned int>(mx);
        const auto map_y = static_cast<unsigned int>(my);
        if (use_maximum) {
          costmap.setCost(map_x, map_y, std::max(costmap.getCost(map_x, map_y), cost));
        } else {
          costmap.setCost(map_x, map_y, cost);
        }
        ++marked;
      }
    }
  }

  return marked;
}

unsigned int markFootprintObstacle(
  nav2_costmap_2d::Costmap2D & costmap,
  const DynamicObstacle & obstacle,
  const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  unsigned char cost,
  bool use_maximum)
{
  if (footprint_spec.size() < 3) {
    return 0;
  }

  geometry_msgs::msg::PolygonStamped oriented;
  nav2_costmap_2d::transformFootprint(
    obstacle.x, obstacle.y, obstacle.yaw, footprint_spec, oriented);
  return markPolygonCells(costmap, oriented.polygon.points, cost, use_maximum);
}

}  // namespace football_navigation
