#pragma once
#include <std_msgs/msg/header.hpp>
#include <vector>

namespace sensor_msgs
{
namespace msg
{
struct LaserScan
{
  std_msgs::msg::Header header;
  float angle_min{0.0f};
  float angle_max{0.0f};
  float angle_increment{0.0f};
  float time_increment{0.0f};
  float scan_time{0.0f};
  float range_min{0.0f};
  float range_max{0.0f};
  std::vector<float> ranges;
  std::vector<float> intensities;
};
}  // namespace msg
}  // namespace sensor_msgs
