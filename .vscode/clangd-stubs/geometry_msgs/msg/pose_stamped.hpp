#pragma once
#include <std_msgs/msg/header.hpp>

namespace geometry_msgs
{
namespace msg
{
struct Point
{
  double x {0.0};
  double y {0.0};
  double z {0.0};
};

struct Quaternion
{
  double x {0.0};
  double y {0.0};
  double z {0.0};
  double w {1.0};
};

struct Pose
{
  Point position;
  Quaternion orientation;
};

struct PoseStamped
{
  std_msgs::msg::Header header;
  Pose pose;
};
}  // namespace msg
}  // namespace geometry_msgs
