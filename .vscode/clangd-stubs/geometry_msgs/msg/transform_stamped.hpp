#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace geometry_msgs
{
namespace msg
{
struct Transform
{
  Vector3 translation;
  Quaternion rotation;
};

struct TransformStamped
{
  std_msgs::msg::Header header;
  std::string child_frame_id;
  Transform transform;
};
}  // namespace msg
}  // namespace geometry_msgs
