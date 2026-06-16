#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace geometry_msgs
{
namespace msg
{
struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

struct PoseWithCovariance
{
  Pose pose;
};

struct TwistWithCovariance
{
  Twist twist;
};
}  // namespace msg
}  // namespace geometry_msgs

namespace nav_msgs
{
namespace msg
{
struct Odometry
{
  std_msgs::msg::Header header;
  std::string child_frame_id;
  geometry_msgs::msg::PoseWithCovariance pose;
  geometry_msgs::msg::TwistWithCovariance twist;
  using SharedPtr = std::shared_ptr<Odometry>;
  using ConstSharedPtr = std::shared_ptr<const Odometry>;
};
}  // namespace msg
}  // namespace nav_msgs
