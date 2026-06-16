#pragma once
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <std_msgs/msg/header.hpp>

namespace sensor_msgs
{
namespace msg
{
struct Imu
{
  std_msgs::msg::Header header;
  geometry_msgs::msg::Vector3 angular_velocity;
  geometry_msgs::msg::Vector3 linear_acceleration;
  using ConstSharedPtr = std::shared_ptr<const Imu>;
};
}  // namespace msg
}  // namespace sensor_msgs
