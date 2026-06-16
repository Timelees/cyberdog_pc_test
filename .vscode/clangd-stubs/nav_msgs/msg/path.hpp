#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <std_msgs/msg/header.hpp>
#include <vector>

namespace nav_msgs
{
namespace msg
{
struct Path
{
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  using SharedPtr = std::shared_ptr<Path>;
  using ConstSharedPtr = std::shared_ptr<const Path>;
};
}  // namespace msg
}  // namespace nav_msgs
