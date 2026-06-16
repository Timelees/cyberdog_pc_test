#pragma once
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

namespace rclcpp
{
class Node;
}

namespace tf2_ros
{
class StaticTransformBroadcaster
{
public:
  explicit StaticTransformBroadcaster(rclcpp::Node & node) { (void)node; }

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    (void)transform;
  }
};
}  // namespace tf2_ros
