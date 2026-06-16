#pragma once
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

namespace rclcpp
{
class Node;
}

namespace tf2_ros
{
class TransformBroadcaster
{
public:
  explicit TransformBroadcaster(rclcpp::Node & node) { (void)node; }
  explicit TransformBroadcaster(const std::shared_ptr<rclcpp::Node> & node) { (void)node; }

  void sendTransform(const geometry_msgs::msg::TransformStamped & transform)
  {
    (void)transform;
  }
};
}  // namespace tf2_ros
