#pragma once
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nav2_util
{
class NodeThread
{
public:
  explicit NodeThread(rclcpp::Node::SharedPtr) {}
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor *) {}
};
}  // namespace nav2_util
