#pragma once
#include <memory>

namespace rclcpp
{
class Node;

namespace executors
{
class SingleThreadedExecutor
{
public:
  template<typename NodeT>
  void add_node(std::shared_ptr<NodeT>)
  {}

  void spin() {}
};
}  // namespace executors
}  // namespace rclcpp
