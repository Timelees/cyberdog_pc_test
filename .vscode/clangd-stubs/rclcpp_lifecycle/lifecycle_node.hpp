#pragma once
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace rclcpp_lifecycle
{
class State
{};

namespace node_interfaces
{
class LifecycleNodeInterface
{
public:
  enum class CallbackReturn
  {
    SUCCESS = 0,
    FAILURE = 1,
    ERROR = 2
  };

  virtual CallbackReturn on_configure(const State &) { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_activate(const State &) { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_deactivate(const State &) { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_cleanup(const State &) { return CallbackReturn::SUCCESS; }
  virtual CallbackReturn on_shutdown(const State &) { return CallbackReturn::SUCCESS; }
};
}  // namespace node_interfaces

class LifecycleNode : public rclcpp::Node, public node_interfaces::LifecycleNodeInterface
{
public:
  LifecycleNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options)
  {}

  using rclcpp::Node::get_parameter;
  using rclcpp::Node::declare_parameter;
  using rclcpp::Node::has_parameter;
};
}  // namespace rclcpp_lifecycle
