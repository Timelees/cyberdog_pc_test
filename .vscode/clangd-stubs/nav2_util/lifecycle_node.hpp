#pragma once
#include <memory>
#include <string>
#include <nav2_util/node_thread.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <bondcpp/bond.hpp>

namespace nav2_util
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleNode(
    const std::string & node_name,
    const std::string & ns = "",
    bool use_rclcpp_node = false,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode(node_name, options)
  {
    (void)ns;
    (void)use_rclcpp_node;
  }

  virtual ~LifecycleNode() = default;

  void createBond() {}
  void destroyBond() {}

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return nullptr;
  }

  using rclcpp_lifecycle::LifecycleNode::declare_parameter;
  using rclcpp_lifecycle::LifecycleNode::get_parameter;
  using rclcpp_lifecycle::LifecycleNode::has_parameter;

  template<typename SrvT>
  typename rclcpp::Client<SrvT>::SharedPtr create_client(const std::string &)
  {
    return nullptr;
  }

protected:
  bool use_rclcpp_node_ {false};
  rclcpp::Node::SharedPtr rclcpp_node_;
  std::unique_ptr<NodeThread> rclcpp_thread_;
  std::unique_ptr<bond::Bond> bond_ {nullptr};
};
}  // namespace nav2_util
