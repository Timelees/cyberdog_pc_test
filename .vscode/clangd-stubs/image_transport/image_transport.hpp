#pragma once
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <string>

namespace image_transport
{
class ImageTransport
{
public:
  explicit ImageTransport(const rclcpp::Node::SharedPtr &) {}
};

class SubscriberFilter
{};
}  // namespace image_transport
