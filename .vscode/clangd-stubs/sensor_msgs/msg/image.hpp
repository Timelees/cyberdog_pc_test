#pragma once
#include <memory>
#include <std_msgs/msg/header.hpp>

namespace sensor_msgs
{
namespace msg
{
struct Image
{
  std_msgs::msg::Header header;
  using SharedPtr = std::shared_ptr<Image>;
  using ConstSharedPtr = std::shared_ptr<const Image>;
};
struct PointCloud2
{
  using SharedPtr = std::shared_ptr<PointCloud2>;
  using ConstSharedPtr = std::shared_ptr<const PointCloud2>;
};
}  // namespace msg
}  // namespace sensor_msgs
