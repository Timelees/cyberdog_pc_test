#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <string>

namespace cyberdog_visions_interfaces
{
namespace srv
{
struct Reloc
{
  struct Request
  {
    int reloc_id {0};
  };
  struct Reply
  {
    int status {0};
  };
  struct Response
  {
    int reloc_id {0};
    bool is_verified {false};
    float confidence {0.0f};
    Reply reply;
    geometry_msgs::msg::PoseStamped pose;
  };
};
}  // namespace srv
}  // namespace cyberdog_visions_interfaces
