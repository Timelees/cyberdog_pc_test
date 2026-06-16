#pragma once
#include <cstdint>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace lifecycle_msgs
{
namespace msg
{
struct Transition
{
  uint8_t id{0};
  std::string label;
};

struct TransitionEvent
{
  std_msgs::msg::Header header;
  Transition transition;
};
}  // namespace msg
}  // namespace lifecycle_msgs
