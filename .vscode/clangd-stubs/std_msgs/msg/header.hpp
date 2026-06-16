#pragma once
#include <string>

namespace builtin_interfaces
{
namespace msg
{
struct Time
{
  int32_t sec {0};
  uint32_t nanosec {0};

  template<typename StampT>
  Time & operator=(const StampT &)
  {
    return *this;
  }
};
}  // namespace msg
}  // namespace builtin_interfaces

namespace std_msgs
{
namespace msg
{
struct Header
{
  builtin_interfaces::msg::Time stamp;
  std::string frame_id;
};
}  // namespace msg
}  // namespace std_msgs
