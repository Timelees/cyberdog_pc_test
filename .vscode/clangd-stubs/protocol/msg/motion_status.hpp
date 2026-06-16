#pragma once
#include <memory>

namespace protocol
{
namespace msg
{
struct MotionStatus
{
  int motion_id {0};
  using SharedPtr = std::shared_ptr<MotionStatus>;
  using ConstSharedPtr = std::shared_ptr<const MotionStatus>;
};
}  // namespace msg
}  // namespace protocol
