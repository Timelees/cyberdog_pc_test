#pragma once

namespace message_filters
{
namespace sync_policies
{
template<typename... T>
struct ApproximateTime
{
  explicit ApproximateTime(int) {}
};
}  // namespace sync_policies
}  // namespace message_filters
