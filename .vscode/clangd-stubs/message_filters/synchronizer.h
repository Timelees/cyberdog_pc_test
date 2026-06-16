#pragma once
#include <message_filters/subscriber.h>

namespace message_filters
{
template<typename Policy>
class Synchronizer
{
public:
  template<typename... Args>
  Synchronizer(Args &&...) {}

  template<typename CallbackT>
  Connection registerCallback(CallbackT)
  {
    return Connection();
  }
};
}  // namespace message_filters
