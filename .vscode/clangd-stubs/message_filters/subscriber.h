#pragma once
namespace message_filters
{
template<typename T>
class Subscriber
{
public:
  template<typename... A>
  Subscriber(A &&...) {}
};

class Connection
{
public:
  void disconnect() {}
};
}  // namespace message_filters
