#pragma once
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_ERROR(...) ((void)0)
#define RCLCPP_WARN(...) ((void)0)
#define RCLCPP_DEBUG(...) ((void)0)
#define RCLCPP_FATAL(...) ((void)0)
#define RCLCPP_INFO_STREAM(...) ((void)0)
#define RCLCPP_ERROR_STREAM(...) ((void)0)
#define RCLCPP_WARN_STREAM(...) ((void)0)
#define RCLCPP_DEBUG_STREAM(...) ((void)0)
#define RCLCPP_FATAL_STREAM(...) ((void)0)
#define RCLCPP_INFO_ONCE(...) ((void)0)
#define RCLCPP_ERROR_ONCE(...) ((void)0)
#define RCLCPP_WARN_STREAM_ONCE(...) ((void)0)
#define RCLCPP_INFO_STREAM_ONCE(...) ((void)0)
#define RCLCPP_ERROR_STREAM_ONCE(...) ((void)0)
#define RCLCPP_INFO_THROTTLE(...) ((void)0)
#define RCLCPP_ERROR_THROTTLE(...) ((void)0)
#define RCLCPP_WARN_THROTTLE(...) ((void)0)
#define RCLCPP_DEBUG_THROTTLE(...) ((void)0)
#define RCLCPP_FATAL_THROTTLE(...) ((void)0)
#define RCLCPP_INFO_STREAM_THROTTLE(...) ((void)0)
#define RCLCPP_ERROR_STREAM_THROTTLE(...) ((void)0)
#define RCLCPP_WARN_STREAM_THROTTLE(...) ((void)0)
#define RCLCPP_DEBUG_STREAM_THROTTLE(...) ((void)0)
#define RCLCPP_FATAL_STREAM_THROTTLE(...) ((void)0)

#define RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT 1
#define RMW_QOS_POLICY_RELIABILITY_RELIABLE 2

namespace rclcpp
{
namespace node_interfaces
{
class NodeBaseInterface
{
public:
  using SharedPtr = std::shared_ptr<NodeBaseInterface>;
};
}  // namespace node_interfaces

class Logger
{
public:
  Logger() = default;
  explicit Logger(const char *) {}
};

inline Logger get_logger(const char * name = "clangd")
{
  (void)name;
  return Logger();
}

inline Logger get_logger(const std::string & name)
{
  return get_logger(name.c_str());
}

class ParameterValue
{
public:
  ParameterValue() = default;
  template<typename T>
  explicit ParameterValue(const T &) {}
};

enum ParameterType
{
  PARAMETER_BOOL,
  PARAMETER_STRING,
  PARAMETER_INTEGER,
  PARAMETER_DOUBLE
};

class Parameter
{
public:
  ParameterType get_type() const { return PARAMETER_BOOL; }
  bool as_bool() const { return false; }
  std::string as_string() const { return ""; }
  int as_int() const { return 0; }
  double as_double() const { return 0.0; }
  std::string as_gstring() const { return ""; }
};

struct rmw_qos_profile_t
{};

class Time
{
public:
  Time() = default;
  template<typename StampT>
  explicit Time(const StampT &) {}
  int64_t nanoseconds() const { return 0; }
  double seconds() const { return 0.0; }
};

inline Time operator-(const Time & lhs, const Time & rhs)
{
  (void)rhs;
  return lhs;
}

class Clock
{
public:
  using SharedPtr = std::shared_ptr<Clock>;
};

class NodeOptions
{};

struct KeepLast
{
  explicit KeepLast(int depth) : depth_(depth) {}
  int depth_{10};
};

class QoS
{
public:
  QoS(int = 10) {}
  explicit QoS(const KeepLast & keep_last) : depth_(keep_last.depth_) {}
  QoS & reliable() { return *this; }
  QoS & best_effort() { return *this; }
  QoS & reliability(int) { return *this; }
  QoS & durability(int) { return *this; }

private:
  int depth_{10};
};

class SensorDataQoS : public QoS
{
public:
  SensorDataQoS() : QoS(5) {}
  rmw_qos_profile_t get_rmw_qos_profile() const { return {}; }
};

template<typename MsgT>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<MsgT>>;
  using ConstSharedPtr = std::shared_ptr<const Subscription<MsgT>>;
};

template<typename SrvT>
class Service
{
public:
  using SharedPtr = std::shared_ptr<Service<SrvT>>;
};

template<typename SrvT>
class Client
{
public:
  using SharedPtr = std::shared_ptr<Client<SrvT>>;
  using SharedFuture = std::shared_future<std::shared_ptr<typename SrvT::Response>>;

  SharedFuture async_send_request(std::shared_ptr<typename SrvT::Request>)
  {
    return SharedFuture();
  }

  template<typename CallbackT>
  SharedFuture async_send_request(
    std::shared_ptr<typename SrvT::Request>, CallbackT)
  {
    return SharedFuture();
  }

  bool wait_for_service(std::chrono::nanoseconds = std::chrono::seconds(0))
  {
    return true;
  }
};

template<typename MsgT>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<MsgT>>;

  void publish(const MsgT & message) { (void)message; }
  void publish(std::unique_ptr<MsgT> message) { (void)message; }
};

class PublisherBase
{
public:
  using SharedPtr = std::shared_ptr<PublisherBase>;
};

class SubscriptionBase
{
public:
  using SharedPtr = std::shared_ptr<SubscriptionBase>;
};

class TimerBase
{
public:
  using SharedPtr = std::shared_ptr<TimerBase>;
};

enum class CallbackGroupType
{
  MutuallyExclusive,
  Reentrant
};

class CallbackGroup
{
public:
  using SharedPtr = std::shared_ptr<CallbackGroup>;
};

struct SubscriptionOptions
{
  CallbackGroup::SharedPtr callback_group;
};

class Node
{
public:
  using SharedPtr = std::shared_ptr<Node>;

  explicit Node(const std::string &, const NodeOptions & = NodeOptions()) {}

  const char * get_name() const { return "clangd_node"; }
  const char * get_namespace() const { return ""; }
  Logger get_logger() const { return rclcpp::get_logger(); }
  Time now() const { return Time(); }

  node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return nullptr;
  }

  template<typename T>
  T declare_parameter(const std::string &, const T & default_value = T())
  {
    return default_value;
  }

  void declare_parameter(const std::string &) {}

  template<typename T>
  bool get_parameter(const std::string &, T &) { return true; }

  Parameter get_parameter(const std::string &) { return Parameter(); }

  bool has_parameter(const std::string &) { return false; }

  CallbackGroup::SharedPtr create_callback_group(CallbackGroupType)
  {
    return nullptr;
  }

  template<typename MsgT>
  typename Publisher<MsgT>::SharedPtr create_publisher(const std::string &, const QoS &)
  {
    return nullptr;
  }

  template<typename MsgT, typename CallbackT>
  typename Subscription<MsgT>::SharedPtr create_subscription(
    const std::string &, const QoS &, CallbackT,
    const SubscriptionOptions & = SubscriptionOptions())
  {
    return nullptr;
  }

  template<typename CallbackT>
  TimerBase::SharedPtr create_wall_timer(
    std::chrono::nanoseconds, CallbackT, CallbackGroup::SharedPtr = nullptr)
  {
    return nullptr;
  }

  template<typename SrvT, typename CallbackT>
  typename Service<SrvT>::SharedPtr create_service(
    const std::string &, CallbackT)
  {
    return nullptr;
  }

  template<typename SrvT>
  typename Client<SrvT>::SharedPtr create_client(const std::string &)
  {
    return nullptr;
  }
};

inline Parameter get_parameter(const std::string &) { return Parameter(); }
inline bool ok() { return true; }
void shutdown();
void init(int, char **);
template<typename NodeT>
void spin(std::shared_ptr<NodeT>) {}
}  // namespace rclcpp
