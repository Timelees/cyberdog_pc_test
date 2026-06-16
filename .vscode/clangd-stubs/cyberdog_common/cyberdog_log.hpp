#pragma once
#include <rclcpp/rclcpp.hpp>

inline rclcpp::Clock::SharedPtr global_steady_clock = nullptr;

#define LOGGER_INSTANCE(name) \
  cyberdog::common::CyberdogLogger cyberdog_logger(name)

#define LOGGER_MAIN_INSTANCE(name) LOGGER_INSTANCE(name)

#define DEBUG(...) ((void)0)
#define INFO(...) ((void)0)
#define WARN(...) ((void)0)
#define ERROR(...) ((void)0)
#define FATAL(...) ((void)0)
#define DEBUG_STREAM(...) ((void)0)
#define INFO_STREAM(...) ((void)0)
#define WARN_STREAM(...) ((void)0)
#define ERROR_STREAM(...) ((void)0)
#define FATAL_STREAM(...) ((void)0)
#define DEBUG_ONCE(...) ((void)0)
#define INFO_ONCE(...) ((void)0)
#define WARN_ONCE(...) ((void)0)
#define ERROR_ONCE(...) ((void)0)
#define FATAL_ONCE(...) ((void)0)
#define DEBUG_STREAM_ONCE(...) ((void)0)
#define INFO_STREAM_ONCE(...) ((void)0)
#define WARN_STREAM_ONCE(...) ((void)0)
#define ERROR_STREAM_ONCE(...) ((void)0)
#define FATAL_STREAM_ONCE(...) ((void)0)
#define DEBUG_MILLSECONDS(...) ((void)0)
#define INFO_MILLSECONDS(...) ((void)0)
#define WARN_MILLSECONDS(...) ((void)0)
#define ERROR_MILLSECONDS(...) ((void)0)
#define FATAL_MILLSECONDS(...) ((void)0)
#define DEBUG_STREAM_MILLSECONDS(...) ((void)0)
#define INFO_STREAM_MILLSECONDS(...) ((void)0)
#define WARN_STREAM_MILLSECONDS(...) ((void)0)
#define ERROR_STREAM_MILLSECONDS(...) ((void)0)
#define FATAL_STREAM_MILLSECONDS(...) ((void)0)

namespace cyberdog
{
namespace common
{
class CyberdogLogger
{
public:
  explicit CyberdogLogger(const char *) {}
};
}  // namespace common
}  // namespace cyberdog
