#pragma once
#include <iostream>
#include <sstream>

struct LogMessageVoidify
{
  void operator&(std::ostream &) const {}
};

class LogMessage
{
public:
  LogMessage() {}
  std::ostream & stream() { return std::cerr; }
};

#define LOG(severity) LogMessage().stream()
#define VLOG(level) if (false) LogMessage().stream()
#define LOG_IF(severity, condition) if (condition) LogMessage().stream()
#define VLOG_IF(level, condition) if (false) LogMessage().stream()
#define CHECK(condition) if (!(condition)) LogMessage().stream()
#define DCHECK(condition) if (false) LogMessage().stream()
#define CHECK_NOTNULL(ptr) (ptr)
#define CHECK_EQ(a, b) if ((a) != (b)) LogMessage().stream()
#define CHECK_NE(a, b) if ((a) == (b)) LogMessage().stream()
#define CHECK_LT(a, b) if (!((a) < (b))) LogMessage().stream()
#define CHECK_LE(a, b) if (!((a) <= (b))) LogMessage().stream()
#define CHECK_GT(a, b) if (!((a) > (b))) LogMessage().stream()
#define CHECK_GE(a, b) if (!((a) >= (b))) LogMessage().stream()
#define CHECK_NEAR(a, b, tolerance) if (false) LogMessage().stream()

#define LOG_DEBUG_STREAM(msg) ((void)0)
#define LOG_INFO_STREAM(msg) ((void)0)
#define LOG_WARN_STREAM(msg) ((void)0)
#define LOG_ERROR_STREAM(msg) ((void)0)
