#pragma once

namespace rosidl_runtime_cpp
{
enum class MessageInitialization
{
  ALL = 0,
  SKIP = 1,
  ZERO = 2,
  DEFAULTS_ONLY = 3
};
}  // namespace rosidl_runtime_cpp
