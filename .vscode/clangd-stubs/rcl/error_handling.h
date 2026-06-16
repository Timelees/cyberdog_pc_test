#pragma once
#include <cstddef>

struct rcutils_error_string_t
{
  char str[256];
};

inline rcutils_error_string_t rcl_get_error_string() { return {}; }
inline void rcl_reset_error() {}
