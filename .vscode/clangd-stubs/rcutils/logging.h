#pragma once

typedef int rcutils_ret_t;
#define RCUTILS_RET_OK 0
#define RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID 1

inline rcutils_ret_t rcutils_logging_severity_level_from_string(
  const char *, void *, int *)
{
  return RCUTILS_RET_OK;
}

inline rcutils_ret_t rcutils_logging_set_logger_level(const char *, int)
{
  return RCUTILS_RET_OK;
}

inline const char * rcutils_get_error_string() { return ""; }
inline void rcutils_reset_error() {}
