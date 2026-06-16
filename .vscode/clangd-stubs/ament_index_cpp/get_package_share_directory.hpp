#pragma once
#include <string>

namespace ament_index_cpp
{
inline std::string get_package_share_directory(const std::string & package_name)
{
  (void)package_name;
  return "/tmp/package_share";
}
}  // namespace ament_index_cpp
