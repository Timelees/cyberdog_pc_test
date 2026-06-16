#pragma once
#include <memory>
#include <string>

namespace cyberdog_visions_interfaces
{
namespace srv
{
struct FinishMap
{
  struct Request
  {
    bool finish {false};
    std::string map_name;
  };
  struct Response
  {
    bool success {false};
    std::string message;
  };
};
}  // namespace srv
}  // namespace cyberdog_visions_interfaces
