#pragma once
#include <memory>

namespace std_srvs
{
namespace srv
{
struct SetBool
{
  struct Request
  {
    bool data {false};
  };
  struct Response
  {
    bool success {false};
    std::string message;
  };
};
}  // namespace srv
}  // namespace std_srvs
