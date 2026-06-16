#pragma once
#include <functional>
#include <memory>
#include <string>

namespace bond
{
class Bond
{
public:
  Bond(const std::string &, const std::string &, bool = true) {}
  void start() {}
  void setFormedCallback(std::function<void()>) {}
  void setBrokenCallback(std::function<void()>) {}
  void breakBond() {}
};
}  // namespace bond
