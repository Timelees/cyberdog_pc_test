#pragma once
#include <memory>
#include <string>

namespace YAML
{
class Node
{
public:
  template<typename T>
  Node operator[](const T &) const { return Node(); }

  template<typename T>
  Node & operator[](const T &) { return *this; }

  bool IsMap() const { return true; }
  bool IsSequence() const { return false; }
  bool IsScalar() const { return false; }
};

template<typename T>
struct convert
{
  static Node encode(const T &) { return Node(); }
  static bool decode(const Node &, T &) { return true; }
};
}  // namespace YAML
