#pragma once
#include <cstddef>
#include <vector>

namespace rosidl_runtime_cpp
{
template<typename T, std::size_t MaxSize = 0>
class BoundedVector : public std::vector<T>
{};
}  // namespace rosidl_runtime_cpp
