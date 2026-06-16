#pragma once
#include <type_traits>

namespace rosidl_runtime_cpp
{
template<typename T>
struct has_fixed_size : std::false_type
{};

template<typename T>
struct has_bounded_size : std::false_type
{};

template<typename T>
struct is_message : std::true_type
{};
}  // namespace rosidl_runtime_cpp
