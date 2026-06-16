#pragma once
#include <memory>

namespace pcl
{
template<typename T>
class PointCloud
{
public:
  using Ptr = std::shared_ptr<PointCloud<T>>;
};
}  // namespace pcl
