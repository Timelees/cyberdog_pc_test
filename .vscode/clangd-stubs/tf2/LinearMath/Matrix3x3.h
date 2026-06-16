#pragma once
#include <tf2/LinearMath/Quaternion.h>

namespace tf2
{
class Matrix3x3
{
public:
  Matrix3x3() = default;
  explicit Matrix3x3(const Quaternion & q) { (void)q; }

  void getRPY(double & roll, double & pitch, double & yaw) const;
};
}  // namespace tf2
