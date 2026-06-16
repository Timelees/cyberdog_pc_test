#pragma once
#include <tf2/LinearMath/Quaternion.h>

namespace tf2
{
class Vector3
{
public:
  Vector3() = default;
  Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

private:
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
};

class Transform
{
public:
  Transform() = default;

  void setOrigin(const Vector3 & origin) { origin_ = origin; }
  void setRotation(const Quaternion & rotation) { rotation_ = rotation; }

  const Vector3 & getOrigin() const { return origin_; }
  const Quaternion & getRotation() const { return rotation_; }
  Transform inverse() const { return *this; }

private:
  Vector3 origin_;
  Quaternion rotation_;
};

inline Transform operator*(const Transform & lhs, const Transform & rhs)
{
  (void)rhs;
  return lhs;
}
}  // namespace tf2
