#pragma once

namespace tf2
{
class Quaternion
{
public:
  Quaternion() = default;
  Quaternion(double x, double y, double z, double w) : x_(x), y_(y), z_(z), w_(w) {}

  void setRPY(double roll, double pitch, double yaw);
  void getRPY(double & roll, double & pitch, double & yaw) const;
  void setValue(double x, double y, double z, double w);
  void normalize();
  double length2() const { return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_; }

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double w() const { return w_; }

private:
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double w_{1.0};
};
}  // namespace tf2
