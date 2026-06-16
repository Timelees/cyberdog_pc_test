#pragma once
#include <memory>

namespace cv
{
template<typename T>
class Ptr
{
public:
  Ptr() : ptr_(nullptr) {}
  explicit Ptr(T * p) : ptr_(p) {}
  T * operator->() { return ptr_; }
  const T * operator->() const { return ptr_; }
  T & operator*() { return *ptr_; }
private:
  T * ptr_;
};

class Size
{
public:
  Size(int = 0, int = 0) {}
};

class Mat
{
public:
  Mat clone() const { return Mat(); }
  void copyTo(Mat &) const {}
};

template<typename T>
Ptr<T> makePtr()
{
  return Ptr<T>(new T());
}
}  // namespace cv
