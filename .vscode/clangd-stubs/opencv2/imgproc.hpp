#pragma once
#include <opencv2/core.hpp>

namespace cv
{
class CLAHE
{
public:
  void apply(const Mat &, Mat &) {}
};

inline Ptr<CLAHE> createCLAHE(double = 2.0, Size = Size(8, 8))
{
  return makePtr<CLAHE>();
}
}  // namespace cv
