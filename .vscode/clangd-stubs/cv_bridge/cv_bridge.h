#pragma once
#include <memory>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>

namespace cv_bridge
{
class Exception : public std::runtime_error
{
public:
  explicit Exception(const std::string & what)
  : std::runtime_error(what)
  {}
};

class CvImage
{
public:
  cv::Mat image;
  std::string encoding;
  using Ptr = std::shared_ptr<CvImage>;
  using ConstPtr = std::shared_ptr<const CvImage>;
};

inline CvImage::Ptr toCvCopy(const sensor_msgs::msg::Image::ConstSharedPtr &)
{
  return std::make_shared<CvImage>();
}

inline CvImage::ConstPtr toCvShare(
  const sensor_msgs::msg::Image::ConstSharedPtr &, const std::string & = "")
{
  return std::make_shared<CvImage>();
}
}  // namespace cv_bridge
