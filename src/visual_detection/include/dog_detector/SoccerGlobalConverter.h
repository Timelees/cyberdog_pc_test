#pragma once

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Transform.h>
#include <opencv2/core.hpp>

#include <optional>
#include <array>
#include <vector>
#include <array>

namespace dog_detector {

class SoccerGlobalConverter final : public rclcpp::Node {
 public:
  SoccerGlobalConverter();

 private:
  void onDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& message);
  void onSoccer(const geometry_msgs::msg::PointStamped::ConstSharedPtr& message);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message);
  void tryPublish();

  int tag_id_{1};
  std::string tag_family_{"36h11"};
  std::string global_frame_{"tag_global"};
  double tag_size_m_{0.0};
  double ground_z_m_{0.0};
  double soccer_radius_m_{0.11};
  bool use_tag_plane_homography_{true};
  std::array<double, 9> output_axis_mapping_{{1.0, 0.0, 0.0,
                                             0.0, 1.0, 0.0,
                                             0.0, 0.0, 1.0}};
  int tag_calibration_samples_{30};
  bool calibration_valid_{false};
  tf2::Transform global_T_tag_;
  std::optional<tf2::Transform> global_T_camera_;
  cv::Mat tag_plane_from_image_;
  std::vector<std::array<cv::Point2d, 4>> tag_corner_samples_;
  rclcpp::Time soccer_stamp_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  geometry_msgs::msg::PointStamped::ConstSharedPtr soccer_pixel_;
  rclcpp::Time last_published_soccer_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr soccer_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr global_soccer_pub_;
};

}  // namespace dog_detector
