#include "dog_detector/SoccerGlobalConverter.h"

#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace dog_detector {
namespace {

bool sameFamily(std::string configured, std::string detected) {
  constexpr const char* prefix = "tag";
  if (configured.rfind(prefix, 0) == 0) configured.erase(0, 3);
  if (detected.rfind(prefix, 0) == 0) detected.erase(0, 3);
  return configured == detected;
}

tf2::Transform makeTransform(double x, double y, double z,
                              double qx, double qy, double qz, double qw) {
  tf2::Quaternion rotation(qx, qy, qz, qw);
  if (rotation.length2() < 1e-12) {
    throw std::runtime_error("tag1_in_tag_global quaternion must be non-zero");
  }
  rotation.normalize();
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(x, y, z));
  transform.setRotation(rotation);
  return transform;
}

}  // namespace

SoccerGlobalConverter::SoccerGlobalConverter() : Node("soccer_global_converter") {
  calibration_valid_ = declare_parameter<bool>("calibration_valid", false);
  tag_id_ = declare_parameter<int>("tag_id", 1);
  tag_family_ = declare_parameter<std::string>("tag_family", "36h11");
  global_frame_ = declare_parameter<std::string>("global_frame", "tag_global");
  tag_size_m_ = declare_parameter<double>("tag_size_m", 0.0);
  ground_z_m_ = declare_parameter<double>("ground_z_m", 0.0);
  soccer_radius_m_ = declare_parameter<double>("soccer_radius_m", 0.11);
  use_tag_plane_homography_ =
      declare_parameter<bool>("use_tag_plane_homography", true);
  tag_calibration_samples_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("tag_calibration_samples", 30)));
  const auto axis_mapping = declare_parameter<std::vector<double>>(
      "output_axis_mapping", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  if (axis_mapping.size() != output_axis_mapping_.size()) {
    throw std::runtime_error("output_axis_mapping must contain 9 row-major values");
  }
  std::copy(axis_mapping.begin(), axis_mapping.end(), output_axis_mapping_.begin());
  const auto tag_x = declare_parameter<double>("tag1_in_tag_global.x", 0.0);
  const auto tag_y = declare_parameter<double>("tag1_in_tag_global.y", 0.0);
  const auto tag_z = declare_parameter<double>("tag1_in_tag_global.z", 0.0);
  const auto tag_qx = declare_parameter<double>("tag1_in_tag_global.qx", 0.0);
  const auto tag_qy = declare_parameter<double>("tag1_in_tag_global.qy", 0.0);
  const auto tag_qz = declare_parameter<double>("tag1_in_tag_global.qz", 0.0);
  const auto tag_qw = declare_parameter<double>("tag1_in_tag_global.qw", 1.0);
  global_T_tag_ = makeTransform(tag_x, tag_y, tag_z, tag_qx, tag_qy, tag_qz, tag_qw);

  const auto tag_topic = declare_parameter<std::string>("tag_detections_topic", "/field_camera/tag_detections");
  const auto soccer_topic = declare_parameter<std::string>("soccer_pixel_topic", "/visual_detection/soccer");
  const auto camera_info_topic = declare_parameter<std::string>("camera_info_topic", "/field_camera/camera_info");
  const auto output_topic = declare_parameter<std::string>("soccer_global_topic", "/field/soccer_global");

  tag_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      tag_topic, rclcpp::SensorDataQoS(),
      [this](apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr message) {
        onDetections(message);
      });
  soccer_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      soccer_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
        onSoccer(message);
      });
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        onCameraInfo(message);
      });
  global_soccer_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(output_topic, 10);

  if (!calibration_valid_) {
    RCLCPP_WARN(get_logger(), "global conversion disabled until calibration_valid is true");
  }
  if (tag_size_m_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "tag_size_m must be set to the measured tag edge length in meters");
  }
  if (soccer_radius_m_ < 0.0) {
    RCLCPP_WARN(get_logger(), "soccer_radius_m is negative; using zero");
    soccer_radius_m_ = 0.0;
  }
  RCLCPP_INFO(get_logger(), "soccer radius compensation: %.3f m", soccer_radius_m_);
  RCLCPP_INFO(get_logger(), "ground projection method: %s",
              use_tag_plane_homography_ ? "Tag1 plane homography" : "ball-center ray");
  RCLCPP_INFO(get_logger(),
              "waiting for the first valid Tag%d detection to initialize the fixed camera pose",
              tag_id_);
}

void SoccerGlobalConverter::onDetections(
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& message) {
  // The field camera and Tag1 are fixed. Once their relative pose has been
  // solved, keep it even when a robot later occludes the tag.
  if (global_T_camera_) return;
  if (!calibration_valid_ || tag_size_m_ <= 0.0 || !camera_info_) return;
  const apriltag_msgs::msg::AprilTagDetection* selected = nullptr;
  for (const auto& detection : message->detections) {
    if (detection.id == tag_id_ && sameFamily(detection.family, tag_family_)) {
      selected = &detection;
      break;
    }
  }
  if (!selected) return;

  const auto& k = camera_info_->k;
  if (k[0] <= 0.0 || k[4] <= 0.0) return;
  const double half = tag_size_m_ * 0.5;
  const std::vector<cv::Point3d> object_points{
      {-half, -half, 0.0}, {half, -half, 0.0},
      {half, half, 0.0}, {-half, half, 0.0}};
  std::array<cv::Point2d, 4> sample;
  for (std::size_t i = 0; i < sample.size(); ++i) {
    sample[i] = cv::Point2d(selected->corners[i].x, selected->corners[i].y);
  }
  tag_corner_samples_.push_back(sample);
  if (static_cast<int>(tag_corner_samples_.size()) < tag_calibration_samples_) {
    return;
  }
  std::vector<cv::Point2d> image_points(4, cv::Point2d(0.0, 0.0));
  for (const auto& corners : tag_corner_samples_) {
    for (std::size_t i = 0; i < corners.size(); ++i) {
      image_points[i] += corners[i];
    }
  }
  for (auto& corner : image_points) {
    corner *= 1.0 / static_cast<double>(tag_corner_samples_.size());
  }

  // The PnP tag frame uses +Y toward the tag image bottom and +Z away from
  // the visible face. The configured physical Tag1 frame flips Y and Z so a
  // horizontal, upward-facing tag has +Z up. Map rectified image pixels
  // directly to that physical ground plane. This avoids amplifying the small
  // pitch/roll error of a distant, single-tag PnP estimate.
  const std::vector<cv::Point2d> physical_tag_points{
      {-half, half}, {half, half}, {half, -half}, {-half, -half}};
  tag_plane_from_image_ = cv::findHomography(image_points, physical_tag_points, 0);
  if (tag_plane_from_image_.empty()) {
    RCLCPP_WARN(get_logger(), "could not initialize Tag1 ground-plane homography");
    return;
  }

  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7], k[8]);
  cv::Mat distortion(camera_info_->d, true);
  cv::Mat rvec, tvec;
  if (!cv::solvePnP(object_points, image_points, camera_matrix, distortion,
                    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "could not solve Tag1 pose");
    return;
  }
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  tf2::Matrix3x3 basis(
      rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
      rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
      rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
  tf2::Transform camera_T_tag;
  const tf2::Matrix3x3 raw_to_physical_tag1(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
  camera_T_tag.setBasis(basis * raw_to_physical_tag1);
  camera_T_tag.setOrigin(tf2::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

  global_T_camera_ = global_T_tag_ * camera_T_tag.inverse();
  const auto& origin = global_T_camera_->getOrigin();
  RCLCPP_INFO(get_logger(),
              "Tag%d initialized from %zu averaged detections and the same CameraInfo used by apriltag_ros "
              "(fx=%.2f, fy=%.2f); fixed camera pose in %s: [x=%.3f, y=%.3f, z=%.3f]. "
              "Later tag loss will not stop conversion",
              tag_id_, tag_corner_samples_.size(), k[0], k[4], global_frame_.c_str(),
              origin.x(), origin.y(), origin.z());
  tryPublish();
}

void SoccerGlobalConverter::onSoccer(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr& message) {
  soccer_pixel_ = message;
  soccer_stamp_ = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type());
  tryPublish();
}

void SoccerGlobalConverter::onCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& message) {
  const bool first_valid_info = !camera_info_ && message->k[0] > 0.0 && message->k[4] > 0.0;
  camera_info_ = message;
  if (first_valid_info) {
    RCLCPP_INFO(get_logger(), "camera intrinsics received (fx=%.2f, fy=%.2f)",
                message->k[0], message->k[4]);
  }
}

void SoccerGlobalConverter::tryPublish() {
  if (!calibration_valid_ || !global_T_camera_ || !soccer_pixel_ || !camera_info_) return;
  if (last_published_soccer_stamp_ == soccer_stamp_) return;

  tf2::Vector3 point;
  if (use_tag_plane_homography_ && !tag_plane_from_image_.empty()) {
    std::vector<cv::Point2d> image_pixel{{soccer_pixel_->point.x, soccer_pixel_->point.y}};
    std::vector<cv::Point2d> tag_point;
    cv::perspectiveTransform(image_pixel, tag_point, tag_plane_from_image_);
    if (tag_point.empty() || !std::isfinite(tag_point.front().x) ||
        !std::isfinite(tag_point.front().y)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "invalid Tag1 homography projection");
      return;
    }
    point = global_T_tag_ * tf2::Vector3(tag_point.front().x, tag_point.front().y, 0.0);
  } else {
    const auto& k = camera_info_->k;
    if (k[0] <= 0.0 || k[4] <= 0.0) return;
    const std::vector<cv::Point2d> pixel{{soccer_pixel_->point.x, soccer_pixel_->point.y}};
    std::vector<cv::Point2d> normalized;
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7], k[8]);
    const cv::Mat distortion(camera_info_->d, true);
    cv::undistortPoints(pixel, normalized, camera_matrix, distortion);
    const tf2::Vector3 ray_camera(normalized.front().x, normalized.front().y, 1.0);
    const tf2::Vector3 camera_origin = global_T_camera_->getOrigin();
    const tf2::Vector3 ray_global = global_T_camera_->getBasis() * ray_camera;
    if (std::abs(ray_global.z()) < 1e-6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "soccer ray is parallel to the ground plane");
      return;
    }
    // Legacy mode: the incoming pixel is assumed to be the ball center.
    const double soccer_center_z = ground_z_m_ + soccer_radius_m_;
    const double scale = (soccer_center_z - camera_origin.z()) / ray_global.z();
    if (scale <= 0.0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "soccer ray intersects the ground behind the camera; check Tag1 pose and ground_z_m");
      return;
    }
    point = camera_origin + scale * ray_global;
  }

  geometry_msgs::msg::PointStamped output;
  output.header = soccer_pixel_->header;
  output.header.frame_id = global_frame_;
  output.point.x = output_axis_mapping_[0] * point.x() +
                   output_axis_mapping_[1] * point.y() +
                   output_axis_mapping_[2] * point.z();
  output.point.y = output_axis_mapping_[3] * point.x() +
                   output_axis_mapping_[4] * point.y() +
                   output_axis_mapping_[5] * point.z();
  const double point_z = use_tag_plane_homography_ ? point.z() : ground_z_m_;
  output.point.z = output_axis_mapping_[6] * point.x() +
                   output_axis_mapping_[7] * point.y() +
                   output_axis_mapping_[8] * point_z;
  global_soccer_pub_->publish(output);
  last_published_soccer_stamp_ = soccer_stamp_;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                       "soccer global position: [x=%.3f, y=%.3f, z=%.3f] %s",
                       output.point.x, output.point.y, output.point.z,
                       global_frame_.c_str());
}

}  // namespace dog_detector

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_detector::SoccerGlobalConverter>());
  rclcpp::shutdown();
  return 0;
}
