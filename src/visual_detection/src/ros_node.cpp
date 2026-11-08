#include "dog_detector/ResultJson.h"
#include "dog_detector/VideoDetectionPipeline.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <cstdlib>
#include <sstream>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>

namespace dog_detector {
namespace {

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r");
  return value.substr(first, last - first + 1);
}

std::string yamlValue(const std::unordered_map<std::string, std::string>& values,
                     const std::string& key,
                     const std::string& fallback) {
  const auto found = values.find(key);
  if (found == values.end()) return fallback;
  std::string value = found->second;
  const auto comment = value.find(" #");
  if (comment != std::string::npos) value = value.substr(0, comment);
  value = trim(value);
  if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
                            (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

std::vector<double> yamlDoubleVector(const std::unordered_map<std::string, std::string>& values,
                                     const std::string& key,
                                     std::vector<double> fallback) {
  const auto found = values.find(key);
  if (found == values.end()) return fallback;
  std::string value = found->second;
  const auto comment = value.find(" #");
  if (comment != std::string::npos) value = value.substr(0, comment);
  const auto first = value.find('[');
  const auto last = value.find(']');
  if (first == std::string::npos || last == std::string::npos || last < first) return fallback;
  std::istringstream input(value.substr(first + 1, last - first - 1));
  std::vector<double> parsed;
  std::string token;
  while (std::getline(input, token, ',')) {
    try {
      parsed.push_back(std::stod(trim(token)));
    } catch (const std::exception&) {
      return fallback;
    }
  }
  return parsed.empty() ? fallback : parsed;
}

std::unordered_map<std::string, std::string> loadParameterDefaults(const std::string& path) {
  std::unordered_map<std::string, std::string> values;
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line.front() == '#') continue;
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const auto key = trim(line.substr(0, colon));
    if (key == "ros__parameters" || key.empty()) continue;
    values[key] = trim(line.substr(colon + 1));
  }
  return values;
}

}  // namespace

class VisualDetectionNode : public rclcpp::Node {
 public:
  VisualDetectionNode() : Node("visual_detection") {
    const auto share = ament_index_cpp::get_package_share_directory("visual_detection");
    const auto defaults = loadParameterDefaults(share + "/config/visual_detection.yaml");
    VideoPipelineConfig config;
    config.model_path = declare_parameter<std::string>("model_path", share + "/models/best.onnx");
    config.device = declare_parameter<std::string>("device", yamlValue(defaults, "device", "cuda"));
    config.cuda_device_id = std::stoi(yamlValue(defaults, "cuda_device_id", "0"));
    config.cuda_device_id = declare_parameter<int>("cuda_device_id", config.cuda_device_id);
    config.confidence_threshold = static_cast<float>(std::stod(yamlValue(defaults, "confidence_threshold", "0.25")));
    config.confidence_threshold = static_cast<float>(declare_parameter<double>("confidence_threshold", config.confidence_threshold));
    config.nms_threshold = static_cast<float>(std::stod(yamlValue(defaults, "nms_threshold", "0.45")));
    config.nms_threshold = static_cast<float>(declare_parameter<double>("nms_threshold", config.nms_threshold));

    video_stream_ = declare_parameter<std::string>("video_stream", yamlValue(defaults, "video_stream", ""));
    username_ = declare_parameter<std::string>("username", yamlValue(defaults, "username", ""));
    password_ = declare_parameter<std::string>("password", yamlValue(defaults, "password", ""));
    preview_ = declare_parameter<bool>("preview", yamlValue(defaults, "preview", "true") == "true");
    const auto image_topic = declare_parameter<std::string>("image_topic", yamlValue(defaults, "image_topic", "/image_raw"));
    const auto result_topic = declare_parameter<std::string>("result_topic", yamlValue(defaults, "result_topic", "/visual_detection/result"));
    camera_image_topic_ = declare_parameter<std::string>("camera_image_topic", "/field_camera/image_rect");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/field_camera/camera_info");
    camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "field_camera_optical_frame");
    camera_fx_ = declare_parameter<double>("camera_fx", std::stod(yamlValue(defaults, "camera_fx", "0.0")));
    camera_fy_ = declare_parameter<double>("camera_fy", std::stod(yamlValue(defaults, "camera_fy", "0.0")));
    camera_cx_ = declare_parameter<double>("camera_cx", std::stod(yamlValue(defaults, "camera_cx", "0.0")));
    camera_cy_ = declare_parameter<double>("camera_cy", std::stod(yamlValue(defaults, "camera_cy", "0.0")));
    camera_distortion_ = declare_parameter<std::vector<double>>(
        "camera_distortion", yamlDoubleVector(defaults, "camera_distortion", std::vector<double>(5, 0.0)));
    max_grab_frames_ = declare_parameter<int>("max_grab_frames", std::stoi(yamlValue(defaults, "max_grab_frames", "8")));
    ffmpeg_capture_options_ = declare_parameter<std::string>(
        "ffmpeg_capture_options",
        yamlValue(defaults, "ffmpeg_capture_options", "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0"));

    pipeline_ = std::make_unique<VideoDetectionPipeline>(std::move(config));
    result_publisher_ = create_publisher<std_msgs::msg::String>(result_topic, 10);
    camera_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(camera_image_topic_, rclcpp::SensorDataQoS());
    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, rclcpp::SensorDataQoS());
    soccer_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("/visual_detection/soccer", 10);
    soccer_ground_contact_publisher_ =
        create_publisher<geometry_msgs::msg::PointStamped>(
            "/visual_detection/soccer_ground_contact", 10);

    if (!video_stream_.empty()) {
      openStream();
      stream_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                        [this]() { onStreamTimer(); });
      RCLCPP_INFO(get_logger(), "RTSP detection enabled; publishing results to %s",
                  result_topic.c_str());
    } else {
      image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
          image_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::Image::ConstSharedPtr message) { onImage(message); });
      RCLCPP_INFO(get_logger(), "image topic mode: subscribing to %s, publishing to %s",
                  image_topic.c_str(), result_topic.c_str());
    }
  }

  ~VisualDetectionNode() override {
    if (preview_) {
      cv::destroyWindow(preview_window_);
    }
  }

 private:
  static std::string withCredentials(const std::string& stream,
                                     const std::string& username,
                                     const std::string& password) {
    if (username.empty() || stream.find("@") != std::string::npos) {
      return stream;
    }
    const auto scheme_end = stream.find("://");
    if (scheme_end == std::string::npos) {
      return stream;
    }
    const auto insert_at = scheme_end + 3;
    return stream.substr(0, insert_at) + username + ":" + password + "@" +
           stream.substr(insert_at);
  }

  void openStream() {
    const auto url = withCredentials(video_stream_, username_, password_);
    // OpenCV's FFmpeg backend reads this before VideoCapture::open(). TCP avoids
    // RTP UDP reordering/loss, while nobuffer/low_delay prevent stale frames.
    if (!ffmpeg_capture_options_.empty()) {
      setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", ffmpeg_capture_options_.c_str(), 1);
    }
    capture_.release();
    if (!capture_.open(url, cv::CAP_FFMPEG)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "failed to open RTSP stream; retrying: %s", video_stream_.c_str());
      return;
    }
    capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    RCLCPP_INFO(get_logger(), "RTSP stream connected");
  }

  void publishFrame(const cv::Mat& frame, double timestamp) {
    cv::Mat calibrated_frame;
    cv::Mat output_camera_matrix;
    const bool has_distortion = std::any_of(camera_distortion_.begin(), camera_distortion_.end(),
                                            [](double value) { return std::abs(value) > 1e-12; });
    if (has_distortion && camera_fx_ > 0.0 && camera_fy_ > 0.0) {
      if (undistort_map1_.empty() || undistort_size_ != frame.size()) {
        const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            camera_fx_, 0.0, camera_cx_, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 1.0);
        // alpha=0 crops invalid border pixels while preserving a rectified image.
        rectified_camera_matrix_ = cv::getOptimalNewCameraMatrix(
            camera_matrix, cv::Mat(camera_distortion_), frame.size(), 0.0, frame.size());
        cv::initUndistortRectifyMap(camera_matrix, cv::Mat(camera_distortion_), cv::Mat(),
                                    rectified_camera_matrix_, frame.size(), CV_16SC2,
                                    undistort_map1_, undistort_map2_);
        undistort_size_ = frame.size();
      }
      output_camera_matrix = rectified_camera_matrix_;
      cv::remap(frame, calibrated_frame, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
    } else {
      calibrated_frame = frame;
      output_camera_matrix = (cv::Mat_<double>(3, 3) <<
          camera_fx_, 0.0, camera_cx_, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 1.0);
    }

    const auto stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
    auto camera_image = std_msgs::msg::Header();
    camera_image.stamp = stamp;
    camera_image.frame_id = camera_frame_id_;
    camera_image_publisher_->publish(
        *cv_bridge::CvImage(camera_image, "bgr8", calibrated_frame).toImageMsg());

    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = camera_image;
    camera_info.width = static_cast<uint32_t>(calibrated_frame.cols);
    camera_info.height = static_cast<uint32_t>(calibrated_frame.rows);
    camera_info.distortion_model = "plumb_bob";
    // The published image is already rectified, so report the output matrix and zero D.
    camera_info.d.assign(5, 0.0);
    camera_info.k = {
        output_camera_matrix.at<double>(0, 0), 0.0, output_camera_matrix.at<double>(0, 2),
        0.0, output_camera_matrix.at<double>(1, 1), output_camera_matrix.at<double>(1, 2),
        0.0, 0.0, 1.0};
    camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.p = {
        output_camera_matrix.at<double>(0, 0), 0.0, output_camera_matrix.at<double>(0, 2),
        0.0, 0.0, output_camera_matrix.at<double>(1, 1), output_camera_matrix.at<double>(1, 2),
        0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info_publisher_->publish(camera_info);

    const auto result = pipeline_->processImage(calibrated_frame, frame_index_++, timestamp);
    std_msgs::msg::String output;
    output.data = frameResultToJson(result);
    result_publisher_->publish(output);
    publishObjectPoints(result, timestamp);
    if (preview_) {
      cv::Mat display = calibrated_frame.clone();
      pipeline_->drawPreview(&display, result);
      cv::imshow(preview_window_, display);
      if (cv::waitKey(1) == 27) {
        RCLCPP_INFO(get_logger(), "preview requested shutdown");
        rclcpp::shutdown();
      }
    }
  }


  void publishObjectPoints(const FrameResult& result, double timestamp) {
    // Publish the best real detection, or the tracker's short prediction during
    // brief detector misses. This keeps the downstream global position stream
    // continuous without extrapolating indefinitely.
    if (result.soccer.has_value()) {
      const auto& soccer = *result.soccer;
      auto message = geometry_msgs::msg::PointStamped();
      message.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
      message.header.frame_id = "image";
      message.point.x = soccer.image_point.x;
      message.point.y = soccer.image_point.y;
      message.point.z = soccer.confidence;
      soccer_publisher_->publish(message);
      if (soccer.ground_contact_point.has_value()) {
        message.point.x = soccer.ground_contact_point->x;
        message.point.y = soccer.ground_contact_point->y;
        soccer_ground_contact_publisher_->publish(message);
      }
    }
  }

  void onStreamTimer() {
    if (!capture_.isOpened()) {
      openStream();
      return;
    }
    cv::Mat frame;
    bool received = false;
    const int grab_count = std::max(1, max_grab_frames_);
    // Drain frames accumulated while ONNX inference was running. The final
    // retrieved frame is the newest available frame, minimizing control lag.
    for (int i = 0; i < grab_count && capture_.grab(); ++i) {
      received = true;
    }
    if (!received || !capture_.retrieve(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "RTSP frame read failed; reconnecting");
      capture_.release();
      return;
    }
    try {
      publishFrame(frame, now().seconds());
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "frame detection failed: %s", error.what());
    }
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
    try {
      const auto image = cv_bridge::toCvShare(message, "bgr8");
      const double timestamp = static_cast<double>(message->header.stamp.sec) +
                               static_cast<double>(message->header.stamp.nanosec) * 1e-9;
      publishFrame(image->image, timestamp);
    } catch (const cv_bridge::Exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "image conversion failed: %s", error.what());
    } catch (const std::exception& error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "frame detection failed: %s", error.what());
    }
  }

  std::unique_ptr<VideoDetectionPipeline> pipeline_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr soccer_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      soccer_ground_contact_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::TimerBase::SharedPtr stream_timer_;
  cv::VideoCapture capture_;
  std::string video_stream_;
  std::string username_;
  std::string password_;
  std::string camera_image_topic_;
  cv::Size undistort_size_;
  cv::Mat undistort_map1_;
  cv::Mat undistort_map2_;
  cv::Mat rectified_camera_matrix_;
  std::string camera_info_topic_;
  std::string camera_frame_id_;
  double camera_fx_{0.0};
  double camera_fy_{0.0};
  double camera_cx_{0.0};
  double camera_cy_{0.0};
  std::vector<double> camera_distortion_;
  int max_grab_frames_{8};
  std::string ffmpeg_capture_options_;
  bool preview_ = true;
  const std::string preview_window_ = "visual_detection";
  int frame_index_ = 0;
};

}  // namespace dog_detector

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dog_detector::VisualDetectionNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("visual_detection"), "failed to start: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
