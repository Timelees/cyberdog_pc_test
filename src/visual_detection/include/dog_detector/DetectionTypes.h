#pragma once

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace dog_detector {

// 单个模型检测框，坐标仍然是图像像素坐标。
struct Detection {
  int class_id = -1;
  std::string class_name;
  float confidence = 0.0F;
  cv::Rect2f box;

  // 后续把检测框转成点时，统一使用框中心作为目标位置。
  cv::Point2f center() const {
    return {box.x + box.width * 0.5F, box.y + box.height * 0.5F};
  }
};

// 对外输出的点坐标，坐标以图像像素表示。
struct DetectionPoint {
  int class_id = -1;
  std::string class_name;
  float confidence = 0.0F;
  cv::Point2f image_point;
  // Ground-plane projection pixel. For soccer this is the bottom-center of the
  // detection box, which is a better approximation of the contact point than
  // projecting the box center through an assumed ball-center height.
  std::optional<cv::Point2f> ground_contact_point;
  // true 表示 soccer 当前帧被遮挡或未检出，坐标来自轨迹预测。
  bool predicted = false;
};

// 单帧检测结果，既保留原始检测框，也保留转换后的点坐标。
struct FrameResult {
  int frame_index = 0;
  double timestamp_sec = 0.0;
  std::vector<Detection> detections;
  std::vector<DetectionPoint> points;
  // 当前帧最可信的 soccer 点，可能是真实检测，也可能是预测结果。
  std::optional<DetectionPoint> soccer;
};

// 整个视频的检测结果，用于 CLI 文件输出或后处理。
struct VideoResult {
  std::string video_path;
  int frame_count = 0;
  double fps = 0.0;
  std::vector<FrameResult> frames;
};

}  // namespace dog_detector
