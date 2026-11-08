#pragma once

#include "dog_detector/DetectionTypes.h"
#include "dog_detector/SoccerTracker.h"
#include "dog_detector/YoloV8Detector.h"

#include <functional>
#include <optional>
#include <string>

namespace dog_detector {

// 视频检测管线配置，贯穿模型加载、后处理阈值、输出文件和 tracker 参数。
struct VideoPipelineConfig {
  std::string model_path = "models/best.onnx";
  std::string output_json_path;
  std::string output_video_path;
  float confidence_threshold = 0.25F;
  float nms_threshold = 0.45F;
  std::string device = "cuda";
  int cuda_device_id = 0;
  SoccerTrackerConfig tracker;
};

// 负责读取视频、调用 YOLO 检测、预测 soccer 并写出结果。
class VideoDetectionPipeline {
 public:
  // 服务端通过回调拿到每一帧结果，从而做到边检测边发送。
  using FrameCallback = std::function<void(const FrameResult&)>;

  explicit VideoDetectionPipeline(VideoPipelineConfig config);

  // 处理完整视频；如果提供 frame_callback，会在每帧处理完成后立即调用。
  VideoResult process(const std::string& video_path, const FrameCallback& frame_callback = {});

  FrameResult processImage(const cv::Mat& frame, int frame_index, double timestamp_sec);

  // 绘制检测预览，供 ROS 节点显示实时结果。
  void drawPreview(cv::Mat* frame, const FrameResult& result) const;

 private:
  // 处理单帧足球检测，并更新图像坐标系中的 soccer tracker。
  FrameResult processFrame(const cv::Mat& frame, int frame_index, double timestamp_sec);
  // 可选输出标注视频时使用，把框、点和预测结果画到帧上。
  void drawFrame(cv::Mat* frame, const FrameResult& result) const;
  // 将完整视频结果写成 JSON 文件。
  void writeJson(const VideoResult& result) const;

  VideoPipelineConfig config_;
  YoloV8Detector detector_;
  SoccerTracker soccer_tracker_;
  std::optional<cv::Point2f> last_soccer_contact_offset_;
};

}  // namespace dog_detector
