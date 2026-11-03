#include "dog_detector/VideoDetectionPipeline.h"

#include "dog_detector/DataConfig.h"
#include "dog_detector/ResultJson.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <utility>

namespace dog_detector {
namespace {

// 标注视频时使用固定颜色区分类别和预测点。
cv::Scalar colorForClass(const std::string& name, bool predicted) {
  if (predicted) {
    return {0, 255, 255};
  }
  if (name == "goal") {
    return {60, 180, 75};
  }
  if (name == "soccer") {
    return {0, 165, 255};
  }
  return {220, 90, 60};
}

// OpenCV 绘制 API 多数使用整数坐标，这里集中做四舍五入。
cv::Point roundedPoint(const cv::Point2f& point) {
  return {static_cast<int>(std::round(point.x)), static_cast<int>(std::round(point.y))};
}

// Rect2f 来自检测输出，绘制前转成 Rect。
cv::Rect roundedRect(const cv::Rect2f& rect) {
  return {static_cast<int>(std::round(rect.x)),
          static_cast<int>(std::round(rect.y)),
          static_cast<int>(std::round(rect.width)),
          static_cast<int>(std::round(rect.height))};
}

}  // namespace

VideoDetectionPipeline::VideoDetectionPipeline(VideoPipelineConfig config)
    : config_(std::move(config)),
      // 标签从 data.yaml 读取，确保模型类别和输出 JSON 中的名称一致。
      detector_(YoloV8Config{config_.model_path,
                             loadClassNames(config_.labels_path),
                             640,
                             640,
                             config_.confidence_threshold,
                             config_.nms_threshold,
                             config_.device,
                             config_.cuda_device_id,
                             "images",
                             "output0"}),
      soccer_tracker_(config_.tracker) {}

VideoResult VideoDetectionPipeline::process(const std::string& video_path,
                                            const FrameCallback& frame_callback) {
  cv::VideoCapture capture(video_path);
  if (!capture.isOpened()) {
    throw std::runtime_error("failed to open video: " + video_path);
  }

  VideoResult result;
  result.video_path = video_path;
  result.fps = capture.get(cv::CAP_PROP_FPS);
  if (result.fps <= 1e-6) {
    // 少数视频没有正确写入 fps，给一个保守默认值保证时间戳可计算。
    result.fps = 25.0;
  }

  // 如果指定了输出视频，边检测边写带标注的 mp4。
  const int width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
  const int height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
  cv::VideoWriter writer;
  if (!config_.output_video_path.empty()) {
    const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    writer.open(config_.output_video_path, fourcc, result.fps, cv::Size(width, height));
    if (!writer.isOpened()) {
      throw std::runtime_error("failed to open output video: " + config_.output_video_path);
    }
  }

  cv::Mat frame;
  int frame_index = 0;
  while (capture.read(frame)) {
    // 时间戳按帧号和 fps 推导，用于 tracker 估计速度。
    const double timestamp = frame_index / result.fps;
    FrameResult frame_result = processFrame(frame, frame_index, timestamp);
    if (writer.isOpened()) {
      cv::Mat annotated = frame.clone();
      drawFrame(&annotated, frame_result);
      writer.write(annotated);
    }
    if (frame_callback) {
      // 服务端在这里拿到单帧结果并立即通过 TCP 发送。
      frame_callback(frame_result);
    }
    result.frames.push_back(std::move(frame_result));
    ++frame_index;
  }
  result.frame_count = frame_index;

  if (!config_.output_json_path.empty()) {
    writeJson(result);
  }
  return result;
}

FrameResult VideoDetectionPipeline::processImage(const cv::Mat& frame, int frame_index, double timestamp_sec) {
  if (frame.empty()) throw std::invalid_argument("input image is empty");
  return processFrame(frame, frame_index, timestamp_sec);
}

void VideoDetectionPipeline::drawPreview(cv::Mat* frame, const FrameResult& result) const {
  if (frame == nullptr || frame->empty()) throw std::invalid_argument("preview image is empty");
  drawFrame(frame, result);
}

std::optional<Detection> VideoDetectionPipeline::bestDetectionByName(
    const std::vector<Detection>& detections,
    const std::string& class_name) const {
  std::optional<Detection> best;
  for (const auto& detection : detections) {
    if (detection.class_name != class_name) {
      continue;
    }
    if (!best.has_value() || detection.confidence > best->confidence) {
      best = detection;
    }
  }
  return best;
}

FrameResult VideoDetectionPipeline::processFrame(const cv::Mat& frame,
                                                 int frame_index,
                                                 double timestamp_sec) {
  FrameResult result;
  result.frame_index = frame_index;
  result.timestamp_sec = timestamp_sec;
  result.detections = detector_.detect(frame);

  // goal 是坐标原点。当前帧未检测到 goal 时沿用上一帧，减少短暂漏检造成的坐标跳变。
  const auto goal = bestDetectionByName(result.detections, "goal");
  if (goal.has_value()) {
    last_goal_origin_ = goal->center();
  }
  result.goal_origin = last_goal_origin_;

  std::optional<TimedPoint> observed_soccer;
  std::optional<DetectionPoint> best_soccer_point;

  for (const auto& detection : result.detections) {
    // 每个检测框都输出图像坐标；goal-relative 坐标仅在球门原点可用时计算。
    const auto point = detection.center();
    DetectionPoint detection_point;
    detection_point.class_id = detection.class_id;
    detection_point.class_name = detection.class_name;
    detection_point.confidence = detection.confidence;
    detection_point.image_point = point;
    if (detection.class_name == "soccer") {
      detection_point.ground_contact_point = {
          detection.box.x + detection.box.width * 0.5F,
          detection.box.y + detection.box.height};
      last_soccer_contact_offset_ =
          *detection_point.ground_contact_point - detection_point.image_point;
    }
    if (result.goal_origin.has_value()) {
      const auto origin = *result.goal_origin;
      detection_point.goal_relative_point = {point.x - origin.x, origin.y - point.y};
    }
    result.points.push_back(detection_point);

    if (detection.class_name == "soccer" &&
        (!best_soccer_point.has_value() ||
         detection_point.confidence > best_soccer_point->confidence)) {
      best_soccer_point = detection_point;
    }
  }

  if (best_soccer_point.has_value()) {
    // tracker 的历史只记录真实检测到的 soccer，不把预测结果继续喂回去。
    observed_soccer = TimedPoint{frame_index, timestamp_sec, best_soccer_point->goal_relative_point};
  }

  // soccer 漏检时根据最近真实点的速度做短时间预测。
  const auto track = soccer_tracker_.update(observed_soccer, frame_index, timestamp_sec);
  if (track.valid) {
    DetectionPoint soccer_point;
    if (best_soccer_point.has_value() && !track.predicted) {
      soccer_point = *best_soccer_point;
    } else {
      soccer_point.class_id = classIdByName("soccer");
      soccer_point.class_name = "soccer";
      soccer_point.confidence = 0.0F;
      soccer_point.goal_relative_point = track.point;
      soccer_point.predicted = true;
      if (result.goal_origin.has_value()) {
        // 预测点本身在 goal_relative 坐标系中，绘制时再转换回图像坐标。
        const auto origin = *result.goal_origin;
        soccer_point.image_point = {origin.x + track.point.x, origin.y - track.point.y};
      }
      if (last_soccer_contact_offset_.has_value()) {
        soccer_point.ground_contact_point =
            soccer_point.image_point + *last_soccer_contact_offset_;
      }
      result.points.push_back(soccer_point);
    }
    result.soccer = soccer_point;
  }

  return result;
}

void VideoDetectionPipeline::drawFrame(cv::Mat* frame, const FrameResult& result) const {
  // 先画原始检测框，便于检查模型检测质量。
  for (const auto& detection : result.detections) {
    const auto color = colorForClass(detection.class_name, false);
    cv::rectangle(*frame, roundedRect(detection.box), color, 2);
    const std::string label = detection.class_name + " " +
                              std::to_string(static_cast<int>(detection.confidence * 100.0F)) + "%";
    cv::putText(*frame,
                label,
                {static_cast<int>(detection.box.x), std::max(0, static_cast<int>(detection.box.y) - 5)},
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv::LINE_AA);
  }

  if (result.goal_origin.has_value()) {
    // 用十字标出当前使用的 goal 原点。
    const auto origin = *result.goal_origin;
    cv::drawMarker(*frame, roundedPoint(origin), {255, 255, 255}, cv::MARKER_CROSS, 18, 2);
  }

  // 再画转换后的点；预测的 soccer 点会使用更大的黄色点。
  for (const auto& point : result.points) {
    const auto color = colorForClass(point.class_name, point.predicted);
    cv::circle(*frame, roundedPoint(point.image_point), point.predicted ? 7 : 4, color, -1, cv::LINE_AA);
    if (point.class_name == "soccer") {
      const std::string label = point.predicted ? "soccer predicted" : "soccer";
      cv::putText(*frame,
                  label,
                  {static_cast<int>(point.image_point.x) + 6, static_cast<int>(point.image_point.y) - 6},
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5,
                  color,
                  1,
                  cv::LINE_AA);
    }
  }
}

int VideoDetectionPipeline::classIdByName(const std::string& class_name) const {
  const auto& names = detector_.classNames();
  const auto found = std::find(names.begin(), names.end(), class_name);
  if (found == names.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(names.begin(), found));
}

void VideoDetectionPipeline::writeJson(const VideoResult& result) const {
  std::ofstream output(config_.output_json_path);
  if (!output.is_open()) {
    throw std::runtime_error("failed to write json: " + config_.output_json_path);
  }

  // JSON 生成逻辑集中在 ResultJson.cpp，保证文件输出和 TCP FRAME 格式一致。
  output << videoResultToJson(result);
}

}  // namespace dog_detector
