#pragma once

#include <opencv2/core.hpp>

#include <deque>
#include <optional>

namespace dog_detector {

// soccer 轨迹缓存与遮挡预测参数。
struct SoccerTrackerConfig {
  // 用最近多少个真实观测点估计速度。
  size_t history_size = 8;
  // 连续丢失多少帧后停止预测，避免长时间无依据外推。
  int max_missing_frames = 15;
};

// 带时间戳的图像像素点。
struct TimedPoint {
  int frame_index = 0;
  double timestamp_sec = 0.0;
  cv::Point2f point;
};

// tracker 当前输出状态。
struct SoccerTrackState {
  bool valid = false;
  bool predicted = false;
  cv::Point2f point;
};

// 简单常速度 tracker：有真实观测时更新历史，遮挡时按最近速度预测。
class SoccerTracker {
 public:
  explicit SoccerTracker(SoccerTrackerConfig config = {});

  // observed 有值表示当前帧检测到了 soccer；无值表示需要尝试预测。
  SoccerTrackState update(const std::optional<TimedPoint>& observed,
                          int frame_index,
                          double timestamp_sec);

  // 新视频或需要重置轨迹时清空缓存。
  void reset();

 private:
  // 用历史队列首尾点估算平均速度。
  cv::Point2f estimateVelocity() const;

  SoccerTrackerConfig config_;
  std::deque<TimedPoint> history_;
  int missing_frames_ = 0;
};

}  // namespace dog_detector
