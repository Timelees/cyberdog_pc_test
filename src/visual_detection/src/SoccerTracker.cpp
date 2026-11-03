#include "dog_detector/SoccerTracker.h"

#include <algorithm>

namespace dog_detector {

SoccerTracker::SoccerTracker(SoccerTrackerConfig config) : config_(config) {}

SoccerTrackState SoccerTracker::update(const std::optional<TimedPoint>& observed,
                                       int frame_index,
                                       double timestamp_sec) {
  if (observed.has_value()) {
    // 有真实观测时更新历史队列，并清零丢失计数。
    history_.push_back(*observed);
    while (history_.size() > config_.history_size) {
      history_.pop_front();
    }
    missing_frames_ = 0;
    return SoccerTrackState{true, false, observed->point};
  }

  ++missing_frames_;
  if (history_.empty() || missing_frames_ > config_.max_missing_frames) {
    // 没有历史或丢失太久时停止预测，避免坐标无限漂移。
    return SoccerTrackState{};
  }

  // 遮挡期间使用常速度模型：最后一次真实位置 + 平均速度 * 时间差。
  const auto velocity = estimateVelocity();
  const auto& last = history_.back();
  const double dt = std::max(0.0, timestamp_sec - last.timestamp_sec);
  cv::Point2f predicted{
      last.point.x + velocity.x * static_cast<float>(dt),
      last.point.y + velocity.y * static_cast<float>(dt),
  };
  return SoccerTrackState{true, true, predicted};
}

void SoccerTracker::reset() {
  // 切换视频或重新开始跟踪时使用。
  history_.clear();
  missing_frames_ = 0;
}

cv::Point2f SoccerTracker::estimateVelocity() const {
  if (history_.size() < 2) {
    // 只有一个点无法估计速度，退化为原地预测。
    return {0.0F, 0.0F};
  }

  const auto& first = history_.front();
  const auto& last = history_.back();
  const double dt = last.timestamp_sec - first.timestamp_sec;
  if (dt <= 1e-6) {
    // 防止异常时间戳导致除零。
    return {0.0F, 0.0F};
  }

  return {
      static_cast<float>((last.point.x - first.point.x) / dt),
      static_cast<float>((last.point.y - first.point.y) / dt),
  };
}

}  // namespace dog_detector
