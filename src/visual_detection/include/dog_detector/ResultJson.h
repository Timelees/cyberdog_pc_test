#pragma once

#include "dog_detector/DetectionTypes.h"

#include <string>

namespace dog_detector {

// 将单帧结果序列化为一行 JSON，供 TCP FRAME 消息使用。
std::string frameResultToJson(const FrameResult& frame);
// 将完整视频结果序列化为 JSON 文件内容，供 CLI 离线结果保存。
std::string videoResultToJson(const VideoResult& result);

}  // namespace dog_detector
