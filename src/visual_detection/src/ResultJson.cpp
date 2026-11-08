#include "dog_detector/ResultJson.h"

#include <iomanip>
#include <sstream>

namespace dog_detector {
namespace {

// 手写 JSON 时必须转义字符串字段，避免路径或类别名中的特殊字符破坏格式。
std::string jsonEscape(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (char c : value) {
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += c;
        break;
    }
  }
  return escaped;
}

// 单帧 JSON 同时服务于 TCP FRAME 消息和完整视频 JSON 中的 frames 数组。
void appendFrameJson(std::ostream& output, const FrameResult& frame, int indent, bool pretty) {
  const std::string pad(indent, ' ');
  const std::string pad2(indent + 2, ' ');
  const char* newline = pretty ? "\n" : "";
  const char* comma_newline = pretty ? ",\n" : ",";
  const char* colon_space = pretty ? ": " : ":";
  const char* item_space = pretty ? ", " : ",";

  output << std::fixed << std::setprecision(3);
  output << (pretty ? pad : "") << "{" << newline;
  output << (pretty ? pad2 : "") << "\"frame_index\"" << colon_space << frame.frame_index
         << comma_newline;
  output << (pretty ? pad2 : "") << "\"timestamp_sec\"" << colon_space << frame.timestamp_sec
         << comma_newline;
  output << (pretty ? pad2 : "") << "\"points\"" << colon_space << "[";
  for (size_t i = 0; i < frame.points.size(); ++i) {
    // Each point is reported in image pixel coordinates.
    const auto& point = frame.points[i];
    if (i != 0) {
      output << item_space;
    }
    output << "{\"class_id\"" << colon_space << point.class_id
           << item_space << "\"class_name\"" << colon_space << "\"" << jsonEscape(point.class_name)
           << "\""
           << item_space << "\"confidence\"" << colon_space << point.confidence
           << item_space << "\"predicted\"" << colon_space << (point.predicted ? "true" : "false")
           << item_space << "\"image_point\"" << colon_space << "{\"x\"" << colon_space
           << point.image_point.x << item_space << "\"y\"" << colon_space << point.image_point.y
           << "}}";
  }
  output << "]" << newline;
  output << (pretty ? pad : "") << "}";
}

}  // namespace

std::string frameResultToJson(const FrameResult& frame) {
  std::ostringstream output;
  // TCP 协议中一条 FRAME 后面跟一段完整的单帧 JSON。
  appendFrameJson(output, frame, 0, false);
  return output.str();
}

std::string videoResultToJson(const VideoResult& result) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(3);
  output << "{\n";
  output << "  \"video_path\": \"" << jsonEscape(result.video_path) << "\",\n";
  output << "  \"fps\": " << result.fps << ",\n";
  output << "  \"frame_count\": " << result.frame_count << ",\n";
  // 明确坐标系，避免客户端把图像 y 轴方向理解反。
  output << "  \"coordinate_system\": \"origin=top-left, x=right, y=down, unit=pixel\",\n";
  output << "  \"frames\": [\n";
  for (size_t i = 0; i < result.frames.size(); ++i) {
    appendFrameJson(output, result.frames[i], 4, true);
    output << (i + 1 == result.frames.size() ? "\n" : ",\n");
  }
  output << "  ]\n";
  output << "}\n";
  return output.str();
}

}  // namespace dog_detector
