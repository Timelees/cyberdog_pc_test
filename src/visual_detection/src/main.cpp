#include "dog_detector/DetectionServer.h"
#include "dog_detector/VideoDetectionPipeline.h"

#include <muduo/net/EventLoop.h>
#include <muduo/net/InetAddress.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// dog_detect 同时支持离线处理和 --server 模式，帮助信息把两种入口都列出来。
void printUsage(const char* program) {
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " --video input.mp4 [--model models/best.onnx]\n"
      << "      [--json result.json] [--out annotated.mp4] [--conf 0.25] [--nms 0.45]\n"
      << "      [--device cuda|cpu] [--cuda-device 0]\n"
      << "  " << program
      << " --server [--port 9090] [--model models/best.onnx]\n"
      << "      [--device cuda|cpu] [--cuda-device 0]\n";
}

// 命令行参数多数是 --key value 形式，这个函数统一检查 value 是否存在。
std::string requireValue(int* index, int argc, char** argv) {
  if (*index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[*index]);
  }
  ++(*index);
  return argv[*index];
}

}  // namespace

int main(int argc, char** argv) {
  // 默认配置在结构体中定义，这里只根据命令行覆盖用户指定的字段。
  dog_detector::VideoPipelineConfig config;
  std::string video_path;
  bool server_mode = false;
  uint16_t port = 9090;

  try {
    for (int i = 1; i < argc; ++i) {
      // 保持参数解析简单直观，后续需要更多参数时继续追加分支。
      const std::string arg = argv[i];
      if (arg == "--model") {
        config.model_path = requireValue(&i, argc, argv);
      } else if (arg == "--video") {
        video_path = requireValue(&i, argc, argv);
      } else if (arg == "--json") {
        config.output_json_path = requireValue(&i, argc, argv);
      } else if (arg == "--out") {
        config.output_video_path = requireValue(&i, argc, argv);
      } else if (arg == "--conf") {
        config.confidence_threshold = std::stof(requireValue(&i, argc, argv));
      } else if (arg == "--nms") {
        config.nms_threshold = std::stof(requireValue(&i, argc, argv));
      } else if (arg == "--history") {
        config.tracker.history_size = static_cast<size_t>(std::stoul(requireValue(&i, argc, argv)));
      } else if (arg == "--max-missing") {
        config.tracker.max_missing_frames = std::stoi(requireValue(&i, argc, argv));
      } else if (arg == "--device") {
        config.device = requireValue(&i, argc, argv);
      } else if (arg == "--cuda-device") {
        config.cuda_device_id = std::stoi(requireValue(&i, argc, argv));
      } else if (arg == "--server") {
        server_mode = true;
      } else if (arg == "--port") {
        port = static_cast<uint16_t>(std::stoul(requireValue(&i, argc, argv)));
      } else if (arg == "--help" || arg == "-h") {
        printUsage(argv[0]);
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }

    if (server_mode) {
      // 服务端模式启动 muduo 事件循环，视频检测请求由 TCP 客户端触发。
      muduo::net::EventLoop loop;
      muduo::net::InetAddress listen_addr(port);
      dog_detector::DetectionServer server(&loop, listen_addr, config);
      server.start();
      std::cout << "dog detector server listening on port " << port << "\n";
      loop.loop();
      return 0;
    }

    if (video_path.empty()) {
      // 离线模式必须指定视频路径。
      printUsage(argv[0]);
      return 1;
    }

    // 离线模式：处理完整视频，最后输出摘要和可选文件路径。
    dog_detector::VideoDetectionPipeline pipeline(config);
    const auto result = pipeline.process(video_path);
    std::cout << "processed " << result.frame_count << " frames at " << result.fps << " fps\n";
    if (!config.output_json_path.empty()) {
      std::cout << "json: " << config.output_json_path << "\n";
    }
    if (!config.output_video_path.empty()) {
      std::cout << "annotated video: " << config.output_video_path << "\n";
    }
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
