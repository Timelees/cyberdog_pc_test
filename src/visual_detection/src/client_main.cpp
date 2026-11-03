#include "dog_detector/DetectionClient.h"

#include <muduo/net/EventLoop.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// dog_client 只负责向服务端发送视频请求，并实时打印服务端返回的 FRAME JSON。
void printUsage(const char* program) {
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " --video input.mp4 [--host 127.0.0.1] [--port 9090]\n"
      << "      [--json result.json] [--out annotated.mp4]\n";
}

// 统一解析 --key value 形式的参数。
std::string requireValue(int* index, int argc, char** argv) {
  if (*index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[*index]);
  }
  ++(*index);
  return argv[*index];
}

}  // namespace

int main(int argc, char** argv) {
  // 客户端默认连接本机 9090，其他字段由命令行覆盖。
  dog_detector::DetectionClientConfig config;

  try {
    for (int i = 1; i < argc; ++i) {
      // 参数数量少，手写解析比引入额外 CLI 库更轻。
      const std::string arg = argv[i];
      if (arg == "--host") {
        config.host = requireValue(&i, argc, argv);
      } else if (arg == "--port") {
        config.port = static_cast<uint16_t>(std::stoul(requireValue(&i, argc, argv)));
      } else if (arg == "--video") {
        config.video_path = requireValue(&i, argc, argv);
      } else if (arg == "--json") {
        config.json_path = requireValue(&i, argc, argv);
      } else if (arg == "--out") {
        config.output_video_path = requireValue(&i, argc, argv);
      } else if (arg == "--help" || arg == "-h") {
        printUsage(argv[0]);
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }

    if (config.video_path.empty()) {
      // 客户端请求必须包含视频路径，否则服务端不知道要检测什么。
      printUsage(argv[0]);
      return 1;
    }

    // muduo 客户端是异步模型，connect 后进入事件循环等待 START/FRAME/END。
    muduo::net::EventLoop loop;
    dog_detector::DetectionClient client(&loop, config);
    client.connect();
    loop.loop();
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
