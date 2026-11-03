#include "dog_detector/DetectionServer.h"

#include "dog_detector/ResultJson.h"

#include <muduo/net/Buffer.h>
#include <muduo/net/EventLoop.h>
#include <muduo/net/InetAddress.h>

#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace dog_detector {
namespace {

// 客户端请求是一行 key=value 文本，便于用 nc 或自写客户端调试。
struct Request {
  std::string video_path;
  std::string json_path;
  std::string output_video_path;
};

// 支持两种写法：video=/a.mp4 json=/b.json out=/c.mp4，或者只发 /a.mp4。
Request parseRequest(const std::string& text) {
  std::istringstream input(text);
  std::string token;
  Request request;
  while (input >> token) {
    const auto equals = token.find('=');
    if (equals == std::string::npos) {
      if (request.video_path.empty()) {
        request.video_path = token;
      }
      continue;
    }

    const auto key = token.substr(0, equals);
    const auto value = token.substr(equals + 1);
    if (key == "video") {
      request.video_path = value;
    } else if (key == "json") {
      request.json_path = value;
    } else if (key == "out" || key == "output_video") {
      request.output_video_path = value;
    }
  }

  if (request.video_path.empty()) {
    throw std::runtime_error("request requires a video path");
  }
  return request;
}

}  // namespace

DetectionServer::DetectionServer(muduo::net::EventLoop* loop,
                                 const muduo::net::InetAddress& listen_addr,
                                 VideoPipelineConfig config)
    : config_(std::move(config)), server_(loop, listen_addr, "dog_detector") {
  server_.setConnectionCallback(
      [this](const muduo::net::TcpConnectionPtr& connection) { onConnection(connection); });
  server_.setMessageCallback([this](const muduo::net::TcpConnectionPtr& connection,
                                    muduo::net::Buffer* buffer,
                                    muduo::Timestamp receive_time) {
    onMessage(connection, buffer, receive_time);
  });
}

void DetectionServer::start() {
  server_.start();
}

void DetectionServer::onConnection(const muduo::net::TcpConnectionPtr& connection) {
  if (connection->connected()) {
    // 连接建立后发协议提示；真正的检测要等客户端发送请求。
    connection->send("Send: video=/path/input.mp4 json=/path/result.json out=/path/annotated.mp4\n");
    connection->send("Protocol: START ..., FRAME {json}, END ...\n");
  }
}

void DetectionServer::onMessage(const muduo::net::TcpConnectionPtr& connection,
                                muduo::net::Buffer* buffer,
                                muduo::Timestamp) {
  const std::string text = buffer->retrieveAllAsString();
  auto base_config = config_;

  // 视频检测可能耗时较长，放到后台线程里，避免阻塞 muduo 的 IO 线程。
  std::thread([connection, text, base_config]() mutable {
    try {
      const auto request = parseRequest(text);
      base_config.output_json_path = request.json_path;
      base_config.output_video_path = request.output_video_path;

      connection->send("START video=" + request.video_path + "\n");
      connection->send("STATUS loading_model device=" + base_config.device + "\n");
      VideoDetectionPipeline pipeline(base_config);
      connection->send("STATUS processing_video\n");
      // 每处理完一帧就发送一条 FRAME JSON，实现边检测边输出坐标。
      const auto result = pipeline.process(request.video_path, [connection](const FrameResult& frame) {
        connection->send("FRAME " + frameResultToJson(frame) + "\n");
      });

      // END 表示视频处理结束，同时附带总帧数和可选输出文件路径。
      std::ostringstream response;
      response << "END frame_count=" << result.frame_count << " fps=" << result.fps;
      if (!base_config.output_json_path.empty()) {
        response << " json=" << base_config.output_json_path;
      }
      if (!base_config.output_video_path.empty()) {
        response << " out=" << base_config.output_video_path;
      }
      response << "\n";
      connection->send(response.str());
    } catch (const std::exception& error) {
      connection->send(std::string("ERROR ") + error.what() + "\n");
    }
    connection->shutdown();
  }).detach();
}

}  // namespace dog_detector
