#include "dog_detector/DetectionClient.h"

#include <muduo/net/Buffer.h>
#include <muduo/net/EventLoop.h>
#include <muduo/net/InetAddress.h>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dog_detector {

DetectionClient::DetectionClient(muduo::net::EventLoop* loop, DetectionClientConfig config)
    : loop_(loop),
      config_(std::move(config)),
      client_(loop_, muduo::net::InetAddress(config_.host, config_.port), "dog_detector_client") {
  // muduo 使用回调驱动网络事件，这里把连接和消息回调绑定到类成员。
  client_.setConnectionCallback(
      [this](const muduo::net::TcpConnectionPtr& connection) { onConnection(connection); });
  client_.setMessageCallback([this](const muduo::net::TcpConnectionPtr& connection,
                                    muduo::net::Buffer* buffer,
                                    muduo::Timestamp receive_time) {
    onMessage(connection, buffer, receive_time);
  });
}

void DetectionClient::connect() {
  // 异步连接，连接成功后会进入 onConnection。
  client_.connect();
}

void DetectionClient::onConnection(const muduo::net::TcpConnectionPtr& connection) {
  if (connection->connected()) {
    // 连接建立后立刻发送一行视频检测请求。
    connection->send(makeRequest());
  } else {
    // 服务端主动断开或检测结束后退出客户端事件循环。
    loop_->quit();
  }
}

void DetectionClient::onMessage(const muduo::net::TcpConnectionPtr&,
                                muduo::net::Buffer* buffer,
                                muduo::Timestamp) {
  // TCP 是字节流，可能半包/粘包，所以先累计到 pending_ 再按换行拆消息。
  pending_ += buffer->retrieveAllAsString();

  size_t newline = std::string::npos;
  while ((newline = pending_.find('\n')) != std::string::npos) {
    std::string line = pending_.substr(0, newline);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    pending_.erase(0, newline + 1);
    handleLine(line);
  }
}

std::string DetectionClient::makeRequest() const {
  if (config_.video_path.empty()) {
    throw std::runtime_error("client requires --video");
  }

  std::ostringstream request;
  // 服务端解析 key=value 文本；路径中如果有空格，需要调用方自行避免或扩展协议。
  request << "video=" << config_.video_path;
  if (!config_.json_path.empty()) {
    request << " json=" << config_.json_path;
  }
  if (!config_.output_video_path.empty()) {
    request << " out=" << config_.output_video_path;
  }
  request << "\n";
  return request.str();
}

void DetectionClient::handleLine(const std::string& line) {
  if (line.rfind("FRAME ", 0) == 0) {
    // 当前客户端直接把每帧 JSON 打到 stdout；后续可在这里接控制逻辑。
    std::cout << line.substr(6) << "\n";
    return;
  }

  std::cout << line << "\n";
  if (line.rfind("END ", 0) == 0 || line.rfind("ERROR ", 0) == 0) {
    // END/ERROR 表示本次请求结束。
    loop_->quit();
  }
}

}  // namespace dog_detector
