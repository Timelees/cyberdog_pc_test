#pragma once

#include <muduo/net/TcpClient.h>

#include <string>

namespace dog_detector {

// TCP 客户端配置：连接服务端后发送视频路径，并持续接收逐帧坐标。
struct DetectionClientConfig {
  std::string host = "127.0.0.1";
  uint16_t port = 9090;
  std::string video_path;
  std::string json_path;
  std::string output_video_path;
};

// muduo TCP 客户端。协议是按行收包：START、FRAME JSON、END 或 ERROR。
class DetectionClient {
 public:
  DetectionClient(muduo::net::EventLoop* loop, DetectionClientConfig config);

  // 发起异步连接，真正的请求在 onConnection 里发送。
  void connect();

 private:
  // 连接建立后发送检测请求；断开时退出事件循环。
  void onConnection(const muduo::net::TcpConnectionPtr& connection);
  // muduo 可能一次收到半行或多行，这里把字节流拆成按行协议。
  void onMessage(const muduo::net::TcpConnectionPtr& connection,
                 muduo::net::Buffer* buffer,
                 muduo::Timestamp receive_time);
  // 构造服务端能识别的一行请求：video=... json=... out=...
  std::string makeRequest() const;
  // 处理完整的一行响应。FRAME 行当前直接打印 JSON。
  void handleLine(const std::string& line);

  muduo::net::EventLoop* loop_;
  DetectionClientConfig config_;
  muduo::net::TcpClient client_;
  std::string pending_;
};

}  // namespace dog_detector
