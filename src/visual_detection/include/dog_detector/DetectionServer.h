#pragma once

#include "dog_detector/VideoDetectionPipeline.h"

#include <muduo/net/TcpServer.h>

#include <string>

namespace dog_detector {

// muduo TCP 服务端。收到视频请求后，边检测边向客户端发送每帧坐标。
class DetectionServer {
 public:
  DetectionServer(muduo::net::EventLoop* loop,
                  const muduo::net::InetAddress& listen_addr,
                  VideoPipelineConfig config);

  // 启动监听；事件循环由 main 中的 EventLoop::loop 驱动。
  void start();

 private:
  // 新连接建立时发送简单的协议提示。
  void onConnection(const muduo::net::TcpConnectionPtr& connection);
  // 每条请求触发一次视频检测，并在线程中持续发送 FRAME。
  void onMessage(const muduo::net::TcpConnectionPtr& connection,
                 muduo::net::Buffer* buffer,
                 muduo::Timestamp receive_time);

  VideoPipelineConfig config_;
  muduo::net::TcpServer server_;
};

}  // namespace dog_detector
