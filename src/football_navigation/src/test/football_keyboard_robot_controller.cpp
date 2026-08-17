// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class TerminalMode
{
public:
  TerminalMode()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("keyboard controller requires an interactive terminal");
    }
    if (tcgetattr(STDIN_FILENO, &saved_) != 0) {
      throw std::runtime_error("failed to read terminal settings");
    }
    termios raw = saved_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error("failed to enable keyboard input mode");
    }
    active_ = true;
  }

  ~TerminalMode()
  {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
  }

private:
  termios saved_{};
  bool active_{false};
};

bool keyAvailable()
{
  fd_set descriptors;
  FD_ZERO(&descriptors);
  FD_SET(STDIN_FILENO, &descriptors);
  timeval timeout{};
  return select(STDIN_FILENO + 1, &descriptors, nullptr, nullptr, &timeout) > 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("football_keyboard_robot_controller");
  const auto robot_namespace = node->declare_parameter<std::string>(
    "robot_namespace", "cyberdog_2");
  const auto default_topic = "/" + robot_namespace + "/cmd_vel";
  const auto cmd_vel_topic = node->declare_parameter<std::string>(
    "cmd_vel_topic", default_topic);
  const double linear_speed = node->declare_parameter<double>("linear_speed_mps", 0.35);
  const double angular_speed = node->declare_parameter<double>("angular_speed_rps", 0.9);
  const double publish_rate_hz = node->declare_parameter<double>("publish_rate_hz", 20.0);
  if (robot_namespace.empty() || cmd_vel_topic.empty() || linear_speed <= 0.0 ||
      angular_speed <= 0.0 || publish_rate_hz <= 0.0) {
    throw std::invalid_argument("keyboard controller parameters must be non-empty and positive");
  }

  const auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(1).reliable());
  TerminalMode terminal_mode;
  geometry_msgs::msg::Twist command;

  std::cout << "Controlling " << robot_namespace << " on " << cmd_vel_topic << "\n"
            << "w/s: forward/reverse, a/d: turn left/right, space: stop, q: quit\n";

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  while (rclcpp::ok()) {
    while (keyAvailable()) {
      char key = 0;
      if (read(STDIN_FILENO, &key, 1) != 1) {
        break;
      }
      switch (key) {
        case 'w': command.linear.x = linear_speed; break;
        case 's': command.linear.x = -linear_speed; break;
        case 'a': command.angular.z = angular_speed; break;
        case 'd': command.angular.z = -angular_speed; break;
        case 'x': command.linear.x = 0.0; break;
        case 'z': command.angular.z = 0.0; break;
        case ' ':
          command = geometry_msgs::msg::Twist{};
          break;
        case 'q':
          rclcpp::shutdown();
          break;
        default:
          break;
      }
    }
    publisher->publish(command);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(period);
  }

  publisher->publish(geometry_msgs::msg::Twist{});
  return 0;
}
