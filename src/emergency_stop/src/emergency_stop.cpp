#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>

#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr int32_t kRecoveryStand = 111;
constexpr int32_t kSlowWalk = 303;
}

class EmergencyStopNode : public rclcpp::Node
{
public:
  EmergencyStopNode()
  : Node("emergency_stop")
  {
    robot_namespace_ = declare_parameter<std::string>("robot_namespace", "cyberdog_1");
    while (!robot_namespace_.empty() && robot_namespace_.front() == '/') robot_namespace_.erase(0, 1);
    while (!robot_namespace_.empty() && robot_namespace_.back() == '/') robot_namespace_.pop_back();
    servo_pub_ = create_publisher<protocol::msg::MotionServoCmd>(
      "/" + robot_namespace_ + "/motion_servo_cmd", rclcpp::SystemDefaultsQoS());
    result_client_ = create_client<protocol::srv::MotionResultCmd>(
      "/" + robot_namespace_ + "/motion_result_cmd");
    RCLCPP_INFO(
      get_logger(),
      "e=emergency stop to recovery stand (111), space=zero SERVO_END, r=recovery stand, q=quit");
  }

  void run()
  {
    termios old{};
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old) != 0) {
      RCLCPP_ERROR(get_logger(), "run emergency_stop from an interactive terminal");
      return;
    }
    termios raw = old;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
      timeval timeout{0, 100000};
      if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout) <= 0) continue;
      char key = 0;
      if (read(STDIN_FILENO, &key, 1) != 1) continue;
      if (key == 'q' || key == 'Q') break;
      if (key == 'e' || key == 'E') {
        publish_servo_end();
        call_motion(kRecoveryStand, "emergency stop to recovery stand");
      }
      else if (key == 'r' || key == 'R') call_motion(kRecoveryStand, "recovery stand");
      else if (key == ' ') publish_servo_end();
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old);
  }

private:
  void publish_servo_end()
  {
    protocol::msg::MotionServoCmd msg;
    msg.motion_id = kSlowWalk;
    msg.cmd_type = protocol::msg::MotionServoCmd::SERVO_END;
    msg.cmd_source = protocol::msg::MotionServoCmd::APP;
    msg.value = 2;
    msg.vel_des = {0.0F, 0.0F, 0.0F};
    msg.rpy_des = {0.0F, 0.0F, 0.0F};
    msg.pos_des = {0.0F, 0.0F, 0.0F};
    msg.acc_des = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    msg.ctrl_point = {0.0F, 0.0F, 0.0F};
    msg.foot_pose = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    msg.step_height = {0.0F, 0.0F};
    servo_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "zero SERVO_END published");
  }

  void call_motion(int32_t motion_id, const std::string & label)
  {
    if (!result_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "motion_result_cmd unavailable; %s not sent", label.c_str());
      return;
    }
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    req->motion_id = motion_id;
    req->cmd_source = protocol::srv::MotionResultCmd::Request::APP;
    req->vel_des = {0.0F, 0.0F, 0.0F};
    req->rpy_des = {0.0F, 0.0F, 0.0F};
    req->pos_des = {0.0F, 0.0F, 0.0F};
    req->acc_des = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    req->ctrl_point = {0.0F, 0.0F, 0.0F};
    req->foot_pose = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    req->step_height = {0.0F, 0.0F};
    auto future = result_client_->async_send_request(req);
    const auto result = rclcpp::spin_until_future_complete(
      shared_from_this(), future, std::chrono::seconds(5));
    if (result == rclcpp::FutureReturnCode::SUCCESS && future.get()->result) {
      RCLCPP_WARN(get_logger(), "%s accepted", label.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s failed", label.c_str());
    }
  }

  std::string robot_namespace_;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr servo_pub_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr result_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmergencyStopNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
