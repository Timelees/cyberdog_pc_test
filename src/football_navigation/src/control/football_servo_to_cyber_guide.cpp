// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_bridge/srv/usr_cmd.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class FootballServoToCyberGuide : public rclcpp::Node
{
public:
  FootballServoToCyberGuide()
  : Node("football_servo_to_cyber_guide"), startup_time_(now())
  {
    input_motion_id_ = declare_parameter<int>("input_motion_id", 309);
    gazebo_gait_type_ = declare_parameter<int>("gazebo_gait_type", 5);
    recovery_duration_sec_ = declare_parameter<double>("recovery_duration_sec", 4.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.35);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 20.0);
    max_x_ = declare_parameter<double>("max_linear_x", 0.26);
    max_y_ = declare_parameter<double>("max_linear_y", 0.26);
    max_yaw_ = declare_parameter<double>("max_angular_z", 1.0);
    max_accel_x_ = declare_parameter<double>("max_linear_accel_x", 2.5);
    max_accel_y_ = declare_parameter<double>("max_linear_accel_y", 2.0);
    max_accel_yaw_ = declare_parameter<double>("max_angular_accel", 3.2);
    angular_z_scale_ = declare_parameter<double>("angular_z_scale", 1.0);
    step_height_ = declare_parameter<double>("step_height", 0.06);
    service_name_ = declare_parameter<std::string>("usr_cmd_service", "/usr_cmd");
    if (input_motion_id_ <= 0 || gazebo_gait_type_ < 0 || recovery_duration_sec_ < 0.0 ||
      command_timeout_sec_ <= 0.0 || command_rate_hz_ <= 0.0 ||
      max_accel_x_ <= 0.0 || max_accel_y_ <= 0.0 || max_accel_yaw_ <= 0.0 ||
      !std::isfinite(angular_z_scale_) || angular_z_scale_ == 0.0)
    {
      throw std::invalid_argument("invalid cyber_guide servo adapter parameters");
    }

    client_ = create_client<ros_bridge::srv::UsrCmd>(service_name_);
    motion_sub_ = create_subscription<protocol::msg::MotionServoCmd>(
      "motion_servo_cmd", rclcpp::SystemDefaultsQoS(),
      [this](const protocol::msg::MotionServoCmd::SharedPtr msg) {
        if (!msg) {
          return;
        }
        latest_ = *msg;
        latest_time_ = now();
        have_command_ = true;
      });
    status_pub_ = create_publisher<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS());
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "football/control_valid", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / command_rate_hz_)),
      std::bind(&FootballServoToCyberGuide::tick, this));
    RCLCPP_INFO(
      get_logger(), "motion_id %d -> cyber_guide gait %d via %s (yaw scale %.1f)",
      input_motion_id_, gazebo_gait_type_, service_name_.c_str(), angular_z_scale_);
  }

private:
  static float component(const std::vector<float> & values, std::size_t index)
  {
    return index < values.size() && std::isfinite(values[index]) ? values[index] : 0.0F;
  }

  static double slew(double target, double current, double acceleration, double dt)
  {
    const double max_delta = acceleration * std::max(0.0, dt);
    return current + std::clamp(target - current, -max_delta, max_delta);
  }

  void tick()
  {
    const auto stamp = now();
    const bool service_ready = client_->service_is_ready();
    if (service_ready && !service_seen_) {
      service_seen_ = true;
      startup_time_ = stamp;
      RCLCPP_INFO(get_logger(), "cyber_guide service ready; starting recovery stand");
    }
    const bool recovering = (stamp - startup_time_).seconds() < recovery_duration_sec_;
    const bool valid = service_seen_ && service_ready && !recovering;

    if (recovering || !service_seen_) {
      limited_x_ = 0.0;
      limited_y_ = 0.0;
      limited_yaw_ = 0.0;
      last_limit_time_ = stamp;
    }

    std_msgs::msg::Bool valid_msg;
    valid_msg.data = valid;
    valid_pub_->publish(valid_msg);

    protocol::msg::MotionStatus status;
    status.motion_id = valid ? input_motion_id_ : 0;
    status.switch_status = valid ? protocol::msg::MotionStatus::NORMAL :
      protocol::msg::MotionStatus::TRANSITIONING;
    status.order_process_bar = valid ? 100 : static_cast<int8_t>(std::clamp(
      recovery_duration_sec_ > 0.0 ?
      100.0 * (stamp - startup_time_).seconds() / recovery_duration_sec_ : 100.0,
      0.0, 99.0));
    status_pub_->publish(status);

    if (!service_ready || request_pending_) {
      return;
    }

    auto request = std::make_shared<ros_bridge::srv::UsrCmd::Request>();
    request->control_mode = recovering ? 1 : 3;  // RECOVERY_STAND, then LOCOMOTION
    request->gait_type = gazebo_gait_type_;
    request->foot_height = step_height_;

    const bool fresh = have_command_ &&
      (stamp - latest_time_).seconds() <= command_timeout_sec_;
    const bool accepted = fresh && latest_.motion_id == input_motion_id_ &&
      latest_.cmd_type != protocol::msg::MotionServoCmd::SERVO_END;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_yaw = 0.0;
    if (valid && accepted) {
      target_x = std::clamp(
        static_cast<double>(component(latest_.vel_des, 0)), -max_x_, max_x_);
      target_y = std::clamp(
        static_cast<double>(component(latest_.vel_des, 1)), -max_y_, max_y_);
      target_yaw = std::clamp(
        angular_z_scale_ * static_cast<double>(component(latest_.vel_des, 2)),
        -max_yaw_, max_yaw_);
      if (!latest_.step_height.empty() && std::isfinite(latest_.step_height.front())) {
        request->foot_height = latest_.step_height.front();
      }
    }

    const double dt = last_limit_time_.nanoseconds() == 0 ?
      0.0 : std::max(0.0, (stamp - last_limit_time_).seconds());
    limited_x_ = slew(target_x, limited_x_, max_accel_x_, dt);
    limited_y_ = slew(target_y, limited_y_, max_accel_y_, dt);
    limited_yaw_ = slew(target_yaw, limited_yaw_, max_accel_yaw_, dt);
    last_limit_time_ = stamp;
    request->cmd_velx = limited_x_;
    request->cmd_vely = limited_y_;
    request->cmd_yaw = limited_yaw_;

    request_pending_ = true;
    client_->async_send_request(
      request, [this](rclcpp::Client<ros_bridge::srv::UsrCmd>::SharedFuture future) {
        request_pending_ = false;
        try {
          if (!future.get()->cmd_result) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000, "cyber_guide rejected usr_cmd");
          }
        } catch (const std::exception & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "usr_cmd call failed: %s", error.what());
        }
      });
  }

  int input_motion_id_{309};
  int gazebo_gait_type_{5};
  double recovery_duration_sec_{4.0};
  double command_timeout_sec_{0.35};
  double command_rate_hz_{20.0};
  double max_x_{0.26};
  double max_y_{0.26};
  double max_yaw_{1.0};
  double max_accel_x_{2.5};
  double max_accel_y_{2.0};
  double max_accel_yaw_{3.2};
  double angular_z_scale_{1.0};
  double step_height_{0.06};
  std::string service_name_;
  bool have_command_{false};
  bool service_seen_{false};
  bool request_pending_{false};
  protocol::msg::MotionServoCmd latest_;
  rclcpp::Time startup_time_;
  rclcpp::Time latest_time_;
  rclcpp::Time last_limit_time_;
  double limited_x_{0.0};
  double limited_y_{0.0};
  double limited_yaw_{0.0};
  rclcpp::Client<ros_bridge::srv::UsrCmd>::SharedPtr client_;
  rclcpp::Subscription<protocol::msg::MotionServoCmd>::SharedPtr motion_sub_;
  rclcpp::Publisher<protocol::msg::MotionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootballServoToCyberGuide>());
  rclcpp::shutdown();
  return 0;
}
