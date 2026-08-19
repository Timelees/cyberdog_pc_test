#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class FootballCmdVelToServo : public rclcpp::Node
{
public:
  FootballCmdVelToServo()
  : Node("football_cmd_vel_to_servo")
  {
    motion_id_ = declare_parameter<int>("motion_id", 303);
    timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.25);
    start_retry_sec_ = declare_parameter<double>("servo_start_retry_sec", 0.50);
    max_x_ = declare_parameter<double>("max_linear_x", 0.30);
    max_y_ = declare_parameter<double>("max_linear_y", 0.20);
    max_w_ = declare_parameter<double>("max_angular_z", 0.80);
    cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    servo_topic_ = declare_parameter<std::string>("motion_servo_cmd_topic", "motion_servo_cmd");
    control_valid_topic_ = declare_parameter<std::string>(
      "control_valid_topic", "football/control_valid");
    motion_status_topic_ = declare_parameter<std::string>(
      "motion_status_topic", "motion_status");
    servo_pub_ = create_publisher<protocol::msg::MotionServoCmd>(servo_topic_, rclcpp::SystemDefaultsQoS());
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!msg) return;
        latest_ = *msg;
        latest_time_ = now();
        have_command_ = true;
      });
    control_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      control_valid_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        control_valid_ = msg && msg->data;
      });
    motion_status_sub_ = create_subscription<protocol::msg::MotionStatus>(
      motion_status_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const protocol::msg::MotionStatus::SharedPtr msg) {
        if (!msg) return;
        const bool ready = msg->motion_id == motion_id_ &&
          (msg->switch_status == protocol::msg::MotionStatus::NORMAL ||
          msg->switch_status == protocol::msg::MotionStatus::TRANSITIONING);
        if (have_motion_status_ && ready != motion_ready_) {
          if (ready) {
            RCLCPP_INFO(
              get_logger(), "slow-walk servo ready: motion_id=%d switch_status=%d",
              msg->motion_id, msg->switch_status);
          } else {
            RCLCPP_WARN(
              get_logger(),
              "slow-walk servo lost: motion_id=%d switch_status=%d; restarting servo session",
              msg->motion_id, msg->switch_status);
          }
        }
        have_motion_status_ = true;
        motion_ready_ = ready;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&FootballCmdVelToServo::tick, this));
    RCLCPP_WARN(get_logger(), "real cmd_vel adapter active: %s -> %s, watchdog=%.2fs",
      cmd_topic_.c_str(), servo_topic_.c_str(), timeout_sec_);
  }

private:
  void tick()
  {
    const auto stamp = now();
    const bool fresh = have_command_ && (stamp - latest_time_).seconds() <= timeout_sec_;
    geometry_msgs::msg::Twist command = fresh ? latest_ : geometry_msgs::msg::Twist();
    command.linear.x = std::clamp(command.linear.x, -max_x_, max_x_);
    command.linear.y = std::clamp(command.linear.y, -max_y_, max_y_);
    command.angular.z = std::clamp(command.angular.z, -max_w_, max_w_);
    if (!control_valid_ && !servo_session_active_) {
      return;
    }
    protocol::msg::MotionServoCmd msg;
    msg.motion_id = motion_id_;
    if (!control_valid_) {
      msg.cmd_type = protocol::msg::MotionServoCmd::SERVO_END;
      command = geometry_msgs::msg::Twist();
    } else if (!servo_session_active_ ||
      (!motion_ready_ &&
      (last_start_time_.nanoseconds() == 0 ||
      (stamp - last_start_time_).seconds() >= start_retry_sec_)))
    {
      // Establish slow-walk before Nav2 is allowed to send its first goal.
      // Waiting for a non-zero cmd_vel here creates a circular dependency with
      // the tracking client's motion_status=303 safety gate.
      msg.cmd_type = protocol::msg::MotionServoCmd::SERVO_START;
      last_start_time_ = stamp;
    } else {
      msg.cmd_type = protocol::msg::MotionServoCmd::SERVO_DATA;
    }
    msg.cmd_source = protocol::msg::MotionServoCmd::APP;
    msg.value = 2;
    msg.vel_des = {static_cast<float>(command.linear.x), static_cast<float>(command.linear.y),
      static_cast<float>(command.angular.z)};
    msg.rpy_des = {0.0F, 0.0F, 0.0F};
    msg.pos_des = {0.0F, 0.0F, 0.0F};
    msg.acc_des = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    msg.ctrl_point = {0.0F, 0.0F, 0.0F};
    msg.foot_pose = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    msg.step_height = {0.06F, 0.06F};
    servo_pub_->publish(msg);
    servo_session_active_ = control_valid_;
    if (!control_valid_) {
      motion_ready_ = false;
      last_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
  }

  int motion_id_{303};
  double timeout_sec_{0.25};
  double start_retry_sec_{0.50};
  double max_x_{0.30};
  double max_y_{0.20};
  double max_w_{0.80};
  std::string cmd_topic_;
  std::string servo_topic_;
  std::string control_valid_topic_;
  std::string motion_status_topic_;
  bool have_command_{false};
  bool control_valid_{false};
  bool servo_session_active_{false};
  bool have_motion_status_{false};
  bool motion_ready_{false};
  geometry_msgs::msg::Twist latest_;
  rclcpp::Time latest_time_;
  rclcpp::Time last_start_time_;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr servo_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_valid_sub_;
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootballCmdVelToServo>());
  rclcpp::shutdown();
  return 0;
}
