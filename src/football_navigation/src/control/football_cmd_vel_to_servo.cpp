#include <chrono>
#include <cmath>
#include <algorithm>
#include <deque>
#include <memory>
#include <stdexcept>
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
    // Keep the real-robot command path aligned with cyberdog_nav2's
    // velocity_adaptor: navigation uses the dedicated follow gait (309), the
    // ALGO command source, and a short moving average before reaching motion.
    motion_id_ = declare_parameter<int>("motion_id", 309);
    cmd_source_ = declare_parameter<int>("cmd_source", 4);
    // CyberDog's AB-navigation executor calls ChangeGait(309) with value=2
    // before following a path.  Keep the direct servo bridge equivalent.
    gait_value_ = declare_parameter<int>("gait_value", 2);
    timeout_sec_ = declare_parameter<double>("cmd_vel_timeout_sec", 0.25);
    start_retry_sec_ = declare_parameter<double>("servo_start_retry_sec", 0.50);
    smoothing_window_sec_ = declare_parameter<double>("smoothing_window_sec", 0.25);
    deadband_ = declare_parameter<double>("velocity_deadband", 0.005);
    max_x_ = declare_parameter<double>("max_linear_x", 0.26);
    max_y_ = declare_parameter<double>("max_linear_y", 0.20);
    max_w_ = declare_parameter<double>("max_angular_z", 1.0);
    cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "football/cmd_vel");
    servo_topic_ = declare_parameter<std::string>("motion_servo_cmd_topic", "motion_servo_cmd");
    control_valid_topic_ = declare_parameter<std::string>(
      "control_valid_topic", "football/control_valid");
    motion_status_topic_ = declare_parameter<std::string>(
      "motion_status_topic", "motion_status");
    if (motion_id_ <= 0 || cmd_source_ < 0 || cmd_source_ > 4 ||
      timeout_sec_ <= 0.0 || start_retry_sec_ <= 0.0 ||
      smoothing_window_sec_ <= 0.0 || deadband_ < 0.0 ||
      max_x_ < 0.0 || max_y_ < 0.0 || max_w_ < 0.0)
    {
      throw std::invalid_argument("invalid cmd_vel adapter parameters");
    }
    servo_pub_ = create_publisher<protocol::msg::MotionServoCmd>(servo_topic_, rclcpp::SystemDefaultsQoS());
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!msg) return;
        const auto stamp = now();
        geometry_msgs::msg::Twist bounded = *msg;
        bounded.linear.x = std::clamp(bounded.linear.x, -max_x_, max_x_);
        bounded.linear.y = std::clamp(bounded.linear.y, -max_y_, max_y_);
        bounded.angular.z = std::clamp(bounded.angular.z, -max_w_, max_w_);
        history_.push_back({stamp, bounded});
        sum_x_ += bounded.linear.x;
        sum_y_ += bounded.linear.y;
        sum_w_ += bounded.angular.z;
        trimHistory(stamp);
        latest_.linear.x = sum_x_ / static_cast<double>(history_.size());
        latest_.linear.y = sum_y_ / static_cast<double>(history_.size());
        latest_.angular.z = sum_w_ / static_cast<double>(history_.size());
        latest_time_ = stamp;
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
              get_logger(), "navigation gait ready: motion_id=%d switch_status=%d",
              msg->motion_id, msg->switch_status);
          } else {
            RCLCPP_WARN(
              get_logger(),
              "navigation gait lost: motion_id=%d switch_status=%d; restarting servo session",
              msg->motion_id, msg->switch_status);
          }
        }
        have_motion_status_ = true;
        motion_ready_ = ready;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&FootballCmdVelToServo::tick, this));
    RCLCPP_WARN(
      get_logger(),
      "real cmd_vel adapter active: %s -> %s, motion_id=%d source=%d "
      "watchdog=%.2fs smoothing=%.2fs",
      cmd_topic_.c_str(), servo_topic_.c_str(), motion_id_, cmd_source_,
      timeout_sec_, smoothing_window_sec_);
  }

private:
  struct TimedTwist
  {
    rclcpp::Time stamp;
    geometry_msgs::msg::Twist twist;
  };

  void trimHistory(const rclcpp::Time & stamp)
  {
    while (!history_.empty() &&
      (stamp - history_.front().stamp).seconds() > smoothing_window_sec_)
    {
      sum_x_ -= history_.front().twist.linear.x;
      sum_y_ -= history_.front().twist.linear.y;
      sum_w_ -= history_.front().twist.angular.z;
      history_.pop_front();
    }
  }

  void clearHistory()
  {
    history_.clear();
    sum_x_ = 0.0;
    sum_y_ = 0.0;
    sum_w_ = 0.0;
  }

  void tick()
  {
    const auto stamp = now();
    const bool fresh = have_command_ && (stamp - latest_time_).seconds() <= timeout_sec_;
    geometry_msgs::msg::Twist command = fresh ? latest_ : geometry_msgs::msg::Twist();
    if (!fresh) {
      clearHistory();
    }
    if (std::fabs(command.linear.x) < deadband_) command.linear.x = 0.0;
    if (std::fabs(command.linear.y) < deadband_) command.linear.y = 0.0;
    if (std::fabs(command.angular.z) < deadband_) command.angular.z = 0.0;
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
      // Establish the navigation gait before Nav2 is allowed to send its first goal.
      // Waiting for a non-zero cmd_vel here creates a circular dependency with
      // the tracking client's expected-motion safety gate.
      msg.cmd_type = protocol::msg::MotionServoCmd::SERVO_START;
      last_start_time_ = stamp;
    } else {
      msg.cmd_type = protocol::msg::MotionServoCmd::SERVO_DATA;
    }
    msg.cmd_source = cmd_source_;
    msg.value = gait_value_;
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
      clearHistory();
      have_command_ = false;
      motion_ready_ = false;
      last_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
  }

  int motion_id_{309};
  int cmd_source_{4};
  int gait_value_{2};
  double timeout_sec_{0.25};
  double start_retry_sec_{0.50};
  double smoothing_window_sec_{0.25};
  double deadband_{0.005};
  double max_x_{0.26};
  double max_y_{0.20};
  double max_w_{1.0};
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
  std::deque<TimedTwist> history_;
  double sum_x_{0.0};
  double sum_y_{0.0};
  double sum_w_{0.0};
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
