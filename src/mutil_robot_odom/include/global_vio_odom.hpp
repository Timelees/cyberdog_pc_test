#ifndef MUTIL_ROBOT_ODOM__GLOBAL_VIO_ODOM_HPP_
#define MUTIL_ROBOT_ODOM__GLOBAL_VIO_ODOM_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tf2_ros
{
class StaticTransformBroadcaster;
class TransformBroadcaster;
}  // namespace tf2_ros

class GlobalVioOdomNode final : public rclcpp::Node
{
public:
  GlobalVioOdomNode();
  ~GlobalVioOdomNode();

private:
  struct RobotRelay
  {
    std::string namespace_name;
    std::string input_topic;
    std::string output_topic;
    std::string child_frame_id;
    std::uint64_t input_count{0};
    std::uint64_t output_count{0};
    std::uint64_t last_status_output_count{0};
    rclcpp::Time last_odom_time;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
  };

  std::string trim_slashes(const std::string & value) const;
  std::string ensure_abs_topic(const std::string & topic) const;
  std::string build_input_topic(
    const std::string & namespace_name,
    const std::string & base_topic) const;
  std::string build_output_topic(
    const std::string & namespace_name,
    const std::string & leaf_topic) const;
  std::string build_child_frame_id(const std::string & namespace_name) const;

  void create_relays(const rclcpp::QoS & sub_qos, const rclcpp::QoS & pub_qos);
  void on_odom(std::size_t robot_index, const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_map_anchor_if_needed();
  void publish_robot_tf(const RobotRelay & relay, const nav_msgs::msg::Odometry & odom);
  void log_status();

  double status_period_sec_{5.0};
  std::vector<std::string> robot_namespaces_;
  std::string input_odom_topic_{"/odom_global"};
  std::string output_topic_prefix_{"/global_vio"};
  std::string output_odom_topic_{"odom"};
  std::string target_frame_id_{"tag_global"};
  std::string base_frame_name_{"base_link"};

  bool subscribe_best_effort_{true};
  bool publish_best_effort_{false};
  bool publish_tf_{true};
  bool publish_map_anchor_tf_{false};
  int tf_publish_every_n_{1};

  std::atomic<bool> shutting_down_{false};
  mutable std::mutex state_mutex_;
  std::vector<RobotRelay> relays_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Time last_status_time_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

#endif  // MUTIL_ROBOT_ODOM__GLOBAL_VIO_ODOM_HPP_
