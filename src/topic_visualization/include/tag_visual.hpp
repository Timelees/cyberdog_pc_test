#ifndef TOPIC_VISUALIZATION__TAG_VISUAL_HPP_
#define TOPIC_VISUALIZATION__TAG_VISUAL_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
namespace tf2_ros
{
class TransformBroadcaster;
class StaticTransformBroadcaster;
}  // namespace tf2_ros

class TagsVisualNode final : public rclcpp::Node
{
public:
  TagsVisualNode();
  ~TagsVisualNode();

private:
  std::string trim_slashes(const std::string & value) const;
  std::string select_namespace(int namespace_index);
  std::string build_input_topic(const std::string & base_topic) const;
  std::string build_output_topic(const std::string & base_topic) const;

  void on_odom_global(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_static_tag_anchor_if_needed();
  void publish_fixed_frame_anchor_if_needed();
  void publish_robot_tf(const nav_msgs::msg::Odometry & odom);
  void append_path_pose(const nav_msgs::msg::Odometry & odom);
  void log_status();

  // params
  double status_period_sec_{5.0};
  std::vector<std::string> namespaces_;
  std::string selected_namespace_;
  bool subscribe_best_effort_{true};
  bool publish_best_effort_{false};

  std::string input_odom_topic_{"/odom_global"};
  std::string target_frame_id_{"tag_0_observation"};
  std::string output_child_frame_id_{"base_link"};
  std::string output_topic_prefix_{"/viz/tags"};

  bool publish_tf_{true};
  int tf_publish_every_n_{1};

  bool path_enabled_{true};
  std::size_t path_max_poses_{2000};
  int path_publish_every_n_{5};

  // state
  std::atomic<bool> shutting_down_{false};
  std::uint64_t odom_count_{0};
  std::uint64_t last_status_count_{0};
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_status_time_;
  mutable std::mutex state_mutex_;
  nav_msgs::msg::Path path_;

  // ros
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_relay_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

#endif  // TOPIC_VISUALIZATION__TAG_VISUAL_HPP_