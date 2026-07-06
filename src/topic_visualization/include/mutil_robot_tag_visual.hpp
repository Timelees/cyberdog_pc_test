#ifndef TOPIC_VISUALIZATION__MUTIL_ROBOT_TAG_VISUAL_HPP_
#define TOPIC_VISUALIZATION__MUTIL_ROBOT_TAG_VISUAL_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MutilRobotTagVisualNode final : public rclcpp::Node
{
public:
  MutilRobotTagVisualNode();
  ~MutilRobotTagVisualNode();

private:
  struct RobotVisual
  {
    std::string namespace_name;
    std::string input_topic;
    std::size_t color_index{0};
    std::uint64_t odom_count{0};
    std::uint64_t last_status_count{0};
    rclcpp::Time last_odom_time;
    nav_msgs::msg::Path path;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  };

  std::string trim_slashes(const std::string & value) const;
  std::string ensure_abs_topic(const std::string & topic) const;
  std::string build_input_topic(
    const std::string & namespace_name,
    const std::string & base_topic) const;
  std::string build_output_topic(
    const std::string & namespace_name,
    const std::string & leaf_topic) const;

  void create_robot_visuals(const rclcpp::QoS & sub_qos, const rclcpp::QoS & pub_qos);
  void on_odom_global(std::size_t robot_index, const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_robot_marker(const RobotVisual & visual, const nav_msgs::msg::Odometry & odom);
  void publish_pose_text(const RobotVisual & visual, const nav_msgs::msg::Odometry & odom);
  void append_path_pose(RobotVisual & visual, const nav_msgs::msg::Odometry & odom);
  void log_status();
  std::array<float, 4> robot_color(std::size_t index) const;

  double status_period_sec_{5.0};
  std::vector<std::string> robot_namespaces_;
  std::string input_topic_prefix_{"/global_vio"};
  std::string input_odom_topic_{"odom"};
  std::string target_frame_id_{"tag_global"};
  std::string output_topic_prefix_{"/global_vio"};

  bool subscribe_best_effort_{true};
  bool publish_best_effort_{false};
  bool path_enabled_{true};
  bool pose_text_enabled_{true};
  bool robot_marker_enabled_{true};
  int path_publish_every_n_{1};
  int marker_publish_every_n_{5};
  std::size_t path_max_poses_{2000};
  double pose_text_z_offset_{0.6};
  double pose_text_scale_{0.18};

  std::atomic<bool> shutting_down_{false};
  mutable std::mutex state_mutex_;
  std::vector<RobotVisual> robots_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_text_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Time last_status_time_;
};

#endif  // TOPIC_VISUALIZATION__MUTIL_ROBOT_TAG_VISUAL_HPP_
