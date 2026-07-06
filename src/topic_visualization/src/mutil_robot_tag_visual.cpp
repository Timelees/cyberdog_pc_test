#include "mutil_robot_tag_visual.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kQuaternionNormEpsilon = 1e-6;
}  // namespace

MutilRobotTagVisualNode::MutilRobotTagVisualNode()
: Node("mutil_robot_tag_visual")
{
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto sensor_qos = rclcpp::SensorDataQoS();

  status_period_sec_ = declare_parameter<double>("status_period_sec", 5.0);
  robot_namespaces_ = declare_parameter<std::vector<std::string>>(
    "robot_namespaces", std::vector<std::string>{"cyberdog_1", "cyberdog_2"});
  input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/odom_global");
  target_frame_id_ = trim_slashes(declare_parameter<std::string>("target_frame_id", "tag_global"));
  output_topic_prefix_ = declare_parameter<std::string>("output_topic_prefix", "/viz/tags_multi");
  base_frame_name_ = trim_slashes(declare_parameter<std::string>("base_frame_name", "base_link"));

  subscribe_best_effort_ = declare_parameter<bool>("subscribe_best_effort", true);
  publish_best_effort_ = declare_parameter<bool>("publish_best_effort", false);
  relay_odom_enabled_ = declare_parameter<bool>("relay_odom_enabled", true);
  path_enabled_ = declare_parameter<bool>("path_enabled", true);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  pose_text_enabled_ = declare_parameter<bool>("pose_text_enabled", true);
  robot_marker_enabled_ = declare_parameter<bool>("robot_marker_enabled", true);
  publish_map_anchor_tf_ = declare_parameter<bool>("publish_map_anchor_tf", false);
  tf_publish_every_n_ = declare_parameter<int>("tf_publish_every_n", 1);
  path_publish_every_n_ = declare_parameter<int>("path_publish_every_n", 1);
  marker_publish_every_n_ = declare_parameter<int>("marker_publish_every_n", 5);
  path_max_poses_ = static_cast<std::size_t>(declare_parameter<int>("path_max_poses", 2000));
  pose_text_z_offset_ = declare_parameter<double>("pose_text_z_offset", 0.6);
  pose_text_scale_ = declare_parameter<double>("pose_text_scale", 0.18);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  const auto sub_qos = subscribe_best_effort_ ? sensor_qos : reliable_qos;
  const auto pub_qos = publish_best_effort_ ? sensor_qos : reliable_qos;

  if (pose_text_enabled_) {
    pose_text_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      ensure_abs_topic(output_topic_prefix_ + "/pose_text"), pub_qos);
  }
  if (robot_marker_enabled_) {
    robot_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      ensure_abs_topic(output_topic_prefix_ + "/robot_markers"), pub_qos);
  }

  publish_map_anchor_if_needed();
  create_robot_visuals(sub_qos, pub_qos);

  if (status_period_sec_ > 0.0) {
    last_status_time_ = now();
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(status_period_sec_));
    status_timer_ = create_wall_timer(period, std::bind(&MutilRobotTagVisualNode::log_status, this));
  }

  RCLCPP_INFO(
    get_logger(),
    "mutil_robot_tag_visual ready: robots=%zu input_suffix=%s fixed_frame=%s output_prefix=%s",
    robots_.size(),
    input_odom_topic_.c_str(),
    target_frame_id_.c_str(),
    ensure_abs_topic(output_topic_prefix_).c_str());
}

MutilRobotTagVisualNode::~MutilRobotTagVisualNode()
{
  shutting_down_.store(true);
  if (status_timer_) {
    status_timer_.reset();
  }
}

std::string MutilRobotTagVisualNode::trim_slashes(const std::string & value) const
{
  const auto start = value.find_first_not_of('/');
  if (start == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of('/');
  return value.substr(start, end - start + 1);
}

std::string MutilRobotTagVisualNode::ensure_abs_topic(const std::string & topic) const
{
  const auto normalized = trim_slashes(topic);
  if (normalized.empty()) {
    return "/";
  }
  return "/" + normalized;
}

std::string MutilRobotTagVisualNode::build_namespaced_topic(
  const std::string & namespace_name,
  const std::string & base_topic) const
{
  const auto ns = trim_slashes(namespace_name);
  const auto topic = trim_slashes(base_topic);
  if (ns.empty()) {
    return ensure_abs_topic(topic);
  }
  if (topic.empty()) {
    return "/" + ns;
  }
  const auto prefix = ns + "/";
  if (topic.rfind(prefix, 0) == 0) {
    return "/" + topic;
  }
  return "/" + ns + "/" + topic;
}

std::string MutilRobotTagVisualNode::build_output_topic(
  const std::string & namespace_name,
  const std::string & leaf_topic) const
{
  const auto prefix = trim_slashes(output_topic_prefix_);
  const auto ns = trim_slashes(namespace_name);
  const auto leaf = trim_slashes(leaf_topic);

  std::string topic;
  if (!prefix.empty()) {
    topic += prefix;
  }
  if (!ns.empty()) {
    if (!topic.empty()) {
      topic += "/";
    }
    topic += ns;
  }
  if (!leaf.empty()) {
    if (!topic.empty()) {
      topic += "/";
    }
    topic += leaf;
  }
  return ensure_abs_topic(topic);
}

std::string MutilRobotTagVisualNode::build_child_frame_id(const std::string & namespace_name) const
{
  const auto ns = trim_slashes(namespace_name);
  const auto base = trim_slashes(base_frame_name_);
  if (ns.empty()) {
    return base.empty() ? "base_link" : base;
  }
  if (base.empty()) {
    return ns + "_base_link";
  }
  return ns + "_" + base;
}

void MutilRobotTagVisualNode::create_robot_visuals(
  const rclcpp::QoS & sub_qos,
  const rclcpp::QoS & pub_qos)
{
  if (robot_namespaces_.empty()) {
    RCLCPP_WARN(get_logger(), "robot_namespaces is empty, no robot odom will be visualized");
    return;
  }

  for (std::size_t index = 0; index < robot_namespaces_.size(); ++index) {
    const auto namespace_name = trim_slashes(robot_namespaces_[index]);
    if (namespace_name.empty()) {
      RCLCPP_WARN(get_logger(), "skip empty robot namespace at index %zu", index);
      continue;
    }

    RobotVisual visual;
    visual.namespace_name = namespace_name;
    visual.input_topic = build_namespaced_topic(namespace_name, input_odom_topic_);
    visual.child_frame_id = build_child_frame_id(namespace_name);
    visual.color_index = index;
    visual.path.header.frame_id = target_frame_id_;

    if (relay_odom_enabled_) {
      visual.odom_publisher = create_publisher<nav_msgs::msg::Odometry>(
        build_output_topic(namespace_name, "odom"), pub_qos);
    }
    if (path_enabled_) {
      visual.path_publisher = create_publisher<nav_msgs::msg::Path>(
        build_output_topic(namespace_name, "path"), pub_qos);
    }

    robots_.push_back(std::move(visual));
    const auto robot_index = robots_.size() - 1;
    robots_[robot_index].odom_subscription = create_subscription<nav_msgs::msg::Odometry>(
      robots_[robot_index].input_topic,
      sub_qos,
      [this, robot_index](const nav_msgs::msg::Odometry::SharedPtr msg) {
        on_odom_global(robot_index, msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "visualize robot[%zu] ns=%s input=%s child_frame=%s odom_out=%s path_out=%s",
      robot_index,
      robots_[robot_index].namespace_name.c_str(),
      robots_[robot_index].input_topic.c_str(),
      robots_[robot_index].child_frame_id.c_str(),
      build_output_topic(namespace_name, "odom").c_str(),
      build_output_topic(namespace_name, "path").c_str());
  }
}

void MutilRobotTagVisualNode::on_odom_global(
  std::size_t robot_index,
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (shutting_down_.load() || !msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (robot_index >= robots_.size()) {
    return;
  }

  auto & visual = robots_[robot_index];
  nav_msgs::msg::Odometry out = *msg;
  if (!target_frame_id_.empty()) {
    out.header.frame_id = target_frame_id_;
  }
  out.child_frame_id = visual.child_frame_id;
  if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
    out.header.stamp = now();
  }

  ++visual.odom_count;
  visual.last_odom_time = rclcpp::Time(out.header.stamp);

  if (visual.odom_publisher) {
    visual.odom_publisher->publish(out);
  }

  publish_robot_tf(visual, out);
  append_path_pose(visual, out);
  publish_robot_marker(visual, out);
  publish_pose_text(visual, out);
}

void MutilRobotTagVisualNode::publish_map_anchor_if_needed()
{
  if (!publish_map_anchor_tf_ || !static_tf_broadcaster_ || target_frame_id_.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = "map";
  tf.child_frame_id = target_frame_id_;
  tf.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(tf);
  RCLCPP_INFO(get_logger(), "published static TF anchor: map -> %s", target_frame_id_.c_str());
}

void MutilRobotTagVisualNode::publish_robot_tf(
  const RobotVisual & visual,
  const nav_msgs::msg::Odometry & odom)
{
  if (!publish_tf_ || !tf_broadcaster_ || target_frame_id_.empty() || visual.child_frame_id.empty()) {
    return;
  }
  if (visual.odom_count % static_cast<std::uint64_t>(std::max(1, tf_publish_every_n_)) != 0) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  tf.header = odom.header;
  tf.header.frame_id = target_frame_id_;
  tf.child_frame_id = visual.child_frame_id;
  tf.transform.translation.x = odom.pose.pose.position.x;
  tf.transform.translation.y = odom.pose.pose.position.y;
  tf.transform.translation.z = odom.pose.pose.position.z;
  tf.transform.rotation = odom.pose.pose.orientation;

  tf2::Quaternion q(
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z,
    tf.transform.rotation.w);
  if (q.length2() <= kQuaternionNormEpsilon) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    q.normalize();
  }
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(tf);
}

void MutilRobotTagVisualNode::publish_robot_marker(
  const RobotVisual & visual,
  const nav_msgs::msg::Odometry & odom)
{
  if (!robot_marker_enabled_ || !robot_marker_publisher_) {
    return;
  }
  if (visual.odom_count % static_cast<std::uint64_t>(std::max(1, marker_publish_every_n_)) != 0) {
    return;
  }

  const auto color = robot_color(visual.color_index);
  visualization_msgs::msg::Marker marker;
  marker.header = odom.header;
  marker.header.frame_id = target_frame_id_;
  marker.ns = "robot_arrow";
  marker.id = static_cast<int>(visual.color_index);
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = odom.pose.pose;
  marker.scale.x = 0.65;
  marker.scale.y = 0.14;
  marker.scale.z = 0.14;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  robot_marker_publisher_->publish(marker);
}

void MutilRobotTagVisualNode::publish_pose_text(
  const RobotVisual & visual,
  const nav_msgs::msg::Odometry & odom)
{
  if (!pose_text_enabled_ || !pose_text_publisher_) {
    return;
  }
  if (visual.odom_count % static_cast<std::uint64_t>(std::max(1, marker_publish_every_n_)) != 0) {
    return;
  }

  tf2::Quaternion q(
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  if (q.length2() > kQuaternionNormEpsilon) {
    q.normalize();
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  std::ostringstream text;
  text << std::fixed << std::setprecision(2)
       << visual.namespace_name
       << " x:" << odom.pose.pose.position.x
       << " y:" << odom.pose.pose.position.y
       << " yaw:" << yaw;

  const auto color = robot_color(visual.color_index);
  visualization_msgs::msg::Marker marker;
  marker.header = odom.header;
  marker.header.frame_id = target_frame_id_;
  marker.ns = "robot_pose_text";
  marker.id = static_cast<int>(visual.color_index);
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = odom.pose.pose.position;
  marker.pose.position.z += pose_text_z_offset_;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = pose_text_scale_;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.text = text.str();
  pose_text_publisher_->publish(marker);
}

void MutilRobotTagVisualNode::append_path_pose(
  RobotVisual & visual,
  const nav_msgs::msg::Odometry & odom)
{
  if (!path_enabled_ || !visual.path_publisher) {
    return;
  }
  if (visual.odom_count % static_cast<std::uint64_t>(std::max(1, path_publish_every_n_)) != 0) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  pose.header.frame_id = target_frame_id_;
  pose.pose = odom.pose.pose;

  visual.path.header = pose.header;
  visual.path.poses.push_back(std::move(pose));
  if (visual.path.poses.size() > path_max_poses_) {
    const auto overflow = visual.path.poses.size() - path_max_poses_;
    visual.path.poses.erase(
      visual.path.poses.begin(),
      visual.path.poses.begin() + static_cast<std::ptrdiff_t>(overflow));
  }
  visual.path_publisher->publish(visual.path);
}

void MutilRobotTagVisualNode::log_status()
{
  if (shutting_down_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto current_time = now();
  const auto dt = (current_time - last_status_time_).seconds();
  std::ostringstream stream;
  stream << "mutil tag visual stats:";
  for (auto & visual : robots_) {
    const auto hz = dt > 1e-6 ?
      static_cast<double>(visual.odom_count - visual.last_status_count) / dt : 0.0;
    std::string age_text = "never";
    if (visual.last_odom_time.nanoseconds() > 0) {
      age_text = std::to_string((current_time - visual.last_odom_time).seconds()) + "s";
    }
    stream << " " << visual.namespace_name
           << "(count=" << visual.odom_count
           << ",hz=" << std::fixed << std::setprecision(2) << hz
           << ",age=" << age_text << ")";
    visual.last_status_count = visual.odom_count;
  }
  last_status_time_ = current_time;
  RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
}

std::array<float, 4> MutilRobotTagVisualNode::robot_color(std::size_t index) const
{
  static const std::array<std::array<float, 4>, 8> colors{{
    {0.95F, 0.15F, 0.15F, 1.0F},
    {0.10F, 0.65F, 0.95F, 1.0F},
    {0.10F, 0.80F, 0.35F, 1.0F},
    {0.95F, 0.70F, 0.10F, 1.0F},
    {0.75F, 0.35F, 0.95F, 1.0F},
    {0.95F, 0.40F, 0.70F, 1.0F},
    {0.20F, 0.85F, 0.80F, 1.0F},
    {0.75F, 0.75F, 0.75F, 1.0F},
  }};
  return colors[index % colors.size()];
}

int main(int argc, char ** argv)
{
  int exit_code = 0;
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MutilRobotTagVisualNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("mutil_robot_tag_visual"), "fatal error: %s", e.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
