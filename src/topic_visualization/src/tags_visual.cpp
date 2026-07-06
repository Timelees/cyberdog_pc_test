#include "tag_visual.hpp"

#include <chrono>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kQuaternionNormEpsilon = 1e-6;
}  // namespace

TagsVisualNode::TagsVisualNode()
: Node("tags_visual")
{
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto sensor_qos = rclcpp::SensorDataQoS();

  status_period_sec_ = declare_parameter<double>("status_period_sec", 5.0);
  namespaces_ = declare_parameter<std::vector<std::string>>("namespace", std::vector<std::string>{});
  selected_namespace_ = select_namespace(declare_parameter<int>("namespace_index", 0));

  input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/odom_global");
  target_frame_id_ = declare_parameter<std::string>("target_frame_id", "tag_0_observation");
  output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "base_link");
  output_topic_prefix_ = declare_parameter<std::string>("output_topic_prefix", "/viz/tags");

  subscribe_best_effort_ = declare_parameter<bool>("subscribe_best_effort", true);
  publish_best_effort_ = declare_parameter<bool>("publish_best_effort", false);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  tf_publish_every_n_ = declare_parameter<int>("tf_publish_every_n", 1);

  path_enabled_ = declare_parameter<bool>("path_enabled", true);
  path_max_poses_ = static_cast<std::size_t>(declare_parameter<int>("path_max_poses", 2000));
  path_publish_every_n_ = declare_parameter<int>("path_publish_every_n", 5);

  // Marker visuals are intentionally omitted here to keep this node compatible with
  // lightweight clangd stubs used in this workspace. RViz can visualize TF + Path directly.

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  const auto sub_qos = subscribe_best_effort_ ? sensor_qos : reliable_qos;
  const auto pub_qos = publish_best_effort_ ? sensor_qos : reliable_qos;

  const auto input_topic = build_input_topic(input_odom_topic_);
  const auto relay_odom_topic = build_output_topic("odom");
  const auto relay_path_topic = build_output_topic("path");

  pub_odom_relay_ = create_publisher<nav_msgs::msg::Odometry>(relay_odom_topic, pub_qos);
  if (path_enabled_) {
    pub_path_ = create_publisher<nav_msgs::msg::Path>(relay_path_topic, pub_qos);
    path_.header.frame_id = target_frame_id_;
  }

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    input_topic,
    sub_qos,
    std::bind(&TagsVisualNode::on_odom_global, this, std::placeholders::_1));

  publish_fixed_frame_anchor_if_needed();
  publish_static_tag_anchor_if_needed();

  if (status_period_sec_ > 0.0) {
    last_status_time_ = now();
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(status_period_sec_));
    status_timer_ = create_wall_timer(period, std::bind(&TagsVisualNode::log_status, this));
  }

  RCLCPP_INFO(
    get_logger(),
    "tags_visual ready: input=%s relay_odom=%s relay_path=%s fixed(tag)=%s child=%s",
    input_topic.c_str(),
    relay_odom_topic.c_str(),
    relay_path_topic.c_str(),
    target_frame_id_.c_str(),
    output_child_frame_id_.c_str());
}

TagsVisualNode::~TagsVisualNode()
{
  shutting_down_.store(true);
  if (status_timer_) {
    status_timer_.reset();
  }
}

std::string TagsVisualNode::trim_slashes(const std::string & value) const
{
  auto start = value.find_first_not_of('/');
  if (start == std::string::npos) {
    return "";
  }
  auto end = value.find_last_not_of('/');
  return value.substr(start, end - start + 1);
}

std::string TagsVisualNode::select_namespace(int namespace_index)
{
  if (namespaces_.empty()) {
    RCLCPP_INFO(get_logger(), "namespace list is empty, use raw input topics");
    return "";
  }
  if (namespace_index < 0 || namespace_index >= static_cast<int>(namespaces_.size())) {
    RCLCPP_WARN(
      get_logger(),
      "namespace_index=%d out of range, fallback to namespace[0]=%s",
      namespace_index,
      namespaces_.front().c_str());
    return namespaces_.front();
  }
  RCLCPP_INFO(
    get_logger(),
    "selected namespace[%d]=%s",
    namespace_index,
    namespaces_[namespace_index].c_str());
  return namespaces_[namespace_index];
}

std::string TagsVisualNode::build_input_topic(const std::string & base_topic) const
{
  const auto normalized_topic = trim_slashes(base_topic);
  if (selected_namespace_.empty()) {
    return normalized_topic.empty() ? "/" : ("/" + normalized_topic);
  }
  const auto normalized_ns = trim_slashes(selected_namespace_);
  if (normalized_topic.empty()) {
    return "/" + normalized_ns;
  }
  const auto prefix = normalized_ns + "/";
  if (normalized_topic.rfind(prefix, 0) == 0) {
    return "/" + normalized_topic;
  }
  return "/" + normalized_ns + "/" + normalized_topic;
}

std::string TagsVisualNode::build_output_topic(const std::string & base_topic) const
{
  const auto prefix = trim_slashes(output_topic_prefix_);
  const auto topic = trim_slashes(base_topic);
  if (prefix.empty()) {
    return topic.empty() ? std::string{} : ("/" + topic);
  }
  if (topic.empty()) {
    return "/" + prefix;
  }
  return "/" + prefix + "/" + topic;
}

void TagsVisualNode::publish_fixed_frame_anchor_if_needed()
{
  if (!static_tf_broadcaster_ || target_frame_id_.empty()) {
    return;
  }

  const bool publish_anchor = declare_parameter<bool>("publish_fixed_frame_anchor", true);
  if (!publish_anchor) {
    return;
  }

  const auto anchor_child = trim_slashes(
    declare_parameter<std::string>("fixed_frame_anchor_child", "tag_0_observation"));
  if (anchor_child.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = target_frame_id_;
  tf.child_frame_id = anchor_child;
  tf.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(tf);
  RCLCPP_INFO(
    get_logger(),
    "published fixed-frame anchor on /tf_static: %s -> %s",
    target_frame_id_.c_str(),
    anchor_child.c_str());
}

void TagsVisualNode::publish_static_tag_anchor_if_needed()
{
  if (!static_tf_broadcaster_ || target_frame_id_.empty()) {
    return;
  }

  // Optional convenience: publish map -> <tag_frame> identity so RViz can use either frame as Fixed Frame.
  // This does NOT override apriltag's own TF chain; it only provides a root anchor when users keep Fixed Frame "map".
  const bool publish_anchor = declare_parameter<bool>("publish_map_anchor_tf", false);
  if (!publish_anchor) {
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

void TagsVisualNode::publish_robot_tf(const nav_msgs::msg::Odometry & odom)
{
  if (!publish_tf_ || !tf_broadcaster_) {
    return;
  }
  if (odom_count_ % static_cast<std::uint64_t>(std::max(1, tf_publish_every_n_)) != 0) {
    return;
  }

  const auto parent = target_frame_id_.empty() ? odom.header.frame_id : target_frame_id_;
  const auto child = output_child_frame_id_.empty() ? odom.child_frame_id : output_child_frame_id_;
  if (parent.empty() || child.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  tf.header = odom.header;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = odom.pose.pose.position.x;
  tf.transform.translation.y = odom.pose.pose.position.y;
  tf.transform.translation.z = odom.pose.pose.position.z;
  tf.transform.rotation = odom.pose.pose.orientation;

  // Normalize quaternion (RViz dislikes near-zero norm)
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

  if (tf.header.stamp.sec == 0 && tf.header.stamp.nanosec == 0) {
    tf.header.stamp = now();
  }
  tf_broadcaster_->sendTransform(tf);
}

void TagsVisualNode::append_path_pose(const nav_msgs::msg::Odometry & odom)
{
  if (!path_enabled_ || !pub_path_) {
    return;
  }
  if (odom_count_ % static_cast<std::uint64_t>(std::max(1, path_publish_every_n_)) != 0) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  pose.header.frame_id = target_frame_id_.empty() ? odom.header.frame_id : target_frame_id_;
  pose.pose = odom.pose.pose;

  path_.header = pose.header;
  path_.poses.push_back(std::move(pose));
  if (path_.poses.size() > path_max_poses_) {
    const auto overflow = path_.poses.size() - path_max_poses_;
    path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<std::ptrdiff_t>(overflow));
  }
  pub_path_->publish(path_);
}

void TagsVisualNode::on_odom_global(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (shutting_down_.load() || !msg) {
    return;
  }

  nav_msgs::msg::Odometry out = *msg;
  if (!target_frame_id_.empty()) {
    out.header.frame_id = target_frame_id_;
  }
  if (!output_child_frame_id_.empty()) {
    out.child_frame_id = output_child_frame_id_;
  }
  if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
    out.header.stamp = now();
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++odom_count_;
    last_odom_time_ = rclcpp::Time(out.header.stamp);
  }

  if (pub_odom_relay_) {
    pub_odom_relay_->publish(out);
  }

  publish_robot_tf(out);
  append_path_pose(out);
}

void TagsVisualNode::log_status()
{
  if (shutting_down_.load()) {
    return;
  }
  std::uint64_t count = 0;
  std::uint64_t last_status_count = 0;
  rclcpp::Time last;
  rclcpp::Time last_status;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    count = odom_count_;
    last_status_count = last_status_count_;
    last = last_odom_time_;
    last_status = last_status_time_;
  }
  auto age_text = std::string("never");
  if (last.nanoseconds() > 0) {
    age_text = std::to_string((now() - last).seconds()) + "s";
  }
  const auto now_time = now();
  const auto dt = (now_time - last_status).seconds();
  const auto hz = dt > 1e-6 ? static_cast<double>(count - last_status_count) / dt : 0.0;
  RCLCPP_INFO(
    get_logger(),
    "odom_global relay stats: count=%llu hz=%.2f last_age=%s ns=%s",
    static_cast<unsigned long long>(count),
    hz,
    age_text.c_str(),
    selected_namespace_.empty() ? "(none)" : selected_namespace_.c_str());
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_status_count_ = count;
    last_status_time_ = now_time;
  }
}

int main(int argc, char ** argv)
{
  int exit_code = 0;
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TagsVisualNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("tags_visual"), "fatal error: %s", e.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
