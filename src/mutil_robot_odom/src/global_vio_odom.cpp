#include "global_vio_odom.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kQuaternionNormEpsilon = 1e-6;
}  // namespace

GlobalVioOdomNode::GlobalVioOdomNode()
: Node("global_vio_odom")
{
  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  const auto sensor_qos = rclcpp::SensorDataQoS();

  status_period_sec_ = declare_parameter<double>("status_period_sec", 5.0);
  robot_namespaces_ = declare_parameter<std::vector<std::string>>(
    "robot_namespaces", std::vector<std::string>{"cyberdog_1"});
  input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/odom_global");
  output_topic_prefix_ = declare_parameter<std::string>("output_topic_prefix", "/global_vio");
  output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "odom");
  target_frame_id_ = trim_slashes(declare_parameter<std::string>("target_frame_id", "tag_global"));
  base_frame_name_ = trim_slashes(declare_parameter<std::string>("base_frame_name", "base_link"));

  subscribe_best_effort_ = declare_parameter<bool>("subscribe_best_effort", true);
  publish_best_effort_ = declare_parameter<bool>("publish_best_effort", false);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_map_anchor_tf_ = declare_parameter<bool>("publish_map_anchor_tf", false);
  tf_publish_every_n_ = declare_parameter<int>("tf_publish_every_n", 1);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  const auto sub_qos = subscribe_best_effort_ ? sensor_qos : reliable_qos;
  const auto pub_qos = publish_best_effort_ ? sensor_qos : reliable_qos;

  publish_map_anchor_if_needed();
  create_relays(sub_qos, pub_qos);

  if (status_period_sec_ > 0.0) {
    last_status_time_ = now();
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(status_period_sec_));
    status_timer_ = create_wall_timer(period, std::bind(&GlobalVioOdomNode::log_status, this));
  }

  RCLCPP_INFO(
    get_logger(),
    "global_vio_odom ready: robots=%zu input_suffix=%s output_prefix=%s frame=%s",
    relays_.size(),
    input_odom_topic_.c_str(),
    ensure_abs_topic(output_topic_prefix_).c_str(),
    target_frame_id_.c_str());
}

GlobalVioOdomNode::~GlobalVioOdomNode()
{
  shutting_down_.store(true);
  if (status_timer_) {
    status_timer_.reset();
  }
}

std::string GlobalVioOdomNode::trim_slashes(const std::string & value) const
{
  const auto start = value.find_first_not_of('/');
  if (start == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of('/');
  return value.substr(start, end - start + 1);
}

std::string GlobalVioOdomNode::ensure_abs_topic(const std::string & topic) const
{
  const auto normalized = trim_slashes(topic);
  if (normalized.empty()) {
    return "/";
  }
  return "/" + normalized;
}

std::string GlobalVioOdomNode::build_input_topic(
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

std::string GlobalVioOdomNode::build_output_topic(
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

std::string GlobalVioOdomNode::build_child_frame_id(const std::string & namespace_name) const
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

void GlobalVioOdomNode::create_relays(
  const rclcpp::QoS & sub_qos,
  const rclcpp::QoS & pub_qos)
{
  if (robot_namespaces_.empty()) {
    RCLCPP_WARN(get_logger(), "robot_namespaces is empty, no odom will be relayed");
    return;
  }

  for (std::size_t index = 0; index < robot_namespaces_.size(); ++index) {
    const auto namespace_name = trim_slashes(robot_namespaces_[index]);
    if (namespace_name.empty()) {
      RCLCPP_WARN(get_logger(), "skip empty robot namespace at index %zu", index);
      continue;
    }

    RobotRelay relay;
    relay.namespace_name = namespace_name;
    relay.input_topic = build_input_topic(namespace_name, input_odom_topic_);
    relay.output_topic = build_output_topic(namespace_name, output_odom_topic_);
    relay.child_frame_id = build_child_frame_id(namespace_name);
    relay.publisher = create_publisher<nav_msgs::msg::Odometry>(relay.output_topic, pub_qos);

    relays_.push_back(std::move(relay));
    const auto robot_index = relays_.size() - 1;
    relays_[robot_index].subscription = create_subscription<nav_msgs::msg::Odometry>(
      relays_[robot_index].input_topic,
      sub_qos,
      [this, robot_index](const nav_msgs::msg::Odometry::SharedPtr msg) {
        on_odom(robot_index, msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "relay robot[%zu] ns=%s input=%s output=%s child_frame=%s",
      robot_index,
      relays_[robot_index].namespace_name.c_str(),
      relays_[robot_index].input_topic.c_str(),
      relays_[robot_index].output_topic.c_str(),
      relays_[robot_index].child_frame_id.c_str());
  }
}

void GlobalVioOdomNode::on_odom(
  std::size_t robot_index,
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (shutting_down_.load() || !msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (robot_index >= relays_.size()) {
    return;
  }

  auto & relay = relays_[robot_index];
  ++relay.input_count;

  nav_msgs::msg::Odometry out = *msg;
  if (!target_frame_id_.empty()) {
    out.header.frame_id = target_frame_id_;
  }
  out.child_frame_id = relay.child_frame_id;
  if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
    out.header.stamp = now();
  }

  relay.publisher->publish(out);
  ++relay.output_count;
  relay.last_odom_time = rclcpp::Time(out.header.stamp);

  publish_robot_tf(relay, out);
}

void GlobalVioOdomNode::publish_map_anchor_if_needed()
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

void GlobalVioOdomNode::publish_robot_tf(
  const RobotRelay & relay,
  const nav_msgs::msg::Odometry & odom)
{
  if (!publish_tf_ || !tf_broadcaster_ || target_frame_id_.empty() || relay.child_frame_id.empty()) {
    return;
  }
  if (relay.output_count % static_cast<std::uint64_t>(std::max(1, tf_publish_every_n_)) != 0) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  tf.header = odom.header;
  tf.header.frame_id = target_frame_id_;
  tf.child_frame_id = relay.child_frame_id;
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

void GlobalVioOdomNode::log_status()
{
  if (shutting_down_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto current_time = now();
  const auto dt = (current_time - last_status_time_).seconds();
  std::ostringstream stream;
  stream << "global vio relay stats:";
  for (auto & relay : relays_) {
    const auto hz = dt > 1e-6 ?
      static_cast<double>(relay.output_count - relay.last_status_output_count) / dt : 0.0;
    std::string age_text = "never";
    if (relay.last_odom_time.nanoseconds() > 0) {
      age_text = std::to_string((current_time - relay.last_odom_time).seconds()) + "s";
    }
    stream << " " << relay.namespace_name
           << "(input=" << relay.input_count
           << ",output=" << relay.output_count
           << ",hz=" << std::fixed << std::setprecision(2) << hz
           << ",age=" << age_text << ")";
    relay.last_status_output_count = relay.output_count;
  }
  last_status_time_ = current_time;
  RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
}

int main(int argc, char ** argv)
{
  int exit_code = 0;
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalVioOdomNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("global_vio_odom"), "fatal error: %s", e.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
