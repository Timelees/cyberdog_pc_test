// Copyright (c) 2026
// SPDX-License-Identifier: MIT

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace
{

std::string trim_slashes(std::string s)
{
  while (!s.empty() && s.front() == '/') s.erase(s.begin());
  while (!s.empty() && s.back() == '/') s.pop_back();
  return s;
}

std::string ensure_abs_topic(const std::string & topic)
{
  if (topic.empty()) return topic;
  if (topic.front() == '/') return topic;
  return "/" + topic;
}

std::string trim(std::string s)
{
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\t')) {
    s.pop_back();
  }
  std::size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {
    ++i;
  }
  return s.substr(i);
}

std::string getenv_str(const char * key)
{
  const char * v = std::getenv(key);
  return v ? std::string(v) : std::string();
}

std::string run_and_capture_first_line(const std::string & cmd)
{
  std::array<char, 256> buf{};
  std::string out;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    return "";
  }
  if (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) {
    out = buf.data();
  }
  (void)pclose(pipe);
  return trim(out);
}

std::string get_robot_namespace_via_cyberdog_bringup()
{
  // Match the launch-side behavior:
  // get_package_share_directory("cyberdog_bringup") + "/bringup" then manual.get_namespace()
  const std::string cmd =
    "python3 -c \""
    "from ament_index_python.packages import get_package_share_directory;"
    "import os,sys;"
    "p=get_package_share_directory('cyberdog_bringup');"
    "sys.path.append(os.path.join(p,'bringup'));"
    "from manual import get_namespace;"
    "print((get_namespace() or '').strip('/'))"
    "\" 2>/dev/null";
  return run_and_capture_first_line(cmd);
}

tf2::Transform pose_to_tf2(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  if (q.length2() <= 1e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    q.normalize();
  }
  tf.setRotation(q);
  return tf;
}

tf2::Transform make_wall_tag_to_global_transform()
{
  // AprilTag on a wall: tag +Z points out from the wall toward the camera
  // (roughly robot +X), and tag +Y points down in the image/tag plane.
  // Define a navigation-friendly global frame:
  //   global +X = tag +Z
  //   global +Y = tag -X
  //   global +Z = tag -Y
  // R_tag_global columns are the global axes expressed in the raw tag frame.
  tf2::Matrix3x3 R_tag_global(
    0.0, -1.0, 0.0,
    0.0,  0.0, -1.0,
    1.0,  0.0, 0.0);

  tf2::Transform T_tag_global;
  T_tag_global.setIdentity();
  T_tag_global.setBasis(R_tag_global);
  return T_tag_global.inverse();
}

tf2::Transform make_rotation_x_pi_transform()
{
  tf2::Transform tf;
  tf.setIdentity();
  tf2::Quaternion q;
  q.setRPY(3.14159265358979323846, 0.0, 0.0);
  q.normalize();
  tf.setRotation(q);
  return tf;
}

tf2::Transform transform_msg_to_tf2(const geometry_msgs::msg::Transform & msg)
{
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
  tf2::Quaternion q(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
  if (q.length2() <= 1e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    q.normalize();
  }
  tf.setRotation(q);
  return tf;
}

geometry_msgs::msg::Pose tf2_to_pose(const tf2::Transform & tf)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  const auto & q = tf.getRotation();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::Transform tf2_to_transform_msg(const tf2::Transform & tf)
{
  geometry_msgs::msg::Transform msg;
  msg.translation.x = tf.getOrigin().x();
  msg.translation.y = tf.getOrigin().y();
  msg.translation.z = tf.getOrigin().z();
  const auto & q = tf.getRotation();
  msg.rotation.x = q.x();
  msg.rotation.y = q.y();
  msg.rotation.z = q.z();
  msg.rotation.w = q.w();
  return msg;
}

}  // namespace


class OdomTransformNode final : public rclcpp::Node
{
public:
  explicit OdomTransformNode(const rclcpp::NodeOptions & options)
  : Node("odom_transform", options),
    tf_buffer_(this->get_clock()),
    static_tf_broadcaster_(this)
  {
    // Parameters
    robot_namespace_ = trim_slashes(this->declare_parameter<std::string>("robot_namespace", ""));
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "odom_slam");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "odom_global");
    target_frame_ = trim_slashes(this->declare_parameter<std::string>("target_frame", ""));
    output_frame_ = trim_slashes(this->declare_parameter<std::string>("output_frame", "tag_global"));
    apply_wall_tag_alignment_ =
      this->declare_parameter<bool>("apply_wall_tag_alignment", true);
    rotate_output_x_180_ =
      this->declare_parameter<bool>("rotate_output_x_180", true);
    status_period_sec_ = this->declare_parameter<double>("status_period_sec", 5.0);
    odom_child_frame_fallback_ =
      trim_slashes(this->declare_parameter<std::string>("odom_child_frame_fallback", "base_link"));
    use_latest_tf_on_failure_ = this->declare_parameter<bool>("use_latest_tf_on_failure", true);
    // Cross-machine CycloneDDS over WiFi: a local RELIABLE subscriber on the robot can
    // prevent remote subscribers from receiving the same topic. BestEffort avoids that.
    odom_subscribe_best_effort_ =
      this->declare_parameter<bool>("odom_subscribe_best_effort", true);
    odom_publish_best_effort_ =
      this->declare_parameter<bool>("odom_publish_best_effort", true);

    // Namespace policy:
    // - Prefer the parameter set by launch (computed via cyberdog_bringup/manual.py:get_namespace()).
    // - If not set, fall back to this node's ROS namespace (e.g. when launched under PushRosNamespace()).
    if (robot_namespace_.empty()) {
      robot_namespace_ = trim_slashes(this->get_namespace());
    }
    // - If still empty (e.g. `ros2 run` default namespace "/"), try environment variables.
    if (robot_namespace_.empty()) {
      robot_namespace_ = trim_slashes(getenv_str("CYBERDOG_NAMESPACE"));
    }
    if (robot_namespace_.empty()) {
      robot_namespace_ = trim_slashes(getenv_str("ROBOT_NAMESPACE"));
    }
    if (robot_namespace_.empty()) {
      robot_namespace_ = trim_slashes(getenv_str("ROS_NAMESPACE"));
    }
    // - Last resort: call cyberdog_bringup/manual.py:get_namespace() via python.
    if (robot_namespace_.empty()) {
      robot_namespace_ = trim_slashes(get_robot_namespace_via_cyberdog_bringup());
    }
    RCLCPP_INFO(get_logger(), "robot_namespace: '%s'", robot_namespace_.c_str());

    // When a robot namespace is known, default to namespaced TF only to reduce cross-host DDS load.
    tf_subscribe_global_ =
      this->declare_parameter<bool>("tf_subscribe_global", robot_namespace_.empty());
    publish_static_tf_to_global_ =
      this->declare_parameter<bool>("publish_static_tf_to_global", robot_namespace_.empty());

    // Default target frame:
    // Tag is fixed; use a global tag frame without robot namespace.
    if (target_frame_.empty()) {
      target_frame_ = "tag_0_observation";
    }
    if (output_frame_.empty()) {
      output_frame_ = target_frame_;
    }
    T_output_tag_.setIdentity();
    if (apply_wall_tag_alignment_) {
      T_output_tag_ = make_wall_tag_to_global_transform();
    }
    if (rotate_output_x_180_) {
      T_output_tag_ = make_rotation_x_pi_transform() * T_output_tag_;
    }

    if (!publish_static_tf_to_global_ && !robot_namespace_.empty()) {
      const auto tf_static_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      const auto ns_tf_static_topic = "/" + robot_namespace_ + "/tf_static";
      ns_static_tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
        ns_tf_static_topic, tf_static_qos);
      RCLCPP_INFO(get_logger(), "static TF will publish on: %s", ns_tf_static_topic.c_str());
    }

    publish_output_tag_static_tf();

    // Force namespaced topics using the computed robot namespace.
    // Use absolute topics so that even if the node is launched without PushRosNamespace,
    // we still subscribe/publish under the robot namespace.
    const std::string odom_topic_abs = robot_namespace_.empty()
      ? ensure_abs_topic(trim_slashes(odom_topic_))
      : ensure_abs_topic(robot_namespace_ + "/" + trim_slashes(odom_topic_));
    const std::string output_topic_abs = robot_namespace_.empty()
      ? ensure_abs_topic(trim_slashes(output_topic_))
      : ensure_abs_topic(robot_namespace_ + "/" + trim_slashes(output_topic_));

    RCLCPP_INFO(get_logger(), "subscribe odom: '%s'", odom_topic_abs.c_str());
    RCLCPP_INFO(get_logger(), "publish  odom: '%s'", output_topic_abs.c_str());
    RCLCPP_INFO(get_logger(), "target frame: '%s'", target_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "output frame: '%s' (wall tag alignment: %s, rotate x 180: %s)",
      output_frame_.c_str(),
      apply_wall_tag_alignment_ ? "enabled" : "disabled",
      rotate_output_x_180_ ? "enabled" : "disabled");

    setup_tf_subscriptions();

    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto odom_sub_qos = odom_subscribe_best_effort_
      ? rclcpp::SensorDataQoS().keep_last(50)
      : rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    const auto odom_pub_qos = odom_publish_best_effort_
      ? rclcpp::SensorDataQoS().keep_last(10)
      : reliable_qos;

    RCLCPP_INFO(
      get_logger(),
      "odom QoS: subscribe=%s publish=%s",
      odom_subscribe_best_effort_ ? "best_effort" : "reliable",
      odom_publish_best_effort_ ? "best_effort" : "reliable");

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_abs, odom_pub_qos);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_abs, odom_sub_qos,
      std::bind(&OdomTransformNode::onOdom, this, std::placeholders::_1));

    if (status_period_sec_ > 0.0) {
      last_status_time_ = now();
      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(status_period_sec_));
      status_timer_ = create_wall_timer(period, std::bind(&OdomTransformNode::log_status, this));
    }
  }

private:
  void setup_tf_subscriptions()
  {
    // Some robot deployments publish TF under the robot namespace (e.g. /cyberdog_1/tf),
    // while others use global /tf. Subscribe to both and feed into the same buffer.
    const auto tf_qos = rclcpp::SensorDataQoS();
    const auto tf_static_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    const auto tf_topic = declare_parameter<std::string>("tf_topic", "/tf");
    const auto tf_static_topic = declare_parameter<std::string>("tf_static_topic", "/tf_static");

    if (tf_subscribe_global_) {
      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic, tf_qos,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { on_tf(msg, false); });
      tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        tf_static_topic, tf_static_qos,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { on_tf(msg, true); });
      RCLCPP_INFO(get_logger(), "listening TF on: %s and %s", tf_topic.c_str(), tf_static_topic.c_str());
    }

    if (!robot_namespace_.empty()) {
      const auto ns_tf = "/" + robot_namespace_ + "/tf";
      const auto ns_tf_static = "/" + robot_namespace_ + "/tf_static";
      tf_sub_ns_ = create_subscription<tf2_msgs::msg::TFMessage>(
        ns_tf, tf_qos,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { on_tf(msg, false); });
      tf_static_sub_ns_ = create_subscription<tf2_msgs::msg::TFMessage>(
        ns_tf_static, tf_static_qos,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { on_tf(msg, true); });

      RCLCPP_INFO(get_logger(), "listening TF on: %s and %s", ns_tf.c_str(), ns_tf_static.c_str());
    }
  }

  void publish_output_tag_static_tf()
  {
    if (output_frame_.empty() || target_frame_.empty() || output_frame_ == target_frame_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = output_frame_;
    tf.child_frame_id = target_frame_;
    tf.transform = tf2_to_transform_msg(T_output_tag_);

    if (publish_static_tf_to_global_) {
      static_tf_broadcaster_.sendTransform(tf);
    } else if (ns_static_tf_pub_) {
      tf2_msgs::msg::TFMessage msg;
      msg.transforms.push_back(tf);
      ns_static_tf_pub_->publish(msg);
    }

    RCLCPP_INFO(
      get_logger(),
      "published static TF for wall tag alignment: %s -> %s",
      output_frame_.c_str(), target_frame_.c_str());
  }

  void on_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static)
  {
    if (!msg) {
      return;
    }
    for (const auto & tf : msg->transforms) {
      try {
        tf_buffer_.setTransform(tf, "odom_transform", is_static);
      } catch (const tf2::TransformException &) {
        // Ignore malformed transforms
      }
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    ++input_count_;

    geometry_msgs::msg::PoseStamped in;
    in.header = msg->header;
    in.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped out;
    if (!convert_with_latched_tag_frame(*msg, out)) {
      if (!use_latest_tf_on_failure_) {
        return;
      }
      try {
        // Temporary path before latch: use latest TF to avoid extrapolation issues.
        const auto tf = tf_buffer_.lookupTransform(target_frame_, in.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(in, out, tf);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF transform failed: %s -> %s (%s); latch fallback unavailable",
          in.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
        return;
      }
    }

    apply_output_frame_alignment(out);

    nav_msgs::msg::Odometry odom_out;
    odom_out.header = out.header;
    odom_out.child_frame_id = msg->child_frame_id;  // keep as-is (twist frame not transformed here)
    odom_out.pose = msg->pose;
    odom_out.pose.pose = out.pose;
    odom_out.twist = msg->twist;

    pub_->publish(odom_out);
    ++output_count_;
  }

  void log_status()
  {
    const auto current_count = output_count_;
    const auto current_time = now();
    const auto dt = (current_time - last_status_time_).seconds();
    const auto hz = dt > 1e-6 ? static_cast<double>(current_count - last_status_output_count_) / dt : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "odom_global stats: input=%llu output=%llu output_hz=%.2f latched=%s frame=%s",
      static_cast<unsigned long long>(input_count_),
      static_cast<unsigned long long>(output_count_),
      hz,
      tag_to_odom_latched_ ? "true" : "false",
      output_frame_.c_str());
    last_status_output_count_ = current_count;
    last_status_time_ = current_time;
  }

  void apply_output_frame_alignment(geometry_msgs::msg::PoseStamped & pose) const
  {
    pose.header.frame_id = output_frame_;
    if (!apply_wall_tag_alignment_) {
      return;
    }

    const auto T_tag_child = pose_to_tf2(pose.pose);
    const auto T_output_child = T_output_tag_ * T_tag_child;
    pose.pose = tf2_to_pose(T_output_child);
  }

  std::vector<std::string> child_frame_candidates(const std::string & child_frame) const
  {
    std::vector<std::string> frames;
    const auto raw = trim_slashes(child_frame.empty() ? odom_child_frame_fallback_ : child_frame);
    if (!raw.empty()) {
      frames.push_back(raw);
    }

    if (!robot_namespace_.empty()) {
      const auto prefix = robot_namespace_ + "/";
      if (raw.rfind(prefix, 0) == 0 && raw.size() > prefix.size()) {
        frames.push_back(raw.substr(prefix.size()));
      } else if (!raw.empty()) {
        frames.push_back(prefix + raw);
      }
    }
    return frames;
  }

  bool latch_tag_to_odom_frame(
    const nav_msgs::msg::Odometry & odom,
    const std::string & child_frame,
    std::string & error)
  {
    if (tag_to_odom_latched_) {
      return true;
    }

    const auto T_odom_child = pose_to_tf2(odom.pose.pose);
    for (const auto & candidate : child_frame_candidates(child_frame)) {
      try {
        // T_child_tag comes from the normal robot TF chain:
        // base_link -> camera_* -> tag_0_observation.
        const auto T_child_tag_msg =
          tf_buffer_.lookupTransform(candidate, target_frame_, tf2::TimePointZero);
        const auto T_child_tag = transform_msg_to_tf2(T_child_tag_msg.transform);
        const auto T_odom_tag = T_odom_child * T_child_tag;

        T_tag_odom_ = T_odom_tag.inverse();
        tag_to_odom_latched_ = true;
        latched_child_frame_ = candidate;
        RCLCPP_INFO(
          get_logger(),
          "latched %s -> %s using odom child frame '%s'",
          target_frame_.c_str(), odom.header.frame_id.c_str(), candidate.c_str());
        return true;
      } catch (const tf2::TransformException & ex) {
        error = ex.what();
      }
    }
    return false;
  }

  bool convert_with_latched_tag_frame(
    const nav_msgs::msg::Odometry & odom,
    geometry_msgs::msg::PoseStamped & out)
  {
    const auto child_frame = trim_slashes(odom.child_frame_id);
    std::string error;
    if (!latch_tag_to_odom_frame(odom, child_frame, error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for TF chain %s -> %s via child frame '%s': %s",
        child_frame.empty() ? odom_child_frame_fallback_.c_str() : child_frame.c_str(),
        target_frame_.c_str(), child_frame.c_str(), error.c_str());
      return false;
    }

    const auto T_odom_child = pose_to_tf2(odom.pose.pose);
    const auto T_tag_child = T_tag_odom_ * T_odom_child;

    out.header = odom.header;
    out.header.frame_id = target_frame_;
    out.pose = tf2_to_pose(T_tag_child);
    return true;
  }

private:
  std::string robot_namespace_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string output_frame_;
  std::string odom_child_frame_fallback_;
  std::string latched_child_frame_;
  bool use_latest_tf_on_failure_{true};
  bool odom_subscribe_best_effort_{true};
  bool odom_publish_best_effort_{true};
  bool tf_subscribe_global_{true};
  bool publish_static_tf_to_global_{true};
  bool apply_wall_tag_alignment_{true};
  bool rotate_output_x_180_{true};
  bool tag_to_odom_latched_{false};
  double status_period_sec_{5.0};
  std::uint64_t input_count_{0};
  std::uint64_t output_count_{0};
  std::uint64_t last_status_output_count_{0};
  rclcpp::Time last_status_time_;
  tf2::Transform T_tag_odom_;
  tf2::Transform T_output_tag_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr ns_static_tf_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_ns_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_ns_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTransformNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}

