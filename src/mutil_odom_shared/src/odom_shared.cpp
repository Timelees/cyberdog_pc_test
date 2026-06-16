#include "odom_shared.hpp"

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kQuaternionNormEpsilon = 1e-6;
}

namespace mutil_odom_shared
{

OdomSharedNode::OdomSharedNode()
: Node("mutil_odom_shared")
{
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
	static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

	namespaces_ = declare_parameter<std::vector<std::string>>(
		"robot_namespaces",
		std::vector<std::string>{});
	if (namespaces_.empty()) {
		namespaces_ = declare_parameter<std::vector<std::string>>(
			"namespace",
			std::vector<std::string>{});
	}
	reference_namespace_index_ = declare_parameter<int>("reference_namespace_index", 0);

	odom_slam_enabled_ = declare_parameter<bool>("odom_slam_enabled", true);
	odom_slam_input_topic_base_ = declare_parameter<std::string>("odom_slam_input_topic", "/odom_slam");
	odom_slam_output_topic_base_ = declare_parameter<std::string>("odom_slam_output_topic", "/odom_shared");

	publish_tf_ = declare_parameter<bool>("publish_tf", true);
	publish_odom_ = declare_parameter<bool>("publish_odom", true);
	align_enabled_ = declare_parameter<bool>("align_enabled", true);
	shared_odom_frame_id_ = declare_parameter<std::string>("shared_odom_frame_id", "shared_odom");
	base_frame_name_ = declare_parameter<std::string>("base_frame_name", "base_link");

	if (
		reference_namespace_index_ < 0 ||
		(!namespaces_.empty() &&
			static_cast<std::size_t>(reference_namespace_index_) >= namespaces_.size()))
	{
		RCLCPP_WARN(get_logger(), "reference_namespace_index 无效，回退到 0");
		reference_namespace_index_ = 0;
	}

	if (!namespaces_.empty()) {
		reference_namespace_ = trim_slashes(namespaces_[static_cast<std::size_t>(reference_namespace_index_)]);
	}

	first_shared_pose_logged_.assign(namespaces_.size(), false);
	publish_shared_odom_frame_anchor();

	if (odom_slam_enabled_) {
		for (std::size_t index = 0; index < namespaces_.size(); ++index) {
			create_robot_relay(trim_slashes(namespaces_[index]), index);
		}
	}

	const auto status_period_sec = declare_parameter<double>("status_period_sec", 5.0);
	if (status_period_sec > 0.0) {
		const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::duration<double>(status_period_sec));
		status_timer_ = create_wall_timer(period, [this]() {
			log_status();
		});
	}

	RCLCPP_INFO(
		get_logger(),
		"全局坐标共享已就绪: robots=%zu, 基准=%s, frame=%s",
		namespaces_.size(),
		reference_namespace_.c_str(),
		shared_odom_frame_id_.c_str());
	RCLCPP_INFO(
		get_logger(),
		"对齐策略: T_global = T_%s_origin^{-1} * T_current（无需手动 spawn 配置）",
		reference_namespace_.c_str());
}

std::string OdomSharedNode::trim_slashes(const std::string & value) const
{
	if (value.empty()) {
		return value;
	}

	std::size_t start = 0;
	while (start < value.size() && value[start] == '/') {
		++start;
	}

	std::size_t end = value.size();
	while (end > start && value[end - 1] == '/') {
		--end;
	}

	return value.substr(start, end - start);
}

std::string OdomSharedNode::build_namespaced_topic(
	const std::string & namespace_name,
	const std::string & base_topic) const
{
	const auto normalized_namespace = trim_slashes(namespace_name);
	const auto normalized_topic = trim_slashes(base_topic);

	if (normalized_namespace.empty()) {
		return normalized_topic.empty() ? std::string{} : ("/" + normalized_topic);
	}

	if (normalized_topic.empty()) {
		return "/" + normalized_namespace;
	}

	return "/" + normalized_namespace + "/" + normalized_topic;
}

std::string OdomSharedNode::build_child_frame_id(const std::string & namespace_name) const
{
	const auto normalized_namespace = trim_slashes(namespace_name);
	if (normalized_namespace.empty()) {
		return base_frame_name_;
	}
	return normalized_namespace + "_" + base_frame_name_;
}

tf2::Transform OdomSharedNode::odometry_to_transform(const nav_msgs::msg::Odometry & odom)
{
	tf2::Transform transform;
	transform.setOrigin(tf2::Vector3(
		odom.pose.pose.position.x,
		odom.pose.pose.position.y,
		odom.pose.pose.position.z));
	tf2::Quaternion rotation(
		odom.pose.pose.orientation.x,
		odom.pose.pose.orientation.y,
		odom.pose.pose.orientation.z,
		odom.pose.pose.orientation.w);
	if (rotation.length2() <= kQuaternionNormEpsilon) {
		rotation.setValue(0.0, 0.0, 0.0, 1.0);
	} else {
		rotation.normalize();
	}
	transform.setRotation(rotation);
	return transform;
}

nav_msgs::msg::Odometry OdomSharedNode::transform_to_odometry(
	const tf2::Transform & transform,
	const std_msgs::msg::Header & header,
	const std::string & child_frame_id)
{
	nav_msgs::msg::Odometry odom;
	odom.header = header;
	odom.child_frame_id = child_frame_id;
	odom.pose.pose.position.x = transform.getOrigin().x();
	odom.pose.pose.position.y = transform.getOrigin().y();
	odom.pose.pose.position.z = transform.getOrigin().z();
	const auto & rotation = transform.getRotation();
	odom.pose.pose.orientation.x = rotation.x();
	odom.pose.pose.orientation.y = rotation.y();
	odom.pose.pose.orientation.z = rotation.z();
	odom.pose.pose.orientation.w = rotation.w();
	return odom;
}

void OdomSharedNode::try_initialize_global_origin(const tf2::Transform & reference_transform)
{
	if (global_origin_ready_) {
		return;
	}

	global_origin_ = reference_transform;
	global_origin_ready_ = true;

	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	tf2::Matrix3x3(global_origin_.getRotation()).getRPY(roll, pitch, yaw);

	RCLCPP_INFO(
		get_logger(),
		"全局原点已建立（%s 首帧 odom_slam） pos=[%.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f]",
		reference_namespace_.c_str(),
		global_origin_.getOrigin().x(),
		global_origin_.getOrigin().y(),
		global_origin_.getOrigin().z(),
		roll,
		pitch,
		yaw);
}

bool OdomSharedNode::apply_global_alignment(
	const nav_msgs::msg::Odometry & input,
	nav_msgs::msg::Odometry & output)
{
	{
		std::lock_guard<std::mutex> lock(align_mutex_);
		if (!global_origin_ready_) {
			return false;
		}
	}

	if (!align_enabled_) {
		output = input;
		output.header.frame_id = shared_odom_frame_id_;
		return true;
	}

	const auto input_transform = odometry_to_transform(input);
	// 全局对齐：以基准机器人启动位置为原点，其余机器人直接使用 odom_slam 在同一地图下的位姿
	const auto aligned_transform = global_origin_.inverse() * input_transform;
	output = transform_to_odometry(
		aligned_transform,
		input.header,
		input.child_frame_id);
	output.header.frame_id = shared_odom_frame_id_;
	return true;
}

void OdomSharedNode::log_first_shared_pose(
	std::size_t robot_index,
	const nav_msgs::msg::Odometry & odom)
{
	if (robot_index >= first_shared_pose_logged_.size() || first_shared_pose_logged_[robot_index]) {
		return;
	}

	first_shared_pose_logged_[robot_index] = true;

	const auto & q = odom.pose.pose.orientation;
	tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

	RCLCPP_INFO(
		get_logger(),
		"%s 在 %s 下首帧位姿 pos=[%.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f]",
		robots_[robot_index].namespace_name.c_str(),
		shared_odom_frame_id_.c_str(),
		odom.pose.pose.position.x,
		odom.pose.pose.position.y,
		odom.pose.pose.position.z,
		roll,
		pitch,
		yaw);
}

void OdomSharedNode::publish_transform(
	const nav_msgs::msg::Odometry & odom,
	const std::string & child_frame_id)
{
	if (!publish_tf_ || !tf_broadcaster_) {
		return;
	}

	geometry_msgs::msg::TransformStamped transform;
	transform.header = odom.header;
	if (transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) {
		transform.header.stamp = now();
	}
	transform.header.frame_id = shared_odom_frame_id_;
	transform.child_frame_id = child_frame_id;
	transform.transform.translation.x = odom.pose.pose.position.x;
	transform.transform.translation.y = odom.pose.pose.position.y;
	transform.transform.translation.z = odom.pose.pose.position.z;
	transform.transform.rotation = odom.pose.pose.orientation;
	tf_broadcaster_->sendTransform(transform);
}

void OdomSharedNode::publish_shared_odom_frame_anchor()
{
	if (!static_tf_broadcaster_ || shared_odom_frame_id_.empty()) {
		return;
	}

	geometry_msgs::msg::TransformStamped anchor;
	anchor.header.stamp = now();
	anchor.header.frame_id = "map";
	anchor.child_frame_id = shared_odom_frame_id_;
	anchor.transform.rotation.w = 1.0;
	static_tf_broadcaster_->sendTransform(anchor);
}

void OdomSharedNode::create_robot_relay(
	const std::string & namespace_name,
	std::size_t robot_index)
{
	if (namespace_name.empty()) {
		return;
	}

	const auto input_topic = build_namespaced_topic(namespace_name, odom_slam_input_topic_base_);
	const auto output_topic = build_namespaced_topic(namespace_name, odom_slam_output_topic_base_);
	if (input_topic.empty() || output_topic.empty()) {
		return;
	}

	RobotRelay relay;
	relay.namespace_name = namespace_name;
	relay.input_topic = input_topic;
	relay.output_topic = output_topic;
	relay.child_frame_id = build_child_frame_id(namespace_name);
	relay.is_reference = static_cast<int>(robot_index) == reference_namespace_index_;

	if (publish_odom_) {
		relay.publisher = create_publisher<nav_msgs::msg::Odometry>(
			output_topic,
			rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
	}

	relay.subscription = create_subscription<nav_msgs::msg::Odometry>(
		input_topic,
		rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
		[this, robot_index](nav_msgs::msg::Odometry::SharedPtr message) {
			if (!message) {
				return;
			}
			handle_slam_odom(robot_index, *message);
		});

	robots_.push_back(std::move(relay));

	RCLCPP_INFO(
		get_logger(),
		"订阅 %s%s: %s -> %s",
		namespace_name.c_str(),
		robots_.back().is_reference ? " [全局基准]" : "",
		input_topic.c_str(),
		output_topic.c_str());
}

void OdomSharedNode::handle_slam_odom(
	std::size_t robot_index,
	const nav_msgs::msg::Odometry & message)
{
	if (robot_index >= robots_.size()) {
		return;
	}

	auto & relay = robots_[robot_index];

	if (relay.is_reference) {
		std::lock_guard<std::mutex> lock(align_mutex_);
		try_initialize_global_origin(odometry_to_transform(message));
	}

	if (!global_origin_ready_) {
		return;
	}

	nav_msgs::msg::Odometry aligned_output;
	if (!apply_global_alignment(message, aligned_output)) {
		return;
	}

	aligned_output.child_frame_id = relay.child_frame_id;
	if (aligned_output.header.stamp.sec == 0 && aligned_output.header.stamp.nanosec == 0) {
		aligned_output.header.stamp = now();
	}

	log_first_shared_pose(robot_index, aligned_output);

	if (publish_odom_ && relay.publisher) {
		relay.publisher->publish(aligned_output);
	}

	if (publish_tf_) {
		publish_transform(aligned_output, relay.child_frame_id);
	}

	relay_counts_[relay.namespace_name]++;
	last_receive_time_[relay.namespace_name] = now();
}

void OdomSharedNode::log_status()
{
	if (robots_.empty()) {
		return;
	}

	RCLCPP_INFO(
		get_logger(),
		"status: 基准=%s, 全局原点=%s, robots=%zu",
		reference_namespace_.c_str(),
		global_origin_ready_ ? "ready" : "waiting",
		robots_.size());

	for (const auto & relay : robots_) {
		auto age_text = std::string("never");
		const auto time_iter = last_receive_time_.find(relay.namespace_name);
		if (time_iter != last_receive_time_.end() && time_iter->second.nanoseconds() > 0) {
			age_text = std::to_string((now() - time_iter->second).seconds()) + "s";
		}

		const auto count_iter = relay_counts_.find(relay.namespace_name);
		const auto count = count_iter != relay_counts_.end() ? count_iter->second : 0U;

		RCLCPP_INFO(
			get_logger(),
			"  %s%s: count=%llu, last_age=%s",
			relay.namespace_name.c_str(),
			relay.is_reference ? " [reference]" : "",
			static_cast<unsigned long long>(count),
			age_text.c_str());
	}
}

}  // namespace mutil_odom_shared

int main(int argc, char ** argv)
{
	int exit_code = 0;
	try {
		rclcpp::init(argc, argv);
		auto node = std::make_shared<mutil_odom_shared::OdomSharedNode>();
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		executor.spin();
	} catch (const std::exception & exception) {
		RCLCPP_ERROR(rclcpp::get_logger("mutil_odom_shared"), "fatal error: %s", exception.what());
		exit_code = 1;
	}

	if (rclcpp::ok()) {
		rclcpp::shutdown();
	}
	return exit_code;
}
