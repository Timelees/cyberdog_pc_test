#include "ros_topic_visual.hpp"

#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kQuaternionNormEpsilon = 1e-6;
constexpr double kMinPoseTextScale = 0.01;
}

RosTopicVisualNode::RosTopicVisualNode()
: Node("ros_topic_visual")
{
	const auto sensor_qos = rclcpp::SensorDataQoS();
	const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
	relay_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

	namespaces_ = declare_parameter<std::vector<std::string>>("namespace", std::vector<std::string>{});
	const auto namespace_index = declare_parameter<int>("namespace_index", 0);
	selected_namespace_ = select_namespace(namespace_index);

	rclcpp::SubscriptionOptions subscription_options;
	subscription_options.callback_group = relay_callback_group_;

	create_relay<sensor_msgs::msg::LaserScan>(
		declare_parameter<bool>("scan_enabled", true),
		build_input_topic(declare_parameter<std::string>("scan_input_topic", "/scan")),
		declare_parameter<std::string>("scan_output_topic", "/scan"),
		sensor_qos,
		sensor_qos,
		"scan",
		subscription_options);

	pose_text_enabled_ = declare_parameter<bool>("pose_text_enabled", true);
	pose_text_topic_ = declare_parameter<std::string>("pose_text_topic", "/base_link_pose_text");
	pose_text_frame_id_ = declare_parameter<std::string>("pose_text_frame_id", "base_link");
	pose_text_z_offset_ = declare_parameter<double>("pose_text_z_offset", 0.6);
	pose_text_scale_ = declare_parameter<double>("pose_text_scale", 0.18);
	if (pose_text_scale_ < kMinPoseTextScale) {
		RCLCPP_WARN(
			get_logger(),
			"pose_text_scale=%.4f is too small, clamp to %.2f",
			pose_text_scale_,
			kMinPoseTextScale);
		pose_text_scale_ = kMinPoseTextScale;
	}

	if (pose_text_enabled_ && !pose_text_topic_.empty()) {
		pose_text_publisher_ =
			create_publisher<visualization_msgs::msg::Marker>(pose_text_topic_, rclcpp::QoS(10).reliable());
	}

	create_odom_relay(
		declare_parameter<bool>("odom_enabled", true),
		"odom",
		build_input_topic(declare_parameter<std::string>("odom_input_topic", "/odom_out")),
		declare_parameter<std::string>("odom_output_topic", "/odom"),
		declare_parameter<bool>("odom_publish_tf", true),
		declare_parameter<std::string>("odom_output_frame_id", "odom"),
		declare_parameter<std::string>("odom_output_child_frame_id", "base_link_leg"),
		reliable_qos,
		reliable_qos,
		subscription_options,
		true);

	create_odom_relay(
		declare_parameter<bool>("vins_odom_enabled", true),
		"vins_odom",
		build_input_topic(declare_parameter<std::string>("vins_odom_input_topic", "/odom_slam")),
		declare_parameter<std::string>("vins_odom_output_topic", "/odom_slam"),
		declare_parameter<bool>("vins_odom_publish_tf", true),
		declare_parameter<std::string>("vins_odom_output_frame_id", "map"),
		declare_parameter<std::string>("vins_odom_output_child_frame_id", "base_link"),
		reliable_qos,
		reliable_qos,
		subscription_options);

	const auto status_period_sec = declare_parameter<double>("status_period_sec", 5.0);
	if (status_period_sec > 0.0) {
		const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::duration<double>(status_period_sec));
		status_timer_ = create_wall_timer(period, [this]() {
			if (shutting_down_.load()) {
				return;
			}
			try {
				log_status();
			} catch (const std::exception & exception) {
				RCLCPP_ERROR(get_logger(), "status timer failed: %s", exception.what());
			}
		});
	}

	RCLCPP_INFO(get_logger(), "ros_topic_visual is ready with %zu relay channels", relay_specs_.size());
}

RosTopicVisualNode::~RosTopicVisualNode()
{
	shutting_down_.store(true);
	if (status_timer_) {
		status_timer_->cancel();
		status_timer_.reset();
	}
}

std::string RosTopicVisualNode::select_namespace(int namespace_index)
{
	if (namespaces_.empty()) {
		RCLCPP_INFO(get_logger(), "namespace list is empty, use raw input topics");
		return "";
	}

	if (namespace_index < 0 || namespace_index >= static_cast<int>(namespaces_.size())) {
		RCLCPP_WARN(
			get_logger(),
			"namespace_index=%d is out of range, fallback to namespace[0]=%s",
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

std::string RosTopicVisualNode::trim_slashes(const std::string & value) const
{
	auto start = value.find_first_not_of('/');
	if (start == std::string::npos) {
		return "";
	}
	auto end = value.find_last_not_of('/');
	return value.substr(start, end - start + 1);
}

std::string RosTopicVisualNode::build_input_topic(const std::string & base_topic) const
{
	const auto normalized_topic = trim_slashes(base_topic);
	if (selected_namespace_.empty()) {
		return normalized_topic.empty() ? "/" : "/" + normalized_topic;
	}

	const auto normalized_namespace = trim_slashes(selected_namespace_);
	if (normalized_topic.empty()) {
		return "/" + normalized_namespace;
	}

	const auto prefix = normalized_namespace + "/";
	if (normalized_topic.rfind(prefix, 0) == 0) {
		return "/" + normalized_topic;
	}

	return "/" + normalized_namespace + "/" + normalized_topic;
}

bool RosTopicVisualNode::is_finite(double value)
{
	return std::isfinite(value);
}

bool RosTopicVisualNode::is_valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
	if (!is_finite(quaternion.x) || !is_finite(quaternion.y) ||
		!is_finite(quaternion.z) || !is_finite(quaternion.w))
	{
		return false;
	}

	const auto norm_squared =
		(quaternion.x * quaternion.x) +
		(quaternion.y * quaternion.y) +
		(quaternion.z * quaternion.z) +
		(quaternion.w * quaternion.w);
	return norm_squared > kQuaternionNormEpsilon;
}

geometry_msgs::msg::Quaternion RosTopicVisualNode::normalize_quaternion(
	const geometry_msgs::msg::Quaternion & quaternion)
{
	geometry_msgs::msg::Quaternion normalized = quaternion;
	const auto norm = std::sqrt(
		(quaternion.x * quaternion.x) +
		(quaternion.y * quaternion.y) +
		(quaternion.z * quaternion.z) +
		(quaternion.w * quaternion.w));
	if (norm <= kQuaternionNormEpsilon) {
		normalized.x = 0.0;
		normalized.y = 0.0;
		normalized.z = 0.0;
		normalized.w = 1.0;
		return normalized;
	}

	normalized.x /= norm;
	normalized.y /= norm;
	normalized.z /= norm;
	normalized.w /= norm;
	return normalized;
}

bool RosTopicVisualNode::extract_rpy(
	const geometry_msgs::msg::Quaternion & quaternion,
	double & roll,
	double & pitch,
	double & yaw)
{
	if (!is_valid_quaternion(quaternion)) {
		return false;
	}

	const auto normalized = normalize_quaternion(quaternion);
	const tf2::Quaternion tf_quaternion(
		normalized.x,
		normalized.y,
		normalized.z,
		normalized.w);
	tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
	return is_finite(roll) && is_finite(pitch) && is_finite(yaw);
}

void RosTopicVisualNode::create_odom_relay(
	bool enabled,
	const std::string & relay_name,
	const std::string & input_topic,
	const std::string & output_topic,
	bool publish_tf,
	const std::string & output_frame_id,
	const std::string & output_child_frame_id,
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
	const rclcpp::SubscriptionOptions & subscription_options,
	bool enable_pose_text)
{
	if (!enabled) {
		RCLCPP_INFO(get_logger(), "skip disabled relay: %s", relay_name.c_str());
		return;
	}

	if (input_topic.empty() || output_topic.empty()) {
		RCLCPP_WARN(
			get_logger(),
			"skip relay %s because topic name is empty",
			relay_name.c_str());
		return;
	}

	auto odom_publisher = create_publisher<nav_msgs::msg::Odometry>(output_topic, publish_qos);
	auto odom_subscription = create_subscription<nav_msgs::msg::Odometry>(
		input_topic,
		subscribe_qos,
		[this, relay_name, odom_publisher, publish_tf, output_frame_id, output_child_frame_id, enable_pose_text](
			nav_msgs::msg::Odometry::SharedPtr message) {
			if (shutting_down_.load()) {
				return;
			}
			try {
				handle_odom_message(
					message,
					relay_name,
					odom_publisher,
					publish_tf,
					output_frame_id,
					output_child_frame_id,
					enable_pose_text);
			} catch (const std::exception & exception) {
				RCLCPP_ERROR_THROTTLE(
					get_logger(),
					*get_clock(),
					5000,
					"relay %s callback failed: %s",
					relay_name.c_str(),
					exception.what());
			}
		},
		subscription_options);

	publishers_.push_back(odom_publisher);
	subscriptions_.push_back(odom_subscription);
	relay_specs_.push_back(RelaySpec{relay_name, input_topic, output_topic});
}

void RosTopicVisualNode::handle_odom_message(
	const nav_msgs::msg::Odometry::SharedPtr & message,
	const std::string & relay_name,
	const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & odom_publisher,
	bool publish_tf,
	const std::string & output_frame_id,
	const std::string & output_child_frame_id,
	bool enable_pose_text)
{
	if (!message) {
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			5000,
			"relay %s received null odom message",
			relay_name.c_str());
		return;
	}

	if (!odom_publisher) {
		RCLCPP_ERROR(get_logger(), "relay %s publisher is unavailable", relay_name.c_str());
		return;
	}

	nav_msgs::msg::Odometry output = *message;

	if (!output_frame_id.empty()) {
		output.header.frame_id = output_frame_id;
	}
	if (!output_child_frame_id.empty()) {
		output.child_frame_id = output_child_frame_id;
	}

	if (!is_finite(output.pose.pose.position.x) ||
		!is_finite(output.pose.pose.position.y) ||
		!is_finite(output.pose.pose.position.z))
	{
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			5000,
			"relay %s skipped message with invalid position",
			relay_name.c_str());
		return;
	}

	if (!is_valid_quaternion(output.pose.pose.orientation)) {
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			5000,
			"relay %s received invalid orientation, use identity quaternion",
			relay_name.c_str());
		output.pose.pose.orientation.x = 0.0;
		output.pose.pose.orientation.y = 0.0;
		output.pose.pose.orientation.z = 0.0;
		output.pose.pose.orientation.w = 1.0;
	} else {
		output.pose.pose.orientation = normalize_quaternion(output.pose.pose.orientation);
	}

	odom_publisher->publish(output);
	update_relay_status(relay_name);

	if (enable_pose_text) {
		publish_pose_text(output);
	}

	if (publish_tf && !output.header.frame_id.empty() && !output.child_frame_id.empty()) {
		geometry_msgs::msg::TransformStamped transform;
		transform.header = output.header;
		if (transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) {
			transform.header.stamp = now();
		}
		transform.child_frame_id = output.child_frame_id;
		transform.transform.translation.x = output.pose.pose.position.x;
		transform.transform.translation.y = output.pose.pose.position.y;
		transform.transform.translation.z = output.pose.pose.position.z;
		transform.transform.rotation = output.pose.pose.orientation;
		tf_broadcaster_->sendTransform(transform);
	}
}

void RosTopicVisualNode::publish_pose_text(const nav_msgs::msg::Odometry & odom)
{
	if (!pose_text_enabled_ || !pose_text_publisher_) {
		return;
	}

	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	const auto has_rpy = extract_rpy(odom.pose.pose.orientation, roll, pitch, yaw);

	std::ostringstream text_stream;
	text_stream << std::fixed << std::setprecision(1)
		<< "[" << (selected_namespace_.empty() ? "default" : selected_namespace_) << "] "
		<< "position:["
		<< odom.pose.pose.position.x << ","
		<< odom.pose.pose.position.y << ","
		<< odom.pose.pose.position.z << "], "
		<< "orientation:[";
	if (has_rpy) {
		text_stream << yaw << "," << pitch << "," << roll;
	} else {
		text_stream << "n/a,n/a,n/a";
	}
	text_stream << "]";

	visualization_msgs::msg::Marker marker;
	if (odom.header.stamp.sec == 0 && odom.header.stamp.nanosec == 0) {
		marker.header.stamp = now();
	} else {
		marker.header.stamp = odom.header.stamp;
	}
	marker.header.frame_id = pose_text_frame_id_.empty() ? odom.child_frame_id : pose_text_frame_id_;
	marker.ns = selected_namespace_.empty() ? "base_link_pose_text" : selected_namespace_ + "_base_link_pose_text";
	marker.id = 0;
	marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = pose_text_z_offset_;
	marker.pose.orientation.w = 1.0;
	marker.scale.z = pose_text_scale_;
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.2f;
	marker.color.a = 1.0f;
	marker.frame_locked = true;
	marker.text = text_stream.str();

	pose_text_publisher_->publish(marker);
}

template<typename MessageT>
void RosTopicVisualNode::create_relay(
	bool enabled,
	const std::string & input_topic,
	const std::string & output_topic,
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
	const std::string & relay_name,
	const rclcpp::SubscriptionOptions & subscription_options)
{
	if (!enabled) {
		RCLCPP_INFO(get_logger(), "skip disabled relay: %s", relay_name.c_str());
		return;
	}

	if (input_topic.empty() || output_topic.empty()) {
		RCLCPP_WARN(
			get_logger(),
			"skip relay %s because topic name is empty",
			relay_name.c_str());
		return;
	}

	auto publisher = create_publisher<MessageT>(output_topic, publish_qos);
	auto subscription = create_subscription<MessageT>(
		input_topic,
		subscribe_qos,
		[this, publisher, relay_name](typename MessageT::SharedPtr message) {
			if (shutting_down_.load()) {
				return;
			}
			if (!message) {
				RCLCPP_WARN_THROTTLE(
					get_logger(),
					*get_clock(),
					5000,
					"relay %s received null message",
					relay_name.c_str());
				return;
			}
			if (!publisher) {
				RCLCPP_ERROR(get_logger(), "relay %s publisher is unavailable", relay_name.c_str());
				return;
			}
			try {
				publisher->publish(*message);
				update_relay_status(relay_name);
			} catch (const std::exception & exception) {
				RCLCPP_ERROR_THROTTLE(
					get_logger(),
					*get_clock(),
					5000,
					"relay %s callback failed: %s",
					relay_name.c_str(),
					exception.what());
			}
		},
		subscription_options);

	publishers_.push_back(publisher);
	subscriptions_.push_back(subscription);
	relay_specs_.push_back(RelaySpec{relay_name, input_topic, output_topic});
}

template void RosTopicVisualNode::create_relay<sensor_msgs::msg::LaserScan>(
	bool enabled,
	const std::string & input_topic,
	const std::string & output_topic,
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
	const std::string & relay_name,
	const rclcpp::SubscriptionOptions & subscription_options);

void RosTopicVisualNode::update_relay_status(const std::string & relay_name)
{
	std::lock_guard<std::mutex> lock(status_mutex_);
	relay_counts_[relay_name] += 1;
	last_receive_time_[relay_name] = now();
}

void RosTopicVisualNode::log_status()
{
	std::unordered_map<std::string, std::uint64_t> relay_counts_copy;
	std::unordered_map<std::string, rclcpp::Time> last_receive_time_copy;
	std::vector<RelaySpec> relay_specs_copy;
	{
		std::lock_guard<std::mutex> lock(status_mutex_);
		relay_counts_copy = relay_counts_;
		last_receive_time_copy = last_receive_time_;
		relay_specs_copy = relay_specs_;
	}

	for (const auto & relay : relay_specs_copy) {
		auto age_text = std::string("never");
		const auto time_iter = last_receive_time_copy.find(relay.name);
		if (time_iter != last_receive_time_copy.end() && time_iter->second.nanoseconds() > 0) {
			const auto age = (now() - time_iter->second).seconds();
			age_text = std::to_string(age) + "s";
		}

		const auto count_iter = relay_counts_copy.find(relay.name);
		const auto count = count_iter != relay_counts_copy.end() ? count_iter->second : 0U;

		RCLCPP_INFO(
			get_logger(),
			"relay %s: %s -> %s, count=%llu, last_age=%s",
			relay.name.c_str(),
			relay.input_topic.c_str(),
			relay.output_topic.c_str(),
			static_cast<unsigned long long>(count),
			age_text.c_str());
	}
}

int main(int argc, char ** argv)
{
	int exit_code = 0;
	try {
		rclcpp::init(argc, argv);
		auto node = std::make_shared<RosTopicVisualNode>();
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		executor.spin();
	} catch (const std::exception & exception) {
		RCLCPP_ERROR(rclcpp::get_logger("ros_topic_visual"), "fatal error: %s", exception.what());
		exit_code = 1;
	}

	if (rclcpp::ok()) {
		rclcpp::shutdown();
	}
	return exit_code;
}
