#include <array>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

constexpr double kQuaternionNormEpsilon = 1e-12;

std::string trim_slashes(const std::string & value)
{
	auto begin = value.find_first_not_of('/');
	if (begin == std::string::npos) {
		return "";
	}
	auto end = value.find_last_not_of('/');
	return value.substr(begin, end - begin + 1);
}

std::string build_topic(const std::string & prefix, const std::string & suffix)
{
	const auto clean_prefix = trim_slashes(prefix);
	const auto clean_suffix = trim_slashes(suffix);
	if (clean_prefix.empty()) {
		return clean_suffix.empty() ? "" : "/" + clean_suffix;
	}
	if (clean_suffix.empty()) {
		return "/" + clean_prefix;
	}
	return "/" + clean_prefix + "/" + clean_suffix;
}

bool is_valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
	const auto norm =
		quaternion.x * quaternion.x +
		quaternion.y * quaternion.y +
		quaternion.z * quaternion.z +
		quaternion.w * quaternion.w;
	return std::isfinite(norm) && norm > kQuaternionNormEpsilon;
}

geometry_msgs::msg::Quaternion normalize_quaternion(
	const geometry_msgs::msg::Quaternion & quaternion)
{
	geometry_msgs::msg::Quaternion normalized = quaternion;
	const auto norm = std::sqrt(
		quaternion.x * quaternion.x +
		quaternion.y * quaternion.y +
		quaternion.z * quaternion.z +
		quaternion.w * quaternion.w);
	normalized.x /= norm;
	normalized.y /= norm;
	normalized.z /= norm;
	normalized.w /= norm;
	return normalized;
}

bool extract_rpy(
	const geometry_msgs::msg::Quaternion & quaternion,
	double & roll,
	double & pitch,
	double & yaw)
{
	if (!is_valid_quaternion(quaternion)) {
		return false;
	}

	const auto normalized = normalize_quaternion(quaternion);
	tf2::Quaternion tf_quaternion(
		normalized.x,
		normalized.y,
		normalized.z,
		normalized.w);
	tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
	return true;
}

}  // namespace

class TagsVisualNode : public rclcpp::Node
{
public:
	TagsVisualNode()
	: Node("tags_visual")
	{
		const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
		const auto subscribe_qos = declare_parameter<bool>("subscribe_best_effort", true)
			? rclcpp::SensorDataQoS()
			: reliable_qos;
		const auto publish_qos = declare_parameter<bool>("publish_best_effort", false)
			? rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()
			: reliable_qos;

		namespaces_ = declare_parameter<std::vector<std::string>>(
			"namespace",
			std::vector<std::string>{"cyberdog_1"});
		const auto namespace_index = declare_parameter<int>("namespace_index", 0);
		selected_namespace_ = select_namespace(namespace_index);

		target_frame_id_ = declare_parameter<std::string>("target_frame_id", "base");
		input_odom_topic_ = build_input_topic(
			declare_parameter<std::string>("input_odom_topic", "/odom_base"));
		input_path_topic_ = build_input_topic(
			declare_parameter<std::string>("input_path_topic", "/path_base"));
		const auto output_prefix =
			declare_parameter<std::string>("output_topic_prefix", "/viz/tags");
		output_odom_topic_ = build_topic(output_prefix, selected_namespace_ + "/odom");
		output_path_topic_ = build_topic(output_prefix, selected_namespace_ + "/path");
		output_input_path_topic_ =
			build_topic(output_prefix, selected_namespace_ + "/global_alignment_path");
		pose_text_topic_ =
			declare_parameter<std::string>("pose_text_topic", build_topic(output_prefix, "pose_text"));
		robot_marker_topic_ =
			declare_parameter<std::string>("robot_marker_topic", build_topic(output_prefix, "robot_marker"));
		image_enabled_ = declare_parameter<bool>("image_enabled", true);
		image_input_topic_ = build_input_topic(
			declare_parameter<std::string>("image_input_topic", "/camera/infra1/image_rect_raw"));
		image_output_topic_ = declare_parameter<std::string>("image_output_topic", "/vins/image1");

		output_child_frame_id_ =
			declare_parameter<std::string>("output_child_frame_id", selected_namespace_ + "_base_link_tag");
		publish_tf_ = declare_parameter<bool>("publish_tf", true);
		pose_text_enabled_ = declare_parameter<bool>("pose_text_enabled", true);
		robot_marker_enabled_ = declare_parameter<bool>("robot_marker_enabled", true);
		input_path_relay_enabled_ = declare_parameter<bool>("input_path_relay_enabled", true);
		const auto path_max_poses = declare_parameter<int>("path_max_poses", 5000);
		path_max_poses_ = path_max_poses > 0 ? static_cast<std::size_t>(path_max_poses) : 0U;
		path_publish_every_n_ = clamp_publish_every_n(
			declare_parameter<int>("path_publish_every_n", 5));
		marker_publish_every_n_ = clamp_publish_every_n(
			declare_parameter<int>("marker_publish_every_n", 5));
		tf_publish_every_n_ = clamp_publish_every_n(
			declare_parameter<int>("tf_publish_every_n", 1));
		pose_text_z_offset_ = declare_parameter<double>("pose_text_z_offset", 0.6);
		pose_text_scale_ = declare_parameter<double>("pose_text_scale", 0.18);
		status_period_sec_ = declare_parameter<double>("status_period_sec", 5.0);

		odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, publish_qos);
		path_publisher_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, publish_qos);
		if (input_path_relay_enabled_ && !input_path_topic_.empty()) {
			input_path_publisher_ =
				create_publisher<nav_msgs::msg::Path>(output_input_path_topic_, publish_qos);
		}
		if (pose_text_enabled_ && !pose_text_topic_.empty()) {
			pose_text_publisher_ =
				create_publisher<visualization_msgs::msg::Marker>(pose_text_topic_, reliable_qos);
		}
		if (robot_marker_enabled_ && !robot_marker_topic_.empty()) {
			robot_marker_publisher_ =
				create_publisher<visualization_msgs::msg::Marker>(robot_marker_topic_, reliable_qos);
		}
		if (publish_tf_) {
			tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
		}
		if (image_enabled_ && !image_input_topic_.empty() && !image_output_topic_.empty()) {
			image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
				image_output_topic_,
				rclcpp::SensorDataQoS());
		}

		odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
			input_odom_topic_,
			subscribe_qos,
			[this](nav_msgs::msg::Odometry::SharedPtr message) {
				handle_odom(message);
			});

		if (input_path_publisher_) {
			path_subscription_ = create_subscription<nav_msgs::msg::Path>(
				input_path_topic_,
				subscribe_qos,
				[this](nav_msgs::msg::Path::SharedPtr message) {
					handle_input_path(message);
				});
		}

		if (image_publisher_) {
			image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
				image_input_topic_,
				rclcpp::SensorDataQoS(),
				[this](sensor_msgs::msg::Image::SharedPtr message) {
					handle_image(message);
				});
		}

		if (status_period_sec_ > 0.0) {
			status_timer_ = create_wall_timer(
				std::chrono::duration_cast<std::chrono::nanoseconds>(
					std::chrono::duration<double>(status_period_sec_)),
				[this]() {
					log_status();
				});
		}

		RCLCPP_INFO(
			get_logger(),
			"tags_visual ready: input=%s, odom=%s, path=%s, fixed_frame=%s",
			input_odom_topic_.c_str(),
			output_odom_topic_.c_str(),
			output_path_topic_.c_str(),
			target_frame_id_.c_str());
	}

	~TagsVisualNode() override
	{
		shutting_down_.store(true);
	}

private:
	std::string select_namespace(int namespace_index) const
	{
		if (namespaces_.empty()) {
			return "cyberdog_1";
		}
		if (namespace_index < 0 || static_cast<std::size_t>(namespace_index) >= namespaces_.size()) {
			RCLCPP_WARN(
				get_logger(),
				"namespace_index=%d out of range, use 0",
				namespace_index);
			return trim_slashes(namespaces_.front());
		}
		const auto selected = trim_slashes(namespaces_[static_cast<std::size_t>(namespace_index)]);
		return selected.empty() ? "cyberdog_1" : selected;
	}

	std::string build_input_topic(const std::string & base_topic) const
	{
		const auto clean_topic = trim_slashes(base_topic);
		if (clean_topic.empty()) {
			return "";
		}
		if (!selected_namespace_.empty() && clean_topic.rfind(selected_namespace_ + "/", 0) != 0) {
			return "/" + selected_namespace_ + "/" + clean_topic;
		}
		return "/" + clean_topic;
	}

	static int clamp_publish_every_n(int value)
	{
		return value > 1 ? value : 1;
	}

	void handle_odom(const nav_msgs::msg::Odometry::SharedPtr & message)
	{
		if (shutting_down_.load() || !message) {
			return;
		}

		auto output = *message;
		if (!sanitize_odom(output)) {
			return;
		}
		if (!target_frame_id_.empty()) {
			output.header.frame_id = target_frame_id_;
		}
		if (!output_child_frame_id_.empty()) {
			output.child_frame_id = output_child_frame_id_;
		}
		output.header.stamp = now();

		odom_publisher_->publish(output);
		append_path(output);
		odom_count_ += 1;
		if (should_publish(odom_count_, tf_publish_every_n_)) {
			publish_tf(output);
		}
		if (should_publish(odom_count_, marker_publish_every_n_)) {
			publish_pose_text(output);
			publish_robot_marker(output);
		}

		{
			std::lock_guard<std::mutex> lock(status_mutex_);
			last_odom_time_ = now();
		}

		if (!first_odom_logged_) {
			first_odom_logged_ = true;
			RCLCPP_INFO(
				get_logger(),
				"tag aligned odom received: %s pos=[%.3f, %.3f, %.3f] frame=%s",
				input_odom_topic_.c_str(),
				output.pose.pose.position.x,
				output.pose.pose.position.y,
				output.pose.pose.position.z,
				output.header.frame_id.c_str());
		}
	}

	void handle_input_path(const nav_msgs::msg::Path::SharedPtr & message)
	{
		if (shutting_down_.load() || !message || !input_path_publisher_) {
			return;
		}

		auto output = *message;
		if (!target_frame_id_.empty()) {
			output.header.frame_id = target_frame_id_;
			for (auto & pose : output.poses) {
				pose.header.frame_id = target_frame_id_;
			}
		}
		if (output.header.stamp.sec == 0 && output.header.stamp.nanosec == 0) {
			output.header.stamp = now();
		}
		input_path_publisher_->publish(output);

		std::lock_guard<std::mutex> lock(status_mutex_);
		input_path_count_ += 1;
		last_input_path_time_ = now();
	}

	void handle_image(const sensor_msgs::msg::Image::SharedPtr & message)
	{
		if (shutting_down_.load() || !message || !image_publisher_) {
			return;
		}

		image_publisher_->publish(*message);
		std::lock_guard<std::mutex> lock(status_mutex_);
		image_count_ += 1;
		last_image_time_ = now();
	}

	bool sanitize_odom(nav_msgs::msg::Odometry & odom)
	{
		if (!std::isfinite(odom.pose.pose.position.x) ||
			!std::isfinite(odom.pose.pose.position.y) ||
			!std::isfinite(odom.pose.pose.position.z))
		{
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				5000,
				"skip tag aligned odom with invalid position");
			return false;
		}

		if (!is_valid_quaternion(odom.pose.pose.orientation)) {
			odom.pose.pose.orientation.x = 0.0;
			odom.pose.pose.orientation.y = 0.0;
			odom.pose.pose.orientation.z = 0.0;
			odom.pose.pose.orientation.w = 1.0;
		} else {
			odom.pose.pose.orientation = normalize_quaternion(odom.pose.pose.orientation);
		}
		return true;
	}

	void append_path(const nav_msgs::msg::Odometry & odom)
	{
		geometry_msgs::msg::PoseStamped pose;
		pose.header = odom.header;
		pose.pose = odom.pose.pose;
		path_.header = odom.header;
		path_.poses.push_back(std::move(pose));

		if (path_max_poses_ > 0 && path_.poses.size() > path_max_poses_) {
			const auto overflow = path_.poses.size() - path_max_poses_;
			path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<std::ptrdiff_t>(overflow));
		}

		if (should_publish(odom_count_ + 1, path_publish_every_n_)) {
			path_publisher_->publish(path_);
		}
	}

	static bool should_publish(std::uint64_t count, int every_n)
	{
		return every_n <= 1 || count % static_cast<std::uint64_t>(every_n) == 0;
	}

	void publish_tf(const nav_msgs::msg::Odometry & odom)
	{
		if (!tf_broadcaster_ || odom.header.frame_id.empty() || odom.child_frame_id.empty()) {
			return;
		}

		geometry_msgs::msg::TransformStamped transform;
		transform.header = odom.header;
		transform.child_frame_id = odom.child_frame_id;
		transform.transform.translation.x = odom.pose.pose.position.x;
		transform.transform.translation.y = odom.pose.pose.position.y;
		transform.transform.translation.z = odom.pose.pose.position.z;
		transform.transform.rotation = odom.pose.pose.orientation;
		tf_broadcaster_->sendTransform(transform);
	}

	void publish_pose_text(const nav_msgs::msg::Odometry & odom)
	{
		if (!pose_text_publisher_) {
			return;
		}

		double roll = 0.0;
		double pitch = 0.0;
		double yaw = 0.0;
		const auto has_rpy = extract_rpy(odom.pose.pose.orientation, roll, pitch, yaw);

		std::ostringstream text_stream;
		text_stream << std::fixed << std::setprecision(2)
			<< selected_namespace_ << " tag "
			<< "pos:["
			<< odom.pose.pose.position.x << ","
			<< odom.pose.pose.position.y << ","
			<< odom.pose.pose.position.z << "] ypr:[";
		if (has_rpy) {
			text_stream << yaw << "," << pitch << "," << roll;
		} else {
			text_stream << "n/a,n/a,n/a";
		}
		text_stream << "]";

		visualization_msgs::msg::Marker marker;
		marker.header = odom.header;
		marker.ns = "tag_aligned_pose_text";
		marker.id = 0;
		marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.pose.position.x = odom.pose.pose.position.x;
		marker.pose.position.y = odom.pose.pose.position.y;
		marker.pose.position.z = odom.pose.pose.position.z + pose_text_z_offset_;
		marker.pose.orientation.w = 1.0;
		marker.scale.z = pose_text_scale_;
		marker.color.r = 1.0F;
		marker.color.g = 0.95F;
		marker.color.b = 0.2F;
		marker.color.a = 1.0F;
		marker.text = text_stream.str();
		pose_text_publisher_->publish(marker);
	}

	void publish_robot_marker(const nav_msgs::msg::Odometry & odom)
	{
		if (!robot_marker_publisher_) {
			return;
		}

		visualization_msgs::msg::Marker marker;
		marker.header = odom.header;
		marker.ns = "tag_aligned_robot_arrow";
		marker.id = 0;
		marker.type = visualization_msgs::msg::Marker::ARROW;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.pose = odom.pose.pose;
		marker.scale.x = 0.6;
		marker.scale.y = 0.12;
		marker.scale.z = 0.12;
		marker.color.r = 0.1F;
		marker.color.g = 0.8F;
		marker.color.b = 1.0F;
		marker.color.a = 1.0F;
		robot_marker_publisher_->publish(marker);
	}

	void log_status()
	{
		std::uint64_t odom_count = 0;
		std::uint64_t input_path_count = 0;
		std::uint64_t image_count = 0;
		rclcpp::Time last_odom_time;
		rclcpp::Time last_input_path_time;
		rclcpp::Time last_image_time;
		{
			std::lock_guard<std::mutex> lock(status_mutex_);
			odom_count = odom_count_;
			input_path_count = input_path_count_;
			image_count = image_count_;
			last_odom_time = last_odom_time_;
			last_input_path_time = last_input_path_time_;
			last_image_time = last_image_time_;
		}

		RCLCPP_INFO(
			get_logger(),
			"tags_visual: odom %s -> %s count=%llu last_age=%s, path_relay count=%llu last_age=%s, image %s -> %s count=%llu last_age=%s",
			input_odom_topic_.c_str(),
			output_odom_topic_.c_str(),
			static_cast<unsigned long long>(odom_count),
			age_text(last_odom_time).c_str(),
			static_cast<unsigned long long>(input_path_count),
			age_text(last_input_path_time).c_str(),
			image_input_topic_.c_str(),
			image_output_topic_.c_str(),
			static_cast<unsigned long long>(image_count),
			age_text(last_image_time).c_str());
	}

	std::string age_text(const rclcpp::Time & time) const
	{
		if (time.nanoseconds() <= 0) {
			return "never";
		}
		return std::to_string((now() - time).seconds()) + "s";
	}

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr input_path_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_text_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_marker_publisher_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::TimerBase::SharedPtr status_timer_;
	nav_msgs::msg::Path path_;
	std::atomic<bool> shutting_down_{false};
	std::mutex status_mutex_;
	std::vector<std::string> namespaces_;
	std::string selected_namespace_;
	std::string target_frame_id_;
	std::string input_odom_topic_;
	std::string input_path_topic_;
	std::string output_odom_topic_;
	std::string output_path_topic_;
	std::string output_input_path_topic_;
	std::string pose_text_topic_;
	std::string robot_marker_topic_;
	std::string image_input_topic_;
	std::string image_output_topic_;
	std::string output_child_frame_id_;
	std::size_t path_max_poses_{5000};
	std::uint64_t odom_count_{0};
	std::uint64_t input_path_count_{0};
	std::uint64_t image_count_{0};
	rclcpp::Time last_odom_time_;
	rclcpp::Time last_input_path_time_;
	rclcpp::Time last_image_time_;
	double pose_text_z_offset_{0.6};
	double pose_text_scale_{0.18};
	double status_period_sec_{5.0};
	int path_publish_every_n_{5};
	int marker_publish_every_n_{5};
	int tf_publish_every_n_{1};
	bool publish_tf_{true};
	bool pose_text_enabled_{true};
	bool robot_marker_enabled_{true};
	bool input_path_relay_enabled_{true};
	bool image_enabled_{true};
	bool first_odom_logged_{false};
};

int main(int argc, char ** argv)
{
	int exit_code = 0;
	try {
		rclcpp::init(argc, argv);
		auto node = std::make_shared<TagsVisualNode>();
		rclcpp::executors::SingleThreadedExecutor executor;
		executor.add_node(node);
		executor.spin();
	} catch (const std::exception & exception) {
		RCLCPP_ERROR(rclcpp::get_logger("tags_visual"), "fatal error: %s", exception.what());
		exit_code = 1;
	}

	if (rclcpp::ok()) {
		rclcpp::shutdown();
	}
	return exit_code;
}
