#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

struct RelaySpec
{
	std::string name;
	std::string input_topic;
	std::string output_topic;
};

class RosTopicVisualNode : public rclcpp::Node
{
public:
	RosTopicVisualNode()
	: Node("ros_topic_visual")
	{
		const auto sensor_qos = rclcpp::SensorDataQoS();
		const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

		namespaces_ = declare_parameter<std::vector<std::string>>("namespace", std::vector<std::string>{});
		const auto namespace_index = declare_parameter<int>("namespace_index", 0);
		selected_namespace_ = select_namespace(namespace_index);

		create_relay<sensor_msgs::msg::LaserScan>(
			declare_parameter<bool>("scan_enabled", true),
			build_input_topic(declare_parameter<std::string>("scan_input_topic", "/scan")),
			declare_parameter<std::string>("scan_output_topic", "/scan"),
			sensor_qos,
			sensor_qos,
			"scan");

		create_odom_relay(
			declare_parameter<bool>("odom_enabled", true),
			build_input_topic(declare_parameter<std::string>("odom_input_topic", "/odom_out")),
			declare_parameter<std::string>("odom_output_topic", "/odom"),
			declare_parameter<bool>("odom_publish_tf", true),
			declare_parameter<std::string>("odom_output_frame_id", "odom"),
			declare_parameter<std::string>("odom_output_child_frame_id", "base_link"),
			reliable_qos,
			reliable_qos);


		const auto status_period_sec = declare_parameter<double>("status_period_sec", 5.0);
		if (status_period_sec > 0.0) {
			const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::duration<double>(status_period_sec));
			status_timer_ = create_wall_timer(period, [this]() {log_status();});
		}

		RCLCPP_INFO(get_logger(), "ros_topic_visual is ready with %zu relay channels", relay_specs_.size());
	}

private:
	std::string select_namespace(int namespace_index)
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

	std::string trim_slashes(const std::string & value) const
	{
		auto start = value.find_first_not_of('/');
		if (start == std::string::npos) {
			return "";
		}
		auto end = value.find_last_not_of('/');
		return value.substr(start, end - start + 1);
	}

	std::string build_input_topic(const std::string & base_topic) const
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

	void create_odom_relay(
		bool enabled,
		const std::string & input_topic,
		const std::string & output_topic,
		bool publish_tf,
		const std::string & output_frame_id,
		const std::string & output_child_frame_id,
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos)
	{
		if (!enabled) {
			RCLCPP_INFO(get_logger(), "skip disabled relay: odom");
			return;
		}

		odom_publish_tf_ = publish_tf;
		odom_output_frame_id_ = output_frame_id;
		odom_output_child_frame_id_ = output_child_frame_id;
		pose_text_enabled_ = declare_parameter<bool>("pose_text_enabled", true);
		pose_text_topic_ = declare_parameter<std::string>("pose_text_topic", "/base_link_pose_text");
		pose_text_frame_id_ = declare_parameter<std::string>("pose_text_frame_id", output_child_frame_id);
		pose_text_z_offset_ = declare_parameter<double>("pose_text_z_offset", 0.6);
		pose_text_scale_ = declare_parameter<double>("pose_text_scale", 0.18);

		if (pose_text_enabled_ && !pose_text_topic_.empty()) {
			pose_text_publisher_ =
				create_publisher<visualization_msgs::msg::Marker>(pose_text_topic_, rclcpp::QoS(10).reliable());
		}

		odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, publish_qos);
		odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
			input_topic,
			subscribe_qos,
			[this](nav_msgs::msg::Odometry::SharedPtr message) {
				nav_msgs::msg::Odometry output = *message;

				if (!odom_output_frame_id_.empty()) {
					output.header.frame_id = odom_output_frame_id_;
				}
				if (!odom_output_child_frame_id_.empty()) {
					output.child_frame_id = odom_output_child_frame_id_;
				}

				odom_publisher_->publish(output);
				update_relay_status("odom");
				publish_pose_text(output);

				if (odom_publish_tf_ && !output.header.frame_id.empty() && !output.child_frame_id.empty()) {
					geometry_msgs::msg::TransformStamped transform;
					transform.header = output.header;
					transform.child_frame_id = output.child_frame_id;
					transform.transform.translation.x = output.pose.pose.position.x;
					transform.transform.translation.y = output.pose.pose.position.y;
					transform.transform.translation.z = output.pose.pose.position.z;
					transform.transform.rotation = output.pose.pose.orientation;
					tf_broadcaster_->sendTransform(transform);
				}
			});

		publishers_.push_back(odom_publisher_);
		subscriptions_.push_back(odom_subscription_);
		relay_specs_.push_back(RelaySpec{"odom", input_topic, output_topic});

	}

	void publish_pose_text(const nav_msgs::msg::Odometry & odom)
	{
		if (!pose_text_enabled_ || !pose_text_publisher_) {
			return;
		}

		double roll = 0.0;
		double pitch = 0.0;
		double yaw = 0.0;
		const tf2::Quaternion quaternion(
			odom.pose.pose.orientation.x,
			odom.pose.pose.orientation.y,
			odom.pose.pose.orientation.z,
			odom.pose.pose.orientation.w);
		tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

		std::ostringstream text_stream;
		text_stream << std::fixed << std::setprecision(1)
			<< "[" << (selected_namespace_.empty() ? "default" : selected_namespace_) << "] "
			<< "position:["
			<< odom.pose.pose.position.x << ","
			<< odom.pose.pose.position.y << ","
			<< odom.pose.pose.position.z << "], "
			<< "orientation:["
			<< yaw << ","
			<< pitch << ","
			<< roll << "]";

		visualization_msgs::msg::Marker marker;
		marker.header.stamp = odom.header.stamp;
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
	void create_relay(
		bool enabled,
		const std::string & input_topic,
		const std::string & output_topic,
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const std::string & relay_name)
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
				publisher->publish(*message);
				update_relay_status(relay_name);
			});

		publishers_.push_back(publisher);
		subscriptions_.push_back(subscription);
		relay_specs_.push_back(RelaySpec{relay_name, input_topic, output_topic});

	}

	void update_relay_status(const std::string & relay_name)
	{
		relay_counts_[relay_name] += 1;
		last_receive_time_[relay_name] = now();
	}

	void log_status()
	{
		for (const auto & relay : relay_specs_) {

			auto age_text = std::string("never");
			const auto time_iter = last_receive_time_.find(relay.name);
			if (time_iter != last_receive_time_.end() && time_iter->second.nanoseconds() > 0) {
				const auto age = (now() - time_iter->second).seconds();
				age_text = std::to_string(age) + "s";
			}

		}
	}

	std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
	std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
	std::vector<RelaySpec> relay_specs_;
	std::unordered_map<std::string, std::uint64_t> relay_counts_;
	std::unordered_map<std::string, rclcpp::Time> last_receive_time_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_text_publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::TimerBase::SharedPtr status_timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	bool odom_publish_tf_{true};
	bool pose_text_enabled_{true};
	std::vector<std::string> namespaces_;
	std::string selected_namespace_;
	std::string odom_output_frame_id_;
	std::string odom_output_child_frame_id_;
	std::string pose_text_topic_;
	std::string pose_text_frame_id_;
	double pose_text_z_offset_{0.6};
	double pose_text_scale_{0.18};
};

}  // namespace

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RosTopicVisualNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
