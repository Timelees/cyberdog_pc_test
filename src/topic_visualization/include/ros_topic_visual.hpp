#ifndef TOPIC_VISUALIZATION__ROS_TOPIC_VISUAL_HPP_
#define TOPIC_VISUALIZATION__ROS_TOPIC_VISUAL_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace tf2_ros
{
class TransformBroadcaster;
}

class RosTopicVisualNode : public rclcpp::Node
{
public:
	RosTopicVisualNode();
	~RosTopicVisualNode() override;

private:
	struct RelaySpec
	{
		std::string name;
		std::string input_topic;
		std::string output_topic;
	};

	std::string select_namespace(int namespace_index);
	std::string trim_slashes(const std::string & value) const;
	std::string build_input_topic(const std::string & base_topic) const;

	void create_odom_relay(
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
		bool enable_pose_text = false);

	void handle_odom_message(
		const nav_msgs::msg::Odometry::SharedPtr & message,
		const std::string & relay_name,
		const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & odom_publisher,
		bool publish_tf,
		const std::string & output_frame_id,
		const std::string & output_child_frame_id,
		bool enable_pose_text);

	void publish_pose_text(const nav_msgs::msg::Odometry & odom);

	template<typename MessageT>
	void create_relay(
		bool enabled,
		const std::string & input_topic,
		const std::string & output_topic,
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const std::string & relay_name,
		const rclcpp::SubscriptionOptions & subscription_options);

	void update_relay_status(const std::string & relay_name);
	void log_status();

	static bool is_finite(double value);
	static bool is_valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion);
	static geometry_msgs::msg::Quaternion normalize_quaternion(
		const geometry_msgs::msg::Quaternion & quaternion);
	static bool extract_rpy(
		const geometry_msgs::msg::Quaternion & quaternion,
		double & roll,
		double & pitch,
		double & yaw);

	std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
	std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
	std::vector<RelaySpec> relay_specs_;
	std::unordered_map<std::string, std::uint64_t> relay_counts_;
	std::unordered_map<std::string, rclcpp::Time> last_receive_time_;
	mutable std::mutex status_mutex_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_text_publisher_;
	rclcpp::TimerBase::SharedPtr status_timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::CallbackGroup::SharedPtr relay_callback_group_;
	std::atomic<bool> shutting_down_{false};
	bool pose_text_enabled_{true};
	std::vector<std::string> namespaces_;
	std::string selected_namespace_;
	std::string pose_text_topic_;
	std::string pose_text_frame_id_;
	double pose_text_z_offset_{0.6};
	double pose_text_scale_{0.18};
};

#endif  // TOPIC_VISUALIZATION__ROS_TOPIC_VISUAL_HPP_
