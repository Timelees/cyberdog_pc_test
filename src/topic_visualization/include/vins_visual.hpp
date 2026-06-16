#ifndef TOPIC_VISUALIZATION__VINS_VISUAL_HPP_
#define TOPIC_VISUALIZATION__VINS_VISUAL_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Transform.h"

namespace tf2_ros
{
class TransformBroadcaster;
class StaticTransformBroadcaster;
}

class VinsVisualNode : public rclcpp::Node
{
public:
	VinsVisualNode();
	~VinsVisualNode() override;

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
	std::string remap_frame_id(const std::string & frame_id) const;

	static tf2::Transform odometry_to_transform(const nav_msgs::msg::Odometry & odom);
	static nav_msgs::msg::Odometry transform_to_odometry(
		const tf2::Transform & transform,
		const std_msgs::msg::Header & header,
		const std::string & child_frame_id);
	static geometry_msgs::msg::PoseStamped transform_to_pose_stamped(
		const tf2::Transform & transform,
		const std_msgs::msg::Header & header,
		const std::string & frame_id);
	void append_path_pose(nav_msgs::msg::Path & path, geometry_msgs::msg::PoseStamped pose);
	void try_initialize_alignment(
		const tf2::Transform & leg_transform,
		const tf2::Transform & slam_transform);
	bool apply_slam_alignment(
		const nav_msgs::msg::Odometry & input,
		nav_msgs::msg::Odometry & output);

	void create_image_relay(
		bool enabled,
		const std::string & relay_name,
		const std::string & input_topic,
		const std::string & output_topic,
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const rclcpp::SubscriptionOptions & subscription_options);

	void create_point_cloud_relay(
		bool enabled,
		const std::string & relay_name,
		const std::string & input_topic,
		const std::string & output_topic,
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const rclcpp::SubscriptionOptions & subscription_options,
		bool remap_frame = false,
		const std::string & output_frame_id = "");

	void publish_camera_static_tf();

	void create_path_relay(
		bool enabled,
		const std::string & relay_name,
		const std::string & input_topic,
		const std::string & output_topic,
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const rclcpp::SubscriptionOptions & subscription_options);

	void create_odom_relay(
		bool enabled,
		const std::string & relay_name,
		const std::string & input_topic,
		const std::string & output_topic,
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const rclcpp::SubscriptionOptions & subscription_options,
		bool remap_frame,
		const std::string & output_frame_id = "",
		const std::string & output_child_frame_id = "",
		bool publish_tf = false);

	void create_leg_odom_compare_relay(
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const rclcpp::SubscriptionOptions & subscription_options);

	void create_odom_slam_compare_relay(
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const rclcpp::SubscriptionOptions & subscription_options);

	void create_lifecycle_monitor(
		bool enabled,
		const std::string & relay_name,
		const std::string & input_topic,
		const rclcpp::SubscriptionOptions & subscription_options);

	void update_relay_status(const std::string & relay_name);
	void log_status();

	std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
	std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
	std::vector<RelaySpec> relay_specs_;
	std::unordered_map<std::string, std::uint64_t> relay_counts_;
	std::unordered_map<std::string, rclcpp::Time> last_receive_time_;
	mutable std::mutex status_mutex_;
	std::mutex compare_mutex_;
	rclcpp::TimerBase::SharedPtr status_timer_;
	rclcpp::CallbackGroup::SharedPtr relay_callback_group_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leg_odom_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr slam_odom_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr leg_path_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_publisher_;
	std::atomic<bool> shutting_down_{false};
	std::vector<std::string> namespaces_;
	std::string selected_namespace_;
	std::string input_frame_id_;
	std::string output_frame_id_;
	std::string compare_frame_id_;
	std::string leg_odom_output_frame_id_;
	std::string leg_odom_output_child_frame_id_;
	std::string slam_odom_output_child_frame_id_;
	std::string leg_path_topic_;
	std::string slam_path_topic_;
	std::string camera_static_tf_parent_frame_id_;
	std::string camera_static_tf_child_frame_id_;
	std::string depth_points_output_frame_id_;
	bool frame_remap_enabled_{false};
	bool leg_odom_publish_tf_{true};
	bool odom_slam_align_enabled_{true};
	bool camera_static_tf_enabled_{true};
	bool alignment_initialized_{false};
	bool leg_odom_received_{false};
	std::optional<tf2::Transform> pending_slam_transform_;
	std::optional<tf2::Transform> last_leg_transform_;
	tf2::Transform odom_to_slam_origin_;
	nav_msgs::msg::Path leg_compare_path_;
	nav_msgs::msg::Path slam_compare_path_;
	std::size_t compare_path_max_poses_{10000};
};

#endif  // TOPIC_VISUALIZATION__VINS_VISUAL_HPP_
