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

#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"

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

	int get_executor_threads() const { return executor_threads_; }

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

	static tf2::Transform make_transform(
		double x,
		double y,
		double z,
		double roll,
		double pitch,
		double yaw);
	void update_base_link_to_optical_transform();
	bool update_tag_optical_to_tag_from_detection(
		const apriltag_msgs::msg::AprilTagDetectionArray & detections);
	void on_tag_tf_message(const tf2_msgs::msg::TFMessage::SharedPtr message);
	void create_tag_align_subscribers(const rclcpp::SubscriptionOptions & subscription_options);

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
	void apply_header_stamp_mode(std_msgs::msg::Header & header, const std::string & stamp_mode) const;
	bool should_publish_at_max_hz(double max_hz, rclcpp::Time & last_publish_time);
	bool should_publish_decimated(int decimation, std::uint64_t & counter);
	void publish_odometry_tf(const nav_msgs::msg::Odometry & odom);
	void try_initialize_alignment(
		const tf2::Transform & leg_transform,
		const tf2::Transform & slam_transform);
	bool apply_slam_alignment(
		const nav_msgs::msg::Odometry & input,
		nav_msgs::msg::Odometry & output);
	bool apply_tag_alignment(
		const nav_msgs::msg::Odometry & input,
		nav_msgs::msg::Odometry & output);
	std::string resolve_tag_align_child_frame_id(const std::string & input_child_frame_id) const;

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
	std::mutex tag_pose_mutex_;
	rclcpp::TimerBase::SharedPtr status_timer_;
	rclcpp::CallbackGroup::SharedPtr relay_callback_group_;
	rclcpp::CallbackGroup::SharedPtr image_callback_group_;
	rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
	rclcpp::CallbackGroup::SharedPtr point_cloud_callback_group_;
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
	std::string leg_odom_base_child_frame_id_;
	std::string slam_odom_output_child_frame_id_;
	std::string leg_path_topic_;
	std::string slam_path_topic_;
	std::string camera_static_tf_parent_frame_id_;
	std::string camera_static_tf_child_frame_id_;
	std::string depth_points_output_frame_id_;
	std::string depth_points_stamp_mode_;
	std::string image_stamp_mode_;
	std::string dynamic_tf_stamp_mode_;
	bool frame_remap_enabled_{false};
	bool leg_odom_publish_tf_{true};
	bool leg_odom_publish_base_tf_{true};
	bool odom_slam_publish_tf_{true};
	bool odom_slam_align_enabled_{true};
	bool tag_align_enabled_{false};
	bool tag_align_namespace_child_frame_{true};
	bool tag_align_compose_from_detection_{true};
	bool tag_align_recalibrate_on_tag_update_{false};
	bool camera_static_tf_enabled_{true};
	bool tag_pose_valid_{false};
	bool camera_intrinsics_valid_{false};
	bool alignment_initialized_{false};
	bool leg_odom_received_{false};
	bool tag_vio_alignment_valid_{false};
	std::optional<tf2::Transform> pending_slam_transform_;
	std::optional<tf2::Transform> last_leg_transform_;
	tf2::Transform odom_to_slam_origin_;
	tf2::Transform tag_global_to_tag_;
	tf2::Transform base_link_to_optical_;
	tf2::Transform tag_optical_to_tag_;
	tf2::Transform tag_to_vio_frame_;
	nav_msgs::msg::Path leg_compare_path_;
	nav_msgs::msg::Path slam_compare_path_;
	std::size_t compare_path_max_poses_{10000};
	double image_publish_max_hz_{15.0};
	double point_cloud_publish_max_hz_{5.0};
	double path_publish_max_hz_{5.0};
	int image_publish_decimation_{1};
	int point_cloud_publish_decimation_{2};
	rclcpp::Time last_slam_path_publish_time_;
	rclcpp::Time last_leg_path_publish_time_;
	rclcpp::Time last_point_cloud_publish_time_;
	std::unordered_map<std::string, rclcpp::Time> last_image_publish_times_;
	std::unordered_map<std::string, std::uint64_t> image_publish_counters_;
	std::uint64_t point_cloud_publish_counter_{0};
	std::uint64_t tag_pose_sequence_{0};
	std::uint64_t consumed_tag_pose_sequence_{0};
	double tag_edge_size_{0.3};
	double camera_intrinsics_[4]{};
	int tag_detection_id_{0};
	int executor_threads_{4};
	std::string tag_frame_id_;
	std::string tag_optical_frame_id_;
	std::string tag_detection_topic_;
	std::string tag_camera_info_topic_;
	std::string tag_align_output_child_frame_id_;
};

#endif  // TOPIC_VISUALIZATION__VINS_VISUAL_HPP_
