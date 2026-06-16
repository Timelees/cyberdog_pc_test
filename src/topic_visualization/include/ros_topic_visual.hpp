#ifndef TOPIC_VISUALIZATION__ROS_TOPIC_VISUAL_HPP_
#define TOPIC_VISUALIZATION__ROS_TOPIC_VISUAL_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace tf2_ros
{
class TransformBroadcaster;
class StaticTransformBroadcaster;
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

	/// 单台机器人在 shared_odom 坐标系下的可视化通道
	struct RobotSharedVisual
	{
		std::string namespace_name;
		std::string input_topic;
		std::string output_odom_topic;
		std::string path_topic;
		std::string child_frame_id;
		std::size_t color_index{0};
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
		nav_msgs::msg::Path path;
	};

	std::string select_namespace(int namespace_index);
	std::string trim_slashes(const std::string & value) const;
	std::string build_input_topic(const std::string & base_topic) const;
	std::string build_namespaced_topic(
		const std::string & namespace_name,
		const std::string & base_topic) const;
	std::string build_shared_child_frame_id(const std::string & namespace_name) const;

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

	void create_shared_odom_visuals(
		const rclcpp::QoS & subscribe_qos,
		const rclcpp::QoS & publish_qos,
		const rclcpp::SubscriptionOptions & subscription_options);

	void handle_odom_message(
		const nav_msgs::msg::Odometry::SharedPtr & message,
		const std::string & relay_name,
		const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & odom_publisher,
		bool publish_tf,
		const std::string & output_frame_id,
		const std::string & output_child_frame_id,
		bool enable_pose_text);

	void handle_shared_odom_message(
		std::size_t robot_index,
		const nav_msgs::msg::Odometry::SharedPtr & message);

	void publish_pose_text(
		const nav_msgs::msg::Odometry & odom,
		const std::string & namespace_label,
		const std::string & marker_namespace,
		int marker_id,
		const std::array<float, 4> & color);

	void append_shared_path(nav_msgs::msg::Path & path, const nav_msgs::msg::Odometry & odom);
	void publish_shared_robot_marker(
		const nav_msgs::msg::Odometry & odom,
		const std::string & namespace_name,
		std::size_t color_index);
	void publish_shared_odom_tf(const nav_msgs::msg::Odometry & odom, const std::string & child_frame_id);
	void publish_shared_odom_frame_anchor();
	std::vector<std::string> load_robot_namespaces();

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
	static std::array<float, 4> robot_color(std::size_t color_index);

	std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
	std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
	std::vector<RelaySpec> relay_specs_;
	std::vector<RobotSharedVisual> shared_robots_;
	std::unordered_map<std::string, std::uint64_t> relay_counts_;
	std::unordered_map<std::string, rclcpp::Time> last_receive_time_;
	mutable std::mutex status_mutex_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_text_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shared_pose_text_publisher_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr shared_robot_marker_publisher_;
	rclcpp::TimerBase::SharedPtr status_timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
	rclcpp::CallbackGroup::SharedPtr relay_callback_group_;
	std::atomic<bool> shutting_down_{false};

	// 单机器人 relay 参数
	bool pose_text_enabled_{true};
	std::vector<std::string> namespaces_;
	std::string selected_namespace_;
	std::string pose_text_topic_;
	std::string pose_text_frame_id_;
	double pose_text_z_offset_{0.6};
	double pose_text_scale_{0.18};

	// 多机器人 shared_odom 可视化参数
	bool shared_odom_enabled_{false};
	bool shared_odom_publish_tf_{false};
	bool shared_odom_relay_enabled_{true};
	bool shared_path_enabled_{true};
	bool shared_pose_text_enabled_{true};
	bool shared_robot_marker_enabled_{true};
	std::string shared_robot_marker_topic_;
	std::string shared_odom_input_topic_;
	std::string shared_odom_relay_topic_prefix_;
	std::string shared_path_topic_prefix_;
	std::string shared_pose_text_topic_;
	std::string shared_odom_frame_id_;
	std::string shared_base_frame_name_;
	std::size_t shared_path_max_poses_{5000};
};

#endif  // TOPIC_VISUALIZATION__ROS_TOPIC_VISUAL_HPP_
