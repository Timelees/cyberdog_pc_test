#ifndef MUTIL_ODOM_SHARED__ODOM_SHARED_HPP_
#define MUTIL_ODOM_SHARED__ODOM_SHARED_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Transform.h"

namespace tf2_ros
{
class TransformBroadcaster;
class StaticTransformBroadcaster;
}

namespace mutil_odom_shared
{

struct RobotRelay
{
	std::string namespace_name;
	std::string input_topic;
	std::string output_topic;
	std::string child_frame_id;
	bool is_reference{false};
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
};

/**
 * 多机器人 odom_slam 全局坐标共享节点。
 *
 * 对齐策略（与 vins_visual 中 odom_slam 对齐 leg odom 的思路一致）：
 *   以基准机器人 cyberdog_1 启动后收到的首帧 odom_slam 作为全局原点；
 *   所有机器人（含 cyberdog_1）统一转换：
 *     T_global = T_ref_origin^{-1} * T_current
 *
 * 前提：各机器人 odom_slam 处于同一 VIO/SLAM 全局地图坐标系。
 * 满足该前提时，无需任何手动 spawn 配置，相对位置由 odom_slam 自动给出。
 */
class OdomSharedNode : public rclcpp::Node
{
public:
	OdomSharedNode();

private:
	std::string trim_slashes(const std::string & value) const;
	std::string build_namespaced_topic(
		const std::string & namespace_name,
		const std::string & base_topic) const;
	std::string build_child_frame_id(const std::string & namespace_name) const;

	static tf2::Transform odometry_to_transform(const nav_msgs::msg::Odometry & odom);
	static nav_msgs::msg::Odometry transform_to_odometry(
		const tf2::Transform & transform,
		const std_msgs::msg::Header & header,
		const std::string & child_frame_id);

	void try_initialize_global_origin(const tf2::Transform & reference_transform);
	bool apply_global_alignment(
		const nav_msgs::msg::Odometry & input,
		nav_msgs::msg::Odometry & output);
	void publish_transform(
		const nav_msgs::msg::Odometry & odom,
		const std::string & child_frame_id);
	void publish_shared_odom_frame_anchor();
	void log_first_shared_pose(std::size_t robot_index, const nav_msgs::msg::Odometry & odom);

	void create_robot_relay(const std::string & namespace_name, std::size_t robot_index);
	void handle_slam_odom(std::size_t robot_index, const nav_msgs::msg::Odometry & message);
	void log_status();

	bool odom_slam_enabled_{true};
	bool publish_tf_{true};
	bool publish_odom_{true};
	bool align_enabled_{true};
	int reference_namespace_index_{0};
	std::vector<std::string> namespaces_;
	std::string reference_namespace_;
	std::string odom_slam_input_topic_base_;
	std::string odom_slam_output_topic_base_;
	std::string shared_odom_frame_id_;
	std::string base_frame_name_;

	std::mutex align_mutex_;
	bool global_origin_ready_{false};
	tf2::Transform global_origin_;
	std::vector<bool> first_shared_pose_logged_;

	std::vector<RobotRelay> robots_;
	std::unordered_map<std::string, std::uint64_t> relay_counts_;
	std::unordered_map<std::string, rclcpp::Time> last_receive_time_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
	rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace mutil_odom_shared

#endif  // MUTIL_ODOM_SHARED__ODOM_SHARED_HPP_
