#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
constexpr double kQuaternionNormEpsilon = 1e-6;
constexpr double kOpticalFrameRoll = -1.5707963267948966;
constexpr double kOpticalFramePitch = 0.0;
constexpr double kOpticalFrameYaw = -1.5707963267948966;

std::string trim_slashes(const std::string & value)
{
	const auto start = value.find_first_not_of('/');
	if (start == std::string::npos) {
		return "";
	}
	const auto end = value.find_last_not_of('/');
	return value.substr(start, end - start + 1);
}

std::string scoped_topic(const std::string & name_space, const std::string & topic)
{
	const auto normalized_topic = trim_slashes(topic);
	const auto normalized_namespace = trim_slashes(name_space);
	if (normalized_namespace.empty()) {
		return normalized_topic.empty() ? "/" : "/" + normalized_topic;
	}
	if (normalized_topic.empty()) {
		return "/" + normalized_namespace;
	}
	const auto prefix = normalized_namespace + "/";
	if (normalized_topic.rfind(prefix, 0) == 0) {
		return "/" + normalized_topic;
	}
	return "/" + normalized_namespace + "/" + normalized_topic;
}

tf2::Transform make_transform(
	double x,
	double y,
	double z,
	double roll,
	double pitch,
	double yaw)
{
	tf2::Transform transform;
	transform.setOrigin(tf2::Vector3(x, y, z));
	tf2::Quaternion rotation;
	rotation.setRPY(roll, pitch, yaw);
	rotation.normalize();
	transform.setRotation(rotation);
	return transform;
}

tf2::Transform odometry_to_transform(const nav_msgs::msg::Odometry & odom)
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

nav_msgs::msg::Odometry transform_to_odometry(
	const tf2::Transform & transform,
	const nav_msgs::msg::Odometry & input,
	const std::string & frame_id,
	const std::string & child_frame_id)
{
	auto output = input;
	output.header.frame_id = frame_id;
	output.child_frame_id = child_frame_id;
	output.pose.pose.position.x = transform.getOrigin().x();
	output.pose.pose.position.y = transform.getOrigin().y();
	output.pose.pose.position.z = transform.getOrigin().z();
	const auto & rotation = transform.getRotation();
	output.pose.pose.orientation.x = rotation.x();
	output.pose.pose.orientation.y = rotation.y();
	output.pose.pose.orientation.z = rotation.z();
	output.pose.pose.orientation.w = rotation.w();
	return output;
}

tf2::Transform transform_msg_to_tf2(const geometry_msgs::msg::Transform & msg)
{
	tf2::Transform transform;
	transform.setOrigin(tf2::Vector3(
		msg.translation.x,
		msg.translation.y,
		msg.translation.z));
	tf2::Quaternion rotation(
		msg.rotation.x,
		msg.rotation.y,
		msg.rotation.z,
		msg.rotation.w);
	if (rotation.length2() <= kQuaternionNormEpsilon) {
		rotation.setValue(0.0, 0.0, 0.0, 1.0);
	} else {
		rotation.normalize();
	}
	transform.setRotation(rotation);
	return transform;
}
}

class MutilOdomSharedNode : public rclcpp::Node
{
public:
	MutilOdomSharedNode()
	: Node("mutil_odom_shared")
	{
		robot_namespaces_ = declare_parameter<std::vector<std::string>>(
			"robot_namespaces",
			std::vector<std::string>{"cyberdog_1", "cyberdog_2"});
		odom_slam_input_topic_ =
			declare_parameter<std::string>("odom_slam_input_topic", "/odom_slam");
		odom_slam_output_topic_ =
			declare_parameter<std::string>("odom_slam_output_topic", "/odom_shared");
		shared_odom_frame_id_ =
			declare_parameter<std::string>("shared_odom_frame_id", "shared_odom");
		base_frame_name_ = declare_parameter<std::string>("base_frame_name", "base_link");
		publish_tf_ = declare_parameter<bool>("publish_tf", true);
		publish_odom_ = declare_parameter<bool>("publish_odom", true);
		align_enabled_ = declare_parameter<bool>("align_enabled", true);
		odom_slam_enabled_ = declare_parameter<bool>("odom_slam_enabled", true);

		tag_align_enabled_ = declare_parameter<bool>("tag_align_enabled", true);
		tag_frame_id_ = declare_parameter<std::string>("tag_frame_id", "base");
		tag_optical_frame_id_ =
			declare_parameter<std::string>("tag_optical_frame_id", "camera_infra1_optical_frame");
		tag_tf_topic_ = declare_parameter<std::string>("tag_tf_topic", "/tf");
		tag_align_recalibrate_on_tag_update_ =
			declare_parameter<bool>("tag_align_recalibrate_on_tag_update", false);
		tag_global_to_tag_ = make_transform(
			declare_parameter<double>("tag_global_to_tag_x", 0.0),
			declare_parameter<double>("tag_global_to_tag_y", 0.0),
			declare_parameter<double>("tag_global_to_tag_z", 0.0),
			declare_parameter<double>("tag_global_to_tag_roll", 0.0),
			declare_parameter<double>("tag_global_to_tag_pitch", 0.0),
			declare_parameter<double>("tag_global_to_tag_yaw", 0.0));

		update_base_link_to_optical_transform();
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
		create_tag_tf_subscription();
		if (odom_slam_enabled_) {
			create_odom_subscriptions();
		} else {
			RCLCPP_INFO(get_logger(), "skip odom_slam subscriptions because odom_slam_enabled=false");
		}

		RCLCPP_INFO(
			get_logger(),
			"mutil_odom_shared ready: robots=%zu, input=%s, output=%s, tag_align=%s",
			robot_namespaces_.size(),
			odom_slam_input_topic_.c_str(),
			odom_slam_output_topic_.c_str(),
			tag_align_enabled_ ? tag_frame_id_.c_str() : "disabled");
	}

private:
	struct RobotState
	{
		std::string name_space;
		std::string child_frame_id;
		bool alignment_valid{false};
		std::uint64_t consumed_tag_pose_sequence{0};
		tf2::Transform shared_to_vio_frame;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
	};

	void update_base_link_to_optical_transform()
	{
		const auto base_to_camera = make_transform(
			declare_parameter<double>("camera_static_tf_x", 0.26932),
			declare_parameter<double>("camera_static_tf_y", 0.0),
			declare_parameter<double>("camera_static_tf_z", 0.11543),
			declare_parameter<double>("camera_static_tf_roll", 0.0),
			declare_parameter<double>("camera_static_tf_pitch", 0.19),
			declare_parameter<double>("camera_static_tf_yaw", 0.0));
		const auto aligned_to_optical = make_transform(
			0.0,
			0.0,
			0.0,
			kOpticalFrameRoll,
			kOpticalFramePitch,
			kOpticalFrameYaw);
		base_link_to_optical_ = base_to_camera * aligned_to_optical;
	}

	void create_tag_tf_subscription()
	{
		if (!tag_align_enabled_) {
			return;
		}

		tag_tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
			tag_tf_topic_,
			rclcpp::QoS(100),
			[this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
				if (!message) {
					return;
				}
				for (const auto & transform : message->transforms) {
					if (transform.header.frame_id != tag_optical_frame_id_ ||
						transform.child_frame_id != tag_frame_id_)
					{
						continue;
					}
					tag_optical_to_tag_ = transform_msg_to_tf2(transform.transform);
					tag_pose_valid_ = true;
					++tag_pose_sequence_;
				}
			});
	}

	void create_odom_subscriptions()
	{
		const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
		for (const auto & robot_namespace : robot_namespaces_) {
			auto state = std::make_shared<RobotState>();
			state->name_space = robot_namespace;
			const auto normalized_namespace = trim_slashes(robot_namespace);
			state->child_frame_id = normalized_namespace.empty()
				? base_frame_name_
				: normalized_namespace + "/" + base_frame_name_;

			const auto input_topic = scoped_topic(robot_namespace, odom_slam_input_topic_);
			const auto output_topic = scoped_topic(robot_namespace, odom_slam_output_topic_);
			if (publish_odom_) {
				state->publisher = create_publisher<nav_msgs::msg::Odometry>(output_topic, qos);
			}
			state->subscription = create_subscription<nav_msgs::msg::Odometry>(
				input_topic,
				qos,
				[this, state](const nav_msgs::msg::Odometry::SharedPtr message) {
					on_odom_slam(state, message);
				});
			robot_states_.push_back(state);
			RCLCPP_INFO(
				get_logger(),
				"shared odom relay: %s -> %s",
				input_topic.c_str(),
				output_topic.c_str());
		}
	}

	void on_odom_slam(
		const std::shared_ptr<RobotState> & state,
		const nav_msgs::msg::Odometry::SharedPtr message)
	{
		if (!state || !message) {
			return;
		}

		const auto vio_to_base_link = odometry_to_transform(*message);
		tf2::Transform shared_to_base_link;
		std::string frame_id = shared_odom_frame_id_;

		if (tag_align_enabled_) {
			if (!tag_pose_valid_) {
				RCLCPP_WARN_THROTTLE(
					get_logger(),
					*get_clock(),
					5000,
					"skip %s because tag TF %s -> %s is unavailable",
					state->name_space.c_str(),
					tag_optical_frame_id_.c_str(),
					tag_frame_id_.c_str());
				return;
			}

			const auto observed_tag_to_base_link =
				tag_global_to_tag_ * tag_optical_to_tag_.inverse() * base_link_to_optical_.inverse();
			if (!state->alignment_valid ||
				(tag_align_recalibrate_on_tag_update_ &&
				state->consumed_tag_pose_sequence != tag_pose_sequence_))
			{
				state->shared_to_vio_frame =
					observed_tag_to_base_link * vio_to_base_link.inverse();
				state->consumed_tag_pose_sequence = tag_pose_sequence_;
				state->alignment_valid = true;
				RCLCPP_INFO(
					get_logger(),
					"%s calibrated %s -> %s, translation=[%.3f, %.3f, %.3f]",
					state->name_space.c_str(),
					message->header.frame_id.c_str(),
					tag_frame_id_.c_str(),
					state->shared_to_vio_frame.getOrigin().x(),
					state->shared_to_vio_frame.getOrigin().y(),
					state->shared_to_vio_frame.getOrigin().z());
			}
			shared_to_base_link = state->shared_to_vio_frame * vio_to_base_link;
			frame_id = tag_frame_id_;
		} else if (align_enabled_) {
			if (!state->alignment_valid) {
				state->shared_to_vio_frame = vio_to_base_link.inverse();
				state->alignment_valid = true;
			}
			shared_to_base_link = state->shared_to_vio_frame * vio_to_base_link;
		} else {
			shared_to_base_link = vio_to_base_link;
		}

		auto output = transform_to_odometry(
			shared_to_base_link,
			*message,
			frame_id,
			state->child_frame_id);
		if (output.header.stamp.sec == 0 && output.header.stamp.nanosec == 0) {
			output.header.stamp = now();
		}
		if (publish_odom_ && state->publisher) {
			state->publisher->publish(output);
		}
		if (publish_tf_) {
			publish_tf(output);
		}
	}

	void publish_tf(const nav_msgs::msg::Odometry & odom)
	{
		if (!tf_broadcaster_) {
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

	std::vector<std::string> robot_namespaces_;
	std::vector<std::shared_ptr<RobotState>> robot_states_;
	std::string odom_slam_input_topic_;
	std::string odom_slam_output_topic_;
	std::string shared_odom_frame_id_;
	std::string base_frame_name_;
	std::string tag_frame_id_;
	std::string tag_optical_frame_id_;
	std::string tag_tf_topic_;
	bool publish_tf_{true};
	bool publish_odom_{true};
	bool align_enabled_{true};
	bool odom_slam_enabled_{true};
	bool tag_align_enabled_{true};
	bool tag_align_recalibrate_on_tag_update_{false};
	bool tag_pose_valid_{false};
	std::uint64_t tag_pose_sequence_{0};
	tf2::Transform tag_global_to_tag_;
	tf2::Transform base_link_to_optical_;
	tf2::Transform tag_optical_to_tag_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tag_tf_subscription_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MutilOdomSharedNode>());
	rclcpp::shutdown();
	return 0;
}
