#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{

constexpr double kQuaternionNormEpsilon = 1e-12;

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

void fill_pose_from_transform(const tf2::Transform & transform, nav_msgs::msg::Odometry & odom)
{
	odom.pose.pose.position.x = transform.getOrigin().x();
	odom.pose.pose.position.y = transform.getOrigin().y();
	odom.pose.pose.position.z = transform.getOrigin().z();

	const auto & rotation = transform.getRotation();
	odom.pose.pose.orientation.x = rotation.x();
	odom.pose.pose.orientation.y = rotation.y();
	odom.pose.pose.orientation.z = rotation.z();
	odom.pose.pose.orientation.w = rotation.w();
}

std::array<double, 36> rotate_pose_covariance(
	const std::array<double, 36> & covariance,
	const tf2::Matrix3x3 & rotation)
{
	std::array<double, 36> rotated{};
	double block_rotation[6][6]{};

	for (int row = 0; row < 3; ++row) {
		for (int col = 0; col < 3; ++col) {
			const auto value = rotation.getRow(row)[col];
			block_rotation[row][col] = value;
			block_rotation[row + 3][col + 3] = value;
		}
	}

	for (int row = 0; row < 6; ++row) {
		for (int col = 0; col < 6; ++col) {
			double value = 0.0;
			for (int i = 0; i < 6; ++i) {
				for (int j = 0; j < 6; ++j) {
					value += block_rotation[row][i] * covariance[i * 6 + j] * block_rotation[col][j];
				}
			}
			rotated[row * 6 + col] = value;
		}
	}

	return rotated;
}

geometry_msgs::msg::TransformStamped odom_to_transform_stamped(
	const nav_msgs::msg::Odometry & odom,
	const std::string & child_frame_id)
{
	geometry_msgs::msg::TransformStamped transform;
	transform.header = odom.header;
	transform.child_frame_id = child_frame_id;
	transform.transform.translation.x = odom.pose.pose.position.x;
	transform.transform.translation.y = odom.pose.pose.position.y;
	transform.transform.translation.z = odom.pose.pose.position.z;
	transform.transform.rotation = odom.pose.pose.orientation;
	return transform;
}

tf2::Transform make_wall_tag_to_z_up_transform()
{
	tf2::Matrix3x3 rotation;
	// Wall-mounted AprilTag frame: base.z points out of the wall and base.y points down.
	// Apply an additional +90 degree rotation around tag_map.x for RViz alignment.
	rotation.setValue(
		0.0, 0.0, 1.0,
		0.0, 1.0, 0.0,
		-1.0, 0.0, 0.0);
	tf2::Quaternion quaternion;
	rotation.getRotation(quaternion);
	quaternion.normalize();

	tf2::Transform transform;
	transform.setIdentity();
	transform.setRotation(quaternion);
	return transform;
}

}  // namespace

class GlobalAlignmentNode : public rclcpp::Node
{
public:
	GlobalAlignmentNode()
	: Node("global_alignment"),
	  tf_buffer_(get_clock()),
	  tf_listener_(tf_buffer_)
	{
		input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/cyberdog_1/odom_slam");
		output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/cyberdog_1/odom_base");
		output_path_topic_ = declare_parameter<std::string>("output_path_topic", "/cyberdog_1/path_base");
		target_frame_id_ = declare_parameter<std::string>("target_frame_id", "base");
		output_frame_id_ = declare_parameter<std::string>("output_frame_id", "tag_map");
		use_wall_tag_z_up_frame_ = declare_parameter<bool>("use_wall_tag_z_up_frame", true);
		publish_output_frame_tf_ = declare_parameter<bool>("publish_output_frame_tf", true);
		source_frame_id_ = declare_parameter<std::string>("source_frame_id", "");
		robot_frame_id_ = declare_parameter<std::string>("robot_frame_id", "base_link");
		output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "");
		publish_tf_ = declare_parameter<bool>("publish_tf", false);
		tf_child_frame_id_ = declare_parameter<std::string>("tf_child_frame_id", "base_link_global");
		path_max_poses_ = declare_parameter<int>("path_max_poses", 2000);
		path_publish_every_n_ = declare_parameter<int>("path_publish_every_n", 5);
		use_current_time_ = declare_parameter<bool>("use_current_time", true);
		const auto subscribe_best_effort = declare_parameter<bool>("subscribe_best_effort", true);
		const auto publish_best_effort = declare_parameter<bool>("publish_best_effort", false);

		if (target_frame_id_.empty()) {
			throw std::runtime_error("target_frame_id must not be empty");
		}
		if (output_frame_id_.empty()) {
			output_frame_id_ = target_frame_id_;
		}
		if (use_wall_tag_z_up_frame_ && output_frame_id_ == target_frame_id_) {
			throw std::runtime_error("output_frame_id must differ from target_frame_id when use_wall_tag_z_up_frame is true");
		}
		if (input_odom_topic_.empty() || output_odom_topic_.empty()) {
			throw std::runtime_error("input_odom_topic and output_odom_topic must not be empty");
		}
		if (path_max_poses_ < 0) {
			path_max_poses_ = 0;
		}
		if (path_publish_every_n_ < 1) {
			path_publish_every_n_ = 1;
		}

		const auto subscribe_qos = subscribe_best_effort ?
			rclcpp::SensorDataQoS() :
			rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
		const auto publish_qos = publish_best_effort ?
			rclcpp::QoS(rclcpp::KeepLast(50)).best_effort() :
			rclcpp::QoS(rclcpp::KeepLast(50)).reliable();

		odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, publish_qos);
		if (!output_path_topic_.empty()) {
			path_publisher_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, publish_qos);
		}
		if (publish_tf_) {
			tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
		}
		output_from_target_ = use_wall_tag_z_up_frame_ ?
			make_wall_tag_to_z_up_transform() :
			tf2::Transform::getIdentity();
		if (publish_output_frame_tf_ && output_frame_id_ != target_frame_id_) {
			static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
			publish_output_frame_tf();
		}

		odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
			input_odom_topic_,
			subscribe_qos,
			[this](nav_msgs::msg::Odometry::SharedPtr message) {
				handle_odom(message);
			});

		RCLCPP_INFO(
			get_logger(),
			"global_alignment waiting for TF %s <- %s, output_frame=%s, input=%s output=%s",
			target_frame_id_.c_str(),
			robot_frame_id_.c_str(),
			output_frame_id_.c_str(),
			input_odom_topic_.c_str(),
			output_odom_topic_.c_str());
	}

private:
	void handle_odom(const nav_msgs::msg::Odometry::SharedPtr & message)
	{
		if (!message) {
			return;
		}

		const std::string source_frame =
			source_frame_id_.empty() ? message->header.frame_id : source_frame_id_;
		if (source_frame.empty()) {
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				2000,
				"skip odom without header.frame_id; set source_frame_id if the input omits it");
			return;
		}

		const auto source_to_child = odometry_to_transform(*message);
		if (!alignment_initialized_ && !try_initialize_alignment(*message, source_frame, source_to_child)) {
			return;
		}

		if (source_frame != initialized_source_frame_) {
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				5000,
				"input odom frame changed from initialized frame %s to %s; keep using locked global alignment",
				initialized_source_frame_.c_str(),
				source_frame.c_str());
		}

		const auto output_to_child = output_from_source_ * source_to_child;

		auto output = *message;
		output.header.frame_id = output_frame_id_;
		if (use_current_time_) {
			output.header.stamp = now();
		}
		if (!output_child_frame_id_.empty()) {
			output.child_frame_id = output_child_frame_id_;
		}
		fill_pose_from_transform(output_to_child, output);
		output.pose.covariance = rotate_pose_covariance(
			message->pose.covariance,
			output_from_source_.getBasis());

		odom_count_ += 1;
		odom_publisher_->publish(output);
		append_and_publish_path(output, odom_count_);

		if (tf_broadcaster_) {
			const auto child_frame = tf_child_frame_id_.empty() ? output.child_frame_id : tf_child_frame_id_;
			if (!child_frame.empty()) {
				tf_broadcaster_->sendTransform(odom_to_transform_stamped(output, child_frame));
			}
		}
	}

	bool try_initialize_alignment(
		const nav_msgs::msg::Odometry & odom,
		const std::string & source_frame,
		const tf2::Transform & source_to_child)
	{
		try {
			const auto target_to_robot_msg = tf_buffer_.lookupTransform(
				target_frame_id_,
				robot_frame_id_,
				tf2::TimePointZero);
			const auto target_to_robot = transform_msg_to_tf2(target_to_robot_msg.transform);
			const auto source_to_robot = source_to_child * lookup_child_to_robot(odom);
			target_from_source_ = target_to_robot * source_to_robot.inverse();
			output_from_source_ = output_from_target_ * target_from_source_;
			initialized_source_frame_ = source_frame;
			alignment_initialized_ = true;

			RCLCPP_INFO(
				get_logger(),
				"global alignment locked: %s <- %s via initial %s, translation=[%.3f, %.3f, %.3f]. Future odom is converted with this fixed transform, independent of tag visibility.",
				output_frame_id_.c_str(),
				source_frame.c_str(),
				robot_frame_id_.c_str(),
				output_from_source_.getOrigin().x(),
				output_from_source_.getOrigin().y(),
				output_from_source_.getOrigin().z());
			return true;
		} catch (const tf2::TransformException & exception) {
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				2000,
				"waiting for alignment TF %s <- %s: %s",
				target_frame_id_.c_str(),
				robot_frame_id_.c_str(),
				exception.what());
		}

		return false;
	}

	tf2::Transform lookup_child_to_robot(const nav_msgs::msg::Odometry & odom)
	{
		if (robot_frame_id_.empty() ||
			odom.child_frame_id.empty() ||
			odom.child_frame_id == robot_frame_id_)
		{
			return tf2::Transform::getIdentity();
		}

		const auto child_to_robot_msg = tf_buffer_.lookupTransform(
			odom.child_frame_id,
			robot_frame_id_,
			tf2::TimePointZero);
		return transform_msg_to_tf2(child_to_robot_msg.transform);
	}

	void publish_output_frame_tf()
	{
		if (!static_tf_broadcaster_ || output_frame_id_ == target_frame_id_) {
			return;
		}

		const auto target_from_output = output_from_target_.inverse();
		geometry_msgs::msg::TransformStamped transform;
		transform.header.stamp = now();
		transform.header.frame_id = target_frame_id_;
		transform.child_frame_id = output_frame_id_;
		transform.transform.translation.x = target_from_output.getOrigin().x();
		transform.transform.translation.y = target_from_output.getOrigin().y();
		transform.transform.translation.z = target_from_output.getOrigin().z();
		const auto & rotation = target_from_output.getRotation();
		transform.transform.rotation.x = rotation.x();
		transform.transform.rotation.y = rotation.y();
		transform.transform.rotation.z = rotation.z();
		transform.transform.rotation.w = rotation.w();
		static_tf_broadcaster_->sendTransform(transform);
	}

	void append_and_publish_path(const nav_msgs::msg::Odometry & odom, std::uint64_t odom_count)
	{
		if (!path_publisher_) {
			return;
		}

		geometry_msgs::msg::PoseStamped pose;
		pose.header = odom.header;
		pose.pose = odom.pose.pose;
		path_.header = odom.header;
		path_.poses.push_back(std::move(pose));

		if (path_max_poses_ > 0 && path_.poses.size() > static_cast<std::size_t>(path_max_poses_)) {
			const auto overflow = path_.poses.size() - static_cast<std::size_t>(path_max_poses_);
			path_.poses.erase(path_.poses.begin(), path_.poses.begin() + static_cast<std::ptrdiff_t>(overflow));
		}

		if (odom_count % static_cast<std::uint64_t>(path_publish_every_n_) == 0) {
			path_publisher_->publish(path_);
		}
	}

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	nav_msgs::msg::Path path_;
	tf2::Transform target_from_source_;
	tf2::Transform output_from_target_;
	tf2::Transform output_from_source_;
	std::string input_odom_topic_;
	std::string output_odom_topic_;
	std::string output_path_topic_;
	std::string target_frame_id_;
	std::string output_frame_id_;
	std::string source_frame_id_;
	std::string initialized_source_frame_;
	std::string robot_frame_id_;
	std::string output_child_frame_id_;
	std::string tf_child_frame_id_;
	int path_max_poses_{2000};
	int path_publish_every_n_{5};
	std::uint64_t odom_count_{0};
	bool publish_tf_{false};
	bool publish_output_frame_tf_{true};
	bool use_wall_tag_z_up_frame_{true};
	bool use_current_time_{true};
	bool alignment_initialized_{false};
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GlobalAlignmentNode>());
	rclcpp::shutdown();
	return 0;
}
