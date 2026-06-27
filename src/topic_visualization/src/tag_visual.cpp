#include "tag_visual.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include <opencv2/calib3d.hpp>

namespace
{
constexpr double kQuaternionNormEpsilon = 1e-6;
constexpr double kOpticalFrameRoll = -1.5707963267948966;
constexpr double kOpticalFramePitch = 0.0;
constexpr double kOpticalFrameYaw = -1.5707963267948966;

std::string normalize_stamp_mode(const std::string & stamp_mode, const std::string & fallback)
{
	if (stamp_mode == "original" || stamp_mode == "now" || stamp_mode == "zero") {
		return stamp_mode;
	}
	return fallback;
}

geometry_msgs::msg::TransformStamped make_static_transform(
	const rclcpp::Time & stamp,
	const std::string & parent_frame_id,
	const std::string & child_frame_id,
	double x,
	double y,
	double z,
	double roll,
	double pitch,
	double yaw)
{
	geometry_msgs::msg::TransformStamped transform;
	transform.header.stamp = stamp;
	transform.header.frame_id = parent_frame_id;
	transform.child_frame_id = child_frame_id;
	transform.transform.translation.x = x;
	transform.transform.translation.y = y;
	transform.transform.translation.z = z;

	tf2::Quaternion rotation;
	rotation.setRPY(roll, pitch, yaw);
	rotation.normalize();
	transform.transform.rotation.x = rotation.x();
	transform.transform.rotation.y = rotation.y();
	transform.transform.rotation.z = rotation.z();
	transform.transform.rotation.w = rotation.w();
	return transform;
}
}

VinsVisualNode::VinsVisualNode()
: Node("vins_visual")
{
	const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
	const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
	const auto viz_odom_sub_qos = declare_parameter<bool>("odom_slam_subscribe_best_effort", true)
		? rclcpp::SensorDataQoS().keep_last(1)
		: reliable_qos;
	const auto viz_odom_pub_qos = declare_parameter<bool>("odom_slam_publish_best_effort", true)
		? rclcpp::SensorDataQoS().keep_last(1)
		: reliable_qos;
	relay_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	image_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	odom_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	point_cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
	static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

	namespaces_ = declare_parameter<std::vector<std::string>>("namespace", std::vector<std::string>{});
	const auto namespace_index = declare_parameter<int>("namespace_index", 0);
	selected_namespace_ = select_namespace(namespace_index);

	frame_remap_enabled_ = declare_parameter<bool>("frame_remap_enabled", false);
	input_frame_id_ = declare_parameter<std::string>("input_frame_id", "vodom");
	output_frame_id_ = declare_parameter<std::string>("output_frame_id", "vodom");
	tag_align_enabled_ = declare_parameter<bool>("tag_align_enabled", false);
	tag_frame_id_ = declare_parameter<std::string>("tag_frame_id", "base");
	tag_align_output_child_frame_id_ =
		declare_parameter<std::string>("tag_align_output_child_frame_id", "");
	tag_align_namespace_child_frame_ =
		declare_parameter<bool>("tag_align_namespace_child_frame", true);
	tag_align_compose_from_detection_ =
		declare_parameter<bool>("tag_align_compose_from_detection", false);
	tag_align_recalibrate_on_tag_update_ =
		declare_parameter<bool>("tag_align_recalibrate_on_tag_update", false);
	tag_global_to_tag_ = make_transform(
		declare_parameter<double>("tag_global_to_tag_x", 0.0),
		declare_parameter<double>("tag_global_to_tag_y", 0.0),
		declare_parameter<double>("tag_global_to_tag_z", 0.0),
		declare_parameter<double>("tag_global_to_tag_roll", 0.0),
		declare_parameter<double>("tag_global_to_tag_pitch", 0.0),
		declare_parameter<double>("tag_global_to_tag_yaw", 0.0));
	tag_edge_size_ = declare_parameter<double>("tag_edge_size", 0.3);
	tag_detection_id_ = declare_parameter<int>("tag_detection_id", 0);
	tag_optical_frame_id_ =
		declare_parameter<std::string>("tag_optical_frame_id", "camera_infra1_optical_frame");
	tag_detection_topic_ =
		declare_parameter<std::string>("tag_detection_topic", "/apriltag/detections");
	tag_camera_info_topic_ = declare_parameter<std::string>(
		"tag_camera_info_topic", "/camera/infra1/camera_info");
	compare_frame_id_ = declare_parameter<std::string>("compare_frame_id", "odom");
	if (tag_align_enabled_) {
		compare_frame_id_ = tag_frame_id_;
	}
	compare_path_max_poses_ = static_cast<std::size_t>(
		declare_parameter<int>("compare_path_max_poses", 2000));
	executor_threads_ = declare_parameter<int>("executor_threads", 6);
	image_publish_max_hz_ = declare_parameter<double>("image_publish_max_hz", 15.0);
	point_cloud_publish_max_hz_ = declare_parameter<double>("point_cloud_publish_max_hz", 5.0);
	path_publish_max_hz_ = declare_parameter<double>("path_publish_max_hz", 5.0);
	image_publish_decimation_ = declare_parameter<int>("image_publish_decimation", 1);
	if (image_publish_decimation_ < 1) {
		image_publish_decimation_ = 1;
	}
	point_cloud_publish_decimation_ = declare_parameter<int>("point_cloud_publish_decimation", 2);
	if (point_cloud_publish_decimation_ < 1) {
		point_cloud_publish_decimation_ = 1;
	}

	leg_odom_output_frame_id_ = declare_parameter<std::string>("leg_odom_output_frame_id", "odom");
	leg_odom_output_child_frame_id_ =
		declare_parameter<std::string>("leg_odom_output_child_frame_id", "base_link_leg");
	leg_odom_base_child_frame_id_ =
		declare_parameter<std::string>("leg_odom_base_child_frame_id", "base_link");
	slam_odom_output_child_frame_id_ =
		declare_parameter<std::string>("slam_odom_output_child_frame_id", "base_link");
	leg_odom_publish_tf_ = declare_parameter<bool>("leg_odom_publish_tf", true);
	leg_odom_publish_base_tf_ = declare_parameter<bool>("leg_odom_publish_base_tf", true);
	odom_slam_publish_tf_ = declare_parameter<bool>("odom_slam_publish_tf", true);
	odom_slam_align_enabled_ = declare_parameter<bool>("odom_slam_align_enabled", true);
	camera_static_tf_enabled_ = declare_parameter<bool>("camera_static_tf_enabled", true);
	camera_static_tf_parent_frame_id_ =
		declare_parameter<std::string>("camera_static_tf_parent_frame_id", "base_link");
	camera_static_tf_child_frame_id_ =
		declare_parameter<std::string>("camera_static_tf_child_frame_id", "camera_link");
	depth_points_output_frame_id_ =
		declare_parameter<std::string>("depth_points_output_frame_id", "");
	depth_points_stamp_mode_ = normalize_stamp_mode(
		declare_parameter<std::string>("depth_points_stamp_mode", "now"),
		"now");
	image_stamp_mode_ = normalize_stamp_mode(
		declare_parameter<std::string>("image_stamp_mode", "now"),
		"now");
	dynamic_tf_stamp_mode_ = normalize_stamp_mode(
		declare_parameter<std::string>("dynamic_tf_stamp_mode", "now"),
		"now");
	leg_path_topic_ = declare_parameter<std::string>("compare_leg_path_topic", "/compare/leg_path");
	slam_path_topic_ = declare_parameter<std::string>("compare_slam_path_topic", "/compare/slam_path");

	leg_compare_path_.header.frame_id = compare_frame_id_;
	slam_compare_path_.header.frame_id = compare_frame_id_;

	rclcpp::SubscriptionOptions subscription_options;
	subscription_options.callback_group = relay_callback_group_;

	rclcpp::SubscriptionOptions image_subscription_options;
	image_subscription_options.callback_group = image_callback_group_;

	rclcpp::SubscriptionOptions odom_subscription_options;
	odom_subscription_options.callback_group = odom_callback_group_;

	rclcpp::SubscriptionOptions point_cloud_subscription_options;
	point_cloud_subscription_options.callback_group = point_cloud_callback_group_;

	create_image_relay(
		declare_parameter<bool>("image0_enabled", true),
		"image0",
		build_input_topic(declare_parameter<std::string>("image0_input_topic", "/vins/image0")),
		declare_parameter<std::string>("image0_output_topic", "/vins/image0"),
		sensor_qos,
		sensor_qos,
		image_subscription_options);

	create_image_relay(
		declare_parameter<bool>("image1_enabled", true),
		"image1",
		build_input_topic(declare_parameter<std::string>("image1_input_topic", "/vins/image1")),
		declare_parameter<std::string>("image1_output_topic", "/vins/image1"),
		sensor_qos,
		sensor_qos,
		image_subscription_options);

	create_image_relay(
		declare_parameter<bool>("image2_enabled", true),
		"image2",
		build_input_topic(declare_parameter<std::string>("image2_input_topic", "/vins/image2")),
		declare_parameter<std::string>("image2_output_topic", "/vins/image2"),
		sensor_qos,
		sensor_qos,
		image_subscription_options);

	create_point_cloud_relay(
		declare_parameter<bool>("depth_points_enabled", true),
		"depth_points",
		build_input_topic(
			declare_parameter<std::string>(
				"depth_points_input_topic", "/camera/depth/color/points")),
		declare_parameter<std::string>(
			"depth_points_output_topic", "/camera/depth/color/points"),
		sensor_qos,
		sensor_qos,
		point_cloud_subscription_options,
		frame_remap_enabled_,
		depth_points_output_frame_id_);

	create_leg_odom_compare_relay(viz_odom_sub_qos, viz_odom_pub_qos, odom_subscription_options);
	create_odom_slam_compare_relay(viz_odom_sub_qos, viz_odom_pub_qos, odom_subscription_options);

	if (tag_align_enabled_) {
		create_tag_align_subscribers(subscription_options);
	}

	create_odom_relay(
		declare_parameter<bool>("odometry_enabled", false),
		"odometry",
		build_input_topic(declare_parameter<std::string>("odometry_input_topic", "/vins/odometry")),
		declare_parameter<std::string>("odometry_output_topic", "/vins/odometry"),
		reliable_qos,
		reliable_qos,
		subscription_options,
		frame_remap_enabled_);

	create_odom_relay(
		declare_parameter<bool>("imuodom_slam_enabled", false),
		"imuodom_slam",
		build_input_topic(declare_parameter<std::string>("imuodom_slam_input_topic", "/vins/imuodom_slam")),
		declare_parameter<std::string>("imuodom_slam_output_topic", "/vins/imuodom_slam"),
		reliable_qos,
		reliable_qos,
		subscription_options,
		frame_remap_enabled_);

	create_odom_relay(
		declare_parameter<bool>("odometryleft_enabled", false),
		"odometryleft",
		build_input_topic(declare_parameter<std::string>("odometryleft_input_topic", "/vins/odometryleft")),
		declare_parameter<std::string>("odometryleft_output_topic", "/vins/odometryleft"),
		reliable_qos,
		reliable_qos,
		subscription_options,
		frame_remap_enabled_);

	create_odom_relay(
		declare_parameter<bool>("odometryright_enabled", false),
		"odometryright",
		build_input_topic(declare_parameter<std::string>("odometryright_input_topic", "/vins/odometryright")),
		declare_parameter<std::string>("odometryright_output_topic", "/vins/odometryright"),
		reliable_qos,
		reliable_qos,
		subscription_options,
		frame_remap_enabled_);

	create_odom_relay(
		declare_parameter<bool>("reloc_odom_enabled", false),
		"reloc_odom",
		build_input_topic(declare_parameter<std::string>("reloc_odom_input_topic", "/vins/reloc_odom")),
		declare_parameter<std::string>("reloc_odom_output_topic", "/vins/reloc_odom"),
		reliable_qos,
		reliable_qos,
		subscription_options,
		false);

	create_path_relay(
		declare_parameter<bool>("path_enabled", false),
		"path",
		build_input_topic(declare_parameter<std::string>("path_input_topic", "/vins/path")),
		declare_parameter<std::string>("path_output_topic", "/vins/path"),
		reliable_qos,
		reliable_qos,
		subscription_options);

	create_path_relay(
		declare_parameter<bool>("pathleft_enabled", false),
		"pathleft",
		build_input_topic(declare_parameter<std::string>("pathleft_input_topic", "/vins/pathleft")),
		declare_parameter<std::string>("pathleft_output_topic", "/vins/pathleft"),
		reliable_qos,
		reliable_qos,
		subscription_options);

	create_path_relay(
		declare_parameter<bool>("pathright_enabled", false),
		"pathright",
		build_input_topic(declare_parameter<std::string>("pathright_input_topic", "/vins/pathright")),
		declare_parameter<std::string>("pathright_output_topic", "/vins/pathright"),
		reliable_qos,
		reliable_qos,
		subscription_options);

	create_lifecycle_monitor(
		declare_parameter<bool>("vinsfollowing_transition_enabled", true),
		"vinsfollowing_transition",
		build_input_topic(declare_parameter<std::string>(
			"vinsfollowing_transition_input_topic", "/vinsfollowing/transition_event")),
		subscription_options);

	create_lifecycle_monitor(
		declare_parameter<bool>("vinslocalization_transition_enabled", true),
		"vinslocalization_transition",
		build_input_topic(declare_parameter<std::string>(
			"vinslocalization_transition_input_topic", "/vinslocalization/transition_event")),
		subscription_options);

	create_lifecycle_monitor(
		declare_parameter<bool>("vinsmapping_transition_enabled", true),
		"vinsmapping_transition",
		build_input_topic(declare_parameter<std::string>(
			"vinsmapping_transition_input_topic", "/vinsmapping/transition_event")),
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

	if (frame_remap_enabled_) {
		RCLCPP_INFO(
			get_logger(),
			"frame remap enabled: %s -> %s",
			input_frame_id_.c_str(),
			output_frame_id_.c_str());
	}

	if (tag_align_enabled_) {
		RCLCPP_INFO(
			get_logger(),
			"tag_align compose mode: optical=%s, detection_topic=%s",
			tag_optical_frame_id_.c_str(),
			tag_detection_topic_.c_str());
	}

	publish_camera_static_tf();

	if (tag_align_enabled_) {
		update_base_link_to_optical_transform();
	}

	RCLCPP_INFO(
		get_logger(),
		"vins_visual is ready with %zu relay channels, compare frame=%s, odom_slam_align=%s, tag_align=%s",
		relay_specs_.size(),
		compare_frame_id_.c_str(),
		odom_slam_align_enabled_ ? "true" : "false",
		tag_align_enabled_ ? tag_frame_id_.c_str() : "disabled");
}

VinsVisualNode::~VinsVisualNode()
{
	shutting_down_.store(true);
	if (status_timer_) {
		status_timer_->cancel();
		status_timer_.reset();
	}
}

std::string VinsVisualNode::select_namespace(int namespace_index)
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

std::string VinsVisualNode::trim_slashes(const std::string & value) const
{
	auto start = value.find_first_not_of('/');
	if (start == std::string::npos) {
		return "";
	}
	auto end = value.find_last_not_of('/');
	return value.substr(start, end - start + 1);
}

std::string VinsVisualNode::build_input_topic(const std::string & base_topic) const
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

std::string VinsVisualNode::remap_frame_id(const std::string & frame_id) const
{
	if (!frame_remap_enabled_ || frame_id != input_frame_id_) {
		return frame_id;
	}
	return output_frame_id_;
}

void VinsVisualNode::publish_camera_static_tf()
{
	if (!camera_static_tf_enabled_ || !static_tf_broadcaster_) {
		return;
	}

	if (camera_static_tf_parent_frame_id_.empty() || camera_static_tf_child_frame_id_.empty()) {
		RCLCPP_WARN(get_logger(), "skip camera static TF because frame id is empty");
		return;
	}

	const auto stamp = now();
	std::vector<geometry_msgs::msg::TransformStamped> transforms;
	transforms.push_back(make_static_transform(
		stamp,
		camera_static_tf_parent_frame_id_,
		camera_static_tf_child_frame_id_,
		declare_parameter<double>("camera_static_tf_x", 0.26932),
		declare_parameter<double>("camera_static_tf_y", 0.0),
		declare_parameter<double>("camera_static_tf_z", 0.11543),
		declare_parameter<double>("camera_static_tf_roll", 0.0),
		declare_parameter<double>("camera_static_tf_pitch", 0.19),
		declare_parameter<double>("camera_static_tf_yaw", 0.0)));

	const auto sensor_static_enabled =
		declare_parameter<bool>("camera_sensor_static_tf_enabled", true);
	const auto minimal_for_tag_align =
		tag_align_enabled_ &&
		declare_parameter<bool>("camera_static_tf_minimal_for_tag_align", true);

	if (sensor_static_enabled && !minimal_for_tag_align) {
		const std::vector<std::pair<std::string, std::string>> sensor_frames = {
			{"camera_depth_frame", "camera_depth_optical_frame"},
			{"camera_infra1_frame", "camera_infra1_optical_frame"},
			{"camera_infra2_frame", "camera_infra2_optical_frame"},
			{"camera_accel_frame", "camera_accel_optical_frame"},
			{"camera_gyro_frame", "camera_gyro_optical_frame"},
		};

		for (const auto & [sensor_frame, optical_frame] : sensor_frames) {
			transforms.push_back(make_static_transform(
				stamp,
				camera_static_tf_child_frame_id_,
				sensor_frame,
				0.0,
				0.0,
				0.0,
				0.0,
				0.0,
				0.0));
			transforms.push_back(make_static_transform(
				stamp,
				sensor_frame,
				optical_frame,
				0.0,
				0.0,
				0.0,
				kOpticalFrameRoll,
				kOpticalFramePitch,
				kOpticalFrameYaw));
		}
	}

	transforms.push_back(make_static_transform(
		stamp,
		camera_static_tf_child_frame_id_,
		"camera_aligned_depth_to_infra1_frame",
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0));

	if (minimal_for_tag_align) {
		transforms.push_back(make_static_transform(
			stamp,
			"camera_aligned_depth_to_infra1_frame",
			tag_optical_frame_id_.empty() ? "camera_infra1_optical_frame" : tag_optical_frame_id_,
			0.0,
			0.0,
			0.0,
			kOpticalFrameRoll,
			kOpticalFramePitch,
			kOpticalFrameYaw));
	}

	static_tf_broadcaster_->sendTransform(transforms);
	RCLCPP_INFO(
		get_logger(),
		"camera static TF published: %s -> %s (%zu transforms, minimal_for_tag_align=%s)",
		camera_static_tf_parent_frame_id_.c_str(),
		camera_static_tf_child_frame_id_.c_str(),
		transforms.size(),
		minimal_for_tag_align ? "true" : "false");
}

tf2::Transform VinsVisualNode::odometry_to_transform(const nav_msgs::msg::Odometry & odom)
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
	const auto norm = rotation.length2();
	if (norm <= kQuaternionNormEpsilon) {
		rotation.setValue(0.0, 0.0, 0.0, 1.0);
	} else {
		rotation.normalize();
	}
	transform.setRotation(rotation);
	return transform;
}

nav_msgs::msg::Odometry VinsVisualNode::transform_to_odometry(
	const tf2::Transform & transform,
	const std_msgs::msg::Header & header,
	const std::string & child_frame_id)
{
	nav_msgs::msg::Odometry odom;
	odom.header = header;
	odom.child_frame_id = child_frame_id;
	odom.pose.pose.position.x = transform.getOrigin().x();
	odom.pose.pose.position.y = transform.getOrigin().y();
	odom.pose.pose.position.z = transform.getOrigin().z();
	const auto & rotation = transform.getRotation();
	odom.pose.pose.orientation.x = rotation.x();
	odom.pose.pose.orientation.y = rotation.y();
	odom.pose.pose.orientation.z = rotation.z();
	odom.pose.pose.orientation.w = rotation.w();
	return odom;
}

geometry_msgs::msg::PoseStamped VinsVisualNode::transform_to_pose_stamped(
	const tf2::Transform & transform,
	const std_msgs::msg::Header & header,
	const std::string & frame_id)
{
	geometry_msgs::msg::PoseStamped pose;
	pose.header = header;
	pose.header.frame_id = frame_id;
	pose.pose.position.x = transform.getOrigin().x();
	pose.pose.position.y = transform.getOrigin().y();
	pose.pose.position.z = transform.getOrigin().z();
	const auto & rotation = transform.getRotation();
	pose.pose.orientation.x = rotation.x();
	pose.pose.orientation.y = rotation.y();
	pose.pose.orientation.z = rotation.z();
	pose.pose.orientation.w = rotation.w();
	return pose;
}

void VinsVisualNode::append_path_pose(nav_msgs::msg::Path & path, geometry_msgs::msg::PoseStamped pose)
{
	path.header.stamp = pose.header.stamp;
	path.header.frame_id = pose.header.frame_id;
	path.poses.push_back(std::move(pose));
	if (path.poses.size() > compare_path_max_poses_) {
		const auto overflow = path.poses.size() - compare_path_max_poses_;
		path.poses.erase(path.poses.begin(), path.poses.begin() + static_cast<std::ptrdiff_t>(overflow));
	}
}

void VinsVisualNode::apply_header_stamp_mode(
	std_msgs::msg::Header & header,
	const std::string & stamp_mode) const
{
	if (stamp_mode == "now") {
		header.stamp = now();
		return;
	}

	if (stamp_mode == "zero") {
		header.stamp.sec = 0;
		header.stamp.nanosec = 0;
	}
}

bool VinsVisualNode::should_publish_at_max_hz(
	double max_hz,
	rclcpp::Time & last_publish_time)
{
	if (max_hz <= 0.0) {
		return true;
	}

	const auto current_time = now();
	if (last_publish_time.nanoseconds() == 0) {
		last_publish_time = current_time;
		return true;
	}

	const auto min_interval = 1.0 / max_hz;
	if ((current_time - last_publish_time).seconds() >= min_interval) {
		last_publish_time = current_time;
		return true;
	}

	return false;
}

bool VinsVisualNode::should_publish_decimated(
	int decimation,
	std::uint64_t & counter)
{
	if (decimation <= 1) {
		return true;
	}

	++counter;
	return counter % static_cast<std::uint64_t>(decimation) == 0U;
}

void VinsVisualNode::publish_odometry_tf(const nav_msgs::msg::Odometry & odom)
{
	if (!tf_broadcaster_ || odom.header.frame_id.empty() || odom.child_frame_id.empty()) {
		return;
	}

	geometry_msgs::msg::TransformStamped transform;
	transform.header = odom.header;
	apply_header_stamp_mode(transform.header, dynamic_tf_stamp_mode_);
	if (transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) {
		transform.header.stamp = now();
	}
	transform.child_frame_id = odom.child_frame_id;
	transform.transform.translation.x = odom.pose.pose.position.x;
	transform.transform.translation.y = odom.pose.pose.position.y;
	transform.transform.translation.z = odom.pose.pose.position.z;
	transform.transform.rotation = odom.pose.pose.orientation;
	tf_broadcaster_->sendTransform(transform);
}

void VinsVisualNode::try_initialize_alignment(
	const tf2::Transform & leg_transform,
	const tf2::Transform & slam_transform)
{
	if (alignment_initialized_) {
		return;
	}

	odom_to_slam_origin_ = leg_transform * slam_transform.inverse();
	alignment_initialized_ = true;
	RCLCPP_INFO(
		get_logger(),
		"odom_slam aligned to %s at origin, translation=[%.3f, %.3f, %.3f]",
		compare_frame_id_.c_str(),
		odom_to_slam_origin_.getOrigin().x(),
		odom_to_slam_origin_.getOrigin().y(),
		odom_to_slam_origin_.getOrigin().z());
}

bool VinsVisualNode::apply_slam_alignment(
	const nav_msgs::msg::Odometry & input,
	nav_msgs::msg::Odometry & output)
{
	if (tag_align_enabled_) {
		return apply_tag_alignment(input, output);
	}

	const auto slam_transform = odometry_to_transform(input);

	{
		std::lock_guard<std::mutex> lock(compare_mutex_);
		if (!alignment_initialized_) {
			if (leg_odom_received_ && last_leg_transform_.has_value()) {
				try_initialize_alignment(last_leg_transform_.value(), slam_transform);
			} else {
				pending_slam_transform_ = slam_transform;
				return false;
			}
		}
	}

	if (!odom_slam_align_enabled_) {
		output = input;
		output.header.frame_id = compare_frame_id_;
		output.child_frame_id = slam_odom_output_child_frame_id_;
		return true;
	}

	const auto aligned_transform = odom_to_slam_origin_ * slam_transform;
	output = transform_to_odometry(
		aligned_transform,
		input.header,
		slam_odom_output_child_frame_id_);
	output.header.frame_id = compare_frame_id_;
	return true;
}

std::string VinsVisualNode::resolve_tag_align_child_frame_id(
	const std::string & input_child_frame_id) const
{
	if (!tag_align_output_child_frame_id_.empty()) {
		return tag_align_output_child_frame_id_;
	}

	const auto & child_frame_id = slam_odom_output_child_frame_id_.empty()
		? input_child_frame_id
		: slam_odom_output_child_frame_id_;

	if (tag_align_namespace_child_frame_ && !selected_namespace_.empty()) {
		const auto normalized_namespace = trim_slashes(selected_namespace_);
		if (!normalized_namespace.empty()) {
			return normalized_namespace + "/" + child_frame_id;
		}
	}

	return child_frame_id;
}

bool VinsVisualNode::apply_tag_alignment(
	const nav_msgs::msg::Odometry & input,
	nav_msgs::msg::Odometry & output)
{
	if (input.child_frame_id.empty()) {
		RCLCPP_WARN_THROTTLE(
			get_logger(),
			*get_clock(),
			5000,
			"skip tag_align because odom child_frame_id is empty");
		return false;
	}

	tf2::Transform tag_optical_to_tag;
	tf2::Transform base_link_to_optical;
	std::uint64_t tag_pose_sequence = 0;
	{
		std::lock_guard<std::mutex> lock(tag_pose_mutex_);
		if (!tag_pose_valid_) {
			RCLCPP_WARN_THROTTLE(
				get_logger(),
				*get_clock(),
				5000,
				"skip tag_align because tag pose is unavailable (need visible tag on %s)",
				tag_detection_topic_.c_str());
			return false;
		}
		tag_optical_to_tag = tag_optical_to_tag_;
		base_link_to_optical = base_link_to_optical_;
		tag_pose_sequence = tag_pose_sequence_;
	}

	const auto vio_to_base_link = odometry_to_transform(input);
	const auto observed_tag_to_base_link =
		tag_global_to_tag_ * tag_optical_to_tag.inverse() * base_link_to_optical.inverse();

	tf2::Transform tag_to_vio_frame;
	{
		std::lock_guard<std::mutex> lock(tag_pose_mutex_);
		if (!tag_vio_alignment_valid_ ||
			(tag_align_recalibrate_on_tag_update_ &&
			consumed_tag_pose_sequence_ != tag_pose_sequence))
		{
			tag_to_vio_frame_ = observed_tag_to_base_link * vio_to_base_link.inverse();
			consumed_tag_pose_sequence_ = tag_pose_sequence;
			tag_vio_alignment_valid_ = true;
			RCLCPP_INFO(
				get_logger(),
				"tag_align calibrated %s -> %s from tag observation, translation=[%.3f, %.3f, %.3f]",
				input.header.frame_id.c_str(),
				tag_frame_id_.c_str(),
				tag_to_vio_frame_.getOrigin().x(),
				tag_to_vio_frame_.getOrigin().y(),
				tag_to_vio_frame_.getOrigin().z());
		}
		tag_to_vio_frame = tag_to_vio_frame_;
	}

	const auto aligned_tag_to_base_link = tag_to_vio_frame * vio_to_base_link;
	output = transform_to_odometry(
		aligned_tag_to_base_link,
		input.header,
		resolve_tag_align_child_frame_id(input.child_frame_id));
	output.header.frame_id = tag_frame_id_;
	apply_header_stamp_mode(output.header, dynamic_tf_stamp_mode_);
	if (output.header.stamp.sec == 0 && output.header.stamp.nanosec == 0) {
		output.header.stamp = now();
	}
	return true;
}

tf2::Transform VinsVisualNode::make_transform(
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

void VinsVisualNode::update_base_link_to_optical_transform()
{
	const auto base_to_camera = make_transform(
		get_parameter("camera_static_tf_x").as_double(),
		get_parameter("camera_static_tf_y").as_double(),
		get_parameter("camera_static_tf_z").as_double(),
		get_parameter("camera_static_tf_roll").as_double(),
		get_parameter("camera_static_tf_pitch").as_double(),
		get_parameter("camera_static_tf_yaw").as_double());
	const auto camera_to_aligned = make_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	const auto aligned_to_optical = make_transform(
		0.0,
		0.0,
		0.0,
		kOpticalFrameRoll,
		kOpticalFramePitch,
		kOpticalFrameYaw);

	std::lock_guard<std::mutex> lock(tag_pose_mutex_);
	base_link_to_optical_ = base_to_camera * camera_to_aligned * aligned_to_optical;
}

bool VinsVisualNode::update_tag_optical_to_tag_from_detection(
	const apriltag_msgs::msg::AprilTagDetectionArray & detections)
{
	if (!camera_intrinsics_valid_) {
		return false;
	}

	const apriltag_msgs::msg::AprilTagDetection * selected_detection = nullptr;
	for (const auto & detection : detections.detections) {
		if (detection.id == tag_detection_id_) {
			selected_detection = &detection;
			break;
		}
	}

	if (selected_detection == nullptr) {
		return false;
	}

	const std::vector<cv::Point3d> object_points{
		{-tag_edge_size_ / 2, -tag_edge_size_ / 2, 0},
		{+tag_edge_size_ / 2, -tag_edge_size_ / 2, 0},
		{+tag_edge_size_ / 2, +tag_edge_size_ / 2, 0},
		{-tag_edge_size_ / 2, +tag_edge_size_ / 2, 0},
	};
	const std::vector<cv::Point2d> image_points{
		{selected_detection->corners[0].x, selected_detection->corners[0].y},
		{selected_detection->corners[1].x, selected_detection->corners[1].y},
		{selected_detection->corners[2].x, selected_detection->corners[2].y},
		{selected_detection->corners[3].x, selected_detection->corners[3].y},
	};

	cv::Matx33d camera_matrix = cv::Matx33d::eye();
	camera_matrix(0, 0) = camera_intrinsics_[0];
	camera_matrix(1, 1) = camera_intrinsics_[1];
	camera_matrix(0, 2) = camera_intrinsics_[2];
	camera_matrix(1, 2) = camera_intrinsics_[3];

	cv::Mat rvec;
	cv::Mat tvec;
	if (!cv::solvePnP(object_points, image_points, camera_matrix, {}, rvec, tvec)) {
		return false;
	}

	cv::Mat rotation_matrix;
	cv::Rodrigues(rvec, rotation_matrix);

	tf2::Matrix3x3 rotation(
		rotation_matrix.at<double>(0, 0),
		rotation_matrix.at<double>(0, 1),
		rotation_matrix.at<double>(0, 2),
		rotation_matrix.at<double>(1, 0),
		rotation_matrix.at<double>(1, 1),
		rotation_matrix.at<double>(1, 2),
		rotation_matrix.at<double>(2, 0),
		rotation_matrix.at<double>(2, 1),
		rotation_matrix.at<double>(2, 2));
	tf2::Quaternion quaternion;
	rotation.getRotation(quaternion);
	quaternion.normalize();

	tf2::Transform optical_to_tag;
	optical_to_tag.setOrigin(tf2::Vector3(
		tvec.at<double>(0),
		tvec.at<double>(1),
		tvec.at<double>(2)));
	optical_to_tag.setRotation(quaternion);

	std::lock_guard<std::mutex> lock(tag_pose_mutex_);
	tag_optical_to_tag_ = optical_to_tag;
	tag_pose_valid_ = true;
	++tag_pose_sequence_;
	return true;
}

void VinsVisualNode::on_tag_tf_message(const tf2_msgs::msg::TFMessage::SharedPtr message)
{
	if (!message) {
		return;
	}

	for (const auto & transform : message->transforms) {
		if (transform.header.frame_id != tag_optical_frame_id_ ||
			transform.child_frame_id != tag_frame_id_)
		{
			continue;
		}

		tf2::Transform optical_to_tag;
		tf2::convert(transform.transform, optical_to_tag);
		std::lock_guard<std::mutex> lock(tag_pose_mutex_);
		tag_optical_to_tag_ = optical_to_tag;
		tag_pose_valid_ = true;
		++tag_pose_sequence_;
	}
}

void VinsVisualNode::create_tag_align_subscribers(
	const rclcpp::SubscriptionOptions & subscription_options)
{
	const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
	std::string camera_info_topic;

	if (tag_align_compose_from_detection_) {
		auto detection_subscription = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
			tag_detection_topic_,
			reliable_qos,
			[this](const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr message) {
				if (shutting_down_.load() || !message) {
					return;
				}
				if (update_tag_optical_to_tag_from_detection(*message)) {
					update_relay_status("tag_detection");
				}
			},
			subscription_options);
		subscriptions_.push_back(detection_subscription);
		relay_specs_.push_back(RelaySpec{"tag_detection", tag_detection_topic_, ""});

		camera_info_topic = build_input_topic(tag_camera_info_topic_);
		auto camera_info_subscription = create_subscription<sensor_msgs::msg::CameraInfo>(
			camera_info_topic,
			reliable_qos,
			[this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
				if (shutting_down_.load() || !message) {
					return;
				}
				if (message->width == 0 || message->height == 0 ||
					message->p[0] == 0.0 || message->p[5] == 0.0)
				{
					return;
				}
				std::lock_guard<std::mutex> lock(tag_pose_mutex_);
				camera_intrinsics_[0] = message->p[0];
				camera_intrinsics_[1] = message->p[5];
				camera_intrinsics_[2] = message->p[2];
				camera_intrinsics_[3] = message->p[6];
				camera_intrinsics_valid_ = true;
			},
			subscription_options);
		subscriptions_.push_back(camera_info_subscription);
		relay_specs_.push_back(RelaySpec{"tag_camera_info", camera_info_topic, ""});
	}

	auto tf_subscription = create_subscription<tf2_msgs::msg::TFMessage>(
		"/tf",
		rclcpp::QoS(10),
		[this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
			on_tag_tf_message(message);
		},
		subscription_options);
	subscriptions_.push_back(tf_subscription);
	relay_specs_.push_back(RelaySpec{"tag_tf", "/tf", ""});

	RCLCPP_INFO(
		get_logger(),
		"tag_align subscribers ready: tf=/tf, compose_from_detection=%s, detection=%s, camera_info=%s",
		tag_align_compose_from_detection_ ? "true" : "false",
		tag_detection_topic_.c_str(),
		camera_info_topic.c_str());
}

void VinsVisualNode::create_leg_odom_compare_relay(
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
	const rclcpp::SubscriptionOptions & subscription_options)
{
	if (!declare_parameter<bool>("leg_odom_enabled", true)) {
		RCLCPP_INFO(get_logger(), "skip disabled relay: leg_odom");
		return;
	}

	const auto input_topic = build_input_topic(
		declare_parameter<std::string>("leg_odom_input_topic", "/odom_out"));
	const auto output_topic = declare_parameter<std::string>("leg_odom_output_topic", "/odom");

	if (input_topic.empty() || output_topic.empty()) {
		RCLCPP_WARN(get_logger(), "skip leg_odom relay because topic name is empty");
		return;
	}

	leg_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, publish_qos);
	leg_path_publisher_ = create_publisher<nav_msgs::msg::Path>(leg_path_topic_, publish_qos);
	publishers_.push_back(leg_odom_publisher_);
	publishers_.push_back(leg_path_publisher_);

	auto subscription = create_subscription<nav_msgs::msg::Odometry>(
		input_topic,
		subscribe_qos,
		[this](nav_msgs::msg::Odometry::SharedPtr message) {
			if (shutting_down_.load() || !message || !leg_odom_publisher_) {
				return;
			}
			try {
				auto output = *message;
				output.header.frame_id = leg_odom_output_frame_id_;
				output.child_frame_id = leg_odom_output_child_frame_id_;
				leg_odom_publisher_->publish(output);
				update_relay_status("leg_odom");

				const auto leg_transform = odometry_to_transform(output);
				{
					std::lock_guard<std::mutex> lock(compare_mutex_);
					leg_odom_received_ = true;
					last_leg_transform_ = leg_transform;
					if (!alignment_initialized_ && pending_slam_transform_.has_value()) {
						try_initialize_alignment(leg_transform, pending_slam_transform_.value());
						pending_slam_transform_.reset();
					}
				}

				if (leg_path_publisher_) {
					auto pose = transform_to_pose_stamped(
						leg_transform,
						output.header,
						compare_frame_id_);
					append_path_pose(leg_compare_path_, std::move(pose));
					if (should_publish_at_max_hz(path_publish_max_hz_, last_leg_path_publish_time_)) {
						leg_path_publisher_->publish(leg_compare_path_);
						update_relay_status("leg_path");
					}
				}

				if (leg_odom_publish_tf_) {
					publish_odometry_tf(output);
				}

				if (leg_odom_publish_base_tf_) {
					auto base_output = output;
					base_output.child_frame_id = leg_odom_base_child_frame_id_;
					publish_odometry_tf(base_output);
				}
			} catch (const std::exception & exception) {
				RCLCPP_ERROR_THROTTLE(
					get_logger(),
					*get_clock(),
					5000,
					"relay leg_odom callback failed: %s",
					exception.what());
			}
		},
		subscription_options);

	subscriptions_.push_back(subscription);
	relay_specs_.push_back(RelaySpec{"leg_odom", input_topic, output_topic});
	relay_specs_.push_back(RelaySpec{"leg_path", input_topic, leg_path_topic_});
}

void VinsVisualNode::create_odom_slam_compare_relay(
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
	const rclcpp::SubscriptionOptions & subscription_options)
{
	if (!declare_parameter<bool>("odom_slam_enabled", true)) {
		RCLCPP_INFO(get_logger(), "skip disabled relay: odom_slam");
		return;
	}

	const auto input_topic = build_input_topic(
		declare_parameter<std::string>("odom_slam_input_topic", "/odom_slam"));
	const auto output_topic = declare_parameter<std::string>("odom_slam_output_topic", "/odom_slam");

	if (input_topic.empty() || output_topic.empty()) {
		RCLCPP_WARN(get_logger(), "skip odom_slam relay because topic name is empty");
		return;
	}

	slam_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, publish_qos);
	slam_path_publisher_ = create_publisher<nav_msgs::msg::Path>(slam_path_topic_, publish_qos);
	publishers_.push_back(slam_odom_publisher_);
	publishers_.push_back(slam_path_publisher_);

	auto subscription = create_subscription<nav_msgs::msg::Odometry>(
		input_topic,
		subscribe_qos,
		[this](nav_msgs::msg::Odometry::SharedPtr message) {
			if (shutting_down_.load() || !message || !slam_odom_publisher_) {
				return;
			}
			try {
				nav_msgs::msg::Odometry aligned_output;
				if (!apply_slam_alignment(*message, aligned_output)) {
					return;
				}

				slam_odom_publisher_->publish(aligned_output);
				update_relay_status("odom_slam");

				if (odom_slam_publish_tf_) {
					publish_odometry_tf(aligned_output);
				}

				if (slam_path_publisher_) {
					const auto aligned_transform = odometry_to_transform(aligned_output);
					auto pose = transform_to_pose_stamped(
						aligned_transform,
						aligned_output.header,
						compare_frame_id_);
					append_path_pose(slam_compare_path_, std::move(pose));
					if (should_publish_at_max_hz(path_publish_max_hz_, last_slam_path_publish_time_)) {
						slam_path_publisher_->publish(slam_compare_path_);
						update_relay_status("slam_path");
					}
				}
			} catch (const std::exception & exception) {
				RCLCPP_ERROR_THROTTLE(
					get_logger(),
					*get_clock(),
					5000,
					"relay odom_slam callback failed: %s",
					exception.what());
			}
		},
		subscription_options);

	subscriptions_.push_back(subscription);
	relay_specs_.push_back(RelaySpec{"odom_slam", input_topic, output_topic});
	relay_specs_.push_back(RelaySpec{"slam_path", input_topic, slam_path_topic_});
}

void VinsVisualNode::create_image_relay(
	bool enabled,
	const std::string & relay_name,
	const std::string & input_topic,
	const std::string & output_topic,
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
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

	auto publisher = create_publisher<sensor_msgs::msg::Image>(output_topic, publish_qos);
	auto subscription = create_subscription<sensor_msgs::msg::Image>(
		input_topic,
		subscribe_qos,
		[this, publisher, relay_name](sensor_msgs::msg::Image::SharedPtr message) {
			if (shutting_down_.load() || !message || !publisher) {
				return;
			}
			try {
				auto & image_counter = image_publish_counters_[relay_name];
				if (!should_publish_decimated(image_publish_decimation_, image_counter)) {
					return;
				}

				auto & last_publish_time = last_image_publish_times_[relay_name];
				if (!should_publish_at_max_hz(image_publish_max_hz_, last_publish_time)) {
					return;
				}

				if (image_stamp_mode_ == "original") {
					publisher->publish(*message);
				} else {
					auto output = std::make_unique<sensor_msgs::msg::Image>();
					output->header = message->header;
					output->height = message->height;
					output->width = message->width;
					output->encoding = message->encoding;
					output->is_bigendian = message->is_bigendian;
					output->step = message->step;
					output->data = message->data;
					apply_header_stamp_mode(output->header, image_stamp_mode_);
					publisher->publish(std::move(output));
				}
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

void VinsVisualNode::create_point_cloud_relay(
	bool enabled,
	const std::string & relay_name,
	const std::string & input_topic,
	const std::string & output_topic,
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
	const rclcpp::SubscriptionOptions & subscription_options,
	bool remap_frame,
	const std::string & output_frame_id)
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

	auto publisher = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, publish_qos);
	auto subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
		input_topic,
		subscribe_qos,
		[this, publisher, relay_name, remap_frame, output_frame_id](
			sensor_msgs::msg::PointCloud2::SharedPtr message) {
			if (shutting_down_.load() || !message || !publisher) {
				return;
			}
			try {
				if (!should_publish_decimated(
						point_cloud_publish_decimation_,
						point_cloud_publish_counter_) ||
					!should_publish_at_max_hz(
						point_cloud_publish_max_hz_,
						last_point_cloud_publish_time_))
				{
					return;
				}

				const bool needs_copy = remap_frame ||
					!output_frame_id.empty() ||
					depth_points_stamp_mode_ != "original";
				if (!needs_copy) {
					publisher->publish(*message);
				} else {
					auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
					*output = *message;
					if (remap_frame) {
						output->header.frame_id = remap_frame_id(output->header.frame_id);
					}
					if (!output_frame_id.empty()) {
						output->header.frame_id = output_frame_id;
					}
					apply_header_stamp_mode(output->header, depth_points_stamp_mode_);
					publisher->publish(std::move(output));
				}
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

void VinsVisualNode::create_path_relay(
	bool enabled,
	const std::string & relay_name,
	const std::string & input_topic,
	const std::string & output_topic,
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
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

	auto publisher = create_publisher<nav_msgs::msg::Path>(output_topic, publish_qos);
	auto subscription = create_subscription<nav_msgs::msg::Path>(
		input_topic,
		subscribe_qos,
		[this, publisher, relay_name](nav_msgs::msg::Path::SharedPtr message) {
			if (shutting_down_.load() || !message || !publisher) {
				return;
			}
			try {
				auto output = *message;
				output.header.frame_id = remap_frame_id(output.header.frame_id);
				for (auto & pose : output.poses) {
					pose.header.frame_id = remap_frame_id(pose.header.frame_id);
				}
				publisher->publish(output);
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

void VinsVisualNode::create_odom_relay(
	bool enabled,
	const std::string & relay_name,
	const std::string & input_topic,
	const std::string & output_topic,
	const rclcpp::QoS & subscribe_qos,
	const rclcpp::QoS & publish_qos,
	const rclcpp::SubscriptionOptions & subscription_options,
	bool remap_frame,
	const std::string & output_frame_id,
	const std::string & output_child_frame_id,
	bool publish_tf)
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

	auto publisher = create_publisher<nav_msgs::msg::Odometry>(output_topic, publish_qos);
	auto subscription = create_subscription<nav_msgs::msg::Odometry>(
		input_topic,
		subscribe_qos,
		[this, publisher, relay_name, remap_frame, output_frame_id, output_child_frame_id, publish_tf](
			nav_msgs::msg::Odometry::SharedPtr message) {
			if (shutting_down_.load() || !message || !publisher) {
				return;
			}
			try {
				auto output = *message;
				if (remap_frame) {
					output.header.frame_id = remap_frame_id(output.header.frame_id);
				}
				if (!output_frame_id.empty()) {
					output.header.frame_id = output_frame_id;
				}
				if (!output_child_frame_id.empty()) {
					output.child_frame_id = output_child_frame_id;
				}
				publisher->publish(output);
				update_relay_status(relay_name);

				if (publish_tf && tf_broadcaster_ &&
					!output.header.frame_id.empty() && !output.child_frame_id.empty())
				{
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

void VinsVisualNode::create_lifecycle_monitor(
	bool enabled,
	const std::string & relay_name,
	const std::string & input_topic,
	const rclcpp::SubscriptionOptions & subscription_options)
{
	if (!enabled) {
		RCLCPP_INFO(get_logger(), "skip disabled lifecycle monitor: %s", relay_name.c_str());
		return;
	}

	if (input_topic.empty()) {
		RCLCPP_WARN(
			get_logger(),
			"skip lifecycle monitor %s because topic name is empty",
			relay_name.c_str());
		return;
	}

	const auto lifecycle_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
	auto subscription = create_subscription<lifecycle_msgs::msg::TransitionEvent>(
		input_topic,
		lifecycle_qos,
		[this, relay_name](lifecycle_msgs::msg::TransitionEvent::SharedPtr message) {
			if (shutting_down_.load() || !message) {
				return;
			}
			RCLCPP_INFO(
				get_logger(),
				"lifecycle %s: %s -> %s (%s)",
				relay_name.c_str(),
				message->start_state.label.c_str(),
				message->goal_state.label.c_str(),
				message->transition.label.c_str());
			update_relay_status(relay_name);
		},
		subscription_options);

	subscriptions_.push_back(subscription);
	relay_specs_.push_back(RelaySpec{relay_name, input_topic, "(monitor)"});
}

void VinsVisualNode::update_relay_status(const std::string & relay_name)
{
	std::lock_guard<std::mutex> lock(status_mutex_);
	relay_counts_[relay_name] += 1;
	last_receive_time_[relay_name] = now();
}

void VinsVisualNode::log_status()
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

	if (tag_align_enabled_) {
		bool tag_pose_valid = false;
		bool camera_intrinsics_valid = false;
		{
			std::lock_guard<std::mutex> lock(tag_pose_mutex_);
			tag_pose_valid = tag_pose_valid_;
			camera_intrinsics_valid = camera_intrinsics_valid_;
		}
		RCLCPP_INFO(
			get_logger(),
			"tag_align status: pose_valid=%s, camera_intrinsics_valid=%s, output_frame=%s",
			tag_pose_valid ? "true" : "false",
			camera_intrinsics_valid ? "true" : "false",
			tag_frame_id_.c_str());
	}
}

int main(int argc, char ** argv)
{
	int exit_code = 0;
	try {
		rclcpp::init(argc, argv);
		auto node = std::make_shared<VinsVisualNode>();
		rclcpp::executors::MultiThreadedExecutor executor(
			rclcpp::ExecutorOptions(), std::max(2, node->get_executor_threads()));
		executor.add_node(node);
		executor.spin();
	} catch (const std::exception & exception) {
		RCLCPP_ERROR(rclcpp::get_logger("vins_visual"), "fatal error: %s", exception.what());
		exit_code = 1;
	}

	if (rclcpp::ok()) {
		rclcpp::shutdown();
	}
	return exit_code;
}
