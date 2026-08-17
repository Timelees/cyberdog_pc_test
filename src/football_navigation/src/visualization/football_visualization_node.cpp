// Copyright (c) 2026 CyberDog2 football navigation contributors.
//
// Licensed under the Apache License, Version 2.0.
//
// RViz visualization for CyberDog2 football no-map chain.
// Global markers remain in the configured field frame. Local markers use
// base_link and transform only through real TF data.

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "football_navigation/core/football_geometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "football_navigation/visualization/football_visualization_node.hpp"

namespace football_navigation
{
namespace
{

const rclcpp::QoS qos_profile_sensor_data = rclcpp::SensorDataQoS();

std::string resolveTopic(const rclcpp::Node & node, const std::string & topic)
{
  if (topic.empty() || topic.front() == '/') {
    return topic;
  }
  std::string ns = node.get_namespace();
  if (ns == "/") {
    return "/" + topic;
  }
  return ns + "/" + topic;
}

std::vector<std::string> splitCsv(const std::string & csv)
{
  std::vector<std::string> output;
  std::stringstream stream(csv);
  std::string value;
  while (std::getline(stream, value, ',')) {
    const auto first = value.find_first_not_of(" \t\r\n/");
    const auto last = value.find_last_not_of(" \t\r\n/");
    if (first != std::string::npos) {
      output.push_back(value.substr(first, last - first + 1));
    }
  }
  return output;
}

std::string expandNamespace(std::string pattern, const std::string & robot_namespace)
{
  const std::string token = "{namespace}";
  const auto position = pattern.find(token);
  if (position == std::string::npos) {
    throw std::invalid_argument("topic template must contain {namespace}");
  }
  pattern.replace(position, token.size(), robot_namespace);
  return pattern;
}

void costToRgb(unsigned char cost, float & r, float & g, float & b)
{
  if (cost == 0) {
    r = g = b = 0.0f;
    return;
  }
  if (cost >= 253) {
    r = 1.0f;
    g = 0.0f;
    b = 0.0f;
    return;
  }
  if (cost >= 99) {
    r = 1.0f;
    g = 0.55f;
    b = 0.0f;
    return;
  }
  if (cost >= 50) {
    r = 1.0f;
    g = 1.0f;
    b = 0.2f;
    return;
  }
  r = 0.2f;
  g = 0.5f;
  b = 1.0f;
}

}  // namespace

FootballVisualizationNode::FootballVisualizationNode()
  : Node("football_visualization_node")
{
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    field_frame_ = declare_parameter<std::string>("field_frame", "tag_global");
    keyboard_robot_namespace_ = declare_parameter<std::string>(
      "keyboard_robot_namespace", "");

    ball_topic_ = declare_parameter<std::string>("ball_topic", "/football/ball_pose");
    approach_pose_topic_ =
      declare_parameter<std::string>("approach_pose_topic", "/football/approach_pose");
    tracking_pose_topic_ =
      declare_parameter<std::string>("tracking_pose_topic", "tracking_pose");
    goal_pose_topic_ =
      declare_parameter<std::string>("goal_pose_topic", "goal_pose");
    robot_namespaces_csv_ = declare_parameter<std::string>(
      "robot_namespaces_csv",
      "cyberdog_1,cyberdog_2,cyberdog_3,cyberdog_4,cyberdog_5,"
      "cyberdog_6,cyberdog_7,cyberdog_8,cyberdog_9,cyberdog_10");
    robot_odom_topic_template_ = declare_parameter<std::string>(
      "robot_odom_topic_template", "/global_vio/{namespace}/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    motion_servo_cmd_topic_ =
      declare_parameter<std::string>("motion_servo_cmd_topic", "motion_servo_cmd");
    costmap_topic_ = declare_parameter<std::string>(
      "costmap_topic", "local_costmap_tracking/costmap");
    path_topic_ = declare_parameter<std::string>("path_topic", "plan");
    local_trajectory_topic_ =
      declare_parameter<std::string>("local_trajectory_topic", "local_plan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom_global");
    field_marker_topic_ = declare_parameter<std::string>(
      "field_marker_topic", "/football/markers/field");
    robot_marker_topic_template_ = declare_parameter<std::string>(
      "robot_marker_topic_template", "/football/markers/robots/{namespace}");
    ball_marker_topic_ = declare_parameter<std::string>(
      "ball_marker_topic", "/football/markers/ball");
    approach_marker_topic_ = declare_parameter<std::string>(
      "approach_marker_topic", "/football/markers/approach_pose");
    tracking_marker_topic_ = declare_parameter<std::string>(
      "tracking_marker_topic", "/football/markers/tracking_pose");
    path_marker_topic_ = declare_parameter<std::string>(
      "path_marker_topic", "/football/markers/paths");
    goal_marker_topic_ = declare_parameter<std::string>(
      "goal_marker_topic", "/football/markers/goals");
    costmap_marker_topic_ = declare_parameter<std::string>(
      "costmap_marker_topic", "/football/markers/costmap");
    command_marker_topic_ = declare_parameter<std::string>(
      "command_marker_topic", "/football/markers/commands");
    status_marker_topic_ = declare_parameter<std::string>(
      "status_marker_topic", "/football/markers/status");
    show_field_boundary_ = declare_parameter<bool>("show_field_boundary", true);
    show_ball_marker_ = declare_parameter<bool>("show_ball_marker", true);
    show_goal_markers_ = declare_parameter<bool>("show_goal_markers", true);
    team_a_kick_topic_ = declare_parameter<std::string>(
      "team_a_kick_target_topic", "/football/team_a/kick_target");
    team_b_kick_topic_ = declare_parameter<std::string>(
      "team_b_kick_target_topic", "/football/team_b/kick_target");
    goal_width_m_ = declare_parameter<double>("goal_width_m", 1.2);
    goal_post_height_m_ = declare_parameter<double>("goal_post_height_m", 0.45);
    field_center_x_ = declare_parameter<double>("field_center_x", 0.0);
    field_center_y_ = declare_parameter<double>("field_center_y", 0.0);
    field_length_m_ = declare_parameter<double>("field_length_m", 16.0);
    field_width_m_ = declare_parameter<double>("field_width_m", 8.0);
    center_circle_radius_m_ = declare_parameter<double>("center_circle_radius_m", 0.75);
    const double team_a_goal_x = declare_parameter<double>("team_a_attack_goal_x", 8.0);
    const double team_a_goal_y = declare_parameter<double>("team_a_attack_goal_y", 0.0);
    const double team_b_goal_x = declare_parameter<double>("team_b_attack_goal_x", -8.0);
    const double team_b_goal_y = declare_parameter<double>("team_b_attack_goal_y", 0.0);

    show_ego_robot_marker_ = declare_parameter<bool>("show_ego_robot_marker", true);
    show_other_robot_markers_ =
      declare_parameter<bool>("show_other_robot_markers", true);
    show_robot_collision_ellipses_ = declare_parameter<bool>(
      "show_robot_collision_ellipses", true);
    team_a_striker_topic_ = declare_parameter<std::string>(
      "team_a_striker_topic", "/football/team_a/striker");
    team_b_striker_topic_ = declare_parameter<std::string>(
      "team_b_striker_topic", "/football/team_b/striker");
    ego_robot_length_m_ = declare_parameter<double>("ego_robot_length_m", 0.562);
    ego_robot_width_m_ = declare_parameter<double>("ego_robot_width_m", 0.339);
    ego_robot_height_m_ = declare_parameter<double>("ego_robot_height_m", 0.481);

    team_a_kick_default_.header.frame_id = field_frame_;
    team_a_kick_default_.pose.position.x = team_a_goal_x;
    team_a_kick_default_.pose.position.y = team_a_goal_y;
    team_a_kick_default_.pose.orientation.w = 1.0;
    team_b_kick_default_.header.frame_id = field_frame_;
    team_b_kick_default_.pose.position.x = team_b_goal_x;
    team_b_kick_default_.pose.position.y = team_b_goal_y;
    team_b_kick_default_.pose.orientation.w = 1.0;

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 1.5);
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 0.0);
    use_delete_all_before_publish_ = declare_parameter<bool>("use_delete_all_before_publish", false);
    velocity_arrow_scale_ = declare_parameter<double>("velocity_arrow_scale", 1.0);
    other_robot_length_m_ = declare_parameter<double>("other_robot_length_m", 0.562);
    other_robot_width_m_ = declare_parameter<double>("other_robot_width_m", 0.339);
    other_robot_height_m_ = declare_parameter<double>("other_robot_height_m", 0.481);
    collision_ellipse_expansion_m_ = declare_parameter<double>(
      "collision_ellipse_expansion_m", 0.05);
    z_offset_ = declare_parameter<double>("z_offset", 0.05);
    command_panel_x_ = declare_parameter<double>("command_panel_x", -3.0);
    command_panel_y_ = declare_parameter<double>("command_panel_y", -5.0);
    status_panel_x_ = declare_parameter<double>("status_panel_x", 1.0);
    status_panel_y_ = declare_parameter<double>("status_panel_y", -5.0);
    enable_costmap_markers_ = declare_parameter<bool>("enable_costmap_markers", true);
    costmap_min_cost_ = declare_parameter<int>("costmap_min_cost", 1);
    expect_motion_cmds_ = declare_parameter<bool>("expect_motion_cmds", false);
    self_namespace_ = declare_parameter<std::string>("self_namespace", "");
    team_id_ = declare_parameter<std::string>("team_id", "");
    if (!self_namespace_.empty() && self_namespace_.front() == '/') {
      self_namespace_.erase(0, 1);
    }
    if (!self_namespace_.empty()) {
      const std::string namespaced_base = self_namespace_ + "/base_link";
      if (namespaced_base != base_frame_) {
        ego_base_frames_.push_back(namespaced_base);
      }
      if (team_id_.empty() && self_namespace_.find("cyberdog_") == 0) {
        try {
          team_id_ = std::stoi(self_namespace_.substr(9)) <= 5 ? "a" : "b";
        } catch (const std::exception &) {
          team_id_ = "a";
        }
      }
    }
    ego_base_frames_.push_back(base_frame_);

    cmd_vel_topic_ = resolveTopic(*this, cmd_vel_topic_);
    motion_servo_cmd_topic_ = resolveTopic(*this, motion_servo_cmd_topic_);
    tracking_pose_topic_ = resolveTopic(*this, tracking_pose_topic_);
    goal_pose_topic_ = resolveTopic(*this, goal_pose_topic_);
    costmap_topic_ = resolveTopic(*this, costmap_topic_);
    path_topic_ = resolveTopic(*this, path_topic_);
    local_trajectory_topic_ = resolveTopic(*this, local_trajectory_topic_);
    odom_topic_ = resolveTopic(*this, odom_topic_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto marker_qos = rclcpp::QoS(10).reliable();
    field_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      field_marker_topic_, marker_qos);
    robot_namespaces_ = splitCsv(robot_namespaces_csv_);
    for (const auto & robot_namespace : robot_namespaces_) {
      robot_marker_pubs_[robot_namespace] =
        create_publisher<visualization_msgs::msg::MarkerArray>(
        expandNamespace(robot_marker_topic_template_, robot_namespace), marker_qos);
      robot_odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
        expandNamespace(robot_odom_topic_template_, robot_namespace), qos_profile_sensor_data,
        [this, robot_namespace](const nav_msgs::msg::Odometry::SharedPtr msg) {
          if (!msg || msg->header.frame_id != field_frame_) {
            return;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          robot_odoms_[robot_namespace] = *msg;
          robot_odom_times_.insert_or_assign(robot_namespace, now());
        }));
    }
    ball_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      ball_marker_topic_, marker_qos);
    approach_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      approach_marker_topic_, marker_qos);
    tracking_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      tracking_marker_topic_, marker_qos);
    path_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      path_marker_topic_, marker_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      goal_marker_topic_, marker_qos);
    costmap_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      costmap_marker_topic_, marker_qos);
    command_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      command_marker_topic_, marker_qos);
    status_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      status_marker_topic_, marker_qos);

    ball_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ball_topic_, qos_profile_sensor_data,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ball_pose_ = *msg;
        ball_time_ = now();
        have_ball_ = true;
      });

    approach_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      approach_pose_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        approach_pose_ = *msg;
        approach_time_ = now();
        have_approach_ = true;
      });

    tracking_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      tracking_pose_topic_, qos_profile_sensor_data,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        tracking_pose_ = *msg;
        tracking_time_ = now();
        have_tracking_ = true;
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_pose_ = *msg;
        goal_time_ = now();
        have_goal_ = true;
      });

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_vel_ = *msg;
        cmd_vel_time_ = now();
        have_cmd_vel_ = true;
      });

    motion_servo_cmd_sub_ = create_subscription<protocol::msg::MotionServoCmd>(
      motion_servo_cmd_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const protocol::msg::MotionServoCmd::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_servo_cmd_ = *msg;
        motion_servo_cmd_time_ = now();
        have_motion_servo_cmd_ = true;
      });

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        costmap_ = *msg;
        costmap_time_ = now();
        costmap_obstacle_cell_count_ = 0;
        for (const auto value : msg->data) {
          if (static_cast<int>(value) >= costmap_min_cost_) {
            ++costmap_obstacle_cell_count_;
          }
        }
        have_costmap_ =
          msg->header.frame_id == target_frame_ &&
          msg->data.size() ==
          static_cast<std::size_t>(msg->info.width) * msg->info.height;
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        path_ = *msg;
        path_time_ = now();
        have_path_ = true;
      });
    local_trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      local_trajectory_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        local_trajectory_ = *msg;
        local_trajectory_time_ = now();
        have_local_trajectory_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos_profile_sensor_data,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_ = *msg;
        odom_time_ = now();
        have_odom_ = true;
      });

    striker_a_sub_ = create_subscription<std_msgs::msg::String>(
      team_a_striker_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        striker_a_ = msg->data;
        have_striker_a_ = true;
      });
    striker_b_sub_ = create_subscription<std_msgs::msg::String>(
      team_b_striker_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        striker_b_ = msg->data;
        have_striker_b_ = true;
      });

    if (show_goal_markers_) {
      team_a_kick_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        team_a_kick_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          team_a_kick_ = *msg;
          team_a_kick_time_ = now();
          have_team_a_kick_ = true;
        });
      team_b_kick_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        team_b_kick_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          team_b_kick_ = *msg;
          team_b_kick_time_ = now();
          have_team_b_kick_ = true;
        });
    }

    const double safe_rate = std::max(1.0, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / safe_rate)),
      std::bind(&FootballVisualizationNode::publishMarkers, this));

    RCLCPP_INFO(
      get_logger(),
      "football_visualization_node target_frame=%s costmap=%s cmd_vel=%s marker_root=/football/markers",
      target_frame_.c_str(), costmap_topic_.c_str(), cmd_vel_topic_.c_str());
  }

bool FootballVisualizationNode::fresh(const rclcpp::Time & stamp, const rclcpp::Time & current) const
{
    return (current - stamp).seconds() <= stale_timeout_sec_;
  }

void FootballVisualizationNode::setLifetime(visualization_msgs::msg::Marker & marker) const
{
    if (marker_lifetime_sec_ <= 0.0) {
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 0;
      return;
    }
    const auto sec = static_cast<int32_t>(std::floor(marker_lifetime_sec_));
    const auto nsec = static_cast<uint32_t>(
      std::max(0.0, marker_lifetime_sec_ - static_cast<double>(sec)) * 1e9);
    marker.lifetime.sec = sec;
    marker.lifetime.nanosec = nsec;
  }

visualization_msgs::msg::Marker FootballVisualizationNode::makeBaseMarker(
  const std::string & frame,
  const std::string & ns,
  int id,
  int type,
  const rclcpp::Time & stamp) const
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame.empty() ? target_frame_ : frame;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    setLifetime(marker);
    marker.color.a = 1.0;
    return marker;
  }

bool FootballVisualizationNode::transformPoseToTarget(
  const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out)
{
    if (in.header.frame_id.empty()) {
      return false;
    }
    if (in.header.frame_id == target_frame_) {
      out = in;
      return true;
    }
    try {
      out = tf_buffer_->transform(in, target_frame_, tf2::durationFromSec(0.25));
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "visualization transform %s -> %s failed: %s",
        in.header.frame_id.c_str(), target_frame_.c_str(), error.what());
      return false;
    }
  }

bool FootballVisualizationNode::transformPoseInPlace(
  geometry_msgs::msg::Pose & pose,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = frame_id;
    in.header.stamp = stamp;
    in.pose = pose;
    geometry_msgs::msg::PoseStamped out;
    if (!transformPoseToTarget(in, out)) {
      return false;
    }
    pose = out.pose;
    return true;
  }

void FootballVisualizationNode::setColor(
  visualization_msgs::msg::Marker & marker,
  float r,
  float g,
  float b,
  float a) const
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
  }

void FootballVisualizationNode::appendDeleteAll(
  visualization_msgs::msg::MarkerArray & array,
  const rclcpp::Time & stamp) const
{
    auto del = makeBaseMarker(
      target_frame_, "football_clear", 0,
      visualization_msgs::msg::Marker::DELETEALL, stamp);
    array.markers.push_back(del);
  }

void FootballVisualizationNode::appendPoseMarker(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::PoseStamped & pose_in,
  const std::string & ns,
  int id_base,
  const std::string & label,
  float r,
  float g,
  float b,
  double sphere_size,
  bool draw_arrow,
  const rclcpp::Time & stamp)
{
    geometry_msgs::msg::PoseStamped pose;
    if (!transformPoseToTarget(pose_in, pose)) {
      return;
    }

    auto sphere = makeBaseMarker(
      target_frame_, ns, id_base,
      visualization_msgs::msg::Marker::SPHERE, stamp);
    sphere.pose = pose.pose;
    sphere.pose.position.z += z_offset_;
    sphere.scale.x = sphere_size;
    sphere.scale.y = sphere_size;
    sphere.scale.z = sphere_size;
    setColor(sphere, r, g, b, 0.9);
    array.markers.push_back(sphere);

    if (draw_arrow) {
      auto arrow = makeBaseMarker(
        target_frame_, ns, id_base + 1,
        visualization_msgs::msg::Marker::ARROW, stamp);
      arrow.pose = pose.pose;
      arrow.pose.position.z += z_offset_ + 0.04;
      arrow.scale.x = 0.35;
      arrow.scale.y = 0.06;
      arrow.scale.z = 0.10;
      setColor(arrow, r, g, b, 0.95);
      array.markers.push_back(arrow);
    }

    auto text = makeBaseMarker(
      target_frame_, ns, id_base + 2,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
    text.pose = pose.pose;
    text.pose.position.z += ns == "approach_pose" ? 0.55 : 0.35;
    text.scale.z = 0.16;
    std::ostringstream ss;
    ss << label << " (" << std::fixed << std::setprecision(2)
       << pose.pose.position.x << "," << pose.pose.position.y << ")";
    text.text = ss.str();
    setColor(text, r, g, b, 1.0);
    array.markers.push_back(text);
  }

void FootballVisualizationNode::appendGoalMarker(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::PoseStamped & kick_in,
  const std::string & ns,
  int id_base,
  const std::string & label,
  float r,
  float g,
  float b,
  const rclcpp::Time & stamp)
{
    geometry_msgs::msg::PoseStamped kick;
    if (!transformPoseToTarget(kick_in, kick)) {
      return;
    }

    const double gx = kick.pose.position.x;
    const double gy = kick.pose.position.y;
    const double half_w = goal_width_m_ * 0.5;
    const double h = goal_post_height_m_;
    const double inward = (gx >= 0.0) ? -0.15 : 0.15;

    auto goal_line = makeBaseMarker(
      target_frame_, ns, id_base,
      visualization_msgs::msg::Marker::LINE_LIST, stamp);
    goal_line.scale.x = 0.06;
    setColor(goal_line, r, g, b, 0.95);

    auto add_point = [&](double x, double y, double z) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      goal_line.points.push_back(p);
    };

    add_point(gx, gy - half_w, 0.0);
    add_point(gx, gy - half_w, h);
    add_point(gx, gy + half_w, 0.0);
    add_point(gx, gy + half_w, h);
    add_point(gx, gy - half_w, h);
    add_point(gx, gy + half_w, h);
    add_point(gx, gy - half_w, 0.0);
    add_point(gx + inward, gy - half_w, 0.0);
    add_point(gx, gy + half_w, 0.0);
    add_point(gx + inward, gy + half_w, 0.0);
    array.markers.push_back(goal_line);

    auto label_marker = makeBaseMarker(
      target_frame_, ns, id_base + 1,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
    label_marker.pose.position.x = gx + inward * 0.5;
    label_marker.pose.position.y = gy;
    label_marker.pose.position.z = h + 0.15;
    label_marker.scale.z = 0.22;
    label_marker.text = label;
    setColor(label_marker, r, g, b, 1.0);
    array.markers.push_back(label_marker);
  }

void FootballVisualizationNode::appendLine(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::PoseStamped & a_in,
  const geometry_msgs::msg::PoseStamped & b_in,
  const std::string & ns,
  int id,
  const rclcpp::Time & stamp)
{
    geometry_msgs::msg::PoseStamped a;
    geometry_msgs::msg::PoseStamped b;
    if (!transformPoseToTarget(a_in, a) || !transformPoseToTarget(b_in, b)) {
      return;
    }
    auto line = makeBaseMarker(
      target_frame_, ns, id,
      visualization_msgs::msg::Marker::LINE_STRIP, stamp);
    geometry_msgs::msg::Point p0;
    geometry_msgs::msg::Point p1;
    p0.x = a.pose.position.x;
    p0.y = a.pose.position.y;
    p0.z = a.pose.position.z + z_offset_;
    p1.x = b.pose.position.x;
    p1.y = b.pose.position.y;
    p1.z = b.pose.position.z + z_offset_;
    line.points.push_back(p0);
    line.points.push_back(p1);
    line.scale.x = 0.03;
    setColor(line, 1.0, 1.0, 0.0, 0.8);
    array.markers.push_back(line);
  }

void FootballVisualizationNode::appendFieldBoundary(
  visualization_msgs::msg::MarkerArray & array,
  const rclcpp::Time & stamp) const
{
    const double min_x = field_center_x_ - field_length_m_ * 0.5;
    const double max_x = field_center_x_ + field_length_m_ * 0.5;
    const double min_y = field_center_y_ - field_width_m_ * 0.5;
    const double max_y = field_center_y_ + field_width_m_ * 0.5;
    auto boundary = makeBaseMarker(
      target_frame_, "field_boundary", 600,
      visualization_msgs::msg::Marker::LINE_STRIP, stamp);
    boundary.scale.x = 0.06;
    setColor(boundary, 0.95f, 0.95f, 0.95f, 0.95f);
    const std::pair<double, double> corners[] = {
      {min_x, min_y},
      {max_x, min_y},
      {max_x, max_y},
      {min_x, max_y},
      {min_x, min_y}};
    for (const auto & corner : corners) {
      geometry_msgs::msg::Point point;
      point.x = corner.first;
      point.y = corner.second;
      point.z = 0.01;
      boundary.points.push_back(point);
    }
    array.markers.push_back(boundary);

    auto center_line = makeBaseMarker(
      target_frame_, "field_center_line", 601,
      visualization_msgs::msg::Marker::LINE_LIST, stamp);
    center_line.scale.x = 0.035;
    setColor(center_line, 0.85f, 0.85f, 0.85f, 0.8f);
    geometry_msgs::msg::Point line_start;
    line_start.x = field_center_x_;
    line_start.y = min_y;
    line_start.z = 0.01;
    geometry_msgs::msg::Point line_end = line_start;
    line_end.y = max_y;
    center_line.points = {line_start, line_end};
    array.markers.push_back(center_line);

    auto center_circle = makeBaseMarker(
      target_frame_, "field_center_circle", 602,
      visualization_msgs::msg::Marker::LINE_STRIP, stamp);
    center_circle.scale.x = 0.035;
    setColor(center_circle, 0.85f, 0.85f, 0.85f, 0.8f);
    constexpr int circle_segments = 48;
    for (int i = 0; i <= circle_segments; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / circle_segments;
      geometry_msgs::msg::Point point;
      point.x = field_center_x_ + center_circle_radius_m_ * std::cos(angle);
      point.y = field_center_y_ + center_circle_radius_m_ * std::sin(angle);
      point.z = 0.01;
      center_circle.points.push_back(point);
    }
    array.markers.push_back(center_circle);

    auto origin = makeBaseMarker(
      target_frame_, "field_origin", 603,
      visualization_msgs::msg::Marker::SPHERE, stamp);
    origin.pose.position.x = field_center_x_;
    origin.pose.position.y = field_center_y_;
    origin.pose.position.z = 0.025;
    origin.pose.orientation.w = 1.0;
    origin.scale.x = origin.scale.y = origin.scale.z = 0.10;
    setColor(origin, 1.0f, 1.0f, 1.0f, 1.0f);
    array.markers.push_back(origin);
  }

void FootballVisualizationNode::appendPath(
  visualization_msgs::msg::MarkerArray & array,
  const nav_msgs::msg::Path & path,
  const std::string & marker_namespace,
  int id,
  float r,
  float g,
  float b,
  const rclcpp::Time & stamp)
{
    auto line = makeBaseMarker(
      target_frame_, marker_namespace, id,
      visualization_msgs::msg::Marker::LINE_STRIP, stamp);
    line.scale.x = marker_namespace == "local_trajectory" ? 0.055 : 0.035;
    setColor(line, r, g, b, 0.95f);
    for (auto pose : path.poses) {
      if (pose.header.frame_id.empty()) {
        pose.header = path.header;
      }
      geometry_msgs::msg::PoseStamped transformed;
      if (!transformPoseToTarget(pose, transformed)) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = transformed.pose.position.x;
      point.y = transformed.pose.position.y;
      point.z = 0.04;
      line.points.push_back(point);
    }
    if (line.points.size() >= 2) {
      array.markers.push_back(line);
    }
  }

void FootballVisualizationNode::appendFootprint(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Time & stamp) const
{
    auto footprint = makeBaseMarker(
      target_frame_, "footprint", 620,
      visualization_msgs::msg::Marker::LINE_STRIP, stamp);
    footprint.scale.x = 0.035;
    setColor(footprint, 0.0f, 1.0f, 1.0f, 0.95f);
    const double yaw = tf2::getYaw(pose.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double hx = ego_robot_length_m_ * 0.5;
    const double hy = ego_robot_width_m_ * 0.5;
    const std::pair<double, double> corners[] = {
      {hx, hy},
      {hx, -hy},
      {-hx, -hy},
      {-hx, hy},
      {hx, hy}};
    for (const auto & corner : corners) {
      geometry_msgs::msg::Point point;
      point.x = pose.position.x + c * corner.first - s * corner.second;
      point.y = pose.position.y + s * corner.first + c * corner.second;
      point.z = 0.03;
      footprint.points.push_back(point);
    }
    array.markers.push_back(footprint);
  }

void FootballVisualizationNode::appendOdometry(
  visualization_msgs::msg::MarkerArray & array,
  const nav_msgs::msg::Odometry & odom,
  const rclcpp::Time & stamp)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    appendPoseMarker(
      array,
      pose,
      "odometry",
      640,
      "Odometry",
      0.9f,
      0.9f,
      0.1f,
      0.09,
      true,
      stamp);
  }

void FootballVisualizationNode::appendRobot(
  visualization_msgs::msg::MarkerArray & array,
  const std::string & robot_namespace,
  const nav_msgs::msg::Odometry & odom,
  const rclcpp::Time & stamp) const
{
      const auto & pose = odom.pose.pose;
      const bool team_a = [&robot_namespace]() {
          try {
          return std::stoi(robot_namespace.substr(robot_namespace.rfind('_') + 1)) <= 5;
          } catch (const std::exception &) {
          return true;
          }
        }();
      auto box = makeBaseMarker(
        target_frame_, robot_namespace, 1,
        visualization_msgs::msg::Marker::CUBE, stamp);
      box.pose = pose;
      box.pose.position.z += other_robot_height_m_ * 0.5;
      box.scale.x = other_robot_length_m_;
      box.scale.y = other_robot_width_m_;
      box.scale.z = other_robot_height_m_;
      const bool keyboard_robot = robot_namespace == keyboard_robot_namespace_;
      if (robot_namespace == self_namespace_) {
        setColor(box, 0.0, 0.95, 1.0, 0.75);
      } else if (keyboard_robot) {
        setColor(box, 1.0, 0.05, 0.05, 0.85);
      } else if (team_a) {
        setColor(box, 0.15, 0.95, 0.25, 0.65);
      } else {
        setColor(box, 0.25, 0.45, 1.0, 0.65);
      }
      array.markers.push_back(box);

      auto text = makeBaseMarker(
        target_frame_, robot_namespace, 2,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
      text.pose = pose;
      text.pose.position.z += other_robot_height_m_ + 0.05;
      text.scale.z = 0.14;
      std::ostringstream ss;
      ss << robot_namespace << (robot_namespace == self_namespace_ ? " [STRIKER]" : "")
         << " (" << std::fixed << std::setprecision(2)
         << pose.position.x << "," << pose.position.y << ")";
      text.text = ss.str();
      setColor(text, keyboard_robot ? 1.0f : (team_a ? 0.3f : 0.45f),
        keyboard_robot ? 0.15f : (team_a ? 1.0f : 0.65f),
        keyboard_robot ? 0.15f : 1.0f, 1.0f);
      array.markers.push_back(text);

      if (show_robot_collision_ellipses_) {
        const auto ellipse = makeCircumscribedCollisionEllipse(
          other_robot_length_m_, other_robot_width_m_, collision_ellipse_expansion_m_);
        auto collision_area = makeBaseMarker(
          target_frame_, robot_namespace, 3,
          visualization_msgs::msg::Marker::CYLINDER, stamp);
        collision_area.pose = pose;
        collision_area.pose.position.z = 0.012;
        collision_area.scale.x = 2.0 * ellipse.semi_major_m;
        collision_area.scale.y = 2.0 * ellipse.semi_minor_m;
        collision_area.scale.z = 0.018;
        setColor(collision_area, keyboard_robot ? 1.0f : (team_a ? 0.1f : 0.25f),
          keyboard_robot ? 0.05f : (team_a ? 1.0f : 0.55f),
          keyboard_robot ? 0.05f : 1.0f, 0.20f);
        array.markers.push_back(collision_area);

        auto ellipse_outline = makeBaseMarker(
          target_frame_, robot_namespace, 4,
          visualization_msgs::msg::Marker::LINE_STRIP, stamp);
        ellipse_outline.scale.x = 0.025;
        setColor(ellipse_outline, keyboard_robot ? 1.0f : (team_a ? 0.1f : 0.25f),
          keyboard_robot ? 0.05f : (team_a ? 1.0f : 0.55f),
          keyboard_robot ? 0.05f : 1.0f, 0.95f);
        const double yaw = tf2::getYaw(pose.orientation);
        const double cosine = std::cos(yaw);
        const double sine = std::sin(yaw);
        constexpr int ellipse_segments = 48;
        for (int index = 0; index <= ellipse_segments; ++index) {
          const double angle = 2.0 * M_PI * static_cast<double>(index) / ellipse_segments;
          const double local_x = ellipse.semi_major_m * std::cos(angle);
          const double local_y = ellipse.semi_minor_m * std::sin(angle);
          geometry_msgs::msg::Point point;
          point.x = pose.position.x + cosine * local_x - sine * local_y;
          point.y = pose.position.y + sine * local_x + cosine * local_y;
          point.z = 0.025;
          ellipse_outline.points.push_back(point);
        }
        array.markers.push_back(ellipse_outline);
      }
}

void FootballVisualizationNode::appendCostmapPoints(
  visualization_msgs::msg::MarkerArray & array,
  const nav_msgs::msg::OccupancyGrid & grid,
  const rclcpp::Time & stamp)
{
    if (grid.header.frame_id != target_frame_) {
      return;
    }

    auto points_marker = makeBaseMarker(
      target_frame_, "costmap_points", 800,
      visualization_msgs::msg::Marker::POINTS, stamp);
    points_marker.scale.x = grid.info.resolution * 0.9;
    points_marker.scale.y = grid.info.resolution * 0.9;
    points_marker.pose.orientation.w = 1.0;

    const auto width = grid.info.width;
    const auto height = grid.info.height;
    const double ox = grid.info.origin.position.x;
    const double oy = grid.info.origin.position.y;
    const double res = grid.info.resolution;

    for (unsigned int my = 0; my < height; ++my) {
      for (unsigned int mx = 0; mx < width; ++mx) {
        const auto idx = my * width + mx;
        if (idx >= grid.data.size()) {
          continue;
        }
        const auto raw_cost = static_cast<int>(grid.data[idx]);
        if (raw_cost < costmap_min_cost_) {
          continue;
        }
        const auto cost = static_cast<unsigned char>(raw_cost);
        geometry_msgs::msg::Point p;
        p.x = ox + (static_cast<double>(mx) + 0.5) * res;
        p.y = oy + (static_cast<double>(my) + 0.5) * res;
        p.z = 0.02;
        points_marker.points.push_back(p);
        std_msgs::msg::ColorRGBA color;
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        costToRgb(cost, r, g, b);
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = 0.85f;
        points_marker.colors.push_back(color);
      }
    }

    if (!points_marker.points.empty()) {
      array.markers.push_back(points_marker);
    }
  }

std::size_t FootballVisualizationNode::countCostmapBackedRobots()
{
    if (robot_odoms_.empty() ||
      costmap_.header.frame_id != target_frame_ ||
      costmap_.info.resolution <= 0.0 ||
      costmap_.info.width == 0 ||
      costmap_.info.height == 0)
    {
      return 0;
    }

    const double resolution = costmap_.info.resolution;
    const double origin_x = costmap_.info.origin.position.x;
    const double origin_y = costmap_.info.origin.position.y;
    const int search_cells = std::max(
      1,
      static_cast<int>(std::ceil(0.25 / resolution)));
    std::size_t matched = 0;
    for (const auto & robot : robot_odoms_) {
      const auto & pose = robot.second.pose.pose;
      const int center_x = static_cast<int>(std::floor(
          (pose.position.x - origin_x) / resolution));
      const int center_y = static_cast<int>(std::floor(
          (pose.position.y - origin_y) / resolution));
      bool found = false;
      for (int dy = -search_cells; dy <= search_cells && !found; ++dy) {
        for (int dx = -search_cells; dx <= search_cells; ++dx) {
          const int mx = center_x + dx;
          const int my = center_y + dy;
          if (mx < 0 ||
            my < 0 ||
            mx >= static_cast<int>(costmap_.info.width) ||
            my >= static_cast<int>(costmap_.info.height))
          {
            continue;
          }
          const auto index =
            static_cast<std::size_t>(my) * costmap_.info.width +
            static_cast<std::size_t>(mx);
          if (index < costmap_.data.size() &&
            static_cast<int>(costmap_.data[index]) >= 99)
          {
            found = true;
            break;
          }
        }
      }
      if (found) {
        ++matched;
      }
    }
    return matched;
  }

bool FootballVisualizationNode::lookupRobotPoseInTarget(
  const std::string & robot_frame,
  geometry_msgs::msg::Pose & pose) const
{
    try {
      const auto tf = tf_buffer_->lookupTransform(
        target_frame_,
        robot_frame,
        tf2::TimePointZero);
      pose.position.x = tf.transform.translation.x;
      pose.position.y = tf.transform.translation.y;
      pose.position.z = tf.transform.translation.z;
      pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

bool FootballVisualizationNode::lookupEgoPoseInTarget(geometry_msgs::msg::Pose & pose) const
{
    for (const auto & ego_frame : ego_base_frames_) {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          target_frame_,
          ego_frame,
          tf2::TimePointZero);
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.position.z = tf.transform.translation.z;
        pose.orientation = tf.transform.rotation;
        return true;
      } catch (const tf2::TransformException &) {
        continue;
      }
    }
    return false;
  }

void FootballVisualizationNode::appendEgoRobot(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Time & stamp) const
{
    auto box = makeBaseMarker(
      target_frame_, "ego_robot", 60,
      visualization_msgs::msg::Marker::CUBE, stamp);
    box.pose = pose;
    box.pose.position.z += ego_robot_height_m_ * 0.5;
    box.scale.x = ego_robot_length_m_;
    box.scale.y = ego_robot_width_m_;
    box.scale.z = ego_robot_height_m_;
    setColor(box, 0.0, 0.85, 1.0, 0.55);
    array.markers.push_back(box);

    auto text = makeBaseMarker(
      target_frame_, "ego_robot", 61,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
    text.pose = pose;
    text.pose.position.z += ego_robot_height_m_ + 0.08;
    text.scale.z = 0.16;
    std::ostringstream ss;
    ss << (self_namespace_.empty() ? "ego" : self_namespace_)
       << " (" << std::fixed << std::setprecision(2)
       << pose.position.x << "," << pose.position.y << ")";
    text.text = ss.str();
    setColor(text, 0.0, 0.9, 1.0, 1.0);
    array.markers.push_back(text);
  }

void FootballVisualizationNode::appendTwistArrowAt(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::Twist & twist,
  const geometry_msgs::msg::Pose & origin,
  const std::string & ns,
  int id,
  const std::string & label,
  float r,
  float g,
  float b,
  double z_offset_extra,
  const rclcpp::Time & stamp) const
{
    auto arrow = makeBaseMarker(
      target_frame_, ns, id,
      visualization_msgs::msg::Marker::ARROW, stamp);
    geometry_msgs::msg::Point p0;
    geometry_msgs::msg::Point p1;
    p0.x = origin.position.x;
    p0.y = origin.position.y;
    p0.z = origin.position.z + z_offset_extra;
    const auto world_velocity = worldVelocityToBody(
      twist.linear.x, twist.linear.y, -tf2::getYaw(origin.orientation));
    p1.x = p0.x + world_velocity.first * velocity_arrow_scale_;
    p1.y = p0.y + world_velocity.second * velocity_arrow_scale_;
    p1.z = p0.z;
    arrow.points.push_back(p0);
    arrow.points.push_back(p1);
    arrow.scale.x = 0.04;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.12;
    setColor(arrow, r, g, b, 0.95);
    array.markers.push_back(arrow);

    auto text = makeBaseMarker(
      target_frame_, ns, id + 1,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
    text.pose = origin;
    text.pose.position.z += z_offset_extra + 0.20;
    text.scale.z = 0.13;
    std::ostringstream ss;
    ss << label
       << " vx=" << std::fixed << std::setprecision(2) << twist.linear.x
       << " vy=" << twist.linear.y
       << " wz=" << twist.angular.z;
    text.text = ss.str();
    setColor(text, r, g, b, 1.0);
    array.markers.push_back(text);
  }

void FootballVisualizationNode::appendTwistArrow(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::Twist & twist,
  const std::string & ns,
  int id,
  const std::string & label,
  float r,
  float g,
  float b,
  double z,
  const rclcpp::Time & stamp) const
{
    auto arrow = makeBaseMarker(
      target_frame_, ns, id,
      visualization_msgs::msg::Marker::ARROW, stamp);
    geometry_msgs::msg::Point p0;
    geometry_msgs::msg::Point p1;
    p0.x = command_panel_x_;
    p0.y = command_panel_y_;
    p0.z = z;
    p1.x = p0.x + twist.linear.x * velocity_arrow_scale_;
    p1.y = p0.y + twist.linear.y * velocity_arrow_scale_;
    p1.z = z;
    arrow.points.push_back(p0);
    arrow.points.push_back(p1);
    arrow.scale.x = 0.04;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.12;
    setColor(arrow, r, g, b, 0.95);
    array.markers.push_back(arrow);

    auto text = makeBaseMarker(
      target_frame_, ns, id + 1,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
    text.pose.position.x = command_panel_x_;
    text.pose.position.y = command_panel_y_;
    text.pose.position.z = z + 0.20;
    text.scale.z = 0.13;
    std::ostringstream ss;
    ss << label
       << " vx=" << std::fixed << std::setprecision(2) << twist.linear.x
       << " vy=" << twist.linear.y
       << " wz=" << twist.angular.z;
    text.text = ss.str();
    setColor(text, r, g, b, 1.0);
    array.markers.push_back(text);
  }

void FootballVisualizationNode::appendMotionServoArrow(
  visualization_msgs::msg::MarkerArray & array,
  const protocol::msg::MotionServoCmd & cmd,
  const geometry_msgs::msg::Pose * origin,
  const rclcpp::Time & stamp) const
{
    geometry_msgs::msg::Twist twist;
    if (cmd.vel_des.size() > 0) {
      twist.linear.x = cmd.vel_des[0];
    }
    if (cmd.vel_des.size() > 1) {
      twist.linear.y = cmd.vel_des[1];
    }
    if (cmd.vel_des.size() > 2) {
      twist.angular.z = cmd.vel_des[2];
    }
    if (origin != nullptr) {
      appendTwistArrowAt(
        array,
        twist,
        *origin,
        "motion_servo_cmd",
        410,
        "motion_servo_cmd",
        1.0,
        0.0,
        1.0,
        0.28,
        stamp);
    } else {
      appendTwistArrow(
        array,
        twist,
        "motion_servo_cmd",
        410,
        "motion_servo_cmd",
        1.0,
        0.0,
        1.0,
        0.28,
        stamp);
    }
  }

void FootballVisualizationNode::appendStatusText(
  visualization_msgs::msg::MarkerArray & array,
  const rclcpp::Time & stamp,
  const std::size_t costmap_dynamic_robot_count) const
{
    auto text = makeBaseMarker(
      target_frame_, "football_status", 900,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, stamp);
    text.pose.position.x = status_panel_x_;
    text.pose.position.y = status_panel_y_;
    text.pose.position.z = 0.9;
    text.scale.z = 0.14;

    const auto motion_hint = expect_motion_cmds_ ?
      "" :
      "  [motion off / non-striker: approach/tracking/cmd_vel NO is normal]";
    const bool costmap_data_ok =
      have_costmap_ &&
      fresh(costmap_time_, stamp) &&
      costmap_obstacle_cell_count_ > 0;

    std::ostringstream ss;
    ss << "football no-map @ " << target_frame_ << "  (stable view)\n"
       << "ball=" << (have_ball_ ? "OK" : "NO")
       << "  other_robots="
       << (robot_odoms_.empty() ? "NO" : std::to_string(robot_odoms_.size() -
      (robot_odoms_.count(self_namespace_) > 0 ? 1 : 0)))
       << "  costmap="
       << (costmap_data_ok ?
      "OK(" + std::to_string(costmap_obstacle_cell_count_) + " cells)" :
      "NO")
       << "  robot_match=" << costmap_dynamic_robot_count
       << "\n"
       << "striker_a=" << (have_striker_a_ ? striker_a_ : "NO")
       << "  striker_b=" << (have_striker_b_ ? striker_b_ : "NO")
       << "\n"
       << "approach=" << (have_approach_ ? "OK" : "NO")
       << "  tracking=" << (have_tracking_ ? "OK" : "NO")
       << "  cmd_vel=" << (have_cmd_vel_ ? "OK" : "NO")
       << motion_hint;
    text.text = ss.str();
    setColor(text, 1.0, 1.0, 1.0, 1.0);
    array.markers.push_back(text);
  }

void FootballVisualizationNode::publishMarkers()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto stamp = now();

    visualization_msgs::msg::MarkerArray field_markers;
    visualization_msgs::msg::MarkerArray ball_markers;
    visualization_msgs::msg::MarkerArray approach_markers;
    visualization_msgs::msg::MarkerArray tracking_markers;
    visualization_msgs::msg::MarkerArray path_markers;
    visualization_msgs::msg::MarkerArray goal_markers;
    visualization_msgs::msg::MarkerArray costmap_markers;
    visualization_msgs::msg::MarkerArray command_markers;
    visualization_msgs::msg::MarkerArray status_markers;
    if (use_delete_all_before_publish_) {
      appendDeleteAll(field_markers, stamp);
      appendDeleteAll(ball_markers, stamp);
      appendDeleteAll(approach_markers, stamp);
      appendDeleteAll(tracking_markers, stamp);
      appendDeleteAll(path_markers, stamp);
      appendDeleteAll(goal_markers, stamp);
      appendDeleteAll(costmap_markers, stamp);
      appendDeleteAll(command_markers, stamp);
      appendDeleteAll(status_markers, stamp);
    }
    if (show_field_boundary_) {
      appendFieldBoundary(field_markers, stamp);
    }

    if (show_ego_robot_marker_ || show_other_robot_markers_) {
      for (const auto & robot_namespace : robot_namespaces_) {
        visualization_msgs::msg::MarkerArray robot_markers;
        if (use_delete_all_before_publish_) {
          appendDeleteAll(robot_markers, stamp);
        }
        const auto odom = robot_odoms_.find(robot_namespace);
        const auto received = robot_odom_times_.find(robot_namespace);
        if (odom != robot_odoms_.end() && received != robot_odom_times_.end() &&
          fresh(received->second, stamp) &&
          (robot_namespace == self_namespace_ ? show_ego_robot_marker_ : show_other_robot_markers_))
        {
          appendRobot(robot_markers, robot_namespace, odom->second, stamp);
        }
        robot_marker_pubs_.at(robot_namespace)->publish(robot_markers);
      }
    }

    if (show_ball_marker_ && have_ball_ && fresh(ball_time_, stamp)) {
      appendPoseMarker(
        ball_markers,
        ball_pose_,
        "ball_pose",
        1,
        "ball_pose",
        1.0,
        0.55,
        0.0,
        0.18,
        false,
        stamp);
    }
    if (have_approach_ && fresh(approach_time_, stamp)) {
      appendPoseMarker(
        approach_markers,
        approach_pose_,
        "approach_pose",
        10,
        "approach_pose",
        0.0,
        1.0,
        0.0,
        0.12,
        true,
        stamp);
    }
    if (have_tracking_ && fresh(tracking_time_, stamp)) {
      appendPoseMarker(
        tracking_markers,
        tracking_pose_,
        "tracking_pose",
        20,
        "tracking_pose",
        0.0,
        0.3,
        1.0,
        0.12,
        true,
        stamp);
    }
    if (have_goal_ && fresh(goal_time_, stamp)) {
      appendPoseMarker(
        path_markers,
        goal_pose_,
        "goal_pose",
        30,
        "goal_pose",
        0.6,
        0.2,
        1.0,
        0.10,
        true,
        stamp);
    }
    if (show_ball_marker_ &&
      have_ball_ &&
      have_approach_ &&
      fresh(ball_time_, stamp) &&
      fresh(approach_time_, stamp))
    {
      appendLine(
        approach_markers,
        ball_pose_,
        approach_pose_,
        "ball_to_approach",
        50,
        stamp);
    }
    if (enable_costmap_markers_ &&
      have_costmap_ &&
      fresh(costmap_time_, stamp))
    {
      appendCostmapPoints(
        costmap_markers,
        costmap_,
        stamp);
    }
    if (have_path_ && fresh(path_time_, stamp)) {
      appendPath(
        path_markers,
        path_,
        "global_path",
        660,
        0.2f,
        0.55f,
        1.0f,
        stamp);
    }
    if (have_local_trajectory_ &&
      fresh(local_trajectory_time_, stamp))
    {
      appendPath(
        path_markers,
        local_trajectory_,
        "local_trajectory",
        670,
        1.0f,
        0.25f,
        0.85f,
        stamp);
    }
    if (have_odom_ && fresh(odom_time_, stamp)) {
      // The robot pose is already published with its body and label on the
      // namespace-specific marker topic.
    }
    if (show_goal_markers_) {
      const geometry_msgs::msg::PoseStamped & kick_a =
        have_team_a_kick_ ?
        team_a_kick_ :
        team_a_kick_default_;
      const geometry_msgs::msg::PoseStamped & kick_b =
        have_team_b_kick_ ?
        team_b_kick_ :
        team_b_kick_default_;
      appendGoalMarker(
        goal_markers,
        kick_a,
        "team_a_goal",
        500,
        "A队进攻球门",
        0.15,
        0.95,
        0.25,
        stamp);
      appendGoalMarker(
        goal_markers,
        kick_b,
        "team_b_goal",
        520,
        "B队进攻球门",
        0.25,
        0.55,
        1.0,
        stamp);
      if (have_ball_ && fresh(ball_time_, stamp)) {
        appendLine(
          goal_markers,
          ball_pose_,
          team_id_ == "b" ? kick_b : kick_a,
          "shot_direction",
          540,
          stamp);
      }
    }
    if (have_cmd_vel_ && fresh(cmd_vel_time_, stamp)) {
      appendTwistArrow(
        command_markers,
        cmd_vel_,
        "cmd_vel",
        400,
        "cmd_vel",
        0.0,
        1.0,
        1.0,
        0.15,
        stamp);
    }
    if (have_motion_servo_cmd_ &&
      fresh(motion_servo_cmd_time_, stamp))
    {
      appendMotionServoArrow(
        command_markers,
        motion_servo_cmd_,
        nullptr,
        stamp);
    }

    const std::size_t costmap_dynamic_robot_count =
      have_costmap_ &&
      fresh(costmap_time_, stamp) &&
      !robot_odoms_.empty() ?
      countCostmapBackedRobots() :
      0;
    appendStatusText(
      status_markers,
      stamp,
      costmap_dynamic_robot_count);
    field_marker_pub_->publish(field_markers);
    ball_marker_pub_->publish(ball_markers);
    approach_marker_pub_->publish(approach_markers);
    tracking_marker_pub_->publish(tracking_markers);
    path_marker_pub_->publish(path_markers);
    goal_marker_pub_->publish(goal_markers);
    costmap_marker_pub_->publish(costmap_markers);
    command_marker_pub_->publish(command_markers);
    status_marker_pub_->publish(status_markers);
  }


}  // namespace football_navigation
