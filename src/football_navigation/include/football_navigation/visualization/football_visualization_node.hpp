// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <memory>
#include <map>
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

namespace football_navigation
{

class FootballVisualizationNode : public rclcpp::Node
{
public:
  FootballVisualizationNode();

private:
  bool fresh(const rclcpp::Time & stamp, const rclcpp::Time & current) const;
  void setLifetime(visualization_msgs::msg::Marker & marker) const;
  visualization_msgs::msg::Marker makeBaseMarker(
  const std::string & frame,
  const std::string & ns,
  int id,
  int type,
  const rclcpp::Time & stamp) const;
  bool transformPoseToTarget(
  const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out);
  bool transformPoseInPlace(
  geometry_msgs::msg::Pose & pose,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);
  void setColor(
  visualization_msgs::msg::Marker & marker,
  float r,
  float g,
  float b,
  float a = 1.0) const;
  void appendDeleteAll(
  visualization_msgs::msg::MarkerArray & array,
  const rclcpp::Time & stamp) const;
  void appendPoseMarker(
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
  const rclcpp::Time & stamp);
  void appendGoalMarker(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::PoseStamped & kick_in,
  const std::string & ns,
  int id_base,
  const std::string & label,
  float r,
  float g,
  float b,
  const rclcpp::Time & stamp);
  void appendLine(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::PoseStamped & a_in,
  const geometry_msgs::msg::PoseStamped & b_in,
  const std::string & ns,
  int id,
  const rclcpp::Time & stamp);
  void appendFieldBoundary(
  visualization_msgs::msg::MarkerArray & array,
  const rclcpp::Time & stamp) const;
  void appendPath(
  visualization_msgs::msg::MarkerArray & array,
  const nav_msgs::msg::Path & path,
  const std::string & marker_namespace,
  int id,
  float r,
  float g,
  float b,
  const rclcpp::Time & stamp);
  void appendFootprint(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Time & stamp) const;
  void appendOdometry(
  visualization_msgs::msg::MarkerArray & array,
  const nav_msgs::msg::Odometry & odom,
  const rclcpp::Time & stamp);
  void appendRobot(
  visualization_msgs::msg::MarkerArray & array,
  const std::string & robot_namespace,
  const nav_msgs::msg::Odometry & odom,
  const rclcpp::Time & stamp) const;
  void appendCostmapPoints(
  visualization_msgs::msg::MarkerArray & array,
  const nav_msgs::msg::OccupancyGrid & grid,
  const rclcpp::Time & stamp);
  std::size_t countCostmapBackedRobots();
  bool lookupRobotPoseInTarget(
  const std::string & robot_frame,
  geometry_msgs::msg::Pose & pose) const;
  bool lookupEgoPoseInTarget(geometry_msgs::msg::Pose & pose) const;
  void appendEgoRobot(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::Pose & pose,
  const rclcpp::Time & stamp) const;
  void appendTwistArrowAt(
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
  const rclcpp::Time & stamp) const;
  void appendTwistArrow(
  visualization_msgs::msg::MarkerArray & array,
  const geometry_msgs::msg::Twist & twist,
  const std::string & ns,
  int id,
  const std::string & label,
  float r,
  float g,
  float b,
  double z,
  const rclcpp::Time & stamp) const;
  void appendMotionServoArrow(
  visualization_msgs::msg::MarkerArray & array,
  const protocol::msg::MotionServoCmd & cmd,
  const geometry_msgs::msg::Pose * origin,
  const rclcpp::Time & stamp) const;
  void appendStatusText(
  visualization_msgs::msg::MarkerArray & array,
  const rclcpp::Time & stamp,
  const std::size_t costmap_dynamic_robot_count) const;
  void publishMarkers();

  std::string target_frame_;
  std::string base_frame_;
  std::string ball_topic_;
  std::string approach_pose_topic_;
  std::string tracking_pose_topic_;
  std::string goal_pose_topic_;
  std::string robot_namespaces_csv_;
  std::string robot_odom_topic_template_;
  std::string robot_marker_topic_template_;
  std::string cmd_vel_topic_;
  std::string motion_servo_cmd_topic_;
  std::string costmap_topic_;
  std::string path_topic_;
  std::string local_trajectory_topic_;
  std::string odom_topic_;
  std::string field_marker_topic_;
  std::string ball_marker_topic_;
  std::string approach_marker_topic_;
  std::string tracking_marker_topic_;
  std::string path_marker_topic_;
  std::string goal_marker_topic_;
  std::string costmap_marker_topic_;
  std::string command_marker_topic_;
  std::string status_marker_topic_;
  std::string team_a_kick_topic_;
  std::string team_b_kick_topic_;
  bool show_field_boundary_{true};
  bool show_ball_marker_{true};
  bool show_goal_markers_{true};
  bool show_ego_robot_marker_{true};
  bool show_other_robot_markers_{true};
  bool show_robot_collision_ellipses_{true};
  double collision_ellipse_expansion_m_{0.05};
  std::string team_a_striker_topic_;
  std::string team_b_striker_topic_;
  double ego_robot_length_m_{0.562};
  double ego_robot_width_m_{0.339};
  double ego_robot_height_m_{0.481};
  double goal_width_m_{1.2};
  double goal_post_height_m_{0.45};
  double field_center_x_{0.0};
  double field_center_y_{0.0};
  double field_length_m_{16.0};
  double field_width_m_{8.0};
  double center_circle_radius_m_{0.75};

  double publish_rate_hz_;
  double stale_timeout_sec_;
  double marker_lifetime_sec_;
  bool use_delete_all_before_publish_{false};
  double velocity_arrow_scale_;
  double other_robot_length_m_;
  double other_robot_width_m_;
  double other_robot_height_m_;
  double z_offset_;
  double command_panel_x_{-3.0};
  double command_panel_y_{-5.0};
  double status_panel_x_{1.0};
  double status_panel_y_{-5.0};
  bool enable_costmap_markers_{true};
  int costmap_min_cost_{1};
  bool expect_motion_cmds_{false};
  std::string field_frame_;
  std::string self_namespace_;
  std::string keyboard_robot_namespace_;
  std::string team_id_;
  std::vector<std::string> ego_base_frames_;

  std::mutex mutex_;

  bool have_ball_{false};
  bool have_approach_{false};
  bool have_tracking_{false};
  bool have_goal_{false};
  bool have_cmd_vel_{false};
  bool have_motion_servo_cmd_{false};
  bool have_costmap_{false};
  bool have_path_{false};
  bool have_local_trajectory_{false};
  bool have_odom_{false};
  bool have_team_a_kick_{false};
  bool have_team_b_kick_{false};
  bool have_striker_a_{false};
  bool have_striker_b_{false};
  std::size_t costmap_obstacle_cell_count_{0};

  std::string striker_a_;
  std::string striker_b_;

  geometry_msgs::msg::PoseStamped team_a_kick_default_;
  geometry_msgs::msg::PoseStamped team_b_kick_default_;
  geometry_msgs::msg::PoseStamped team_a_kick_;
  geometry_msgs::msg::PoseStamped team_b_kick_;
  geometry_msgs::msg::PoseStamped ball_pose_;
  geometry_msgs::msg::PoseStamped approach_pose_;
  geometry_msgs::msg::PoseStamped tracking_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  std::vector<std::string> robot_namespaces_;
  std::map<std::string, nav_msgs::msg::Odometry> robot_odoms_;
  std::map<std::string, rclcpp::Time> robot_odom_times_;
  geometry_msgs::msg::Twist cmd_vel_;
  protocol::msg::MotionServoCmd motion_servo_cmd_;
  nav_msgs::msg::OccupancyGrid costmap_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path local_trajectory_;
  nav_msgs::msg::Odometry odom_;

  rclcpp::Time ball_time_;
  rclcpp::Time approach_time_;
  rclcpp::Time tracking_time_;
  rclcpp::Time goal_time_;
  rclcpp::Time cmd_vel_time_;
  rclcpp::Time motion_servo_cmd_time_;
  rclcpp::Time costmap_time_;
  rclcpp::Time path_time_;
  rclcpp::Time local_trajectory_time_;
  rclcpp::Time odom_time_;
  rclcpp::Time team_a_kick_time_;
  rclcpp::Time team_b_kick_time_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr field_marker_pub_;
  std::map<std::string,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> robot_marker_pubs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ball_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr approach_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracking_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr costmap_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr command_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr status_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ball_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr approach_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_odom_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<protocol::msg::MotionServoCmd>::SharedPtr motion_servo_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr team_a_kick_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr team_b_kick_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_a_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr striker_b_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace football_navigation
