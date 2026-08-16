// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#ifndef FOOTBALL_NAVIGATION__MULTI_ROBOT_OBSTACLE_LAYER_HPP_
#define FOOTBALL_NAVIGATION__MULTI_ROBOT_OBSTACLE_LAYER_HPP_

#include <cstddef>
#include <mutex>
#include <map>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"

#include "football_navigation/plugins/dynamic_obstacle_marker.hpp"

namespace football_navigation
{

class MultiRobotObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  MultiRobotObstacleLayer();
  ~MultiRobotObstacleLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  struct IndexedObstacle
  {
    std::size_t index{0};
    DynamicObstacle obstacle;
  };

  struct ObstacleTrack
  {
    bool valid{false};
    bool velocity_valid{false};
    DynamicObstacle obstacle;
    rclcpp::Time stamp;
    double vx{0.0};
    double vy{0.0};
    double yaw_rate{0.0};
  };

  void robotOdomCallback(
    const std::string & robot_namespace, std::size_t source_index,
    const nav_msgs::msg::Odometry::SharedPtr msg);
  bool validPose(const geometry_msgs::msg::Pose & pose) const;
  bool stampAcceptable(const rclcpp::Time & stamp) const;
  bool makeObstacleFromPose(
    const geometry_msgs::msg::Pose & pose,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    DynamicObstacle & output) const;
  void updateTracks(
    const std::vector<IndexedObstacle> & observations,
    std::size_t input_size,
    const rclcpp::Time & stamp);
  void appendPredictedSweep(
    const ObstacleTrack & track,
    const rclcpp::Time & now,
    std::vector<DynamicObstacle> & output) const;
  void touchObstacleBounds(
    const DynamicObstacle & obstacle,
    double * min_x, double * min_y, double * max_x, double * max_y);
  void logMasterGridDiagnostics(
    const nav2_costmap_2d::Costmap2D & master_grid,
    unsigned int marked_cells,
    const DynamicObstacle * sample_obstacle,
    unsigned int sample_mx,
    unsigned int sample_my,
    unsigned char sample_cost);
  static double normalizeAngle(double angle);

  bool rolling_window_{false};
  bool use_oriented_footprint_{true};
  double obstacle_radius_{0.55};
  double footprint_padding_{0.04};
  double other_robot_length_m_{0.603};
  double other_robot_width_m_{0.339};
  double data_timeout_{0.70};
  int minimum_obstacle_count_{1};
  double max_message_age_{0.50};
  double future_tolerance_{0.08};
  double transform_tolerance_sec_{0.25};

  bool enable_prediction_{true};
  double prediction_horizon_sec_{0.90};
  double max_prediction_distance_m_{0.40};
  double min_prediction_speed_mps_{0.03};
  double velocity_filter_alpha_{0.45};
  double min_velocity_dt_sec_{0.04};
  double max_velocity_dt_sec_{0.50};
  double max_obstacle_speed_mps_{1.00};
  double max_obstacle_yaw_rate_rps_{2.00};
  double max_observation_jump_m_{0.80};
  double sweep_linear_step_m_{0.08};
  double sweep_angular_step_rad_{0.15};
  int max_sweep_samples_{20};

  unsigned char mark_cost_{254};
  bool use_maximum_{true};
  std::string global_frame_;
  std::string target_frame_;
  std::string self_namespace_;
  std::string robot_namespaces_csv_;
  std::string robot_odom_topic_template_;
  std::vector<std::string> robot_namespaces_;
  std::vector<geometry_msgs::msg::Point> other_robot_footprint_;

  std::mutex data_mutex_;
  std::vector<ObstacleTrack> obstacle_tracks_;
  std::vector<DynamicObstacle> obstacles_to_mark_;
  std::vector<DynamicObstacle> previous_obstacles_;
  rclcpp::Time last_pose_array_stamp_;
  rclcpp::Time last_data_received_time_;
  rclcpp::Time last_master_grid_diagnostics_time_;
  unsigned long long update_bounds_calls_{0};
  unsigned long long update_costs_calls_{0};
  unsigned long long diagnostics_bounds_calls_{0};
  unsigned long long diagnostics_costs_calls_{0};
  std::size_t last_input_count_{0};
  std::size_t last_accepted_count_{0};
  std::size_t last_rejected_invalid_count_{0};
  std::size_t last_rejected_transform_count_{0};
  double last_update_bounds_ms_{0.0};
  double last_update_costs_ms_{0.0};
  bool have_received_data_{false};
  std::map<std::string, rclcpp::Time> robot_odom_times_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_odom_subs_;
};

}  // namespace football_navigation

#endif  // FOOTBALL_NAVIGATION__MULTI_ROBOT_OBSTACLE_LAYER_HPP_
