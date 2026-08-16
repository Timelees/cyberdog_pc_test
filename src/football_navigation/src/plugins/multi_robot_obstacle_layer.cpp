// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include "football_navigation/multi_robot_obstacle_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "dwb_core/trajectory_critic.hpp"
#include "football_navigation/football_geometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

PLUGINLIB_EXPORT_CLASS(
    football_navigation::MultiRobotObstacleLayer,
    nav2_costmap_2d::Layer)

namespace football_navigation
{

  class FootballPreferForwardCritic : public dwb_core::TrajectoryCritic
  {
  public:
    void onInit() override
    {
      const auto node = node_.lock();
      if (!node)
      {
        throw std::runtime_error("FootballPreferForwardCritic node unavailable");
      }

      const std::string prefix = dwb_plugin_name_ + "." + name_ + ".";
      nav2_util::declare_parameter_if_not_declared(
          node, prefix + "penalty", rclcpp::ParameterValue(8.0));
      nav2_util::declare_parameter_if_not_declared(
          node, prefix + "forward_target", rclcpp::ParameterValue(0.38));
      nav2_util::declare_parameter_if_not_declared(
          node, prefix + "lateral_scale", rclcpp::ParameterValue(4.0));
      nav2_util::declare_parameter_if_not_declared(
          node, prefix + "theta_scale", rclcpp::ParameterValue(0.5));
      nav2_util::declare_parameter_if_not_declared(
          node, prefix + "wrong_turn_penalty", rclcpp::ParameterValue(0.5));
      node->get_parameter(prefix + "penalty", penalty_);
      node->get_parameter(prefix + "forward_target", forward_target_);
      node->get_parameter(prefix + "lateral_scale", lateral_scale_);
      node->get_parameter(prefix + "theta_scale", theta_scale_);
      node->get_parameter(prefix + "wrong_turn_penalty", wrong_turn_penalty_);
      if (!std::isfinite(penalty_) || penalty_ < 0.0 ||
          !std::isfinite(forward_target_) || forward_target_ <= 0.0 ||
          !std::isfinite(lateral_scale_) || lateral_scale_ < 0.0 ||
          !std::isfinite(theta_scale_) || theta_scale_ < 0.0 ||
          !std::isfinite(wrong_turn_penalty_) || wrong_turn_penalty_ < 0.0)
      {
        throw std::invalid_argument("invalid FootballPreferForwardCritic parameters");
      }
    }

    bool prepare(
      const geometry_msgs::msg::Pose2D & pose,
      const nav_2d_msgs::msg::Twist2D &,
      const geometry_msgs::msg::Pose2D & goal,
      const nav_2d_msgs::msg::Path2D &) override
    {
      const double goal_dx = goal.x - pose.x;
      const double goal_dy = goal.y - pose.y;
      goal_forward_m_ =
        std::cos(pose.theta) * goal_dx + std::sin(pose.theta) * goal_dy;
      goal_yaw_error_ = std::atan2(
        std::sin(goal.theta - pose.theta),
        std::cos(goal.theta - pose.theta));
      prefer_forward_ = goal_forward_m_ > 0.05;
      return true;
    }

    double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) override
    {
      double score = 0.0;
      if (std::fabs(goal_yaw_error_) > 0.08 &&
        traj.velocity.theta * goal_yaw_error_ < -1e-4)
      {
        score = wrong_turn_penalty_;
      }
      if (!prefer_forward_)
      {
        return score;
      }
      if (traj.velocity.x < 0.0)
      {
        return score + penalty_;
      }
      const double forward_deficit =
          std::max(0.0, forward_target_ - traj.velocity.x) / forward_target_;
      return score + forward_deficit * penalty_ +
             std::fabs(traj.velocity.y) * lateral_scale_ +
             std::fabs(traj.velocity.theta) * theta_scale_;
    }

  private:
    double penalty_{8.0};
    double forward_target_{0.38};
    double lateral_scale_{4.0};
    double theta_scale_{0.5};
    double wrong_turn_penalty_{0.5};
    double goal_forward_m_{0.0};
    double goal_yaw_error_{0.0};
    bool prefer_forward_{false};
  };

  class FootballProgressChecker : public nav2_core::ProgressChecker
  {
  public:
    void initialize(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &plugin_name) override
    {
      plugin_name_ = plugin_name;
      const auto node = parent.lock();
      if (!node)
      {
        throw std::runtime_error(
            "FootballProgressChecker failed to lock lifecycle node");
      }

      clock_ = node->get_clock();
      nav2_util::declare_parameter_if_not_declared(
          node, plugin_name_ + ".required_movement_radius",
          rclcpp::ParameterValue(0.08));
      nav2_util::declare_parameter_if_not_declared(
          node, plugin_name_ + ".movement_time_allowance",
          rclcpp::ParameterValue(4.0));

      node->get_parameter_or(
          plugin_name_ + ".required_movement_radius",
          movement_radius_m_, 0.08);
      double allowance_sec = 4.0;
      node->get_parameter_or(
          plugin_name_ + ".movement_time_allowance",
          allowance_sec, 4.0);

      if (!std::isfinite(movement_radius_m_) || movement_radius_m_ <= 0.0 ||
          !std::isfinite(allowance_sec) || allowance_sec <= 0.0)
      {
        throw std::invalid_argument(
            "invalid FootballProgressChecker parameters");
      }

      movement_time_allowance_ =
          rclcpp::Duration::from_seconds(allowance_sec);
      reset();
    }

    bool check(geometry_msgs::msg::PoseStamped &current_pose) override
    {
      const auto &orientation = current_pose.pose.orientation;
      const double quaternion_norm =
          orientation.x * orientation.x + orientation.y * orientation.y +
          orientation.z * orientation.z + orientation.w * orientation.w;
      const double current_yaw = tf2::getYaw(orientation);
      if (!std::isfinite(current_pose.pose.position.x) ||
          !std::isfinite(current_pose.pose.position.y) ||
          !std::isfinite(quaternion_norm) || quaternion_norm <= 1.0e-8 ||
          !std::isfinite(current_yaw))
      {
        return false;
      }

      if (!baseline_set_ || progressed(current_pose))
      {
        baseline_x_ = current_pose.pose.position.x;
        baseline_y_ = current_pose.pose.position.y;
        baseline_time_ = clock_->now();
        baseline_set_ = true;
        return true;
      }

      return (clock_->now() - baseline_time_) <= movement_time_allowance_;
    }

    void reset() override
    {
      baseline_set_ = false;
    }

  private:
    bool progressed(const geometry_msgs::msg::PoseStamped &pose) const
    {
      const double translation = std::hypot(
          pose.pose.position.x - baseline_x_,
          pose.pose.position.y - baseline_y_);
      return translation >= movement_radius_m_;
    }

    std::string plugin_name_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Duration movement_time_allowance_{
        rclcpp::Duration::from_seconds(4.0)};
    double movement_radius_m_{0.08};
    double baseline_x_{0.0};
    double baseline_y_{0.0};
    rclcpp::Time baseline_time_;
    bool baseline_set_{false};
  };

  class StableComputePathToPoseAction
      : public nav2_behavior_tree::BtActionNode<
            nav2_msgs::action::ComputePathToPose>
  {
  public:
    StableComputePathToPoseAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &configuration)
        : nav2_behavior_tree::BtActionNode<
              nav2_msgs::action::ComputePathToPose>(
              xml_tag_name,
              action_name,
              configuration)
    {
    }

    void on_tick() override
    {
      getInput(
          "goal",
          goal_.goal);

      getInput(
          "planner_id",
          goal_.planner_id);

      goal_.use_start = false;

      if (getInput(
              "start",
              goal_.start))
      {
        goal_.use_start = true;
      }

      /*
       * BtActionNode sends the first Goal immediately after on_tick().
       *
       * The upstream Galactic ComputePathToP implementation sets
       * goal_updated_ here, which makes BtActionNode send the same
       * Goal again immediately after the first handle is accepted.
       *
       * Keep it false. Later replans are initiated by the outer
       * RateController, not by preempting an unfinished planner
       * request.
       */
      goal_updated_ = false;
    }

    void on_wait_for_result() override
    {
      /*
       * Deliberately do not preempt an in-flight planning request.
       *
       * The newest target remains on the Behavior Tree blackboard
       * and is consumed at the next bounded replanning tick.
       */
    }

    BT::NodeStatus on_success() override
    {
      setOutput(
          "path",
          result_.result->path);

      return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
          {
              BT::OutputPort<nav_msgs::msg::Path>(
                  "path",
                  "Path created by the Football stable planner action"),

              BT::InputPort<geometry_msgs::msg::PoseStamped>(
                  "goal",
                  "Destination to plan to"),

              BT::InputPort<geometry_msgs::msg::PoseStamped>(
                  "start",
                  "Optional explicit start pose"),

              BT::InputPort<std::string>(
                  "planner_id",
                  ""),
          });
    }
  };

  class StableFollowPathAction
      : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>
  {
  public:
    StableFollowPathAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &configuration)
        : nav2_behavior_tree::BtActionNode<nav2_msgs::action::FollowPath>(
              xml_tag_name, action_name, configuration)
    {
      getInput("path_xy_update_tolerance", path_xy_tolerance_m_);
      getInput("path_yaw_update_tolerance", path_yaw_tolerance_rad_);
      getInput("path_length_update_tolerance", path_length_tolerance_m_);
      getInput("path_update_min_interval_sec", path_update_min_interval_sec_);
      path_xy_tolerance_m_ = std::max(1.0e-3, path_xy_tolerance_m_);
      path_yaw_tolerance_rad_ = std::max(1.0e-3, path_yaw_tolerance_rad_);
      path_length_tolerance_m_ = std::max(1.0e-3, path_length_tolerance_m_);
      path_update_min_interval_sec_ = std::max(0.0, path_update_min_interval_sec_);
    }

    void on_tick() override
    {
      getInput("path", goal_.path);
      getInput("goal", goal_.goal);
      getInput("controller_id", goal_.controller_id);
      getInput("goal_checker_id", goal_.goal_checker_id);
      getInput("progress_checker_id", goal_.progress_checker_id);
      goal_updated_ = false;
      last_goal_update_time_ = node_->now();
    }

    void on_wait_for_result() override
    {
      const auto now = node_->now();
      if (
        now >= last_goal_update_time_ &&
        (now - last_goal_update_time_).seconds() < path_update_min_interval_sec_)
      {
        return;
      }

      nav_msgs::msg::Path new_path;
      if (getInput("path", new_path) &&
          !new_path.poses.empty() &&
          pathGeometryChanged(goal_.path, new_path))
      {
        goal_.path = new_path;
        int exception_code = nav2_core::NOEXCEPTION;
        config().blackboard->get<int>("exception_code", exception_code);
        if (exception_code == nav2_core::CONTROLLEREXECPTION)
        {
          config().blackboard->set<int>(
              "exception_code", nav2_core::NOEXCEPTION);
        }
        goal_updated_ = true;
      }

      geometry_msgs::msg::PoseStamped new_goal;
      if (getInput("goal", new_goal) &&
          poseGeometryChanged(goal_.goal, new_goal))
      {
        goal_.goal = new_goal;
        goal_updated_ = true;
      }

      updateStringPort("controller_id", goal_.controller_id);
      updateStringPort("goal_checker_id", goal_.goal_checker_id);
      updateStringPort("progress_checker_id", goal_.progress_checker_id);
      if (goal_updated_)
      {
        last_goal_update_time_ = now;
      }
    }

    BT::NodeStatus on_aborted() override
    {
      config().blackboard->set<int>(
          "exception_code", nav2_core::CONTROLLEREXECPTION);
      setOutput("output_exception_code", nav2_core::CONTROLLEREXECPTION);
      return BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts()
    {
      return providedBasicPorts(
          {
              BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
              BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Goal to reach"),
              BT::InputPort<std::string>("controller_id", ""),
              BT::InputPort<std::string>("goal_checker_id", ""),
              BT::InputPort<std::string>("progress_checker_id", ""),
              BT::InputPort<double>(
                  "path_xy_update_tolerance",
                  0.10,
                  "Minimum XY path change required before updating the FollowPath goal"),
              BT::InputPort<double>(
                  "path_yaw_update_tolerance",
                  0.08,
                  "Minimum path yaw change required before updating the FollowPath goal"),
              BT::InputPort<double>(
                  "path_length_update_tolerance",
                  0.18,
                  "Minimum path length change required before updating the FollowPath goal"),
              BT::InputPort<double>(
                  "path_update_min_interval_sec",
                  0.50,
                  "Minimum time between FollowPath goal updates"),
              BT::OutputPort<unsigned int>("output_exception_code", ""),
          });
    }

  private:
    static double pathLength(
        const nav_msgs::msg::Path &path,
        const std::size_t start_index = 0)
    {
      if (path.poses.size() < 2 || start_index >= path.poses.size() - 1)
      {
        return 0.0;
      }
      double total = 0.0;
      for (std::size_t index = start_index + 1;
           index < path.poses.size(); ++index)
      {
        total += std::hypot(
            path.poses[index].pose.position.x -
                path.poses[index - 1].pose.position.x,
            path.poses[index].pose.position.y -
                path.poses[index - 1].pose.position.y);
      }
      return total;
    }

    static double pointSegmentDistance(
        const double point_x,
        const double point_y,
        const double start_x,
        const double start_y,
        const double end_x,
        const double end_y)
    {
      const double delta_x = end_x - start_x;
      const double delta_y = end_y - start_y;
      const double norm_squared = delta_x * delta_x + delta_y * delta_y;
      if (norm_squared <= 1.0e-12)
      {
        return std::hypot(point_x - start_x, point_y - start_y);
      }
      const double projection = std::clamp(
          ((point_x - start_x) * delta_x +
           (point_y - start_y) * delta_y) /
              norm_squared,
          0.0, 1.0);
      return std::hypot(
          point_x - (start_x + projection * delta_x),
          point_y - (start_y + projection * delta_y));
    }

    static double pointToPathDistance(
        const geometry_msgs::msg::PoseStamped &point,
        const nav_msgs::msg::Path &path,
        const std::size_t start_index = 0)
    {
      if (path.poses.empty() || start_index >= path.poses.size())
      {
        return std::numeric_limits<double>::infinity();
      }
      if (start_index == path.poses.size() - 1)
      {
        return std::hypot(
            point.pose.position.x - path.poses.back().pose.position.x,
            point.pose.position.y - path.poses.back().pose.position.y);
      }

      double minimum = std::numeric_limits<double>::infinity();
      for (std::size_t index = start_index + 1;
           index < path.poses.size(); ++index)
      {
        minimum = std::min(
            minimum,
            pointSegmentDistance(
                point.pose.position.x,
                point.pose.position.y,
                path.poses[index - 1].pose.position.x,
                path.poses[index - 1].pose.position.y,
                path.poses[index].pose.position.x,
                path.poses[index].pose.position.y));
      }
      return minimum;
    }

    static std::size_t closestPathIndex(
        const geometry_msgs::msg::PoseStamped &point,
        const nav_msgs::msg::Path &path)
    {
      std::size_t best_index = 0;
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < path.poses.size(); ++index)
      {
        const double candidate = std::hypot(
            point.pose.position.x - path.poses[index].pose.position.x,
            point.pose.position.y - path.poses[index].pose.position.y);
        if (candidate < best_distance)
        {
          best_distance = candidate;
          best_index = index;
        }
      }
      return best_index;
    }

    bool poseGeometryChanged(
        const geometry_msgs::msg::PoseStamped &previous,
        const geometry_msgs::msg::PoseStamped &current) const
    {
      if (previous.header.frame_id != current.header.frame_id)
      {
        return true;
      }
      const double translation = std::hypot(
          previous.pose.position.x - current.pose.position.x,
          previous.pose.position.y - current.pose.position.y);
      const double yaw_delta = std::fabs(
          football_navigation::normalizeAngle(
              tf2::getYaw(previous.pose.orientation) -
              tf2::getYaw(current.pose.orientation)));
      return translation > path_xy_tolerance_m_ ||
             yaw_delta > path_yaw_tolerance_rad_;
    }

    bool pathGeometryChanged(
        const nav_msgs::msg::Path &previous,
        const nav_msgs::msg::Path &current) const
    {
      if (previous.header.frame_id != current.header.frame_id ||
          previous.poses.empty() != current.poses.empty())
      {
        return true;
      }
      if (previous.poses.empty())
      {
        return false;
      }

      const std::size_t previous_suffix_start = closestPathIndex(
          current.poses.front(), previous);
      if (pointToPathDistance(
              current.poses.front(), previous, previous_suffix_start) >
          path_xy_tolerance_m_)
      {
        return true;
      }
      if (std::hypot(
              previous.poses.back().pose.position.x -
                  current.poses.back().pose.position.x,
              previous.poses.back().pose.position.y -
                  current.poses.back().pose.position.y) > path_xy_tolerance_m_)
      {
        return true;
      }
      if (std::fabs(
              pathLength(previous, previous_suffix_start) - pathLength(current)) >
          path_length_tolerance_m_)
      {
        return true;
      }

      constexpr std::size_t sample_count = 11;
      for (std::size_t sample = 0; sample < sample_count; ++sample)
      {
        const double ratio = static_cast<double>(sample) /
                             static_cast<double>(sample_count - 1);
        const std::size_t current_index = static_cast<std::size_t>(std::llround(
            ratio * static_cast<double>(current.poses.size() - 1)));
        if (pointToPathDistance(
                current.poses[current_index], previous, previous_suffix_start) >
            path_xy_tolerance_m_)
        {
          return true;
        }

        const std::size_t previous_index = previous_suffix_start +
                                           static_cast<std::size_t>(std::llround(
                                               ratio * static_cast<double>(
                                                           previous.poses.size() - 1 - previous_suffix_start)));
        if (pointToPathDistance(
                previous.poses[previous_index], current) > path_xy_tolerance_m_)
        {
          return true;
        }
      }
      return false;
    }

    void updateStringPort(
        const std::string &port_name,
        std::string &stored_value)
    {
      std::string new_value;
      if (getInput(port_name, new_value) && new_value != stored_value)
      {
        stored_value = new_value;
        goal_updated_ = true;
      }
    }

    double path_xy_tolerance_m_{0.10};
    double path_yaw_tolerance_rad_{0.08};
    double path_length_tolerance_m_{0.18};
    double path_update_min_interval_sec_{0.50};
    rclcpp::Time last_goal_update_time_;
  };

  /**
   * Football-specific target updater.
   *
   * The upstream CyberDog TargetUpdater rewrites BEHIND-mode orientation to
   * atan2(relative_y, relative_x). That is correct for person following, but
   * wrong for football: once the robot reaches the ball-behind point the
   * relative vector approaches zero, so the requested goal yaw becomes noisy
   * and rotates with base_link. This decorator transforms the user-owned
   * football target into the stable global frame while preserving its yaw.
   */
  class FootballTargetUpdater : public BT::DecoratorNode
  {
  public:
    FootballTargetUpdater(
        const std::string &name,
        const BT::NodeConfiguration &configuration)
        : BT::DecoratorNode(name, configuration)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      nav2_util::declare_parameter_if_not_declared(
          node_, "global_frame", rclcpp::ParameterValue("map"));
      node_->get_parameter("global_frame", global_frame_);

      nav2_util::declare_parameter_if_not_declared(
          node_, "goal_updater_topic", rclcpp::ParameterValue("tracking_pose"));
      node_->get_parameter("goal_updater_topic", goal_updater_topic_);

      nav2_util::declare_parameter_if_not_declared(
          node_, "overtime", rclcpp::ParameterValue(2.5));
      node_->get_parameter("overtime", overtime_sec_);

      nav2_util::declare_parameter_if_not_declared(
          node_, "dist_throttle", rclcpp::ParameterValue(0.02));
      node_->get_parameter("dist_throttle", distance_throttle_m_);

      nav2_util::declare_parameter_if_not_declared(
          node_, "football_yaw_throttle_rad", rclcpp::ParameterValue(0.03));
      node_->get_parameter("football_yaw_throttle_rad", yaw_throttle_rad_);

      nav2_util::declare_parameter_if_not_declared(
          node_, "max_target_distance_m", rclcpp::ParameterValue(4.0));
      node_->get_parameter("max_target_distance_m", max_target_distance_m_);

      nav2_util::declare_parameter_if_not_declared(
          node_, "football_target_startup_grace_sec", rclcpp::ParameterValue(2.0));
      node_->get_parameter(
          "football_target_startup_grace_sec", startup_grace_sec_);

      if (!std::isfinite(overtime_sec_) || overtime_sec_ <= 0.0 ||
          !std::isfinite(distance_throttle_m_) || distance_throttle_m_ < 0.0 ||
          !std::isfinite(yaw_throttle_rad_) || yaw_throttle_rad_ < 0.0 ||
          !std::isfinite(max_target_distance_m_) || max_target_distance_m_ <= 0.0 ||
          !std::isfinite(startup_grace_sec_) || startup_grace_sec_ <= 0.0 ||
          startup_grace_sec_ >= overtime_sec_)
      {
        throw std::invalid_argument("invalid FootballTargetUpdater parameters");
      }

      callback_group_ = node_->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive, false);
      callback_executor_.add_callback_group(
          callback_group_, node_->get_node_base_interface());

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
          node_->get_node_base_interface(), node_->get_node_timers_interface());
      tf_buffer_->setCreateTimerInterface(timer_interface);
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group_;
      target_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_updater_topic_, rclcpp::QoS(1).reliable(),
          std::bind(
              &FootballTargetUpdater::targetCallback,
              this,
              std::placeholders::_1),
          options);
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
          BT::InputPort<unsigned char>("input_tracking_mode", "Ignored in football mode"),
          BT::OutputPort<double>("distance", "Target distance from robot"),
          BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Stable global goal"),
          BT::OutputPort<geometry_msgs::msg::PoseStamped>(
              "transformed_goal", "Stable global goal preserving football yaw"),
          BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
              "output_goals", "Single stable football goal"),
          BT::OutputPort<unsigned int>("output_exception_code", "Failure reason"),
      };
    }

  private:
    static bool finitePose(const geometry_msgs::msg::Pose &pose)
    {
      const auto &q = pose.orientation;
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      return std::isfinite(pose.position.x) &&
             std::isfinite(pose.position.y) &&
             std::isfinite(pose.position.z) &&
             std::isfinite(q.x) && std::isfinite(q.y) &&
             std::isfinite(q.z) && std::isfinite(q.w) && norm > 1e-8;
    }

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      if (!msg || msg->header.frame_id.empty() || !finitePose(msg->pose))
      {
        return;
      }

      const rclcpp::Time source_stamp(msg->header.stamp, node_->get_clock()->get_clock_type());
      const double age = source_stamp.nanoseconds() > 0 ? (node_->now() - source_stamp).seconds() : std::numeric_limits<double>::infinity();
      if (source_stamp.nanoseconds() <= 0 || age > overtime_sec_ || age < -0.08)
      {
        return;
      }

      geometry_msgs::msg::PoseStamped bounded = *msg;
      const double relative_distance = std::hypot(
          bounded.pose.position.x, bounded.pose.position.y);
      if (!std::isfinite(relative_distance))
      {
        return;
      }
      if (relative_distance > max_target_distance_m_)
      {
        const double scale = max_target_distance_m_ / relative_distance;
        bounded.pose.position.x *= scale;
        bounded.pose.position.y *= scale;
      }

      geometry_msgs::msg::PoseStamped global_goal;
      if (!nav2_util::transformPoseInTargetFrame(
              bounded, global_goal, *tf_buffer_, global_frame_, 1.2))
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "FootballTargetUpdater failed to transform %s to %s",
            bounded.header.frame_id.c_str(), global_frame_.c_str());
        return;
      }
      global_goal.header.stamp = node_->now();
      global_goal.pose.position.z = 0.0;

      std::lock_guard<std::mutex> lock(mutex_);
      if (have_goal_)
      {
        const double dx = global_goal.pose.position.x - latest_global_goal_.pose.position.x;
        const double dy = global_goal.pose.position.y - latest_global_goal_.pose.position.y;
        const double yaw_delta = std::fabs(
            football_navigation::normalizeAngle(
                tf2::getYaw(global_goal.pose.orientation) -
                tf2::getYaw(latest_global_goal_.pose.orientation)));
        if (std::hypot(dx, dy) < distance_throttle_m_ &&
            yaw_delta < yaw_throttle_rad_)
        {
          latest_receive_time_ = node_->now();
          relative_distance_m_ = relative_distance;
          return;
        }
      }

      latest_global_goal_ = global_goal;
      relative_distance_m_ = relative_distance;
      latest_receive_time_ = node_->now();
      have_goal_ = true;
    }

    BT::NodeStatus tick() override
    {
      callback_executor_.spin_some();
      std::lock_guard<std::mutex> lock(mutex_);

      const rclcpp::Time tick_time = node_->now();
      if (wait_started_.nanoseconds() <= 0)
      {
        wait_started_ = tick_time;
      }

      setOutput("output_exception_code", nav2_core::NOEXCEPTION);
      if (!have_goal_ || latest_receive_time_.nanoseconds() <= 0)
      {
        // The BT-local subscription is created only after TargetTracking has
        // accepted the action. Keep the tree alive long enough to receive the
        // next bounded planner-target update instead of failing and destroying
        // the subscription in the same DDS discovery race on every retry.
        if ((tick_time - wait_started_).seconds() <= startup_grace_sec_)
        {
          config().blackboard->set<int>(
              "exception_code", static_cast<int>(nav2_core::NOEXCEPTION));
          return BT::NodeStatus::RUNNING;
        }
        config().blackboard->set<int>(
            "exception_code", static_cast<int>(nav2_core::DETECTOREXCEPTION));
        setOutput("output_exception_code", nav2_core::DETECTOREXCEPTION);
        return BT::NodeStatus::FAILURE;
      }

      if ((tick_time - latest_receive_time_).seconds() > overtime_sec_)
      {
        config().blackboard->set<int>(
            "exception_code", static_cast<int>(nav2_core::DETECTOREXCEPTION));
        setOutput("output_exception_code", nav2_core::DETECTOREXCEPTION);
        return BT::NodeStatus::FAILURE;
      }

      config().blackboard->set<float>(
          "distance", static_cast<float>(relative_distance_m_));
      config().blackboard->set<int>(
          "exception_code", static_cast<int>(nav2_core::NOEXCEPTION));

      setOutput("distance", relative_distance_m_);
      setOutput("output_goal", latest_global_goal_);
      setOutput("transformed_goal", latest_global_goal_);
      setOutput(
          "output_goals",
          std::vector<geometry_msgs::msg::PoseStamped>{latest_global_goal_});

      const BT::NodeStatus status = child_node_->executeTick();
      if (status == BT::NodeStatus::FAILURE)
      {
        config().blackboard->set<int>(
            "exception_code", static_cast<int>(nav2_core::PLANNEREXECPTION));
        setOutput("output_exception_code", nav2_core::PLANNEREXECPTION);
      }
      return status;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_executor_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex mutex_;
    std::string global_frame_;
    std::string goal_updater_topic_;
    double overtime_sec_{2.5};
    double distance_throttle_m_{0.02};
    double yaw_throttle_rad_{0.03};
    double max_target_distance_m_{4.0};
    double startup_grace_sec_{2.0};
    double relative_distance_m_{0.0};
    bool have_goal_{false};
    geometry_msgs::msg::PoseStamped latest_global_goal_;
    rclcpp::Time latest_receive_time_;
    rclcpp::Time wait_started_;
  };

  MultiRobotObstacleLayer::MultiRobotObstacleLayer()
  {
    costmap_ = nullptr;
  }

  double MultiRobotObstacleLayer::normalizeAngle(const double angle)
  {
    return football_navigation::normalizeAngle(angle);
  }

  void MultiRobotObstacleLayer::onInitialize()
  {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("target_frame", rclcpp::ParameterValue(std::string("")));
    declareParameter(
        "pose_array_topic",
        rclcpp::ParameterValue(std::string("football/other_robot_poses")));
    declareParameter("obstacle_radius", rclcpp::ParameterValue(0.55));
    declareParameter("use_oriented_footprint", rclcpp::ParameterValue(true));
    declareParameter("footprint_padding", rclcpp::ParameterValue(0.04));
    declareParameter("other_robot_length_m", rclcpp::ParameterValue(0.603));
    declareParameter("other_robot_width_m", rclcpp::ParameterValue(0.339));
    declareParameter("obstacle_timeout_sec", rclcpp::ParameterValue(0.70));
    declareParameter("minimum_obstacle_count", rclcpp::ParameterValue(1));
    declareParameter("pose_array_includes_self", rclcpp::ParameterValue(false));
    declareParameter("self_filter_radius", rclcpp::ParameterValue(0.05));
    declareParameter("max_message_age", rclcpp::ParameterValue(0.50));
    declareParameter("future_tolerance", rclcpp::ParameterValue(0.08));
    declareParameter("transform_tolerance_sec", rclcpp::ParameterValue(0.25));
    declareParameter("enable_prediction", rclcpp::ParameterValue(true));
    declareParameter("prediction_horizon_sec", rclcpp::ParameterValue(0.90));
    declareParameter("max_prediction_distance_m", rclcpp::ParameterValue(0.40));
    declareParameter("min_prediction_speed_mps", rclcpp::ParameterValue(0.03));
    declareParameter("velocity_filter_alpha", rclcpp::ParameterValue(0.45));
    declareParameter("min_velocity_dt_sec", rclcpp::ParameterValue(0.04));
    declareParameter("max_velocity_dt_sec", rclcpp::ParameterValue(0.50));
    declareParameter("max_obstacle_speed_mps", rclcpp::ParameterValue(1.00));
    declareParameter("max_obstacle_yaw_rate_rps", rclcpp::ParameterValue(2.00));
    declareParameter("max_observation_jump_m", rclcpp::ParameterValue(0.80));
    declareParameter("sweep_linear_step_m", rclcpp::ParameterValue(0.08));
    declareParameter("sweep_angular_step_rad", rclcpp::ParameterValue(0.15));
    declareParameter("max_sweep_samples", rclcpp::ParameterValue(20));
    declareParameter("mark_cost", rclcpp::ParameterValue(254));
    declareParameter("use_maximum", rclcpp::ParameterValue(true));

    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error(
          "MultiRobotObstacleLayer failed to lock lifecycle node");
    }

    int cost = 254;
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".target_frame", target_frame_);
    node->get_parameter(name_ + ".pose_array_topic", pose_array_topic_);
    node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);
    node->get_parameter(name_ + ".use_oriented_footprint", use_oriented_footprint_);
    node->get_parameter(name_ + ".footprint_padding", footprint_padding_);
    node->get_parameter(name_ + ".other_robot_length_m", other_robot_length_m_);
    node->get_parameter(name_ + ".other_robot_width_m", other_robot_width_m_);
    node->get_parameter(name_ + ".obstacle_timeout_sec", data_timeout_);
    node->get_parameter(name_ + ".minimum_obstacle_count", minimum_obstacle_count_);
    node->get_parameter(name_ + ".pose_array_includes_self", pose_array_includes_self_);
    node->get_parameter(name_ + ".self_filter_radius", self_filter_radius_);
    node->get_parameter(name_ + ".max_message_age", max_message_age_);
    node->get_parameter(name_ + ".future_tolerance", future_tolerance_);
    node->get_parameter(name_ + ".transform_tolerance_sec", transform_tolerance_sec_);
    node->get_parameter(name_ + ".enable_prediction", enable_prediction_);
    node->get_parameter(name_ + ".prediction_horizon_sec", prediction_horizon_sec_);
    node->get_parameter(name_ + ".max_prediction_distance_m", max_prediction_distance_m_);
    node->get_parameter(name_ + ".min_prediction_speed_mps", min_prediction_speed_mps_);
    node->get_parameter(name_ + ".velocity_filter_alpha", velocity_filter_alpha_);
    node->get_parameter(name_ + ".min_velocity_dt_sec", min_velocity_dt_sec_);
    node->get_parameter(name_ + ".max_velocity_dt_sec", max_velocity_dt_sec_);
    node->get_parameter(name_ + ".max_obstacle_speed_mps", max_obstacle_speed_mps_);
    node->get_parameter(name_ + ".max_obstacle_yaw_rate_rps", max_obstacle_yaw_rate_rps_);
    node->get_parameter(name_ + ".max_observation_jump_m", max_observation_jump_m_);
    node->get_parameter(name_ + ".sweep_linear_step_m", sweep_linear_step_m_);
    node->get_parameter(name_ + ".sweep_angular_step_rad", sweep_angular_step_rad_);
    node->get_parameter(name_ + ".max_sweep_samples", max_sweep_samples_);
    node->get_parameter(name_ + ".mark_cost", cost);
    node->get_parameter(name_ + ".use_maximum", use_maximum_);

    mark_cost_ = static_cast<unsigned char>(std::clamp(cost, 0, 254));
    velocity_filter_alpha_ = std::clamp(velocity_filter_alpha_, 0.0, 1.0);
    prediction_horizon_sec_ = std::max(0.0, prediction_horizon_sec_);
    max_prediction_distance_m_ = std::max(0.0, max_prediction_distance_m_);
    min_prediction_speed_mps_ = std::max(0.0, min_prediction_speed_mps_);
    min_velocity_dt_sec_ = std::max(1.0e-3, min_velocity_dt_sec_);
    max_velocity_dt_sec_ = std::max(min_velocity_dt_sec_, max_velocity_dt_sec_);
    max_obstacle_speed_mps_ = std::max(0.01, max_obstacle_speed_mps_);
    max_obstacle_yaw_rate_rps_ = std::max(0.01, max_obstacle_yaw_rate_rps_);
    max_observation_jump_m_ = std::max(0.05, max_observation_jump_m_);
    sweep_linear_step_m_ = std::max(0.02, sweep_linear_step_m_);
    sweep_angular_step_rad_ = std::max(0.05, sweep_angular_step_rad_);
    max_sweep_samples_ = std::clamp(max_sweep_samples_, 1, 60);
    minimum_obstacle_count_ = std::clamp(minimum_obstacle_count_, 1, 20);
    self_filter_radius_ = std::max(0.0, self_filter_radius_);

    rolling_window_ = layered_costmap_->isRolling();
    global_frame_ = layered_costmap_->getGlobalFrameID();
    if (target_frame_.empty())
    {
      target_frame_ = global_frame_;
    }
    if (target_frame_ != global_frame_)
    {
      throw std::invalid_argument(
          "MultiRobotObstacleLayer target_frame must equal the generated costmap frame");
    }

    if (other_robot_length_m_ > 0.0 && other_robot_width_m_ > 0.0)
    {
      other_robot_footprint_ = makeFootprintFromDimensions(
          other_robot_length_m_, other_robot_width_m_, footprint_padding_);
      for (const auto &point : other_robot_footprint_)
      {
        obstacle_radius_ = std::max(obstacle_radius_, std::hypot(point.x, point.y));
      }
    }

    matchSize();
    current_ = false;
    pose_array_sub_ = rclcpp_node_->create_subscription<geometry_msgs::msg::PoseArray>(
        pose_array_topic_, rclcpp::SensorDataQoS().keep_last(5),
        std::bind(
            &MultiRobotObstacleLayer::poseArrayCallback,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
        logger_,
        "MultiRobotObstacleLayer topic=%s frame=%s prediction=%s horizon=%.2fs "
        "max_distance=%.2fm pose_array_includes_self=%s",
        pose_array_topic_.c_str(), global_frame_.c_str(),
        enable_prediction_ ? "on" : "off",
        prediction_horizon_sec_, max_prediction_distance_m_,
        pose_array_includes_self_ ? "true" : "false");
  }

  void MultiRobotObstacleLayer::activate()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_ = have_received_data_;
  }

  void MultiRobotObstacleLayer::deactivate()
  {
    current_ = false;
  }

  void MultiRobotObstacleLayer::reset()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    obstacle_tracks_.clear();
    obstacles_to_mark_.clear();
    previous_obstacles_.clear();
    last_pose_array_stamp_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    last_data_received_time_ = last_pose_array_stamp_;
    have_received_data_ = false;
    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
    current_ = false;
  }

  bool MultiRobotObstacleLayer::validPose(
      const geometry_msgs::msg::Pose &pose) const
  {
    double yaw = 0.0;
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           yawFromQuaternion(pose.orientation, yaw);
  }

  bool MultiRobotObstacleLayer::stampAcceptable(
      const rclcpp::Time &stamp) const
  {
    if (stamp.nanoseconds() <= 0)
    {
      return false;
    }
    const double age = (clock_->now() - stamp).seconds();
    return age <= max_message_age_ &&
           age >= -future_tolerance_;
  }

  bool MultiRobotObstacleLayer::makeObstacleFromPose(
      const geometry_msgs::msg::Pose &pose,
      const std::string &source_frame,
      const rclcpp::Time &stamp,
      DynamicObstacle &output) const
  {
    if (source_frame.empty() || !validPose(pose))
    {
      return false;
    }

    geometry_msgs::msg::PoseStamped input;
    input.header.frame_id = source_frame;
    input.header.stamp = stamp;
    input.pose = pose;

    geometry_msgs::msg::PoseStamped transformed;
    try
    {
      transformed = tf_->transform(
          input, global_frame_, tf2::durationFromSec(transform_tolerance_sec_));
    }
    catch (const tf2::TransformException &error)
    {
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "MultiRobotObstacleLayer cannot transform %s -> %s: %s",
          source_frame.c_str(), global_frame_.c_str(), error.what());
      return false;
    }

    double yaw = 0.0;
    if (!yawFromQuaternion(transformed.pose.orientation, yaw))
    {
      return false;
    }
    output.x = transformed.pose.position.x;
    output.y = transformed.pose.position.y;
    output.yaw = yaw;
    output.radius = obstacle_radius_;
    return true;
  }

  void MultiRobotObstacleLayer::updateTracks(
      const std::vector<IndexedObstacle> &observations,
      const std::size_t input_size,
      const rclcpp::Time &stamp)
  {
    if (obstacle_tracks_.size() < input_size)
    {
      obstacle_tracks_.resize(input_size);
    }
    for (const auto &observation : observations)
    {
      if (observation.index >= obstacle_tracks_.size())
      {
        continue;
      }
      auto &track = obstacle_tracks_[observation.index];
      bool new_velocity_valid = false;
      double filtered_vx = 0.0;
      double filtered_vy = 0.0;
      double filtered_yaw_rate = 0.0;

      if (track.valid && track.stamp.nanoseconds() > 0)
      {
        const double dt = (stamp - track.stamp).seconds();
        const double dx = observation.obstacle.x - track.obstacle.x;
        const double dy = observation.obstacle.y - track.obstacle.y;
        const double displacement = std::hypot(dx, dy);
        if (dt >= min_velocity_dt_sec_ &&
            dt <= max_velocity_dt_sec_ &&
            displacement <= max_observation_jump_m_)
        {
          double raw_vx = dx / dt;
          double raw_vy = dy / dt;
          const double raw_speed = std::hypot(raw_vx, raw_vy);
          if (raw_speed > max_obstacle_speed_mps_)
          {
            const double scale = max_obstacle_speed_mps_ / raw_speed;
            raw_vx *= scale;
            raw_vy *= scale;
          }
          double raw_yaw_rate = normalizeAngle(
                                    observation.obstacle.yaw - track.obstacle.yaw) /
                                dt;
          raw_yaw_rate = std::clamp(
              raw_yaw_rate,
              -max_obstacle_yaw_rate_rps_,
              max_obstacle_yaw_rate_rps_);

          if (track.velocity_valid)
          {
            filtered_vx = velocity_filter_alpha_ * raw_vx +
                          (1.0 - velocity_filter_alpha_) * track.vx;
            filtered_vy = velocity_filter_alpha_ * raw_vy +
                          (1.0 - velocity_filter_alpha_) * track.vy;
            filtered_yaw_rate = velocity_filter_alpha_ * raw_yaw_rate +
                                (1.0 - velocity_filter_alpha_) * track.yaw_rate;
          }
          else
          {
            filtered_vx = raw_vx;
            filtered_vy = raw_vy;
            filtered_yaw_rate = raw_yaw_rate;
          }
          new_velocity_valid = true;
        }
      }

      track.obstacle = observation.obstacle;
      track.stamp = stamp;
      track.valid = true;
      track.velocity_valid = new_velocity_valid;
      track.vx = new_velocity_valid ? filtered_vx : 0.0;
      track.vy = new_velocity_valid ? filtered_vy : 0.0;
      track.yaw_rate = new_velocity_valid ? filtered_yaw_rate : 0.0;
    }

    // A single malformed or temporarily missing pose must not clear every
    // dynamic obstacle. Unobserved tracks remain valid until data_timeout_.
  }

  void MultiRobotObstacleLayer::poseArrayCallback(
      const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!msg || msg->header.frame_id.empty())
    {
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp, clock_->get_clock_type());
    if (!stampAcceptable(stamp))
    {
      return;
    }

    std::vector<IndexedObstacle> accepted;
    accepted.reserve(std::min<std::size_t>(msg->poses.size(), 20));
    const std::size_t input_size = std::min<std::size_t>(msg->poses.size(), 20);
    std::size_t rejected_invalid = 0;
    std::size_t rejected_transform = 0;
    for (std::size_t index = 0; index < input_size; ++index)
    {
      const auto &pose = msg->poses[index];
      if (!validPose(pose))
      {
        ++rejected_invalid;
        continue;
      }
      // The formal Football PoseArray contract excludes the controlled robot.
      // Only legacy producers that explicitly declare that they include self may
      // use the origin-radius compatibility filter in the ego base frame.
      if (pose_array_includes_self_ &&
          std::hypot(pose.position.x, pose.position.y) < self_filter_radius_)
      {
        ++rejected_invalid;
        continue;
      }
      DynamicObstacle obstacle;
      if (!makeObstacleFromPose(
              pose, msg->header.frame_id, stamp, obstacle))
      {
        ++rejected_transform;
        continue;
      }
      obstacle.source_index = index;
      accepted.push_back(IndexedObstacle{index, obstacle});
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    last_input_count_ = input_size;
    last_accepted_count_ = accepted.size();
    last_rejected_invalid_count_ = rejected_invalid;
    last_rejected_transform_count_ = rejected_transform;
    // Valid peers continue updating even when another pose is absent or
    // untransformable; unobserved tracks expire individually in updateBounds.
    if (input_size < static_cast<std::size_t>(minimum_obstacle_count_) ||
        accepted.size() < static_cast<std::size_t>(minimum_obstacle_count_))
    {
      return;
    }
    if (!stampAcceptable(stamp))
    {
      return;
    }
    if (last_pose_array_stamp_.nanoseconds() > 0 &&
        stamp <= last_pose_array_stamp_)
    {
      return;
    }
    updateTracks(accepted, input_size, stamp);
    last_pose_array_stamp_ = stamp;
    last_data_received_time_ = clock_->now();
    have_received_data_ = true;
  }

  void MultiRobotObstacleLayer::appendPredictedSweep(
      const ObstacleTrack &track,
      const rclcpp::Time &now,
      std::vector<DynamicObstacle> &output) const
  {
    const double age = std::clamp(
        (now - track.stamp).seconds(), 0.0, data_timeout_);
    DynamicObstacle start = track.obstacle;
    if (track.velocity_valid)
    {
      start.x += track.vx * age;
      start.y += track.vy * age;
      start.yaw = normalizeAngle(start.yaw + track.yaw_rate * age);
    }
    output.push_back(start);

    if (!enable_prediction_ || !track.velocity_valid || prediction_horizon_sec_ <= 0.0)
    {
      return;
    }
    const double speed = std::hypot(track.vx, track.vy);
    const double yaw_rate = std::fabs(track.yaw_rate);
    if (speed < min_prediction_speed_mps_ && yaw_rate < 0.05)
    {
      return;
    }

    double horizon = prediction_horizon_sec_;
    if (speed > 1.0e-6 && max_prediction_distance_m_ > 0.0)
    {
      horizon = std::min(horizon, max_prediction_distance_m_ / speed);
    }
    horizon = std::max(0.0, horizon);
    const double travel = speed * horizon;
    const double turn = yaw_rate * horizon;
    const int samples = std::clamp(
        static_cast<int>(std::ceil(std::max(
            travel / sweep_linear_step_m_,
            turn / sweep_angular_step_rad_))),
        1,
        max_sweep_samples_);

    for (int sample = 1; sample <= samples; ++sample)
    {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      const double dt = horizon * ratio;
      DynamicObstacle predicted = start;
      predicted.x += track.vx * dt;
      predicted.y += track.vy * dt;
      predicted.yaw = normalizeAngle(predicted.yaw + track.yaw_rate * dt);
      output.push_back(predicted);
    }
  }

  void MultiRobotObstacleLayer::updateBounds(
      const double robot_x,
      const double robot_y,
      const double robot_yaw,
      double *min_x,
      double *min_y,
      double *max_x,
      double *max_y)
  {
    const auto started = std::chrono::steady_clock::now();
    ++update_bounds_calls_;
    (void)robot_yaw;
    if (rolling_window_)
    {
      updateOrigin(
          robot_x - getSizeInMetersX() / 2.0,
          robot_y - getSizeInMetersY() / 2.0);
    }

    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
    useExtraBounds(min_x, min_y, max_x, max_y);
    for (const auto &obstacle : previous_obstacles_)
    {
      touchObstacleBounds(obstacle, min_x, min_y, max_x, max_y);
    }

    obstacles_to_mark_.clear();
    if (!enabled_)
    {
      previous_obstacles_.clear();
      current_ = true;
      last_update_bounds_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      return;
    }

    const rclcpp::Time current_time = clock_->now();
    bool data_fresh = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      data_fresh = have_received_data_ &&
                   last_data_received_time_.nanoseconds() > 0 &&
                   (current_time - last_data_received_time_).seconds() <= data_timeout_;
      if (!data_fresh)
      {
        for (auto &track : obstacle_tracks_)
        {
          track.valid = false;
          track.velocity_valid = false;
        }
      }

      for (const auto &track : obstacle_tracks_)
      {
        if (!track.valid || track.stamp.nanoseconds() <= 0)
        {
          continue;
        }
        if ((current_time - track.stamp).seconds() > data_timeout_)
        {
          continue;
        }
        appendPredictedSweep(track, current_time, obstacles_to_mark_);
      }
    }

    for (const auto &obstacle : obstacles_to_mark_)
    {
      touchObstacleBounds(obstacle, min_x, min_y, max_x, max_y);
    }
    previous_obstacles_ = obstacles_to_mark_;
    current_ = data_fresh;
    last_update_bounds_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
  }

  void MultiRobotObstacleLayer::touchObstacleBounds(
      const DynamicObstacle &obstacle,
      double *min_x,
      double *min_y,
      double *max_x,
      double *max_y)
  {
    if (use_oriented_footprint_ && other_robot_footprint_.size() >= 3)
    {
      geometry_msgs::msg::PolygonStamped oriented;
      nav2_costmap_2d::transformFootprint(
          obstacle.x, obstacle.y, obstacle.yaw,
          other_robot_footprint_, oriented);
      for (const auto &point : oriented.polygon.points)
      {
        touch(point.x, point.y, min_x, min_y, max_x, max_y);
      }
    }
    else
    {
      touch(
          obstacle.x - obstacle.radius,
          obstacle.y - obstacle.radius,
          min_x, min_y, max_x, max_y);
      touch(
          obstacle.x + obstacle.radius,
          obstacle.y + obstacle.radius,
          min_x, min_y, max_x, max_y);
    }
  }

  void MultiRobotObstacleLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid,
      int min_i,
      int min_j,
      int max_i,
      int max_j)
  {
    const auto started = std::chrono::steady_clock::now();
    (void)min_i;
    (void)min_j;
    (void)max_i;
    (void)max_j;
    ++update_costs_calls_;
    if (!enabled_)
    {
      last_update_costs_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      return;
    }

    unsigned int total_marked = 0;
    const DynamicObstacle *sample_obstacle = nullptr;
    unsigned int sample_mx = 0;
    unsigned int sample_my = 0;
    unsigned char sample_cost = nav2_costmap_2d::NO_INFORMATION;
    for (const auto &obstacle : obstacles_to_mark_)
    {
      unsigned int marked = 0;
      if (use_oriented_footprint_ && other_robot_footprint_.size() >= 3)
      {
        marked = markFootprintObstacle(
            master_grid,
            obstacle,
            other_robot_footprint_,
            mark_cost_,
            use_maximum_);
        if (marked == 0)
        {
          marked = markCircularObstacle(
              master_grid, obstacle, mark_cost_, use_maximum_);
        }
      }
      else
      {
        marked = markCircularObstacle(
            master_grid, obstacle, mark_cost_, use_maximum_);
      }
      total_marked += marked;

      unsigned int mx = 0;
      unsigned int my = 0;
      if (!sample_obstacle &&
          master_grid.worldToMap(obstacle.x, obstacle.y, mx, my))
      {
        sample_obstacle = &obstacle;
        sample_mx = mx;
        sample_my = my;
        sample_cost = master_grid.getCost(mx, my);
      }
    }
    last_update_costs_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    logMasterGridDiagnostics(
        master_grid,
        total_marked,
        sample_obstacle,
        sample_mx,
        sample_my,
        sample_cost);
  }

  void MultiRobotObstacleLayer::logMasterGridDiagnostics(
      const nav2_costmap_2d::Costmap2D &master_grid,
      const unsigned int marked_cells,
      const DynamicObstacle *sample_obstacle,
      const unsigned int sample_mx,
      const unsigned int sample_my,
      const unsigned char sample_cost)
  {
    const rclcpp::Time now = clock_->now();
    if (last_master_grid_diagnostics_time_.nanoseconds() > 0 &&
        (now - last_master_grid_diagnostics_time_).seconds() < 1.0)
    {
      return;
    }

    double elapsed = 1.0;
    if (last_master_grid_diagnostics_time_.nanoseconds() > 0)
    {
      elapsed = std::max(
          1.0e-6,
          (now - last_master_grid_diagnostics_time_).seconds());
    }
    const auto bounds_delta =
        update_bounds_calls_ - diagnostics_bounds_calls_;
    const auto costs_delta =
        update_costs_calls_ - diagnostics_costs_calls_;
    diagnostics_bounds_calls_ = update_bounds_calls_;
    diagnostics_costs_calls_ = update_costs_calls_;
    last_master_grid_diagnostics_time_ = now;

    std::size_t input_count = 0;
    std::size_t accepted_count = 0;
    std::size_t rejected_invalid = 0;
    std::size_t rejected_transform = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      input_count = last_input_count_;
      accepted_count = last_accepted_count_;
      rejected_invalid = last_rejected_invalid_count_;
      rejected_transform = last_rejected_transform_count_;
    }

    if (sample_obstacle)
    {
      RCLCPP_INFO(
          logger_,
          "MultiRobotObstacleLayer master_grid stamp_ns=%ld "
          "updateBounds_hz=%.2f updateCosts_hz=%.2f "
          "input_count=%zu accepted_count=%zu rejected_invalid=%zu "
          "rejected_transform=%zu updateBounds_ms=%.3f updateCosts_ms=%.3f "
          "obstacles=%zu marked_cells=%u "
          "sample_id=%zu sample_xy=(%.3f,%.3f) sample_cell=(%u,%u) "
          "center_cost=%u size=%ux%u resolution=%.3f origin=(%.3f,%.3f)",
          now.nanoseconds(),
          static_cast<double>(bounds_delta) / elapsed,
          static_cast<double>(costs_delta) / elapsed,
          input_count,
          accepted_count,
          rejected_invalid,
          rejected_transform,
          last_update_bounds_ms_,
          last_update_costs_ms_,
          obstacles_to_mark_.size(),
          marked_cells,
          sample_obstacle->source_index,
          sample_obstacle->x,
          sample_obstacle->y,
          sample_mx,
          sample_my,
          static_cast<unsigned int>(sample_cost),
          master_grid.getSizeInCellsX(),
          master_grid.getSizeInCellsY(),
          master_grid.getResolution(),
          master_grid.getOriginX(),
          master_grid.getOriginY());
    }
    else
    {
      RCLCPP_INFO(
          logger_,
          "MultiRobotObstacleLayer master_grid stamp_ns=%ld "
          "updateBounds_hz=%.2f updateCosts_hz=%.2f "
          "input_count=%zu accepted_count=%zu rejected_invalid=%zu "
          "rejected_transform=%zu updateBounds_ms=%.3f updateCosts_ms=%.3f "
          "obstacles=%zu marked_cells=%u sample=OUTSIDE_MASTER_GRID "
          "size=%ux%u resolution=%.3f origin=(%.3f,%.3f)",
          now.nanoseconds(),
          static_cast<double>(bounds_delta) / elapsed,
          static_cast<double>(costs_delta) / elapsed,
          input_count,
          accepted_count,
          rejected_invalid,
          rejected_transform,
          last_update_bounds_ms_,
          last_update_costs_ms_,
          obstacles_to_mark_.size(),
          marked_cells,
          master_grid.getSizeInCellsX(),
          master_grid.getSizeInCellsY(),
          master_grid.getResolution(),
          master_grid.getOriginX(),
          master_grid.getOriginY());
    }
  }

} // namespace football_navigation

PLUGINLIB_EXPORT_CLASS(
    football_navigation::FootballPreferForwardCritic,
    dwb_core::TrajectoryCritic)

PLUGINLIB_EXPORT_CLASS(
    football_navigation::FootballProgressChecker,
    nav2_core::ProgressChecker)

extern "C"
    __attribute__((visibility("default"))) void
    BT_RegisterNodesFromPlugin(
        BT::BehaviorTreeFactory &factory)
{
  BT::NodeBuilder builder =
      [](
          const std::string &name,
          const BT::NodeConfiguration &configuration)
  {
    return std::make_unique<
        football_navigation::
            StableComputePathToPoseAction>(
        name,
        "compute_path_to_p",
        configuration);
  };

  factory.registerBuilder<
      football_navigation::
          StableComputePathToPoseAction>(
      "ComputePathToP",
      builder);

  BT::NodeBuilder follow_builder =
      [](
          const std::string &name,
          const BT::NodeConfiguration &configuration)
  {
    return std::make_unique<
        football_navigation::StableFollowPathAction>(
        name,
        "follow_p",
        configuration);
  };

  factory.registerBuilder<
      football_navigation::StableFollowPathAction>(
      "FollowP",
      follow_builder);

  factory.registerNodeType<
      football_navigation::FootballTargetUpdater>(
      "FootballTargetUpdater");
}
