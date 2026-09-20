// Copyright (c) 2026 CyberDog2 football navigation contributors.
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ignition/msgs/marker.pb.h"
#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
using RosMarker = visualization_msgs::msg::Marker;
using MarkerKey = std::pair<std::string, std::uint64_t>;

std::string groupFromTopic(std::string topic)
{
  if (!topic.empty() && topic.front() == '/') {
    topic.erase(0, 1);
  }
  std::replace(topic.begin(), topic.end(), '/', '_');
  return topic.empty() ? "markers" : topic;
}

double finiteOr(double value, double fallback)
{
  return std::isfinite(value) ? value : fallback;
}
}  // namespace

class FootballGazeboMarkerBridge : public rclcpp::Node
{
public:
  FootballGazeboMarkerBridge()
  : Node("football_gazebo_marker_bridge")
  {
    expected_frame_ = declare_parameter<std::string>("expected_frame", "world");
    marker_service_ = declare_parameter<std::string>("marker_service", "/marker");
    suppress_physical_duplicates_ =
      declare_parameter<bool>("suppress_physical_duplicates", true);
    auto topics = declare_parameter<std::vector<std::string>>(
      "marker_topics",
      std::vector<std::string>{
        "/football/markers/ball",
        "/football/markers/approach_pose",
        "/football/markers/tracking_pose",
        "/football/markers/paths",
        "/football/markers/goals"});
    const auto robot_marker_topic = declare_parameter<std::string>(
      "robot_marker_topic", "/football/markers/robots/cyberdog_1");
    if (!robot_marker_topic.empty()) {
      topics.push_back(robot_marker_topic);
    }
    if (expected_frame_.empty() || marker_service_.empty() || topics.empty()) {
      throw std::invalid_argument("Gazebo marker bridge parameters must not be empty");
    }

    const auto qos = rclcpp::QoS(10).reliable();
    for (const auto & topic : topics) {
      if (topic.empty()) {
        continue;
      }
      const std::string group = groupFromTopic(topic);
      subscriptions_.push_back(
        create_subscription<visualization_msgs::msg::MarkerArray>(
          topic, qos,
          [this, group](const visualization_msgs::msg::MarkerArray::SharedPtr message) {
            if (message) {
              onMarkers(group, *message);
            }
          }));
    }

    RCLCPP_INFO(
      get_logger(), "bridging %zu ROS marker topics into Gazebo service %s (frame=%s)",
      subscriptions_.size(), marker_service_.c_str(), expected_frame_.c_str());
  }

  ~FootballGazeboMarkerBridge() override
  {
    for (const auto & entry : active_markers_) {
      clearGroup(entry.first, entry.second);
    }
  }

private:
  std::string gazeboNamespace(const std::string & group, const std::string & marker_ns) const
  {
    return "football/" + group + "/" + (marker_ns.empty() ? "default" : marker_ns);
  }

  bool request(const ignition::msgs::Marker & marker)
  {
    const bool queued = ignition_node_.Request(marker_service_, marker);
    if (!queued) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Gazebo marker service '%s' is unavailable; is gzclient running?",
        marker_service_.c_str());
    }
    return queued;
  }

  void deleteMarker(const MarkerKey & key)
  {
    ignition::msgs::Marker marker;
    marker.set_action(ignition::msgs::Marker::DELETE_MARKER);
    marker.set_ns(key.first);
    marker.set_id(key.second);
    request(marker);
    text_cache_.erase(key);
  }

  void clearGroup(const std::string &, const std::set<MarkerKey> & markers)
  {
    for (const auto & marker : markers) {
      deleteMarker(marker);
    }
  }

  static void copyPose(const RosMarker & source, ignition::msgs::Marker & target)
  {
    auto * pose = target.mutable_pose();
    pose->mutable_position()->set_x(finiteOr(source.pose.position.x, 0.0));
    pose->mutable_position()->set_y(finiteOr(source.pose.position.y, 0.0));
    pose->mutable_position()->set_z(finiteOr(source.pose.position.z, 0.0));

    const double x = finiteOr(source.pose.orientation.x, 0.0);
    const double y = finiteOr(source.pose.orientation.y, 0.0);
    const double z = finiteOr(source.pose.orientation.z, 0.0);
    const double w = finiteOr(source.pose.orientation.w, 1.0);
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    auto * orientation = pose->mutable_orientation();
    if (norm > 1e-9) {
      orientation->set_x(x / norm);
      orientation->set_y(y / norm);
      orientation->set_z(z / norm);
      orientation->set_w(w / norm);
    } else {
      orientation->set_w(1.0);
    }
  }

  static void copyColor(const RosMarker & source, ignition::msgs::Marker & target)
  {
    const double red = std::clamp(static_cast<double>(source.color.r), 0.0, 1.0);
    const double green = std::clamp(static_cast<double>(source.color.g), 0.0, 1.0);
    const double blue = std::clamp(static_cast<double>(source.color.b), 0.0, 1.0);
    const double alpha = std::clamp(static_cast<double>(source.color.a), 0.0, 1.0);
    auto set = [red, green, blue, alpha](auto * color) {
        color->set_r(red);
        color->set_g(green);
        color->set_b(blue);
        color->set_a(alpha);
      };
    auto * material = target.mutable_material();
    set(material->mutable_ambient());
    set(material->mutable_diffuse());
    set(material->mutable_emissive());
    material->set_lighting(false);

    // Gazebo Classic's MarkerManager does not consistently apply raw RGBA
    // values to dynamic markers (transparent cylinders often render white).
    // Its bundled material scripts are stable across the OGRE render paths,
    // so select the closest named material while retaining RGBA above as a
    // fallback for renderers that honour it directly.
    const bool transparent = alpha < 0.55;
    std::string name;
    if (red > 0.75 && green > 0.15 && green < 0.75 && blue < 0.35) {
      name = transparent ? "Gazebo/OrangeTransparentOverlay" : "Gazebo/Orange";
    } else if (red > 0.70 && green > 0.65 && blue < 0.45) {
      name = transparent ? "Gazebo/YellowTransparent" : "Gazebo/Yellow";
    } else if (green >= red && green > blue * 1.15) {
      name = transparent ? "Gazebo/GreenTransparentOverlay" : "Gazebo/Green";
    } else if (blue >= red && blue >= green) {
      name = transparent ? "Gazebo/BlueTransparentOverlay" : "Gazebo/SkyBlue";
    } else if (red > blue * 1.20) {
      name = transparent ? "Gazebo/RedTransparentOverlay" : "Gazebo/Red";
    } else {
      name = transparent ? "Gazebo/GreyTransparent" : "Gazebo/White";
    }
    auto * script = material->mutable_script();
    script->add_uri("file://media/materials/scripts/gazebo.material");
    script->set_name(name);
  }

  static void copyPoints(const RosMarker & source, ignition::msgs::Marker & target)
  {
    for (const auto & point : source.points) {
      auto * output = target.add_point();
      output->set_x(finiteOr(point.x, 0.0));
      output->set_y(finiteOr(point.y, 0.0));
      output->set_z(finiteOr(point.z, 0.0));
    }
  }

  bool suppressedDuplicate(const std::string & group, const RosMarker & marker) const
  {
    if (!suppress_physical_duplicates_) {
      return false;
    }
    const bool ball_body = group.find("markers_ball") != std::string::npos &&
      marker.ns == "ball_pose" && marker.type == RosMarker::SPHERE;
    const bool robot_body = group.find("markers_robots_") != std::string::npos &&
      marker.type == RosMarker::CUBE;
    return ball_body || robot_body;
  }

  bool convert(
    const std::string & group, const RosMarker & source,
    ignition::msgs::Marker & target, MarkerKey & key)
  {
    if (source.id < 0 || suppressedDuplicate(group, source)) {
      return false;
    }
    if (!source.header.frame_id.empty() && source.header.frame_id != expected_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "ignoring Gazebo marker in frame '%s'; expected '%s'",
        source.header.frame_id.c_str(), expected_frame_.c_str());
      return false;
    }

    target.set_action(ignition::msgs::Marker::ADD_MODIFY);
    target.set_visibility(ignition::msgs::Marker::GUI);
    target.set_ns(gazeboNamespace(group, source.ns));
    target.set_id(static_cast<std::uint64_t>(source.id));
    target.set_layer(2);
    key = {target.ns(), target.id()};
    copyPose(source, target);
    copyColor(source, target);

    auto * scale = target.mutable_scale();
    const double sx = std::max(1e-4, finiteOr(source.scale.x, 1.0));
    const double sy = std::max(1e-4, finiteOr(source.scale.y, sx));
    const double sz = std::max(1e-4, finiteOr(source.scale.z, sx));
    switch (source.type) {
      case RosMarker::CUBE:
        target.set_type(ignition::msgs::Marker::BOX);
        scale->set_x(sx);
        scale->set_y(sy);
        scale->set_z(sz);
        break;
      case RosMarker::SPHERE:
        target.set_type(ignition::msgs::Marker::SPHERE);
        scale->set_x(sx);
        scale->set_y(sy);
        scale->set_z(sz);
        break;
      case RosMarker::CYLINDER:
        target.set_type(ignition::msgs::Marker::CYLINDER);
        scale->set_x(sx);
        scale->set_y(sy);
        scale->set_z(sz);
        break;
      case RosMarker::LINE_STRIP:
        if (source.points.size() < 2) {
          return false;
        }
        target.set_type(ignition::msgs::Marker::LINE_STRIP);
        scale->set_x(1.0);
        scale->set_y(1.0);
        scale->set_z(1.0);
        copyPoints(source, target);
        break;
      case RosMarker::LINE_LIST:
        if (source.points.size() < 2) {
          return false;
        }
        target.set_type(ignition::msgs::Marker::LINE_LIST);
        scale->set_x(1.0);
        scale->set_y(1.0);
        scale->set_z(1.0);
        copyPoints(source, target);
        break;
      case RosMarker::POINTS:
        if (source.points.empty()) {
          return false;
        }
        target.set_type(ignition::msgs::Marker::POINTS);
        scale->set_x(1.0);
        scale->set_y(1.0);
        scale->set_z(1.0);
        copyPoints(source, target);
        break;
      case RosMarker::TEXT_VIEW_FACING:
        if (source.text.empty()) {
          return false;
        }
        target.set_type(ignition::msgs::Marker::TEXT);
        target.set_text(source.text);
        scale->set_x(std::max(0.05, sz));
        scale->set_y(std::max(0.05, sz));
        scale->set_z(std::max(0.05, sz));
        break;
      case RosMarker::ARROW:
        target.set_type(ignition::msgs::Marker::LINE_LIST);
        scale->set_x(1.0);
        scale->set_y(1.0);
        scale->set_z(1.0);
        if (source.points.size() >= 2) {
          copyPoints(source, target);
        } else {
          auto * start = target.add_point();
          start->set_x(0.0);
          start->set_y(0.0);
          start->set_z(0.0);
          auto * end = target.add_point();
          end->set_x(sx);
          end->set_y(0.0);
          end->set_z(0.0);
        }
        break;
      default:
        return false;
    }
    return true;
  }

  void onMarkers(
    const std::string & group,
    const visualization_msgs::msg::MarkerArray & message)
  {
    std::set<MarkerKey> current;
    const bool explicit_clear = std::any_of(
      message.markers.begin(), message.markers.end(),
      [](const RosMarker & marker) {return marker.action == RosMarker::DELETEALL;});
    auto & previous = active_markers_[group];
    if (explicit_clear) {
      clearGroup(group, previous);
      previous.clear();
    }

    for (const auto & source : message.markers) {
      if (source.action == RosMarker::DELETEALL) {
        continue;
      }
      const MarkerKey source_key{
        gazeboNamespace(group, source.ns),
        source.id < 0 ? 0U : static_cast<std::uint64_t>(source.id)};
      if (source.action == RosMarker::DELETE) {
        deleteMarker(source_key);
        continue;
      }
      ignition::msgs::Marker target;
      MarkerKey target_key;
      if (convert(group, source, target, target_key)) {
        if (source.type == RosMarker::TEXT_VIEW_FACING) {
          const auto cached = text_cache_.find(target_key);
          if (cached != text_cache_.end() && cached->second != source.text) {
            // Gazebo Classic updates a text marker's pose but not its string.
            // Recreate only when the string changes to keep live coordinates
            // current without introducing continuous delete/add flicker.
            deleteMarker(target_key);
          }
        }
        if (request(target)) {
          current.insert(target_key);
          if (source.type == RosMarker::TEXT_VIEW_FACING) {
            text_cache_[target_key] = source.text;
          }
        }
      }
    }

    for (const auto & marker : previous) {
      if (current.find(marker) == current.end()) {
        deleteMarker(marker);
      }
    }
    previous = std::move(current);
  }

  std::string expected_frame_;
  std::string marker_service_;
  bool suppress_physical_duplicates_{true};
  ignition::transport::Node ignition_node_;
  std::map<std::string, std::set<MarkerKey>> active_markers_;
  std::map<MarkerKey, std::string> text_cache_;
  std::vector<rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr>
  subscriptions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootballGazeboMarkerBridge>());
  rclcpp::shutdown();
  return 0;
}
