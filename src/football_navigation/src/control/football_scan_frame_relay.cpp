#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class FootballScanFrameRelay : public rclcpp::Node
{
public:
  FootballScanFrameRelay()
  : Node("football_scan_frame_relay")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cyberdog_1/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cyberdog_1/football_test_scan");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "cyberdog_1_base_link");
    output_frame_ = declare_parameter<std::string>("output_frame", "cyberdog_1_laser_frame");
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!msg) return;
        auto output = *msg;
        output.header.frame_id = output_frame_;
        pub_->publish(output);
      });
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::SensorDataQoS());
    static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = output_frame_;
    tf.transform.translation.x = 0.179;
    tf.transform.translation.z = 0.0837;
    tf.transform.rotation.w = 1.0;
    static_tf_->sendTransform(tf);
  }

private:
  std::string input_topic_, output_topic_, parent_frame_, output_frame_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootballScanFrameRelay>());
  rclcpp::shutdown();
  return 0;
}
