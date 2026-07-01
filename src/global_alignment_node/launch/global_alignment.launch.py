from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("global_alignment_node"))
    default_config = str(package_share / "config" / "topic.yaml")

    config_file = LaunchConfiguration("config_file")
    input_odom_topic = LaunchConfiguration("input_odom_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    source_frame_id = LaunchConfiguration("source_frame_id")
    robot_frame_id = LaunchConfiguration("robot_frame_id")
    output_frame_id = LaunchConfiguration("output_frame_id")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="global_alignment_node parameter file"),
        DeclareLaunchArgument(
            "input_odom_topic",
            default_value="/cyberdog_1/odom_slam",
            description="VIO odometry topic to align into the AprilTag base frame"),
        DeclareLaunchArgument(
            "output_odom_topic",
            default_value="/cyberdog_1/odom_base",
            description="Aligned odometry output topic"),
        DeclareLaunchArgument(
            "source_frame_id",
            default_value="",
            description="Override odom source frame; empty uses input odom header.frame_id"),
        DeclareLaunchArgument(
            "robot_frame_id",
            default_value="base_link",
            description="Robot body frame used to compute base <- odom_source alignment"),
        DeclareLaunchArgument(
            "output_frame_id",
            default_value="tag_map",
            description="Z-up output frame for RViz visualization"),
        Node(
            package="global_alignment_node",
            executable="global_alignment_node",
            name="global_alignment",
            output="screen",
            parameters=[
                config_file,
                {
                    "input_odom_topic": input_odom_topic,
                    "output_odom_topic": output_odom_topic,
                    "source_frame_id": source_frame_id,
                    "robot_frame_id": robot_frame_id,
                    "output_frame_id": output_frame_id,
                },
            ],
        ),
    ])
