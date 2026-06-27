from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    apriltag_share = Path(get_package_share_directory("apriltag_ros"))
    tags_config = str(apriltag_share / "cfg" / "tags_36h11.yaml")

    image_topic = LaunchConfiguration("image_topic")
    node_namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/cyberdog_1/camera/infra1/image_rect_raw",
            description="实机矫正后灰度图像话题"),
        DeclareLaunchArgument(
            "namespace",
            default_value="apriltag",
            description="apriltag 节点命名空间"),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            namespace=node_namespace,
            output="screen",
            parameters=[tags_config],
            remappings=[
                ("image_rect", image_topic),
            ],
        ),
    ])
