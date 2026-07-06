from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("mutil_robot_odom"))
    default_config = str(package_share / "config" / "global_vio_odom.yaml")

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="多机器人 odom_global 到 /global_vio 的转发参数文件"),
        Node(
            package="mutil_robot_odom",
            executable="global_vio_odom_node",
            name="global_vio_odom",
            output="screen",
            parameters=[config_file])
    ])
