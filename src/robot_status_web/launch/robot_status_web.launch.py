from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robot_status_web"))
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(share / "config" / "robot_status_web.yaml")),
        Node(
            package="robot_status_web",
            executable="status_server",
            name="robot_status_web",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),
    ])
