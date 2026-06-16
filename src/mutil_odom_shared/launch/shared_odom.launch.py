from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	package_share = Path(get_package_share_directory("mutil_odom_shared"))
	default_config = str(package_share / "config" / "topic.yaml")
	config_file = LaunchConfiguration("config_file")

	return LaunchDescription([
		DeclareLaunchArgument(
			"config_file",
			default_value=default_config,
			description="多机器人 odom 共享参数文件"),
		Node(
			package="mutil_odom_shared",
			executable="mutil_odom_shared_node",
			name="mutil_odom_shared",
			output="screen",
			parameters=[config_file]),
	])
