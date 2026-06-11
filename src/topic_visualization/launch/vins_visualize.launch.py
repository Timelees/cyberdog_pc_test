from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	package_share = Path(get_package_share_directory("topic_visualization"))
	default_config = str(package_share / "config" / "vins_topics.yaml")
	default_rviz = str(package_share / "config" / "mivins_rviz2_config.rviz2.rviz")

	config_file = LaunchConfiguration("config_file")
	rviz_config = LaunchConfiguration("rviz_config")
	use_rviz = LaunchConfiguration("use_rviz")

	return LaunchDescription([
		DeclareLaunchArgument(
			"config_file",
			default_value=default_config,
			description="vins_visual 节点参数文件"),
		DeclareLaunchArgument(
			"rviz_config",
			default_value=default_rviz,
			description="Mivins RViz 配置文件"),
		DeclareLaunchArgument(
			"use_rviz",
			default_value="true",
			description="是否同时启动 RViz2"),
		Node(
			package="topic_visualization",
			executable="vins_visual_node",
			name="vins_visual",
			output="screen",
			parameters=[config_file]),
		Node(
			condition=IfCondition(use_rviz),
			package="rviz2",
			executable="rviz2",
			name="rviz2",
			arguments=["-d", rviz_config],
			output="screen")
	])
