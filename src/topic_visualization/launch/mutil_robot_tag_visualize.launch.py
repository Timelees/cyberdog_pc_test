from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("topic_visualization"))
    default_config = str(package_share / "config" / "mutil_robot_topics.yaml")
    default_rviz = str(package_share / "config" / "mutil_robot_tag.rviz")

    config_file = LaunchConfiguration("config_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="多机器人 tag_global 可视化节点参数文件"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="RViz 配置文件，Fixed Frame 建议设为 tag_global"),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否同时启动 RViz2"),
        Node(
            package="topic_visualization",
            executable="mutil_robot_tag_visual_node",
            name="mutil_robot_tag_visual",
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
