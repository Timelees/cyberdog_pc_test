from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("topic_visualization"))
    default_config = str(package_share / "config" / "tags_topics.yaml")
    default_rviz = str(package_share / "config" / "tags_rviz2_config.rviz2.rviz")

    config_file = LaunchConfiguration("config_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="tags_visual 节点参数文件"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="RViz 配置文件，Fixed Frame 需设为 tag_global"),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否同时启动 RViz2"),
        DeclareLaunchArgument(
            "run_odom_transform",
            default_value="false",
            description="是否在本机启动 odom_transform；机器人端已运行时保持 false"),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="cyberdog_2",
            description="机器人命名空间，与 tags_topics.yaml 中 namespace_index 对应"),
        Node(
            condition=IfCondition(LaunchConfiguration("run_odom_transform")),
            package="apriltag_ros",
            executable="odom_transform_node",
            name="odom_transform",
            output="screen",
            parameters=[{
                "robot_namespace": LaunchConfiguration("robot_namespace"),
                "odom_subscribe_best_effort": True,
                "odom_publish_best_effort": True,
                "tf_subscribe_global": False,
                "publish_static_tf_to_global": False,
            }]),
        Node(
            package="topic_visualization",
            executable="tags_visual_node",
            name="tags_visual",
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
