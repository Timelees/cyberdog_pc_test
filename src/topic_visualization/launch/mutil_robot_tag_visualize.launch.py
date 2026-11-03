from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("topic_visualization"))
    default_config = str(package_share / "config" / "mutil_robot_topics.yaml")
    default_rviz = str(package_share / "config" / "mutil_robot_tag.rviz")

    config_file = LaunchConfiguration("config_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    pose_text_enabled = LaunchConfiguration("pose_text_enabled")
    pose_text_scale = LaunchConfiguration("pose_text_scale")
    pose_text_z_offset = LaunchConfiguration("pose_text_z_offset")

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
        DeclareLaunchArgument(
            "pose_text_enabled",
            default_value="true",
            description="是否显示机器人在 tag_global 下的坐标文本"),
        DeclareLaunchArgument(
            "pose_text_scale",
            default_value="0.18",
            description="坐标文本字号（米）"),
        DeclareLaunchArgument(
            "pose_text_z_offset",
            default_value="0.6",
            description="坐标文本相对机器人位置的 Z 轴偏移（米）"),
        Node(
            package="topic_visualization",
            executable="mutil_robot_tag_visual_node",
            name="mutil_robot_tag_visual",
            output="screen",
            parameters=[
                config_file,
                {
                    "pose_text_enabled": ParameterValue(pose_text_enabled, value_type=bool),
                    "pose_text_scale": ParameterValue(pose_text_scale, value_type=float),
                    "pose_text_z_offset": ParameterValue(pose_text_z_offset, value_type=float),
                },
            ]),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen")
    ])
