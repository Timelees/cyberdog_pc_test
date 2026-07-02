from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_robot_namespace() -> str:
    """
    Best-effort import of Cyberdog's namespace generator.
    Falls back to empty string when cyberdog_bringup is not installed.
    """
    try:
        import os
        import sys

        cyberdog_bringup_share = get_package_share_directory("cyberdog_bringup")
        sys.path.append(os.path.join(cyberdog_bringup_share, "bringup"))
        from manual import get_namespace  # type: ignore

        return str(get_namespace() or "").strip("/")
    except Exception:
        return ""


def launch_setup(context, *args, **kwargs):
    apriltag_share = Path(get_package_share_directory("apriltag_ros"))
    tags_config = str(apriltag_share / "cfg" / "tags_36h11.yaml")

    robot_name = LaunchConfiguration("robot_name").perform(context).strip("/")
    tag_frame = LaunchConfiguration("tag_frame").perform(context).strip("/")
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    image_topic = LaunchConfiguration("image_topic").perform(context).strip()

    # Default robot namespace if not provided by user.
    if not robot_namespace:
        robot_namespace = _get_robot_namespace()

    # Default camera image topic:
    # If user didn't provide a topic, subscribe to `/<robot_namespace>/camera/...` when possible.
    if not image_topic:
        image_topic = f"/{robot_namespace}/camera/infra1/image_rect_raw" if robot_namespace else "/camera/infra1/image_rect_raw"

    if not tag_frame:
        tag_frame = "tag_0_observation"

    return [
        Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[
            tags_config,
            {
                "tag.frames": [tag_frame],
            },
        ],
        remappings=[
            ("image_rect", image_topic),
        ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="",
            description="Rectified infra1 image topic; empty uses /<robot_namespace>/camera/infra1/image_rect_raw"),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value=_get_robot_namespace(),
            description="Top-level robot namespace; empty tries cyberdog_bringup get_namespace()"),
        DeclareLaunchArgument(
            "robot_name",
            default_value="cyberdog_1",
            description="Robot prefix used to build the tag TF child frame"),
        DeclareLaunchArgument(
            "tag_frame",
            default_value="",
            description="AprilTag child frame; empty uses <robot_name>/tag_0_observation"),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="(deprecated) no longer used"),
        OpaqueFunction(function=launch_setup),
    ])
