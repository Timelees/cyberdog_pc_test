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

        from ament_index_python.packages import get_package_share_directory

        cyberdog_bringup_share = get_package_share_directory("cyberdog_bringup")
        sys.path.append(os.path.join(cyberdog_bringup_share, "bringup"))
        from manual import get_namespace  # type: ignore

        return str(get_namespace() or "").strip("/")
    except Exception:
        return ""


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")

    if not robot_namespace:
        robot_namespace = _get_robot_namespace()

    return [
        Node(
            package="apriltag_ros",
            executable="odom_transform_node",
            name="odom_transform",
            output="screen",
            parameters=[{
                "robot_namespace": robot_namespace,
                "odom_topic": LaunchConfiguration("odom_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "odom_subscribe_best_effort": True,
                "odom_publish_best_effort": True,
                # apriltag_node is launched without a ROS namespace, so its tag TF
                # is published on global /tf or /tf_static.
                "tf_subscribe_global": True,
                "publish_static_tf_to_global": False,
                "use_latest_tf_on_failure": False,
            }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_namespace",
            default_value=_get_robot_namespace(),
            description="Top-level robot namespace; empty tries cyberdog_bringup get_namespace()"),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="odom_slam",
            description="Input odometry topic name without namespace"),
        DeclareLaunchArgument(
            "output_topic",
            default_value="odom_global",
            description="Output odometry topic name without namespace"),
        OpaqueFunction(function=launch_setup),
    ])
