from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='cyberdog_1'),
        Node(
            package='emergency_stop', executable='emergency_stop_node',
            name='emergency_stop', output='screen',
            parameters=[{'robot_namespace': LaunchConfiguration('robot_namespace')}]),
    ])
