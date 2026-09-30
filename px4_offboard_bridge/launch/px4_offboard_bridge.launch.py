from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_setpoints", default_value="false"),
        DeclareLaunchArgument("request_offboard", default_value="false"),
        DeclareLaunchArgument("request_arm", default_value="false"),
        Node(
            package="px4_offboard_bridge",
            executable="px4_offboard_bridge.py",
            name="px4_offboard_bridge",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("px4_offboard_bridge"),
                    "config",
                    "px4_offboard_bridge.yaml",
                ]),
                {
                    "enable_setpoints": LaunchConfiguration("enable_setpoints"),
                    "request_offboard": LaunchConfiguration("request_offboard"),
                    "request_arm": LaunchConfiguration("request_arm"),
                },
            ],
        ),
    ])
