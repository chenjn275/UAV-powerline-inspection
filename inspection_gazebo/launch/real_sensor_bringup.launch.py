"""Real sensor bringup for D435i plus an external MID-360 driver.

The Livox node is intentionally supplied as an argument because MID-360
support differs between livox_ros_driver2 and vendor forks. This launch file
does not silently substitute simulated data for a missing device.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_d435i", default_value="true"),
        DeclareLaunchArgument("livox_package", default_value="livox_ros_driver2"),
        DeclareLaunchArgument("livox_executable", default_value="livox_ros_driver2_node"),
        DeclareLaunchArgument("enable_livox", default_value="false"),
        DeclareLaunchArgument("camera_frame", default_value="d435i_link"),
        DeclareLaunchArgument("lidar_frame", default_value="mid360_link"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="d435i",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_d435i")),
            parameters=[{
                "enable_color": True,
                "enable_depth": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_sync": True,
                "align_depth": True,
                "pointcloud.enable": True,
                "publish_tf": True,
                "tf_publish_rate": 30.0,
            }],
        ),
        Node(
            package=LaunchConfiguration("livox_package"),
            executable=LaunchConfiguration("livox_executable"),
            name="mid360",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_livox")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_d435i_tf",
            arguments=["0.18", "0.0", "-0.08", "0.0", "0.0", "0.0",
                       LaunchConfiguration("base_frame"), LaunchConfiguration("camera_frame")],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_mid360_tf",
            arguments=["0.0", "0.0", "0.12", "0.0", "0.0", "0.0",
                       LaunchConfiguration("base_frame"), LaunchConfiguration("lidar_frame")],
        ),
        LogInfo(msg=[
            "D435i bringup enabled; MID-360 bringup is disabled until a Livox driver is installed. "
            "Verify extrinsics before flight."
        ]),
    ])
