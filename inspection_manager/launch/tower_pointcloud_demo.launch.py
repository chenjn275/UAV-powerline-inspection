"""Run the synthetic tower cloud through the PointCloud2 localizer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_px4_setpoints", default_value="false",
            description="Enable guarded PX4 /fmu/in setpoint output after valid DDS feedback.",
        ),
        DeclareLaunchArgument(
            "request_offboard", default_value="false",
            description="Request PX4 Offboard mode after setpoint warmup.",
        ),
        DeclareLaunchArgument(
            "request_arm", default_value="false",
            description="Request PX4 arming after Offboard mode request.",
        ),
        DeclareLaunchArgument(
            "health_sequence", default_value="READY:120",
            description="Synthetic health sequence; keep READY during PX4 Offboard SITL.",
        ),
        DeclareLaunchArgument(
            "obstacle_amplitude", default_value="0.0",
            description="Dynamic obstacle lateral amplitude in metres.",
        ),
        Node(
            package="inspection_manager",
            executable="tower_pointcloud_node.py",
            name="tower_pointcloud_localizer",
            output="screen",
        ),
        Node(
            package="inspection_manager",
            executable="inspection_sensor_sim_node.py",
            name="inspection_sensor_sim",
            output="screen",
            parameters=[{"tower_x": 0.0, "tower_y": 0.0, "tower_height_m": 10.0}],
        ),
        Node(
            package="inspection_manager",
            executable="obstacle_pointcloud_sim_node.py",
            name="obstacle_pointcloud_sim",
            output="screen",
            parameters=[{"amplitude": LaunchConfiguration("obstacle_amplitude")}],
        ),
        Node(
            package="inspection_manager",
            executable="vision_observation_sim_node.py",
            name="vision_observation_sim",
            output="screen",
        ),
        Node(
            package="inspection_manager",
            executable="quality_coverage_sim_node.py",
            name="quality_coverage_sim",
            output="screen",
        ),
        Node(
            package="inspection_manager",
            executable="health_fault_sim_node.py",
            name="health_fault_sim",
            output="screen",
            parameters=[{"sequence": LaunchConfiguration("health_sequence")}],
        ),
        Node(
            package="inspection_manager",
            executable="pointcloud_image_fusion_sim_node.py",
            name="pointcloud_image_fusion",
            output="screen",
        ),
        Node(
            package="inspection_manager",
            executable="inspection_manager_node.py",
            name="inspection_manager",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("inspection_manager"), "config", "vertical_tower.yaml"
            ]), {"require_health": True}],
        ),
        Node(
            package="px4_offboard_bridge",
            executable="px4_offboard_bridge.py",
            name="px4_offboard_bridge",
            output="screen",
            parameters=[{
                "enable_setpoints": LaunchConfiguration("enable_px4_setpoints"),
                "request_offboard": LaunchConfiguration("request_offboard"),
                "request_arm": LaunchConfiguration("request_arm"),
                "land_on_path_end": True,
            }],
        ),
    ])
