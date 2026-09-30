from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("inspection_manager"),
                "config",
                "vertical_tower.yaml",
            ]),
            description="Inspection manager parameter YAML",
        ),
        Node(
            package="inspection_manager",
            executable="inspection_manager_node.py",
            name="inspection_manager",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),
        Node(
            package="inspection_manager",
            executable="perception_sim_node.py",
            name="synthetic_perception",
            output="screen",
        ),
    ])
