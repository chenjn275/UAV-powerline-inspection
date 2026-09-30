from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare("inspection_gazebo"), "worlds", "inspection_line_native.world"
    ])
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
    ])
    model_path = PathJoinSubstitution([
        FindPackageShare("inspection_gazebo"), "models"
    ])

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=[
                model_path,
                ":/usr/share/gazebo-11/models:/usr/share/gazebo-11",
                ":",
                EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value=[
                "/usr/share/gazebo-11:",
                EnvironmentVariable("GAZEBO_RESOURCE_PATH", default_value=""),
            ],
        ),
        SetEnvironmentVariable(
            name="OGRE_RESOURCE_PATH",
            value="/usr/lib/x86_64-linux-gnu/OGRE-1.9.0",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show the Gazebo client; use false for headless checks.",
        ),
        DeclareLaunchArgument(
            "world", default_value=default_world,
            description=(
                "Gazebo world. The default native world avoids unstable ROS sensor "
                "plugins; use inspection_line.world only for plugin experiments."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "gui": LaunchConfiguration("gui"),
                "verbose": "false",
            }.items(),
        ),
        # Gazebo must finish loading the state plugin before the demo node calls
        # /gazebo/set_entity_state.
        TimerAction(
            period=3.0,
            actions=[Node(
                package="inspection_gazebo",
                executable="inspection_gazebo_demo.py",
                name="inspection_gazebo_demo",
                output="screen",
                parameters=[{"duration_s": 45.0}],
            )],
        ),
    ])
