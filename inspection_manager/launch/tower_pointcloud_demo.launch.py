"""Run the synthetic tower cloud through the PointCloud2 localizer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            "path_duration_s", default_value="75.0",
            description="Smooth two-orbit inspection duration, excluding landing.",
        ),
        DeclareLaunchArgument(
            "health_sequence", default_value="READY:120",
            description="Synthetic health sequence; keep READY during PX4 Offboard SITL.",
        ),
        DeclareLaunchArgument(
            "obstacle_amplitude", default_value="0.0",
            description="Dynamic obstacle lateral amplitude in metres.",
        ),
        DeclareLaunchArgument(
            "obstacle_radius", default_value="0.0",
            description="Synthetic obstacle radius; set to 0 for the nominal clear-air run.",
        ),
        DeclareLaunchArgument("tower_x", default_value="0.0"),
        DeclareLaunchArgument("tower_y", default_value="0.0"),
        DeclareLaunchArgument("tower_height_m", default_value="10.0"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/inspection/tower_points",
                             description="PointCloud2 source; use /cloud_registered for FAST-LIO2 output."),
        DeclareLaunchArgument(
            "use_fast_lio", default_value="false",
            description="Run the installed FAST-LIO2 node and consume its /cloud_registered output.",
        ),
        DeclareLaunchArgument(
            "fast_lio_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("fast_lio"), "config", "mid360.yaml"
            ]),
            description="FAST-LIO2 MID-360 parameter file.",
        ),
        DeclareLaunchArgument(
            "fast_lio_rviz", default_value="false",
            description="Open the FAST-LIO2 RViz view when running the visual demo.",
        ),
        DeclareLaunchArgument(
            "use_super_planner", default_value="false",
            description="Run the repository's SUPER/ROG-Map planner and bridge PositionCommand to PX4.",
        ),
        DeclareLaunchArgument(
            "super_config_name", default_value="inspection_super_px4.yaml",
            description="SUPER config file installed with the super_planner package.",
        ),
        DeclareLaunchArgument(
            "super_goal_stride", default_value="8",
            description="Take every Nth manager waypoint as a sequential SUPER goal.",
        ),
        DeclareLaunchArgument(
            "lock_super_reference_path", default_value="true",
            description="Keep one detected inspection path until it is explicitly revoked.",
        ),
        DeclareLaunchArgument(
            "planning_frame", default_value="map",
            description="Frame shared by tower localization, manager, path, and PX4 bridge.",
        ),
        DeclareLaunchArgument("use_synthetic_cloud", default_value="true",
                             description="Publish synthetic sensor images/cloud for deterministic SITL."),
        Node(
            package="inspection_manager",
            executable="tower_pointcloud_node.py",
            name="tower_pointcloud_localizer",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("pointcloud_topic"),
            }],
        ),
        Node(
            package="inspection_manager",
            executable="inspection_sensor_sim_node.py",
            name="inspection_sensor_sim",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_synthetic_cloud")),
            parameters=[{
                "tower_x": ParameterValue(LaunchConfiguration("tower_x"), value_type=float),
                "tower_y": ParameterValue(LaunchConfiguration("tower_y"), value_type=float),
                "tower_height_m": ParameterValue(LaunchConfiguration("tower_height_m"), value_type=float),
                "fast_lio_input": LaunchConfiguration("use_fast_lio"),
                "frame_id": LaunchConfiguration("planning_frame"),
            }],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_fast_lio")),
            remappings=[
                ("odom", "/fastlio/odom"),
                ("path", "/fastlio/path"),
            ],
            parameters=[
                LaunchConfiguration("fast_lio_config_path"),
                {"use_sim_time": False},
            ],
        ),
        Node(
            package="inspection_manager",
            executable="px4_position_odom_bridge.py",
            name="px4_position_odom_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_fast_lio")),
            parameters=[{
                "input_topic": "/px4_offboard_bridge/position_enu",
                "output_topic": "/odom",
                "frame_id": LaunchConfiguration("planning_frame"),
                "child_frame_id": "base_link",
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="fastlio_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("fast_lio_rviz")),
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("fast_lio"), "rviz", "fastlio.rviz"
                ]),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_livox_static_tf",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_fast_lio")),
            arguments=[
                "--x", "0.12", "--y", "0.0", "--z", "0.175",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "livox_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_static_tf",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_fast_lio")),
            arguments=[
                "--x", "0.12", "--y", "0.0", "--z", "0.125",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "imu_link",
            ],
        ),
        Node(
            package="inspection_manager",
            executable="obstacle_pointcloud_sim_node.py",
            name="obstacle_pointcloud_sim",
            output="screen",
            parameters=[{
                "amplitude": ParameterValue(LaunchConfiguration("obstacle_amplitude"), value_type=float),
                "radius": ParameterValue(LaunchConfiguration("obstacle_radius"), value_type=float),
                "frame_id": LaunchConfiguration("planning_frame"),
            }],
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
            ]), {
                "require_health": True,
                "frame_id": LaunchConfiguration("planning_frame"),
            }],
        ),
        Node(
            package="super_planner",
            executable="fsm_node",
            name="super_planner",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_super_planner")),
            parameters=[{
                "config_name": LaunchConfiguration("super_config_name"),
            }],
        ),
        Node(
            package="px4_offboard_bridge",
            executable="super_planner_adapter.py",
            name="super_planner_adapter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_super_planner")),
            parameters=[{
                "reference_path_topic": "/inspection/reference_path",
                "goal_topic": "/inspection/super/goal",
                "position_command_topic": "/planning/pos_cmd",
                "super_status_topic": "/inspection/super/status",
                "position_topic": "/odom",
                "px4_position_enu_topic": "/px4_offboard_bridge/position_enu",
                "reference_frame": LaunchConfiguration("planning_frame"),
                "super_frame_id": "world",
                "goal_stride": ParameterValue(
                    LaunchConfiguration("super_goal_stride"), value_type=int
                ),
                "lock_reference_path": LaunchConfiguration("lock_super_reference_path"),
            }],
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
                "use_super_position_commands": LaunchConfiguration("use_super_planner"),
                "super_command_topic": "/planning/pos_cmd",
                "super_status_topic": "/inspection/super/status",
                "land_on_super_mission_complete": True,
                "path_duration_s": ParameterValue(
                    LaunchConfiguration("path_duration_s"), value_type=float
                ),
                "expected_frame": LaunchConfiguration("planning_frame"),
            }],
        ),
    ])
