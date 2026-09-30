# Real sensor integration gate

The project now has a single launch entry for the physical sensors:

```bash
source /opt/ros/humble/setup.bash
source /home/venom/Documents/ChatGPT/巡检/super_ws/install/setup.bash
ros2 launch inspection_gazebo real_sensor_bringup.launch.py
```

This starts `realsense2_camera_node` with depth/color alignment and pointcloud
enabled, publishes static `base_link -> d435i_link` and `base_link ->
mid360_link` transforms, and leaves the MID-360 driver disabled until its
exact Livox package/executable is selected. To start a confirmed Livox driver:

```bash
ros2 launch inspection_gazebo real_sensor_bringup.launch.py \
  enable_livox:=true livox_package:=livox_ros_driver2 \
  livox_executable:=livox_ros_driver2_node
```

The transforms in the launch file are provisional mounting values. They must
be replaced by measured extrinsics. Completion requires a connected D435i,
MID-360 driver discovery, live `PointCloud2` topics, TF lookup at sensor
timestamps, and a recorded synchronization/calibration check.
