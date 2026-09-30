# Guarded PX4 Offboard adapter

`px4_offboard_bridge` is the control-boundary package for the inspection
reference path. It consumes `/inspection/reference_path` (`nav_msgs/Path` in
ROS ENU/map coordinates), converts each point to PX4 local NED, and can publish
`OffboardControlMode` plus `TrajectorySetpoint` at 30 Hz.

All output is disabled by default. Even when enabled, the node refuses to
publish setpoints until it has a fresh path, a valid `/fmu/out/vehicle_local_position`
message, and a fresh `/fmu/out/timesync_status`. Offboard mode and arming are
separate parameters and remain false by default. This prevents the Gazebo
visualization or a stale path from becoming a flight command.

The runtime `px4_msgs` package must match the PX4 firmware line. The existing
machine has a separate `~/venom_ws` bridge workspace documenting PX4 v1.16.x
and `px4_msgs` `release/1.16`; that workspace is not silently mixed into this
SUPER build. Before enabling output, fill `SUPER/docs/hardware_intake.template.md`,
measure the ENU/NED origin and yaw, verify timesync and estimator reset behavior,
and run SITL before any propeller-on test.

Dry-run launch (safe default):

```bash
source /opt/ros/humble/setup.bash
source super_ws/install/setup.bash
ros2 launch px4_offboard_bridge px4_offboard_bridge.launch.py
```

Only after the PX4-matched workspace is sourced and the alignment checks pass,
explicitly enable the setpoint stream. Arming is still a separate decision:

```bash
source /opt/ros/humble/setup.bash
source /home/venom/venom_ws/install/setup.bash
source "/home/venom/Documents/ChatGPT/巡检/super_ws/install/setup.bash"
ros2 launch px4_offboard_bridge px4_offboard_bridge.launch.py \
  enable_setpoints:=true request_offboard:=false request_arm:=false
```

This package is source/build verified only. It is not evidence of PX4 SITL or
real-aircraft flight until the corresponding PX4, DDS, alignment and flight
logs are archived.
