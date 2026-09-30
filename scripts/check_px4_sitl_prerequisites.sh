#!/usr/bin/env bash
set -u

root="${1:-$HOME/PX4-Autopilot}"
ok=0
fail=0
check() {
  if "$@"; then printf 'OK   %s\n' "$*"; ok=$((ok+1)); else printf 'MISS %s\n' "$*"; fail=$((fail+1)); fi
}

check test -d "$root"
check test -f "$root/Tools/simulation/gazebo-classic/sitl_run.sh"
check test -x "$root/build/px4_sitl_default/bin/px4"
check test -f /home/venom/venom_ws/install/setup.bash
check test -f "$(cd "$(dirname "$0")/.." && pwd)/px4_offboard_bridge/launch/px4_offboard_bridge.launch.py"
check test -f "$root/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_inspection/iris_inspection.sdf"
check test -f "$root/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/inspection_depth_camera/inspection_depth_camera.sdf"
check test -f "$root/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/inspection_lidar/inspection_lidar.sdf"

# These libraries are supplied by the ROS Gazebo Classic packages.  The PX4
# and Gazebo processes can start without them, but camera/lidar/state ROS
# topics will be absent, so report that separately instead of hiding it.
check test -f /opt/ros/humble/lib/libgazebo_ros_openni_kinect.so
check test -f /opt/ros/humble/lib/libgazebo_ros_ray_sensor.so
check test -f /opt/ros/humble/lib/libgazebo_ros_state.so

printf 'PX4 SITL prerequisite summary: %d present, %d missing\n' "$ok" "$fail"
if [ "$fail" -ne 0 ]; then
  printf '%s\n' 'Install/build PX4-Autopilot and rebuild matching px4_msgs before enabling setpoints.'
  exit 2
fi
exit 0
