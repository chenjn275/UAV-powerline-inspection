# Gazebo inspection view

This package provides a deliberately simple visual acceptance scene for the
known-line inspection baseline. Gazebo shows two three-leg triangular lattice
towers, three thin conductors, a faint red line-exclusion indicator, a blue
drone, and green coverage cells that appear as the drone climbs and orbits the
tower. The conductors use a 6 mm visual diameter and the exclusion indicator
is transparent so it cannot be mistaken for a thick wire.

The tower geometry is generated from primitive SDF cylinders by
`scripts/generate_inspection_world.py`; it is a self-contained triangular
lattice visual model rather than a downloaded vendor asset. It is a
visualization and task-state demonstration. It does not publish PX4 setpoints
and it does not prove vehicle dynamics, sensor accuracy, or flight safety.

The demo path is deliberately vertical: the drone climbs from 1.5 m to 9.5 m
while making two orbits around the near tower. Green cells report the planned
vertical scan progress. This is the visual counterpart of the task-book's
``上下飞行巡检`` requirement; it is not a replacement for a real vehicle or
PX4 SITL.

## PX4 Classic inspection world

The SUPER source contains a `yunque-M.dae` RViz mesh, but it is not a URDF or
Gazebo dynamics model. For PX4 Classic use the project `iris_inspection`
airframe. The installer combines the SUPER/PX4 Iris vehicle with the project's
D435i-style depth camera and MID-360-style ray sensor. The inspection world is
generated separately so the triangular towers and thin conductors remain
visible.

Generate and launch it with:

```bash
python3 SUPER/inspection_gazebo/scripts/generate_px4_inspection_world.py
cd ~/PX4-Autopilot
source /opt/ros/humble/setup.bash
export PX4_SITL_WORLD="/home/venom/Documents/ChatGPT/巡检/SUPER/inspection_gazebo/worlds/px4_inspection.world"
export PX4_SIM_MODEL=iris_inspection
make px4_sitl gazebo-classic_iris_inspection
```

The ROS launch file exports the package-local model directory automatically,
and the demo keeps both sensor models mounted to the moving inspection
aircraft. Use `gui:=false` for a headless sensor check if Gazebo Classic's GUI
renderer crashes with the local Wayland/OpenGL driver.

The visual aircraft uses the official PX4 Iris body and propeller meshes copied
into `models/inspection_aircraft`; this visual-only model is separate from the
PX4 SITL `iris_inspection` model, so changing the display model does not change
the flight-control dynamics or MAVLink target.

For a repeatable demonstration that starts PX4, waits for the MAVLink
heartbeat, drives the vertical two-orbit inspection path, and cleans up the
simulator on exit, use the project runner:

```bash
cd "/home/venom/Documents/ChatGPT/巡检"
INSPECTION_DURATION=45 \
  SUPER/scripts/run_px4_inspection_sitl.sh ~/PX4-Autopilot
```

The default run opens the Gazebo client. Set `HEADLESS=1` for a CI or
terminal-only run. The controller sends `SET_POSITION_TARGET_LOCAL_NED`
references over MAVLink, climbs from 1.5 m to 9.5 m while making two orbits,
then requests landing. Set `INSPECTION_DURATION`, `INSPECTION_RADIUS`,
`INSPECTION_Z_MIN`, `INSPECTION_Z_MAX`, or `INSPECTION_ORBITS` to adjust the
reference without editing the script. The PX4/Gazebo console is saved under
`/tmp/super_px4_inspection`.

This runner is the native PX4 SITL path. It does not silently claim ROS sensor
topics: those require the ROS Gazebo Classic plugins (`ros-humble-gazebo-ros`
and `ros-humble-gazebo-plugins`) and a matching PX4 `px4_msgs` workspace.

This verifies PX4 SITL model integration. The installer also places visible
D435i and MID-360 housings next to the PX4 model. The ROS Gazebo sensor plugins
are intentionally optional: on hosts where Gazebo Classic's renderer or ROS
plugin initialization is unstable, embedding them in the PX4 model can crash
`gzserver`. The supported sensor pipeline is the pure-ROS simulator started by
the manager demo:

```bash
source /opt/ros/humble/setup.bash
source /home/venom/Documents/ChatGPT/巡检/super_ws/install/setup.bash
ros2 launch inspection_manager tower_pointcloud_demo.launch.py
```

It publishes D435i-style RGB/depth (`/inspection_d435i/color/image_raw`,
`/inspection_d435i/depth/image_rect_raw`) and MID-360/Livox-style
`sensor_msgs/PointCloud2` (`/livox/lidar`). The vision simulator emits quality,
occlusion, miss/false-positive and per-view confidence fields; fusion rejects
stale, sparse, low-contrast or low-confidence frames before publishing a pose.
Quality-gated coverage counts each accepted camera view once. Check the actual
names on the running system with:

```bash
ros2 topic list | grep -E 'inspection|camera|depth|points|fmu'
```

These topics are simulation data for validating the perception-to-planning
pipeline; they are not proof of D435i or MID-360 hardware equivalence. The
Gazebo ROS sensor plugins remain an optional visualization path and are not
required by the verified PX4/native or pure-ROS simulation paths.

To install the PX4 inspection model with the visible D435i/MID-360 housings
into a local PX4 checkout:

```bash
SUPER/scripts/install_px4_inspection_model.sh ~/PX4-Autopilot
cd ~/PX4-Autopilot
make px4_sitl_default
export PX4_SITL_WORLD="/home/venom/Documents/ChatGPT/巡检/SUPER/inspection_gazebo/worlds/px4_inspection.world"
unset HEADLESS
make px4_sitl gazebo-classic_iris_inspection
```
