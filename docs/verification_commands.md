# Reproduction commands

Run these commands from a Bash terminal in the workspace root:

```bash
cd "/home/venom/Documents/ChatGPT/巡检"
source /opt/ros/humble/setup.bash
source super_ws/install/setup.bash
```

## Rebuild from this checkout

The checkout path contains non-ASCII characters. ROS 2 Humble's IDL adapter
can corrupt that path when its build directory is underneath the checkout.
Keep the build and log directories in `/tmp` and use the included wrapper:

```bash
cd "/home/venom/Documents/ChatGPT/巡检"
source /opt/ros/humble/setup.bash
SUPER/scripts/build_ros2_humble.sh
source super_ws/install/setup.bash
```

For a clean rebuild after changing interface files or switching branches:

```bash
CLEAN_BUILD=1 SUPER/scripts/build_ros2_humble.sh
source super_ws/install/setup.bash
```

Do not use the default `colcon build` layout for this checkout; it puts the
generated IDL files under the UTF-8 path and can produce a missing
`QuadrotorState.idl` error. The wrapper uses an ASCII temporary build path and
copies a self-contained install tree, so the install setup remains usable even
after the temporary build directory is removed.

## Unit and artifact checks

The complete local verification can be run with one command:

```bash
cd "/home/venom/Documents/ChatGPT/巡检/SUPER"
./scripts/verify_simulation.sh
```

It runs the core/manager/Offboard contract tests, Python compilation, scenario
and 150-run artifact validation, and checks the installed ROS packages.

```bash
PYTHONPATH=SUPER/inspection_core \
  python3 -m unittest discover -s SUPER/inspection_core/test -q

ros2 run inspection_core run_baseline_scenario.py \
  --output artifacts/baseline_scenario.json
ros2 run inspection_core run_scenario_suite.py \
  --output artifacts/scenario_suite.json
ros2 run inspection_core run_ablation.py \
  --output artifacts/ablation.json
ros2 run inspection_core run_ablation_batch.py \
  --output artifacts/ablation_batch.json --repetitions 10
ros2 run inspection_core validate_artifacts.py artifacts/*.json
```

## ROS 2 baseline

```bash
export ROS_LOG_DIR=/tmp/super_ros_log
mkdir -p "$ROS_LOG_DIR"
ros2 launch mission_planner click_demo.launch.py
```

The click demo starts the simulator, SUPER planner and RViz. In RViz press
`G` and click a goal. A normal desktop with a usable display and DDS network
permissions is required for RViz and inter-process ROS communication.

## Inspection manager

```bash
ros2 launch inspection_manager inspection_manager.launch.py
ros2 topic echo /inspection/status
ros2 topic echo /inspection/coverage
ros2 topic echo /inspection/reference_path
```

The manager publishes a reference and waits for valid observations. It is not a
PX4 setpoint publisher and must not be treated as flight-control authority.

## Gazebo visual inspection demo

For a scene that makes the inspection task visible, launch the standalone
Gazebo view:

```bash
export ROS_LOG_DIR=/tmp/super_ros_log
mkdir -p "$ROS_LOG_DIR"
ros2 launch inspection_gazebo inspection_gazebo.launch.py
```

The default scene uses the native visualization world without the unstable
Gazebo ROS camera/lidar plugins. It contains two primitive three-leg triangular lattice towers and
6 mm visual-diameter conductors. The blue drone climbs from 1.5 m to 9.5 m
while orbiting the near tower; green cells show vertical inspection progress.
The scene is a visual task demonstration and is not PX4 SITL or a real-flight
acceptance test.

The scene contains two towers, three thin conductors, a faint red exclusion
indicator, a blue simulated drone, and green coverage cells that appear along
the vertical tower scan. The node reports the same progress in:

```bash
ros2 topic echo /inspection/gazebo/status
ros2 topic echo /inspection/gazebo/coverage
```

The plugin world remains available explicitly for experiments:

```bash
ros2 launch inspection_gazebo inspection_gazebo.launch.py \
  world:=/home/venom/Documents/ChatGPT/巡检/SUPER/inspection_gazebo/worlds/inspection_line.world
```

This view is for geometry and task-state communication. It is not a PX4
flight-control or sensor-accuracy acceptance test.

## Native PX4 inspection SITL

After installing the `iris_inspection` model into the PX4 checkout, the
one-command runner starts Gazebo Classic and PX4, waits for the MAVLink
heartbeat, sends the vertical two-orbit tower reference, requests landing,
and cleans up the child processes:

```bash
cd "/home/venom/Documents/ChatGPT/巡检"
INSPECTION_DURATION=45 \
  SUPER/scripts/run_px4_inspection_sitl.sh ~/PX4-Autopilot
```

Use `HEADLESS=1` when no display is available. The runner's log is written to
`/tmp/super_px4_inspection`. This is native PX4 SITL/MAVLink evidence; ROS
sensor topics still require the Gazebo ROS plugins and a matching `px4_msgs`
workspace.

## PX4 Offboard adapter dry run

If `/tmp/microxrce_install` has been cleaned, rebuild the Agent with:

```bash
cd "/home/venom/Documents/ChatGPT/巡检/SUPER"
git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent
./scripts/build_microxrce_agent.sh
export UXRCE_AGENT_BIN=/tmp/microxrce_install/bin/MicroXRCEAgent
```

The adapter is disabled by default and is safe to run with only this workspace:

```bash
source /opt/ros/humble/setup.bash
source super_ws/install/setup.bash
ros2 launch px4_offboard_bridge px4_offboard_bridge.launch.py
ros2 topic echo /px4_offboard_bridge/status
```

The status must remain `DISABLED_DRY_RUN`; no `/fmu/in/*` setpoints are
published. Enabling output requires a PX4-matched `px4_msgs` workspace,
continuous TimesyncStatus and VehicleLocalPosition, a measured origin/yaw
alignment, and an explicit review of `request_offboard` and `request_arm`.
See `px4_offboard_bridge/README.md` for the guarded command and
`hardware_intake.template.md` for the required evidence. This is not a SITL
or real-aircraft result.

For the integrated simulation launch, setpoint output is also explicitly
opt-in and mode/arming requests remain off:

```bash
ros2 launch inspection_manager tower_pointcloud_demo.launch.py \
  enable_px4_setpoints:=true request_offboard:=false request_arm:=false
```
