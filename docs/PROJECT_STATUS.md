# Project status

## Completed in the current baseline

- SUPER is checked out at `ros2-humble`, commit `2ad3419c127a617c6d7df6925e81a14175a9c096`.
- The six existing SUPER packages plus four project packages (`inspection_core`, `inspection_manager`, `inspection_gazebo`, and `px4_offboard_bridge`) build on ROS 2 Humble and install into `super_ws/install` when the ASCII-path build wrapper is used.
- The official click demo starts its simulator and planner nodes after sourcing the rebuilt workspace; RViz/GLFW still requires a desktop display and DDS permissions.
- A standalone Gazebo inspection view now shows 6 mm visual-diameter conductors, two three-leg triangular lattice towers generated from primitive SDF geometry, a faint line-exclusion indicator, and a drone that climbs and orbits the tower while coverage cells progress vertically.
- The installed PX4 `iris_inspection` Classic target has been rebuilt and started against `px4_inspection.world`: the model SDF was selected, `SYS_AUTOSTART=1018` loaded, Gazebo accepted the TCP 4560 connection, and PX4 completed its startup script. This is native SITL startup evidence; it does not enable Offboard control or prove ROS sensor topics.
- `scripts/run_px4_inspection_sitl.sh` now provides a repeatable native-SITL path: it checks the model/target, starts the PX4 target, runs `px4_mavlink_inspection.py` after the UDP link is available, and cleans up the isolated process group. The controller uses a position-only MAVLink mask, confirms arming, stops the Offboard stream for PX4 landing, and waits for disarm. A full elevated run was verified on 2026-09-23: PX4 armed, executed the vertical two-orbit reference trajectory, accepted `NAV_LAND`, and disarmed successfully. ROS sensor topics remain a separate open gate.
- The official click-demo launch file resolves after sourcing the matching shell setup files.
- `inspection_core` now provides a dependency-light line model, parallel-transport frame, time-parameterized circular scan, full-line clearance report, and quality-gated coverage grid.
- A straight single-span scenario and executable unit tests are included.
- ROS Gazebo Classic packages are installed. The inspection world now includes package-local D435i-style depth and MID-360-style ray models, and the launch file exports their model path automatically. Static XML and launch checks pass; live sensor-topic verification requires a normal desktop Gazebo process because the restricted execution environment cannot create Gazebo network interfaces.
- The visual inspection scene now uses a self-contained official PX4 Iris mesh model with explicit camera and MID-360 mounts. Pose updates are serialized per entity to avoid an out-of-order `SetEntityState` backlog that can make the aircraft flicker; the visual model remains separate from the PX4 SITL model.
- Added a robust point-cloud tower observation baseline in `inspection_core/tower_localization.py`: trimmed XY center, percentile height bounds, minimum point/height gates, and an uncertainty estimate for safety-margin inflation. Nineteen inspection-core tests pass; live ROS sensor input is still pending.
- Added `inspection_manager/tower_pointcloud_node.py`, which consumes `sensor_msgs/PointCloud2`, rejects malformed/sparse clouds, publishes the estimated center (`PoseStamped`), height/uncertainty (`Vector3Stamped`), status, and an RViz marker. `inspection_manager` builds with these ROS dependencies in the current environment.
- Fixed the PointCloud2 parser for Humble's structured NumPy return from `sensor_msgs_py.read_points`; a regression test now covers valid XYZ parsing and missing fields. An in-memory replay of the synthetic tower cloud yields center `(4.01, -0.99)`, estimated height `11.40 m`, and 153 accepted points. The live ROS demo launch must show both simulator and localizer nodes before topic output is considered verified.
- Connected localization outputs to `inspection_manager`: matching `PoseStamped` and `Vector3Stamped` observations now determine tower base/top, a rising two-orbit path, radius `max(3.0 m, estimated footprint + 2.0 m standoff)`, and 1.5 m/1.0 m vertical margins. The manager rejects mismatched frames/stamps, clears invalid observations, and stops publishing a path after a 2 s observation timeout. The demo launch now starts simulator, localizer, and manager together. Nine manager tests and nineteen core tests pass; live reference-path topic output still needs desktop DDS verification.
- Added `inspection_gazebo/launch/real_sensor_bringup.launch.py` and `docs/real_sensor_integration.md` for physical D435i startup, provisional base-to-sensor TFs, and an explicit MID-360 Livox driver hook. The host has `realsense2_camera_node`, but no connected camera/Livox device was detected and no Livox driver package is installed. The launch also exposed a host ROS typesupport mismatch in `static_transform_publisher` (`std_msgs` introspection symbol error); this must be repaired before TF runtime validation.
- Added a pure ROS simulation sensor node (`inspection_sensor_sim_node.py`) that publishes D435i-style RGB/depth images and MID-360/Livox-style `PointCloud2` streams without Gazebo ROS plugins. The existing demo launch now feeds its simulated lidar cloud into tower localization and the dynamic path planner; the package builds successfully. This is the supported simulation path on the current host, while Gazebo sensor plugins remain optional visualization only.
- Aligned the demo point cloud to the Gazebo tower origin and 10 m model height, then added the guarded PX4 ROS bridge in dry-run mode to the same launch. Offline end-to-end replay produces a center near `(0,0)`, tower bounds `0.31–9.69 m`, 3.33 m orbit radius, 2.0 m modeled tower-surface clearance, scan altitude `1.81–8.69 m`, and 145 path points; the first ENU point converts to PX4 NED as expected. The bridge stays disabled and publishes no `/fmu/in` commands. Launch processes start, but this sandbox cannot create DDS UDP sockets, so the full ROS graph must be verified on the user's desktop.
- Added synthetic moving-obstacle PointCloud2 input on `/inspection/obstacles/points` and conservative path clearance checks in `inspection_manager`. The manager now rejects a blocked orbit, tries configured fallback radii (`3/4/5/6 m`), republishes a safe path when an obstacle clears, and reports `NO_SAFE_TRAJECTORY_DYNAMIC_OBSTACLE` while all candidates are blocked. Management regression tests are 10/10 and inspection-core tests are 21/21; `inspection_manager` rebuilds cleanly. Desktop DDS topics were verified in the completed simulation run.
- Added `health_fault_sim_node.py` and a health gate to the task manager. The deterministic simulator cycles through healthy operation, stale point-cloud, link-loss, and recovery states; a fault transitions the task to `BRAKING`, retracts the reference path, and recovery requires a fresh quality-gated observation before `SCANNING` resumes. The parser/action and manager transition tests pass in the ROS Humble environment (22 manager tests, 21 core tests).

## Open gates

### Simulation verification update (2026-09-25)

- Core and manager regression suites pass: 21/21 and 10/10.
- The geometry scenario suite also passes: clear straight and bent spans select safe candidates, while the fully blocked case is rejected with `obstacle:blocking_obstacle`; the 150-run artifact validates successfully.
- The manager now consumes `/inspection/coverage_quality`; low-quality observations leave the task waiting, while valid quality-gated observations advance it to scanning and update the reported coverage ratio. Manager regression is now 11/11.
- Native PX4 SITL completes the inspection scan, landing, and disarm.
- `MicroXRCEAgent` v2.4.3 is built for ROS Humble; automatic Agent startup and PX4 DDS synchronization were verified. PX4 creates `vehicle_local_position` and `timesync_status` writers and completes the scan.
- The desktop ROS graph publishes D435i color/depth, Livox-style point cloud, obstacle cloud, fusion pose/status, coverage, and reference-path topics.
- `iris_inspection` now keeps visible D435i/MID-360 mounts without embedding the crashing Gazebo ROS sensor plugins. Sensor data uses the stable pure-ROS simulation nodes.
- The PX4 `iris_inspection` model is now flattened from the stock Iris SDF with lightweight D435i/MID-360 mounts. This fixes the nested-model motor-plugin scope issue: the sensor-equipped model was physically verified to climb about 9.55 m and land/disarm in the latest SITL ULog.

- Hardware facts are still missing: MID-360S model/firmware, D435i firmware and mounting, flight computer, PX4 version, and ownership/contributor record.
- FAST-LIO2 commit `633f681dc5bda92de63379db056016103de5d3c8` and Livox ROS driver 2 commit `960bb5702c1e3a6197ff8fc5c3c9bca4e2e87f37` are locked from the inspected simulation workspace. RealSense ROS remains unselected; PX4 source commit is still unrecorded.
- The ROS task state machine now includes a simulation health/fault gate and deterministic fault injector. A bag replay, dedicated RViz coverage overlay, and full SUPER trajectory adapter are not yet implemented. A guarded PX4 Offboard adapter now exists, but it is disabled by default and has no completed Offboard flight evidence; the Gazebo view remains a visualization baseline.
- The guarded PX4 adapter now has offline ENU-to-NED, origin/yaw, and finite-output contract tests. These tests verify the conversion boundary only; they do not enable `/fmu/in` output or claim an Offboard flight.
- A clean native PX4 SITL run on 2026-09-30 completed the full controller state sequence: TCP 4560 connection, arming, Offboard setpoint stream through 99% of the two-orbit rising reference, `NAV_LAND`, landing detection, and disarm. Its ULog is recorded at `artifacts/px4_ulog_metrics_clean_20260930.json`; estimated vertical excursion was only about 0.92 m, so the physical 8.5 m climb remains supported only by the separate historical ULog.
- The MID-360 simulation chain was run in ROS domain 42: `/livox/imu`, `/livox/lidar`, `/cloud_registered`, and `/odom` each measured approximately 100 Hz. This proves the simulated sensor-to-FAST-LIO2 path only; there is no physical MID-360S rosbag evidence.
- The task book's clearance and performance thresholds remain targets until runs produce raw logs and statistics. The latest clean-environment build completed all 10 packages after regenerating the 6 mm-conductor world.

## Next implementation order

1. Verify the new three-node demo's reference path on the desktop ROS graph; then connect PointCloud2 to D435i/MID-360 drivers and validate TF/calibration with a bag.
2. Verify dynamic-obstacle topic/path switching on the desktop ROS graph, then extend the sampled checker to the vehicle's swept-volume limits.
3. Connect the validated reference path to PX4/SUPER control only after frame/reset, timing, and obstacle checks pass.

### PX4 message workspace audit (2026-09-22)

`/home/venom/venom_ws` contains generated C++ headers, but the installed Python
message file for `VehicleLocalPosition` is zero bytes and importing the message
fails. The guarded bridge now reports this as an incomplete `px4_msgs` build
instead of treating the workspace as usable. A PX4-Autopilot firmware checkout
and a matching rebuilt message workspace are still required before SITL or
Offboard evidence can be collected.

The bridge itself starts in a clean shell with `ROS_LOG_DIR=/tmp/super_ros_log`
and reports `DISABLED_DRY_RUN`; it does not publish `/fmu/in/*`. The current
container cannot create DDS UDP sockets (`getifaddrs`/`TRANSPORT_UDP` permission
errors), so this runtime check is startup-only and is not a ROS graph or SITL
verification.
