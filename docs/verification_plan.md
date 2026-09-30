# Verification plan for the current baseline

The task book defines the acceptance targets. This file records what is executable today and what remains blocked by hardware or an unselected SITL baseline.

## Executable now

1. Build all six SUPER ROS 2 packages from the `ros2-humble` commit.
2. Resolve `mission_planner` from the project install and parse `click_demo.launch.py`.
3. Run the `inspection_core` reference tests:

   ```bash
   cd /home/venom/Documents/ChatGPT/巡检
   PYTHONPATH=SUPER/inspection_core python3 -m unittest discover -s SUPER/inspection_core/test -v
   ```

4. Verify a straight single-span model, a piecewise-linear curved model, an arc scan, full-line clearance and observation-quality coverage bookkeeping.
5. Launch the Gazebo inspection view and observe the drone, thin conductors, triangular towers and progressive vertical coverage cells.
6. Run `px4_offboard_bridge` in its default dry-run mode; it must not publish `/fmu/in/*` topics.
7. Run the integrated tower demo with `health_fault_sim_node.py`; verify that a stale point-cloud/link-loss state retracts the path, reports the conservative action, and resumes only after health and a quality-gated observation recover.
8. Record the real MID-360/Fast-LIO evidence topics with `scripts/record_mid360_fastlio_bag.sh`; keep the generated `ros2 bag info` output beside the bag and replay it in an unrestricted desktop ROS 2 domain.

## Acceptance mapping

| Task-book item | Current evidence | Remaining gate |
| --- | --- | --- |
| T01 SUPER official example | Build and launch-argument verification; local GUI run confirmed by user | Record a desktop launch log and lock the final workspace image |
| T04 line model and scan | `inspection_core` model, parallel-transport frame and tests | Add ROS messages/preview and compare against a surveyed or truth model |
| T06 clearance and candidate search | Full-line clearance primitive and rejection report | Add candidate radius/angle enumeration and SUPER trajectory validation |
| T07 coverage/reconnect | Quality-gated grid, nearest safe-cell selection, explicit `WAITING_FOR_VALID_OBSERVATIONS`, and health-fault brake/reconnect gate | Desktop DDS replay and bag-based timing evidence remain; recorder is now provided |
| T08 visual inspection review | Gazebo scene with thin conductors, triangular towers and progressive vertical coverage cells | Connect the visual scene to live sensor/trajectory truth before using it as acceptance evidence |
| T03 FAST-LIO2/PX4 | Locked FAST-LIO2/Livox commits, verified 100 Hz simulated topics, guarded adapter source/build plus PX4 Classic evidence | Run the provided bag recorder/replay, then complete timesync, axis/reset and Offboard tests |
| T10 D435i/YOLO | Not started; camera and dataset unavailable | Calibrate, label, associate point cloud and implement explicit rejection cases |

The first production safety gate is a continuous or conservative swept-trajectory checker. Waypoint-only checks are insufficient for release.
