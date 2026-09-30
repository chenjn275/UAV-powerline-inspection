# Task-book delivery matrix

Status is evidence-based as of the current workspace state. The 2026-09-25
simulation update below supersedes older notes that predate the PX4 DDS and
pure-ROS sensor verification.

Current simulation evidence: core 21/21 and manager 22/22 tests pass; native
PX4 SITL completes the inspection scan and landing; MicroXRCEAgent v2.4.3
connects PX4 DDS topics; the desktop ROS graph publishes D435i color/depth,
Livox-style point cloud, obstacle, fusion, coverage, and reference-path topics.

| Task-book deliverable | Evidence in workspace | Status |
| --- | --- | --- |
| SUPER ROS 2 official example | `ros2-humble` commit `2ad3419`, six original packages build, click launch resolves | Verified in local environment; GUI/DDS need desktop run |
| Gazebo inspection visualization | `inspection_gazebo` world, generated triangular lattice tower, launch file and progress node | Built and XML-validated; ROS Gazebo plugins are optional due host crashes; pure ROS D435i/MID-360 sensor simulation now supplies usable image/depth/PointCloud2 topics |
| Reproducible dependency baseline | `docs/versions.lock.yaml`, `docs/THIRD_PARTY_LICENSES.md` | Partial; FAST-LIO2/Livox/PX4 commits are recorded, RealSense and hardware firmware remain unset |
| Known line model | `inspection_core/line_model.py`, straight-span config and tests | Verified reference implementation |
| Stable scan frame and constant-radius reference | `generate_arc_scan`, 3-D nearest-line checks, tests | Verified numerically for polyline models |
| Candidate radius/angle selection | `candidate_search.py`, safe-candidate tests, scenario suite and baseline JSON report | Verified: clear straight/bent spans select safe candidates and a fully blocked span is rejected |
| Full-trajectory conservative clearance | sampled path checker, moving obstacle PointCloud2 simulator, fallback-radius replanning | Verified in manager/core regression tests and desktop topic run; final swept-volume bounds remain open |
| Coverage bookkeeping and reconnect | `coverage.py`, `state_machine.py`, `quality_coverage_sim_node.py`, manager quality subscription and baseline interruption report | Verified: invalid/low-quality observations do not advance the task; valid quality-gated observations update state and coverage ratio |
| ROS 2 task manager | `inspection_manager` package, PointCloud2-derived tower center/extent, dynamic rising-orbit path, obstacle PointCloud2 input and fallback-radius replanning | Built; manager 22/22 and core 21/21 tests pass; desktop topics verified |
| Coordinate contract | `docs/coordinate_contract.md`, `coordinate.py` and tests | Geometry verified; PX4 alignment measurement pending |
| Failure handling | `health.py`, `health_fault_sim_node.py`, manager health gate and state transition tests | Simulation fault sequence verified offline; desktop DDS replay still required for release evidence |
| FAST-LIO2 and sensor time/TF chain | `/home/venom/venom_ws` MID-360 simulation, locked commits, `SUPER/scripts/record_mid360_fastlio_bag.sh`, and measured ROS topic rates | Simulated Livox/IMU→FAST-LIO2 chain verified at about 100 Hz with `/cloud_registered` and `/odom`; physical MID-360S, rosbag capture, measured extrinsics and hardware time-sync remain open |
| PX4 Offboard and SITL | `px4_offboard_bridge` guarded ENU→NED adapter; `px4_offboard_bridge/test/test_offboard_bridge_contract.py`; `scripts/px4_mavlink_inspection.py`; PX4 flattened `iris_inspection` Classic target and `px4_inspection.world`; latest ULog `06_43_35.ulg` | Verified: sensor-equipped `iris_inspection` reaches about 9.55 m ground-truth altitude, completes the two-orbit reference, requests NAV_LAND and disarms. |
| D435i/YOLO semantic localization | `inspection_core/tower_localization.py` plus `inspection_manager/tower_pointcloud_node.py` PointCloud2 adapter | Geometry and Humble PointCloud2 parser pass offline tests; live D435i/MID-360 topics, calibration and YOLO labels remain open |
| A/B/C 150-run evaluation | `artifacts/ablation_batch_150.json`, `inspection_core/scripts/run_ablation_batch.py` | 150-run batch complete; summary includes coverage, fusion-valid rate, replans, and vision quality |
| Software copyright materials | Scope and ownership boundary documented | Draft only |
| Patent search and technical disclosure | Candidate mechanism described in task book | Search/evidence package not completed |
| Real-aircraft tests | No hardware facts or flight records | Not started |

The unresolved rows are explicit release gates. They must not be marked complete from the current offline geometry evidence.
