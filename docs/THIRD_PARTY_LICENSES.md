# Third-party and ownership ledger

This ledger is an engineering boundary record for the prototype. It is not a legal opinion. Before publishing code or filing either registration, replace every `review required` entry with a source license, version/commit, and the permitted use.

| Component | Source/version | Current evidence | Intended use | Action |
| --- | --- | --- | --- | --- |
| SUPER | `ros2-humble`, `2ad3419c127a617c6d7df6925e81a14175a9c096` | Package manifests declare BSD for most packages; `mars_quadrotor_msgs` declares Apache License 2.0 | Planning, map, simulator and message baseline | Review upstream license files and retain notices |
| ROS 2 Humble | `/opt/ros/humble` | Installed system distribution | Middleware, launch and message runtime | Record image/package manifest for the delivery environment |
| Eigen/PCL/OpenCV/Boost | System packages used by SUPER | Detected during the successful build | Geometry, point cloud and visualization support | Export package versions and licenses with the release |
| FAST-LIO2 | `633f681dc5bda92de63379db056016103de5d3c8` | Installed and verified in MID-360 simulation | LIO and de-skewed point cloud | Physical sensor validation remains |
| Livox ROS driver 2 | `960bb5702c1e3a6197ff8fc5c3c9bca4e2e87f37` | Installed and verified in MID-360 simulation | LiDAR/IMU driver | Confirm MID-360S firmware/hardware support |
| RealSense ROS | Planned; commit not selected | D435i not verified | RGB acquisition and calibration | Confirm SDK/firmware and model license |
| PX4 | `6ea3539157ca358c70a515878b77077af7d4611d` (`v1.16.0-dirty`) | Local SITL checkout; historical ULog verified | Offboard execution and estimator interface | Clean Gazebo/TCP 4560 rerun remains open |
| `inspection_core` | Workspace-authored, uncommitted | New reference implementation in this repository | Line model, scan geometry, clearance and coverage bookkeeping | Keep ownership and contributor record; license is internal until confirmed |

The project-authored modules must not copy third-party source. They consume documented interfaces and keep the upstream component and project version in the run metadata.
