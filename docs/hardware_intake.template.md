# Hardware and external-version intake

Fill this record before selecting FAST-LIO2, Livox, RealSense or PX4 commits.
Blank or unknown values are intentional blockers for hardware-dependent claims.

| Item | Required value | Evidence/location |
| --- | --- | --- |
| LiDAR exact model | MID-360S or confirmed alternative | nameplate photo / datasheet |
| LiDAR firmware |  | device report |
| LiDAR driver | repository URL + commit | lock file |
| LiDAR point time field |  | message definition / bag inspection |
| IMU source and rate |  | driver config / measured log |
| Flight computer | CPU/GPU/RAM/storage | system report |
| Camera | D435i exact SKU and firmware | device report |
| Camera mounting | rigid transform and optical frame | calibration record |
| PX4 version | tag/commit and airframe | SITL or vehicle report |
| ROS/Gazebo version | distribution and simulator | package/image manifest |
| External-odometry interface | topic/message/frame | bridge configuration |
| ENU/NED origin | measured translation | alignment experiment |
| Initial yaw difference | measured radians/degrees | alignment experiment |
| Estimator reset behavior | tested response | PX4/LIO log |
| Project owne继续rship | developers, unit, contribution terms | signed record |

Until the relevant rows are filled and verified, the project may claim only the
offline geometry and ROS 2 SUPER baseline documented in `DELIVERY_MATRIX.md`.
