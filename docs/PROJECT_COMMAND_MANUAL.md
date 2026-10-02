# 四旋翼输电线路巡检仿真命令手册

本手册对应 `/home/venom/Documents/ChatGPT/巡检/SUPER` 当前版本。所有 ROS 终端必须使用相同的 `ROS_DOMAIN_ID`；下面的完整 PX4 闭环统一使用 `53`。命令中 `true`/`false` 必须拼写完整，不能写成 `ture`。

## 0. 环境变量和目录

```bash
export SUPER_ROOT="/home/venom/Documents/ChatGPT/巡检/SUPER"
export PX4_ROOT="/home/venom/PX4-Autopilot"
export FAST_LIO_WORKSPACE="/home/venom/venom_ws"
export SUPER_WORKSPACE="/home/venom/Documents/ChatGPT/巡检/super_ws"
```

每个新终端按需要执行：

```bash
source /opt/ros/humble/setup.bash
source "$FAST_LIO_WORKSPACE/install/setup.bash"       # 需要 FAST-LIO2/Livox 时
source "$SUPER_WORKSPACE/install/setup.bash"          # 需要项目 ROS 包时
export ROS_DOMAIN_ID=53
```

## 1. 安全停止旧仿真

先杀 ROS 控制器、ROS launch、Agent，再杀 PX4/Gazebo：

```bash
for pattern in '[p]x4_offboard_bridge.py' '[s]uper_planner_adapter.py' '[t]ower_pointcloud_demo.launch.py' '[M]icroXRCEAgent'; do
  for pid in $(pgrep -f "$pattern" || true); do
    kill -TERM "$pid" 2>/dev/null || true
  done
done

for name in px4 gzserver gzclient gazebo rviz2; do
  for pid in $(pgrep -x "$name" || true); do
    kill -TERM "$pid" 2>/dev/null || true
  done
done

sleep 3
pgrep -af 'px4|gzserver|gzclient|gazebo|MicroXRCEAgent|rviz2|px4_offboard_bridge|tower_pointcloud_demo' || true
```

## 2. 推荐：完整可视化自主巡检

此模式包含 Gazebo GUI、RViz、合成 MID-360/IMU、FAST-LIO2、点云定位、自动半径、障碍/健康/质量门、SUPER、PX4 Offboard、自动降落和解锁。

```bash
cd "$SUPER_ROOT"
source /opt/ros/humble/setup.bash

for pattern in '[p]x4_offboard_bridge.py' '[s]uper_planner_adapter.py' '[t]ower_pointcloud_demo.launch.py' '[M]icroXRCEAgent'; do
  for pid in $(pgrep -f "$pattern" || true); do kill -TERM "$pid" 2>/dev/null || true; done
done
for name in px4 gzserver gzclient gazebo rviz2; do
  for pid in $(pgrep -x "$name" || true); do kill -TERM "$pid" 2>/dev/null || true; done
done

env \
ROS_DOMAIN_ID=53 \
PX4_ROOT="$PX4_ROOT" \
FAST_LIO_WORKSPACE="$FAST_LIO_WORKSPACE" \
SUPER_WORKSPACE="$SUPER_WORKSPACE" \
PX4_INSPECTION_USE_ROS_OFFBOARD=1 \
PX4_INSPECTION_USE_FAST_LIO=1 \
PX4_INSPECTION_USE_SUPER_PLANNER=1 \
PX4_ROS_OFFBOARD_TIMEOUT_S=360 \
PX4_SUPER_GOAL_STRIDE=8 \
PX4_ROS_HEALTH_SEQUENCE='READY:360' \
PX4_ROS_OBSTACLE_AMPLITUDE=0 \
PX4_ROS_OBSTACLE_RADIUS=0 \
PX4_ROS_FAST_LIO_RVIZ=true \
HEADLESS=0 \
GAZEBO_MASTER_URI=http://127.0.0.1:11345 \
GAZEBO_IP=127.0.0.1 \
GAZEBO_HOST=127.0.0.1 \
bash scripts/run_px4_inspection_sitl.sh
```

成功标志：

```text
SUPER_PATH_READY goals=19 points=145
FAST_LIO_SENSOR_POSE_CONNECTED
OUTPUTTING_SETPOINT source=SUPER
SUPER_MISSION_COMPLETE
PX4_DISARMED_AFTER_LAND
ROS Offboard inspection SITL completed successfully
```

RViz 启动失败时先确认参数是 `PX4_ROS_FAST_LIO_RVIZ=true`，而不是 `ture`。

## 3. 完整闭环无界面模式

服务器或无显示器运行时使用：

```bash
cd "$SUPER_ROOT"
source /opt/ros/humble/setup.bash
env \
ROS_DOMAIN_ID=53 \
PX4_ROOT="$PX4_ROOT" \
FAST_LIO_WORKSPACE="$FAST_LIO_WORKSPACE" \
SUPER_WORKSPACE="$SUPER_WORKSPACE" \
PX4_INSPECTION_USE_ROS_OFFBOARD=1 \
PX4_INSPECTION_USE_FAST_LIO=1 \
PX4_INSPECTION_USE_SUPER_PLANNER=1 \
PX4_ROS_OFFBOARD_TIMEOUT_S=360 \
PX4_SUPER_GOAL_STRIDE=8 \
PX4_ROS_HEALTH_SEQUENCE='READY:360' \
PX4_ROS_OBSTACLE_AMPLITUDE=0 \
PX4_ROS_OBSTACLE_RADIUS=0 \
PX4_ROS_FAST_LIO_RVIZ=false \
HEADLESS=1 \
GAZEBO_MASTER_URI=http://127.0.0.1:11345 \
GAZEBO_IP=127.0.0.1 \
GAZEBO_HOST=127.0.0.1 \
bash scripts/run_px4_inspection_sitl.sh
```

## 4. 只看 PX4/Gazebo，不启动 ROS 传感器

此模式只验证 PX4 SITL、Gazebo、MAVLink 位置控制和降落，不启动 FAST-LIO2、RViz 或 ROS Offboard。使用 `iris` 时不加载项目的传感器挂载模型；使用 `iris_inspection` 时可看到传感器外形，但仍不发布 ROS 传感器话题。

```bash
cd "$SUPER_ROOT"
export PX4_ROOT="$PX4_ROOT"
export PX4_SIM_MODEL=iris
export INSPECTION_DURATION=45
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_IP=127.0.0.1
export GAZEBO_HOST=127.0.0.1
export HEADLESS=0
bash scripts/run_px4_inspection_sitl.sh
```

## 5. 只运行传感器、FAST-LIO2 和定位，不控制飞机

用于检查 `/livox/*`、`/cloud_registered`、`/odom`、TF 和参考路径，不发布 PX4 setpoint：

```bash
cd "$SUPER_ROOT"
source /opt/ros/humble/setup.bash
source "$FAST_LIO_WORKSPACE/install/setup.bash"
source "$SUPER_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=53

ros2 launch inspection_manager tower_pointcloud_demo.launch.py \
  use_fast_lio:=true \
  fast_lio_rviz:=true \
  use_super_planner:=false \
  enable_px4_setpoints:=false \
  request_offboard:=false \
  request_arm:=false \
  planning_frame:=odom \
  tower_x:=0.0 \
  tower_y:=0.0 \
  tower_height_m:=10.0
```

没有 PX4 位姿时传感器会使用静止 bootstrap；接入 PX4 后日志会出现 `FAST_LIO_SENSOR_POSE_CONNECTED`。

## 6. 不使用 FAST-LIO 的纯几何/任务管理器模式

此模式使用 `/inspection/tower_points` 合成 `PointCloud2`，验证点云定位、半径计算、上升螺旋、障碍门和覆盖率，不启动 FAST-LIO2：

```bash
cd "$SUPER_ROOT"
source /opt/ros/humble/setup.bash
source "$SUPER_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=53

ros2 launch inspection_manager tower_pointcloud_demo.launch.py \
  use_fast_lio:=false \
  fast_lio_rviz:=false \
  use_super_planner:=false \
  enable_px4_setpoints:=false \
  planning_frame:=map \
  tower_x:=0.0 \
  tower_y:=0.0 \
  tower_height_m:=10.0
```

## 7. 动态障碍重规划模式（先不要接 PX4）

该模式让障碍球横向移动，检查候选半径切换和阻塞状态。它用于算法验收，不作为名义飞行命令：

```bash
cd "$SUPER_ROOT"
source /opt/ros/humble/setup.bash
source "$FAST_LIO_WORKSPACE/install/setup.bash"
source "$SUPER_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=53

ros2 launch inspection_manager tower_pointcloud_demo.launch.py \
  use_fast_lio:=true \
  fast_lio_rviz:=true \
  use_super_planner:=false \
  enable_px4_setpoints:=false \
  obstacle_amplitude:=3.0 \
  obstacle_radius:=0.6 \
  health_sequence:='READY:120' \
  planning_frame:=odom
```

观察 `/inspection/status` 是否出现 `REPLANNED radius=...`、`NO_SAFE_TRAJECTORY_DYNAMIC_OBSTACLE` 和障碍清除后的恢复。

## 8. 健康故障/恢复模式

```bash
cd "$SUPER_ROOT"
source /opt/ros/humble/setup.bash
source "$FAST_LIO_WORKSPACE/install/setup.bash"
source "$SUPER_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=53

ros2 launch inspection_manager tower_pointcloud_demo.launch.py \
  use_fast_lio:=true \
  fast_lio_rviz:=false \
  use_super_planner:=false \
  enable_px4_setpoints:=false \
  health_sequence='READY:5,STALE_POINTCLOUD:3,READY:5,LINK_LOST:3,READY:10' \
  planning_frame:=odom
```

预期状态：故障时路径撤回，恢复后等待新的质量有效观测，再恢复扫描。

## 9. 话题、频率、类型和 TF 检查

在完整闭环或传感器模式的另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source "$FAST_LIO_WORKSPACE/install/setup.bash"
source "$SUPER_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=53

ros2 node list | sort
ros2 topic list | sort

ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic type /cloud_registered
ros2 topic type /odom
ros2 topic type /inspection/reference_path

ros2 topic info /livox/lidar -v
ros2 topic info /livox/imu -v
ros2 topic info /cloud_registered -v
ros2 topic info /odom -v
ros2 topic info /inspection/reference_path -v

ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /odom

ros2 topic echo /inspection/fast_lio/input_status --once
ros2 topic echo /inspection/tower/status --once
ros2 topic echo /inspection/status --once
ros2 topic echo /inspection/coverage --once
ros2 topic echo /inspection/reference_path --once
ros2 topic echo /px4_offboard_bridge/status --once

ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 topic echo /tf_static --once
```

核心话题期望：`/livox/lidar` 为 `livox_ros_driver2/msg/CustomMsg`，`/livox/imu` 为 `sensor_msgs/msg/Imu`，`/cloud_registered` 为 `sensor_msgs/msg/PointCloud2`，`/odom` 为 `nav_msgs/msg/Odometry`。仿真中四个核心话题通常约 100 Hz；频率会受主机负载影响。

如果出现 `The message type 'livox_ros_driver2/msg/CustomMsg' is invalid`，重新按下面顺序 source，并检查接口是否存在：

```bash
source /opt/ros/humble/setup.bash
source "$FAST_LIO_WORKSPACE/install/setup.bash"
source "$SUPER_WORKSPACE/install/setup.bash"
ros2 interface show livox_ros_driver2/msg/CustomMsg
ros2 topic type /livox/lidar
```

## 10. 录制和查看 rosbag

在传感器链已经发布后执行：

```bash
cd "$SUPER_ROOT"
source /opt/ros/humble/setup.bash
source "$FAST_LIO_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID=53
export BAG_DURATION=60
bash scripts/record_mid360_fastlio_bag.sh
```

查看最近的包：

```bash
find "$SUPER_ROOT/artifacts/rosbags" -maxdepth 2 -type f -printf '%TY-%Tm-%Td %TH:%TM %p\n' | sort | tail
ros2 bag info "$SUPER_ROOT/artifacts/rosbags/<bag目录>"
```

## 11. 起飞点、降落点、目标位置和半径

### 11.1 ROS/SUPER 模式

当前 ROS/SUPER 模式的起飞点是 PX4 local origin，巡检目标中心来自点云观测，不单独读取一个 `takeoff_x/y` 参数。可以改变合成塔的位置，让识别和规划自动跟随：

```bash
PX4_ROS_TOWER_X=8.0
PX4_ROS_TOWER_Y=-3.0
PX4_ROS_TOWER_HEIGHT_M=12.0
```

将这些变量放入第 2 节完整命令的 `env` 中，或直接在 launch 中传参：

```bash
ros2 launch inspection_manager tower_pointcloud_demo.launch.py \
  tower_x:=8.0 tower_y:=-3.0 tower_height_m:=12.0 \
  planning_frame:=odom
```

任务管理器将根据新点云自动更新中心、半径和高度。默认降落由 `NAV_LAND` 接管，通常在任务完成时的当前水平位置下降；它不是预先写死的第二个 GPS 点。

### 11.2 PX4 原生 MAVLink 模式

原生控制器支持直接设置局部 NED 巡检中心、半径、高度和圈数：

```bash
cd "$SUPER_ROOT"
export PX4_ROOT="$PX4_ROOT"
export PX4_SIM_MODEL=iris_inspection
export HEADLESS=0
bash scripts/run_px4_inspection_sitl.sh \
  --center-north 20 \
  --center-east 10 \
  --radius 6 \
  --z-min 2 \
  --z-max 12 \
  --orbits 3 \
  --duration 60
```

这里的 `--center-north/--center-east` 是 PX4 local NED 的巡检圆心，也是起飞后的 warm-up 目标；它不是修改 Gazebo 模型的出生坐标。当前 Gazebo Classic `sitl_run.sh` 的模型出生点仍由 PX4 外部脚本中的 `gz model ... -x 1.01 -y 0.98 -z 0.83` 设置。若要改变物理出生点，应先备份并修改对应 PX4 脚本或世界文件，再重新运行；不要把 ENU 的 x/y 直接当成 PX4 NED 的 north/east。

### 11.3 坐标对齐参数

查看当前桥接器参数：

```bash
ros2 param list /px4_offboard_bridge
ros2 param get /px4_offboard_bridge origin_ned
ros2 param get /px4_offboard_bridge alignment_yaw_rad
```

真实测量后可在台架测试中设置：

```bash
ros2 param set /px4_offboard_bridge alignment_yaw_rad 0.0
ros2 param set /px4_offboard_bridge origin_ned "[0.0, 0.0, 0.0]"
```

### 11.4 任务管理器可调参数

在任务运行期间可以查看或修改：

```bash
ros2 param list /inspection_manager
ros2 param get /inspection_manager tower_surface_standoff
ros2 param get /inspection_manager minimum_orbit_radius
ros2 param get /inspection_manager tower_base_clearance
ros2 param get /inspection_manager tower_top_clearance
ros2 param get /inspection_manager orbit_count
ros2 param get /inspection_manager replan_radii

ros2 param set /inspection_manager tower_surface_standoff 2.5
ros2 param set /inspection_manager minimum_orbit_radius 4.0
ros2 param set /inspection_manager orbit_count 2.0
ros2 param set /inspection_manager tower_base_clearance 1.5
ros2 param set /inspection_manager tower_top_clearance 1.0
```

参数更新后应检查 `/inspection/status` 的 `VERTICAL_TOWER_READY center=... z=... radius=... points=...`。

## 12. 查看飞行日志和完成状态

```bash
ls -lt /tmp/super_px4_inspection | head
tail -f /tmp/super_px4_inspection/*.ros_offboard.log
tail -f /tmp/super_px4_inspection/*.log

grep -E 'SUPER_PATH_READY|SUPER_GOAL_REACHED index=19/19|SUPER_MISSION_COMPLETE|OUTPUTTING_SETPOINT|PX4_LANDING_ACTIVE|PX4_DISARMED_AFTER_LAND|PX4_MODE_EXITED|Ill corridor|Odom below virtual ground' \
  /tmp/super_px4_inspection/<本次运行>.ros_offboard.log
```

仿真验收脚本：

```bash
cd "$SUPER_ROOT"
bash scripts/verify_simulation.sh
```

预期：`62 passed`、`SIMULATION_VERIFICATION_OK`。

## 13. 改变场景和规划参数的常用写法

完整 ROS/SUPER 命令中可加入：

```bash
PX4_ROS_TOWER_X=8.0                 # 合成塔 x（任务坐标）
PX4_ROS_TOWER_Y=-3.0                # 合成塔 y（任务坐标）
PX4_ROS_TOWER_HEIGHT_M=12.0         # 合成塔高
PX4_SUPER_GOAL_STRIDE=4             # SUPER 目标更密，目标数增加
PX4_ROS_PATH_DURATION_S=90          # PX4 bridge 的参考时间参数
PX4_ROS_HEALTH_SEQUENCE='READY:360' # 名义运行保持健康
PX4_ROS_OBSTACLE_AMPLITUDE=3.0      # 动态障碍横向幅度
PX4_ROS_OBSTACLE_RADIUS=0.6         # 动态障碍半径
PX4_ROS_FAST_LIO_RVIZ=true          # 打开 FAST-LIO RViz
HEADLESS=0                          # 打开 Gazebo GUI
```

名义自动飞行建议保持障碍幅度和半径为 0；动态障碍、健康故障、阻塞路径应先关闭 PX4 setpoint，只检查任务管理器状态和路径撤回/恢复。

## 14. 停止和清理

```bash
for pattern in '[p]x4_offboard_bridge.py' '[s]uper_planner_adapter.py' '[t]ower_pointcloud_demo.launch.py' '[M]icroXRCEAgent'; do
  for pid in $(pgrep -f "$pattern" || true); do kill -TERM "$pid" 2>/dev/null || true; done
done
for name in px4 gzserver gzclient gazebo rviz2; do
  for pid in $(pgrep -x "$name" || true); do kill -TERM "$pid" 2>/dev/null || true; done
done
```

不要使用宽泛的 `pkill -f` 去杀整个 shell；优先使用上面的带方括号进程匹配，避免误杀其他终端或其他项目。

## 15. 关键文件速查

| 文件 | 作用 |
|---|---|
| `scripts/run_px4_inspection_sitl.sh` | PX4/Gazebo/Agent/ROS Offboard 总启动和清理 |
| `inspection_gazebo/worlds/px4_inspection.world` | 输电塔、导线和仿真场景 |
| `inspection_manager/launch/tower_pointcloud_demo.launch.py` | 传感器、FAST-LIO、定位、任务管理、SUPER、桥接器编排 |
| `inspection_manager/inspection_manager/inspection_sensor_sim_node.py` | Livox/IMU/D435i 合成数据 |
| `inspection_manager/inspection_manager/tower_pointcloud_node.py` | 点云中心、高度和范围估计 |
| `inspection_manager/inspection_manager/inspection_manager_node.py` | 半径、净空、障碍和参考路径 |
| `inspection_manager/inspection_manager/path_generation.py` | 上升螺旋几何生成 |
| `px4_offboard_bridge/scripts/super_planner_adapter.py` | 145 点路径到 19 个 SUPER 目标 |
| `px4_offboard_bridge/scripts/px4_offboard_bridge.py` | SUPER ENU 命令到 PX4 NED setpoint、Offboard、Arm、Land |
| `super_planner/config/inspection_super_px4.yaml` | SUPER/ROG-Map 参数 |
| `scripts/record_mid360_fastlio_bag.sh` | MID-360/FAST-LIO 话题录包 |
| `scripts/verify_simulation.sh` | 自动化回归和场景验收 |
