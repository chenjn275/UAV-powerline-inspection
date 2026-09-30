# 四旋翼输电线路巡检项目仿真汇报启动手册

以下命令按终端编号执行。默认使用 `ROS_DOMAIN_ID=42`，避免与其他 ROS 进程串线。

## 0. 清理旧进程

先执行一次：

```bash
pkill -TERM -x px4 2>/dev/null || true
pkill -TERM -x gzserver 2>/dev/null || true
pkill -TERM -x gzclient 2>/dev/null || true
pkill -TERM -x gazebo 2>/dev/null || true
pkill -TERM -x MicroXRCEAgent 2>/dev/null || true
sleep 3
ps -eo pid=,comm=,args= | awk '$2=="px4" || $2=="gzserver" || $2=="gzclient" || $2=="gazebo" || $2=="MicroXRCEAgent" {print}'
```

## 1. PX4 可视化巡检仿真（推荐汇报演示）

该模式启动 Gazebo GUI、PX4、解锁、两圈上升巡检、`NAV_LAND` 和解除锁定。

终端 1：

```bash
cd "/home/venom/Documents/ChatGPT/巡检/SUPER"
export PX4_SIM_MODEL=iris_inspection
export INSPECTION_DURATION=45
export INSPECTION_RADIUS=4.0
export INSPECTION_Z_MIN=1.5
export INSPECTION_Z_MAX=9.5
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_IP=127.0.0.1
export GAZEBO_HOST=127.0.0.1
export PX4_SIM_HOSTNAME=127.0.0.1
unset HEADLESS
bash scripts/run_px4_inspection_sitl.sh
```

观察窗口中的 Iris 模型和项目世界。运行日志在 `/tmp/super_px4_inspection/`，PX4 ULog 在：
`/home/venom/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`。

## 2. 不带传感器的 PX4 原生模式

适合只演示飞控、轨迹和降落，不启动 ROS 传感器节点：

```bash
cd "/home/venom/Documents/ChatGPT/巡检/SUPER"
export PX4_SIM_MODEL=iris
export INSPECTION_DURATION=45
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_IP=127.0.0.1
export GAZEBO_HOST=127.0.0.1
export PX4_SIM_HOSTNAME=127.0.0.1
export HEADLESS=1
bash scripts/run_px4_inspection_sitl.sh
```

## 2.1 修改起飞/降落点、巡检半径和高度

PX4 使用本地 NED 坐标。下面示例从北向 20 m、东向 10 m 的位置起飞，在半径 6 m 内从 2 m 上升到 12 m，巡检 3 圈：

```bash
cd "/home/venom/Documents/ChatGPT/巡检/SUPER"
source /opt/ros/humble/setup.bash
export PX4_SIM_MODEL=iris_inspection
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_IP=127.0.0.1
export GAZEBO_HOST=127.0.0.1
export PX4_SIM_HOSTNAME=127.0.0.1
unset HEADLESS
bash scripts/run_px4_inspection_sitl.sh \
  --center-north 20 \
  --center-east 10 \
  --radius 6 \
  --z-min 2 \
  --z-max 12 \
  --orbits 3 \
  --duration 60
```

## 2.2 查看坐标系、TF 和传感器安装外参

```bash
source /opt/ros/humble/setup.bash
source /home/venom/venom_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run tf2_ros tf2_echo base_link mid360_mount
ros2 run tf2_ros tf2_echo base_link d435i_mount
ros2 topic echo /tf_static --once
ros2 topic echo /cloud_registered --once
ros2 topic echo /livox/lidar/pointcloud --once
```

检查点云坐标系：

```bash
ros2 topic echo /cloud_registered --once | grep -E 'frame_id|stamp'
ros2 topic echo /odom --once | grep -E 'frame_id|child_frame_id|position:'
```

PX4 NED 到 ROS/Gazebo ENU 的对应关系：`north -> y`、`east -> x`、`down -> -z`。

## 2.3 基础状态和诊断命令

```bash
ros2 node list
ros2 topic list | sort
ros2 topic info /cloud_registered -v
ros2 topic info /odom -v
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic bw /cloud_registered
ros2 doctor --report
```

PX4/Gazebo 检查：

```bash
pgrep -a -x px4
pgrep -a -x gzserver
gz model --list
gz topic -l | grep -E 'pose|motor|imu|lidar'
tail -f /tmp/super_px4_inspection/*.controller.log
```

## 3. MID-360 + FAST-LIO2 传感器仿真

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /home/venom/venom_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOG_DIR=/tmp/venom_ros_log
ros2 launch venom_mid360_simulation rm_simulation.launch.py rviz:=false world:=RMUC
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source /home/venom/venom_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOG_DIR=/tmp/venom_ros_log
ros2 launch fast_lio mapping.launch.py use_sim_time:=true rviz:=false
```

## 4. 检查传感器、点云和 FAST-LIO2 输出

```bash
source /opt/ros/humble/setup.bash
source /home/venom/venom_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic list | sort
ros2 topic info /livox/lidar -v
ros2 topic info /livox/imu -v
ros2 topic info /cloud_registered -v
ros2 topic info /odom -v

ros2 topic hz /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /cloud_registered
ros2 topic hz /odom

ros2 topic echo /livox/imu --once
ros2 topic echo /livox/lidar --once
ros2 topic echo /cloud_registered --once
ros2 topic echo /odom --once
```

预期：四个核心话题约 100 Hz；`/livox/lidar` 类型为 `livox_ros_driver2/msg/CustomMsg`，`/cloud_registered` 为 `sensor_msgs/msg/PointCloud2`，`/odom` 为 `nav_msgs/msg/Odometry`。

## 5. 录制 MID-360/FAST-LIO2 rosbag

在传感器仿真和 FAST-LIO2 已启动后执行：

```bash
cd "/home/venom/Documents/ChatGPT/巡检/SUPER"
source /opt/ros/humble/setup.bash
source /home/venom/venom_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export BAG_DURATION=60
bash scripts/record_mid360_fastlio_bag.sh
```

录制目录默认是 `SUPER/artifacts/rosbags/`，脚本会同时生成 `ros2 bag info` 文件。

## 6. 项目任务管理器和纯 ROS 传感器仿真

```bash
source /opt/ros/humble/setup.bash
source "/home/venom/Documents/ChatGPT/巡检/super_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
ros2 launch inspection_manager tower_pointcloud_demo.launch.py
```

检查：

```bash
ros2 topic list | grep -E 'inspection|camera|depth|livox|coverage|reference|obstacle|odom|path'
ros2 topic hz /livox/lidar
ros2 topic echo /inspection/coverage_quality --once
ros2 topic echo /inspection/reference_path --once
```

## 7. 自动化验收

```bash
cd "/home/venom/Documents/ChatGPT/巡检"
bash SUPER/scripts/verify_simulation.sh
```

预期输出：`51 passed`、`SIMULATION_VERIFICATION_OK`。

## 8. 结束仿真

```bash
pkill -TERM -x px4 2>/dev/null || true
pkill -TERM -x gzserver 2>/dev/null || true
pkill -TERM -x gzclient 2>/dev/null || true
pkill -TERM -x gazebo 2>/dev/null || true
pkill -TERM -x MicroXRCEAgent 2>/dev/null || true
```

不要使用 `pkill -f`，避免误杀当前终端或其他任务。

## 9. Gazebo 飞行过程说明

当前飞行链路为：

```text
Gazebo Classic --TCP 4560--> PX4 SITL --UDP 14540--> px4_mavlink_inspection.py
```

启动后，控制器先连续发送约 2 s 的 NED 位置目标 `(north=0, east=0, down=-2)`，再切换 `OFFBOARD` 并发送解锁命令。解锁后，控制器以 20 Hz 发送 `SET_POSITION_TARGET_LOCAL_NED`，按半径 4 m、两圈、1.5～9.5 m 高度生成上升螺旋。轨迹完成后发送 `MAV_CMD_NAV_LAND`，停止轨迹流，等待 PX4 着陆并解锁。

实时查看飞行进度：

```bash
tail -f /tmp/super_px4_inspection/*.controller.log
tail -f /tmp/super_px4_inspection/*.log
gz model --list
gz topic -e -t /gazebo/default/pose/info
```

成功标志：

```text
Simulator connected on TCP port 4560
Armed by external command
Takeoff detected
local_ned=(..., ...,-8...)
Landing detected
Disarmed by landing
inspection reference complete
```

PX4 使用 NED 坐标：`north` 向北、`east` 向东、`down` 向下；ROS/Gazebo 常用 ENU 坐标：`x` 向东、`y` 向北、`z` 向上。传感器固定在 `base_link` 下的 `mid360_mount` 和 `d435i_mount`，飞机运动会通过 TF 带动传感器运动。
