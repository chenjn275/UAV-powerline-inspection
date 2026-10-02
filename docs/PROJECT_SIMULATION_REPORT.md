# 四旋翼输电线路巡检仿真项目汇报报告

**项目**：基于 SUPER、FAST-LIO2 和 PX4 的输电塔自主巡检仿真
**报告版本**：2026-10-02
**代码目录**：`/home/venom/Documents/ChatGPT/巡检/SUPER`

## 1. 汇报结论

当前版本已经完成一条可重复运行的 ROS 2 Humble 仿真闭环：仿真传感器产生 Livox/MID-360 风格的雷达和 IMU 数据，实际运行 FAST-LIO2 进行点云配准和里程计输出，点云定位节点估计输电塔几何范围，任务管理器计算安全巡检轨迹，SUPER/ROG-Map 生成局部轨迹，PX4 ROS 2 Offboard 桥接器发布飞控 setpoint，PX4 SITL 完成起飞、绕塔巡检、自动降落和解锁。

最近一次带 RViz 的完整回归结果：

| 项目 | 结果 |
|---|---|
| ROS 2 启动树 | 正常，RViz OpenGL 4.6 启动 |
| FAST-LIO2 | 正常初始化地图，收到 PX4 位姿 |
| 参考路径 | 145 个点 |
| SUPER 目标 | 19/19 到达 |
| PX4 setpoint | 3,149 个输出样本 |
| 任务结束 | `SUPER_MISSION_COMPLETE` |
| 降落 | `PX4_DISARMED_AFTER_LAND` |
| 提前退出 | 0 次 `PX4_MODE_EXITED` |
| 安全错误 | 0 次走廊错误、0 次虚拟地面错误 |
| 自动化验证 | 62 项通过，`SIMULATION_VERIFICATION_OK` |

证据日志：`/tmp/super_px4_inspection/run_20261002_102538_2183466.ros_offboard.log`。
汇报记录：`artifacts/simulation_run_20261002_super_fastlio.json`。

这里的“完成”指软件仿真闭环完成。传感器输入仍是确定性合成数据，不能作为 MID-360S、D435i 或真实飞机飞行性能的实测证明。

## 2. 系统总体结构

```text
Gazebo Classic 输电塔/导线场景 + PX4 iris_inspection
                    │
                    │ PX4 VehicleLocalPosition / DDS
                    ▼
纯 ROS 仿真传感器
  ├─ Livox CustomMsg: /livox/lidar
  ├─ IMU:             /livox/imu
  ├─ D435i 风格图像:  /inspection_d435i/*
  └─ 动态障碍点云:    /inspection/obstacles/points
                    │
                    ▼
FAST-LIO2 fastlio_mapping
  ├─ 配准点云:        /cloud_registered
  ├─ 里程计:          /odom（桥接后的 ENU 任务坐标）
  └─ 轨迹:            /fastlio/path
                    │
                    ▼
点云定位与任务管理
  ├─ 输电塔中心/高度/范围估计
  ├─ 质量门、健康门、超时门
  ├─ 候选半径和动态障碍净空检查
  └─ 上升螺旋参考路径: /inspection/reference_path
                    │
                    ▼
SUPER / ROG-Map
  ├─ 每隔 N 个参考点发送一个 SUPER goal
  ├─ 根据局部点云地图重规划
  └─ 输出 /planning/pos_cmd
                    │
                    ▼
px4_offboard_bridge
  ├─ 检查 PX4 位姿、VehicleStatus、Timesync
  ├─ ENU → NED 坐标变换
  ├─ 发布 /fmu/in/offboard_control_mode
  ├─ 发布 /fmu/in/trajectory_setpoint
  └─ 任务完成后请求 NAV_LAND
                    │
                    ▼
PX4 SITL：Offboard → Arm → 巡检 → Auto Land → Disarm
```

## 3. 仿真场景和坐标约定

### 3.1 场景

`inspection_gazebo/worlds/px4_inspection.world` 包含：

- 两座由圆柱、斜撑和横担组成的三角格构输电塔，塔高约 10 m；
- 塔顶绝缘子、导线和可视化净空参考物；
- 带有 D435i/MID-360 外形挂载的 `iris_inspection` PX4 模型；
- Gazebo Classic 负责动力学和 PX4 联机，传感器数据使用纯 ROS 仿真节点以避免 Gazebo ROS 传感器插件在当前主机上的崩溃问题。

第一座塔用于当前闭环任务。FAST-LIO 输入模式中，传感器模拟器给固定场景增加 `sensor_origin=(6,0,0)` 的场景偏移，再根据 PX4 当前位姿把场景变换到雷达坐标；因此日志中的 SUPER 目标约在 `x=6` 附近，是坐标契约的一部分，不是随机漂移。

### 3.2 坐标系

| 坐标系 | 作用 |
|---|---|
| PX4 local NED | 飞控内部位置和轨迹 setpoint：north、east、down |
| ROS/Gazebo ENU | 任务规划坐标：x 向东、y 向北、z 向上 |
| `odom` | FAST-LIO/任务的局部规划坐标 |
| `world` | SUPER ROG-Map 使用的规划 frame |
| `base_link` | 飞机机体 |
| `livox_frame` | MID-360/Livox 传感器 |
| `imu_link` | IMU 传感器 |

仿真中发布了 `base_link→livox_frame` 和 `base_link→imu_link` 静态 TF。桥接器使用 `origin_ned` 和 `alignment_yaw_rad` 完成 ENU/NED 对齐；默认原点和偏航对齐都是 0。

## 4. 逐步实现流程

### 4.1 PX4 和仿真世界启动

`scripts/run_px4_inspection_sitl.sh` 首先检查 PX4 构建、Gazebo 模型和 `gazebo-classic_iris_inspection` target，然后启动：

1. PX4 Classic SITL；
2. Gazebo server 和可选 GUI；
3. MicroXRCEAgent UDP 8888；
4. ROS 2 `tower_pointcloud_demo.launch.py`；
5. ROS Offboard bridge。

PX4 的 Gazebo 联机使用 TCP 4560，MAVLink 控制模式使用 UDP 14540，PX4 DDS 到 MicroXRCEAgent 使用 UDP 8888。脚本将 Gazebo/PX4 和 ROS launch 放入独立进程组，任务结束后统一清理，避免遗留控制器抢占下一次运行。

### 4.2 传感器仿真

`inspection_sensor_sim_node.py` 在 `fast_lio_input=true` 时发布真实 FAST-LIO2 输入接口所需的两类消息：

- `/livox/lidar`：`livox_ros_driver2/msg/CustomMsg`，包含点坐标、反射率、线号和扫描内 `offset_time`；
- `/livox/imu`：`sensor_msgs/msg/Imu`，包含模拟加速度和角速度。

同时发布 D435i 风格的深度和彩色图像，用于演示感知和覆盖率接口。雷达点来自确定性塔模型，不是 Gazebo ray-cast，也不含真实 MID-360S 噪声、回波丢失、温漂和实际标定误差。

传感器模拟器会订阅 PX4 `vehicle_local_position`。飞机移动后，固定塔场景在传感器坐标中的位置随之变化，这使 FAST-LIO2 能够看到随运动变化的扫描，而不是单纯重复同一帧世界坐标点云。

### 4.3 FAST-LIO2 配准和里程计

启动已锁定的 FAST-LIO2 `fastlio_mapping` 和 MID-360 配置。其输入、输出关系为：

```text
/livox/lidar + /livox/imu
        ↓
fastlio_mapping
        ├─ /cloud_registered : sensor_msgs/msg/PointCloud2
        ├─ /fastlio/odom     : FAST-LIO 原始里程计
        └─ /fastlio/path     : FAST-LIO 原始轨迹
```

`px4_position_odom_bridge.py` 将 PX4 的 ENU 位姿桥接到任务所需的 `/odom`，确保传感器模拟、FAST-LIO、ROG-Map 和 PX4 桥接器共享同一规划坐标契约。启动阶段必须先收到有效 PX4/FAST-LIO 位姿，SUPER 才会发送第一个 goal；这是为消除 `No odom`/`PlanFromRest failed` 启动竞态而增加的门。

### 4.4 输电塔点云定位

`tower_pointcloud_node.py` 订阅 `/cloud_registered`，执行以下几何估计：

1. 检查 `x/y/z` 字段和有限值；
2. 用 XY 中位数建立初始中心；
3. 用径向 MAD/百分位数排除导线等长条离群点；
4. 计算鲁棒中心、底高、顶高和水平范围不确定度；
5. 发布：
   - `/inspection/tower/center`：`PoseStamped`；
   - `/inspection/tower/extent`：`Vector3Stamped`，x=高度、y=水平范围代理、z=有效点数；
   - `/inspection/tower/status`：`VALID` 或拒绝原因；
   - `/inspection/tower/marker`：RViz 绿色透明圆柱。

当前实现是“已裁剪输电塔 ROI 的几何定位”，不是任意全场景中的语义识别器。真实部署前需要把 YOLO/视觉检测或真实点云分割接入该 ROI 接口，并验证误检、漏检和遮挡场景。

### 4.5 巡检半径和高度计算

任务管理器使用最新有效观测计算：

```text
z_min = tower_base_z + tower_base_clearance
z_max = tower_top_z  - tower_top_clearance
R     = max(minimum_orbit_radius,
            footprint_radius + tower_surface_standoff)
```

当前默认参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `minimum_orbit_radius` | 3.0 m | 最小绕塔半径 |
| `tower_surface_standoff` | 2.0 m | 塔体表面净空 |
| `tower_base_clearance` | 1.5 m | 低于塔底不进入的高度余量 |
| `tower_top_clearance` | 1.0 m | 塔顶上方的高度余量 |
| `orbit_count` | 2.0 | 完整环绕圈数 |
| `orbit_samples_per_turn` | 72 | 每圈离散采样数 |
| `replan_radii` | 3/4/5/6 m | 动态障碍时依次尝试的半径 |

纯几何回归曾得到约 3.33 m 的半径和 1.77–8.73 m 的扫描高度。最近一次 FAST-LIO 可视化日志中，SUPER 目标坐标约为 x=2.75–10.55、y=±3.95，围绕 x≈6 的塔中心形成约 4.55 m 半径；高度约为 1.95–8.65 m。半径随点云估计范围变化，这是设计行为，不是固定飞行圆。

### 4.6 上升螺旋参考轨迹

对中心 `(c_x,c_y)`、半径 `R` 和进度 `u∈[0,1]`，参考路径为：

```text
θ(u) = θ0 + 2π · orbit_count · u
x(u) = c_x + R · cos(θ(u))
y(u) = c_y + R · sin(θ(u))
z(u) = z_min + (z_max-z_min) · u
yaw  = θ(u) + π/2
```

默认 2 圈、每圈 72 个采样点，加上终点共 145 个 `PoseStamped`。航向沿圆周切线方向，飞机绕塔上升时机头跟随飞行方向。任务管理器以至少 5 Hz 发布参考路径，防止路径桥接器把旧路径判为超时。

### 4.7 障碍物和动态重规划

`/inspection/obstacles/points` 是障碍物输入。任务管理器把点云估计为保守球体，并对参考路径按不大于 0.25 m 的步长加密采样。候选半径按照 `replan_radii` 检查：

```text
3 m → 4 m → 5 m → 6 m
```

路径必须同时满足塔体表面净空和障碍物半径+`obstacle_margin`。所有候选均被阻挡时发布 `NO_SAFE_TRAJECTORY_DYNAMIC_OBSTACLE` 并撤回路径；障碍物清除后重新发布安全路径。当前名义 PX4 闭环将障碍物半径和幅度设置为 0，动态障碍场景应先用无飞控模式单独验收。

### 4.8 SUPER 局部规划

`super_planner_adapter.py` 将 145 点参考路径按 `super_goal_stride=8` 抽取为 19 个顺序 goal，逐个发布到 `/inspection/super/goal`。它具备以下门控：

- 未收到有效 `/odom` 或 PX4 位姿时不发送第一个目标；
- 必须收到新鲜 `/planning/pos_cmd` 才推进目标；
- 目标距离小于约 0.6 m 并停留 0.5 s 才算到达；
- 检测到路径重复发布时锁定当前任务，防止 FAST-LIO 微小抖动把 SUPER 重置到第一个目标；
- 收到 `SUPER_MISSION_COMPLETE`、`PX4_LANDING_ACTIVE` 或 `PX4_DISARMED_AFTER_LAND` 后停止重规划和轨迹发布。

SUPER 本身根据 ROG-Map 和当前局部点云生成平滑的 `PositionCommand`。PX4 实际跟踪的是 SUPER 的实时命令，而不是直接跟踪 145 个离散参考点。

### 4.9 PX4 Offboard、降落和解锁

`px4_offboard_bridge.py` 默认是 dry-run。只有 launch 同时设置 `enable_px4_setpoints=true`、`request_offboard=true` 和 `request_arm=true` 时才会：

1. 检查有效 PX4 local position、VehicleStatus 和 Timesync；
2. 以 30 Hz 发布 OffboardControlMode 和 TrajectorySetpoint；
3. 将 SUPER ENU 命令转换为 PX4 local NED；
4. 请求 Offboard 和 Arm；
5. 只在 `SUPER_MISSION_COMPLETE` 后请求 `MAV_CMD_NAV_LAND`；
6. 等待 PX4 进入 Auto Land 并确认 disarm。

当前 ROS/SUPER 模式没有独立的“返航点”参数：起飞点是 PX4/Gazebo local origin，降落由 `NAV_LAND` 在任务完成时接管，通常在任务末端当前位置附近垂直降落。`origin_ned` 和 `alignment_yaw_rad` 用于坐标对齐，不能替代真实机的起降点测量和 PX4 home/failsafe 配置。

## 5. 状态机和异常处理

```text
WAITING_FOR_TOWER_POINTCLOUD
        ↓ 有效中心/高度/范围
WAITING_FOR_HEALTH / WAITING_FOR_VALID_OBSERVATIONS
        ↓ 健康、质量和障碍门通过
SCANNING / REFERENCE_PATH_PUBLISHED
        ↓ FAST-LIO 位姿 + SUPER PositionCommand
PX4 OFFBOARD FOLLOWING
        ↓ 19 个 SUPER goal 完成
SUPER_MISSION_COMPLETE → NAV_LAND → DISARMED
```

健康仿真可注入 `STALE_POINTCLOUD`、`LINK_LOST` 等状态。发生故障时任务进入制动/等待，撤回旧路径；恢复后还必须收到新的质量有效观测，不能直接复用故障前的旧轨迹。点云、路径、SUPER 命令、PX4 状态任何一个超时都会抑制输出。

## 6. RViz 观测项

FAST-LIO 默认 RViz 配置使用黑色背景和 `odom` Fixed Frame。黑色背景有利于观察彩色点云、绿色路径和 TF 轴，不影响算法；汇报时可改为深灰色。

| RViz 项 | 话题/来源 | 解释 |
|---|---|---|
| TF | `/tf`、`/tf_static` | 机体、雷达、IMU 和 odom/world 关系 |
| Odometry | `/odom` | PX4 位姿桥接到 ENU 后的飞机位置 |
| CloudRegistered | `/cloud_registered` | FAST-LIO2 配准点云，通常按高度彩色 |
| CloudEffected | `/cloud_effected` | FAST-LIO 特征/处理点云，可能为空 |
| CloudMap | `/map_cloud` | FAST-LIO 累积地图，配置未发布时可能为空 |
| Path | `/path` 或 `/fastlio/path` | FAST-LIO 估计轨迹，不等于巡检参考路径 |
| Inspection path | `/inspection/reference_path` | 任务管理器生成的上升螺旋 |
| Tower marker | `/inspection/tower/marker` | 点云定位出的塔体范围 |
| Obstacle cloud | `/inspection/obstacles/points` | 动态障碍模拟点云 |

当前 FAST-LIO RViz 配置没有预置所有项目话题，需要在 RViz 点击 `Add` 手动加入 `Path`、`Marker` 和 `PointCloud2`。

## 7. 仿真验证和限制

已完成的验证：

- 62 项 Python 回归测试；
- 150 组几何/障碍/覆盖率批量实验；
- FAST-LIO2 MID-360 输入链路和约 100 Hz 话题检查；
- 合成 rosbag：`artifacts/rosbags/mid360_fastlio_20261002_070303`；
- PX4 ROS 2 Offboard 完整飞行闭环；
- RViz、Gazebo、SUPER、FAST-LIO2、PX4 进程清理和重复启动。

尚未由仿真证明的内容：

- MID-360S 实物噪声、固件和实际回波；
- D435i 实物深度质量和视觉识别；
- 实测 TF 外参、时间同步、振动和遮挡；
- 真实 PX4 飞控参数、动力学、失控保护和通信延迟；
- 真实输电线路环境中的净空法规和飞行安全。

因此当前可以作为“软件仿真闭环汇报和算法联调基线”，不能直接宣称已获得真实飞机放飞许可。

## 8. 迁移到真实飞机前的顺序

1. 固定 PX4、FAST-LIO2、Livox driver 和 ROS 2 的版本/commit；
2. 录入 MID-360S、D435i、伴随计算机和飞控硬件信息；
3. 实测并验证 `base_link→livox_frame`、`base_link→imu_link`、相机 TF；
4. 用静态场景和手持移动录制真实 rosbag，检查时间戳、点云坐标和 IMU 轴；
5. 用真实点云替换合成传感器，先只运行 FAST-LIO2 和定位，不开电机；
6. 台架验证 PX4 DDS、Timesync、Offboard 超时、RC 接管、返航和降落；
7. 系留悬停，再做开阔场地小半径、低高度轨迹；
8. 最后才做输电塔近距离、动态障碍和完整任务。

每一步都应保留原始 rosbag、PX4 ULog、参数快照和人工急停记录。
