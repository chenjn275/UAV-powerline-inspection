# SUPER 项目文件树

```text
SUPER/
├── inspection_core/                 # 核心巡检算法
│   ├── inspection_core/
│   │   ├── line_model.py            # 导线/输电线路模型
│   │   ├── coordinate.py             # 坐标与坐标变换
│   │   ├── candidate_search.py       # 巡检半径与候选轨迹搜索
│   │   ├── dynamic_replanner.py      # 动态避障与重新规划
│   │   ├── coverage.py               # 覆盖率与观测质量
│   │   ├── health.py                 # 传感器/任务健康状态
│   │   ├── state_machine.py          # 巡检任务状态机
│   │   └── tower_localization.py     # 输电塔/导线定位接口
│   ├── config/                       # 算法参数
│   ├── scripts/                      # 基线、场景、消融实验
│   └── test/                         # 核心算法测试
│
├── inspection_gazebo/                # Gazebo 仿真场景
│   ├── worlds/
│   │   ├── px4_inspection.world      # PX4 输电线路世界
│   │   ├── inspection_line.world     # 输电线路场景
│   │   └── inspection_line_native.world
│   ├── models/                       # 仿真传感器/目标模型
│   ├── scripts/                      # 世界生成脚本
│   └── launch/                       # Gazebo 启动文件
│
├── inspection_manager/               # ROS 2 任务管理
│   ├── inspection_manager/
│   │   ├── inspection_manager_node.py       # 总任务状态机节点
│   │   ├── tower_pointcloud_node.py         # PointCloud2 解析与目标输入
│   │   ├── tower_pointcloud_sim_node.py     # 输电塔点云仿真
│   │   ├── obstacle_pointcloud_sim_node.py  # 动态障碍点云仿真
│   │   ├── perception_sim_node.py           # 感知结果仿真
│   │   ├── vision_observation_sim_node.py   # 视觉观测质量仿真
│   │   ├── path_generation.py               # 参考巡检轨迹生成
│   │   ├── quality_coverage_sim_node.py     # 覆盖率与质量输出
│   │   └── health_fault_sim_node.py         # 故障注入与恢复
│   ├── launch/
│   │   ├── tower_pointcloud_demo.launch.py   # 点云/规划/PX4 演示入口
│   │   └── inspection_manager.launch.py      # 通用任务入口
│   ├── config/                               # vertical_tower.yaml 等任务参数
│   └── test/
│
├── px4_offboard_bridge/              # ROS 2 ↔ PX4 Offboard
│   ├── scripts/px4_offboard_bridge.py       # ENU 轨迹转 PX4 NED setpoint
│   ├── config/px4_offboard_bridge.yaml      # 话题、频率、坐标参数
│   ├── launch/px4_offboard_bridge.launch.py # ROS 2 bridge 启动
│   └── test/test_offboard_bridge_contract.py # 坐标和接口契约测试
│
├── super_planner/                    # SUPER C++ 规划器
│   ├── src/                          # 规划器源码
│   ├── include/                      # 头文件
│   ├── Apps/                         # FSM、轨迹优化程序
│   ├── config/                       # 规划参数
│   ├── launch/                       # ROS 启动文件
│   └── rviz/                         # RViz 配置
│
├── mission_planner/                  # 航点/任务规划示例
├── rog_map/                          # ROG Map 地图模块
├── mars_uav_sim/                     # MARS 无人机仿真基础包
│
├── scripts/                          # 项目级运行脚本
│   ├── run_px4_inspection_sitl.sh          # 清理、启动 PX4/Gazebo、运行控制器
│   ├── px4_mavlink_inspection.py           # 起飞/螺旋巡检/NAV_LAND 控制器
│   ├── install_px4_inspection_model.sh     # 安装扁平 iris_inspection 模型
│   ├── record_mid360_fastlio_bag.sh        # 录制 Livox/FAST-LIO2 rosbag
│   ├── verify_simulation.sh                 # 62 项回归和场景验收
│   ├── check_px4_sitl_prerequisites.sh     # 检查 PX4/Gazebo/模型依赖
│   ├── test_px4_mavlink_inspection.py      # MAVLink 控制器单测
│   ├── build_ros2_humble.sh                # ROS 2 工作区构建辅助
│   └── build_microxrce_agent.sh            # PX4 ROS 2 Agent 构建辅助
│
├── artifacts/                        # 仿真证据、批量实验、PX4 ULog 指标
├── docs/                             # 项目文档、版本、交付矩阵、命令手册
│   ├── PROJECT_STATUS.md              # 当前完成状态与边界
│   ├── DELIVERY_MATRIX.md             # 任务-证据交付矩阵
│   ├── PROJECT_DEMO_COMMANDS.md      # 汇报启动、诊断和 TF 命令
│   ├── versions.lock.yaml             # FAST-LIO2/Livox/PX4 版本锁定
│   ├── coordinate_contract.md         # NED/ENU/TF 坐标约定
│   ├── verification_plan.md           # 验证计划和验收标准
│   └── real_sensor_integration.md    # 真实传感器接入边界
├── tools/Micro-XRCE-DDS-Agent/       # PX4 ROS 2 Agent 源码
├── misc/                             # 图片、论文和展示资料
└── log/                              # 运行日志
```
