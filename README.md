# AutoFactory AGV

**AutoFactory AGV 全覆盖巡检与自主识别泊车系统**

> A ROS 2 based factory AGV system integrating full-coverage inspection, Nav2 autonomous navigation, ArUco visual localization, Hybrid A* parking planning, and autonomous parking.

---

## 项目简介

AutoFactory AGV 是一个基于 **ROS 2 Humble + Gazebo Classic + TurtleBot3 Waffle** 的工厂 AGV 仿真系统，面向“全覆盖巡检 → 自主转场 → 视觉识别 → 自动泊车”的完整任务链。

系统已经完成以下功能集成：

- Gazebo 工厂环境与 AGV 仿真
- AMCL 定位与 Nav2 自主导航
- 基于牛耕式路径的全覆盖巡检
- Coverage 与 Parking 任务自动衔接
- 预备停车点自主导航
- ArUco ID=0 识别与稳定锁定
- ArUco 位姿转换到 `map` 坐标系
- Hybrid A* 自主泊车路径规划
- 几何约束泊车路径跟踪
- Nav2 与 Parking Tracker 之间的速度控制权切换
- Coverage RViz 与 Parking/RPP RViz 的阶段化自动切换

---

## 系统完整流程

```text
Factory Simulation
        ↓
AGV Spawn
        ↓
TF + AMCL + Nav2
        ↓
Full-Coverage Inspection
        ↓
Coverage Complete
        ↓
Close Coverage RViz
        ↓
Open RPP / Parking RViz
        ↓
Coverage → Parking Handoff
        ↓
Parking Staging
        ↓
ArUco Recognition
        ↓
Map Localization
        ↓
Hybrid A* Planning
        ↓
Parking Path Tracking
        ↓
Autonomous Parking
        ↓
Mission Complete
```

---

## 主要功能

### 1. Nav2 自主导航

系统使用：

- Nav2
- AMCL
- NavFn
- Regulated Pure Pursuit Controller
- Velocity Smoother
- ROS 2 Humble 自定义 Behavior Tree

普通导航与 Coverage 使用：

```text
general_goal_checker
```

Parking Staging 使用：

```text
staging_goal_checker
```

以避免多 Goal Checker 场景下的选择歧义。

---

### 2. 全覆盖巡检

`agv_coverage` 实现牛耕式全覆盖路径规划，包括：

- OccupancyGrid 自由区域提取
- 障碍物膨胀
- 扫描方向估计
- 平行扫描线生成
- 相邻扫描段连接
- 航点抽稀
- 多批次 `NavigateThroughPoses` 执行
- 覆盖率估计
- 覆盖状态与进度发布

主要话题：

```text
/coverage_path
/coverage_waypoints
/coverage_status
/coverage_progress
```

---

### 3. Parking Staging

机器人在进入视觉识别和最终泊车前，先自主前往预备停车点。

目标：

```text
x   = -6.0
y   = -4.5
yaw = 3.1 rad
```

使用：

```text
planner_id      = StagingGrid
controller_id   = FollowPath
goal_checker_id = staging_goal_checker
```

流程：

```text
Plan
→ FollowPath
→ Position Check
→ Final Heading Alignment
→ Staging Pose Guard
```

---

### 4. ArUco 自主识别

识别参数：

```text
Dictionary    : DICT_6X6_250
Marker ID     : 0
Marker Length : 0.20 m
Stable Frames : 5
```

处理链：

```text
Camera Image
→ ArUco Detection
→ Stable Detection
→ Lock
→ Marker Pose
→ Map Frame Transformation
```

主要输出：

```text
/aruco/locked
/aruco/status
/aruco/marker_pose
/aruco/marker_pose_map
/aruco/map_status
/aruco/debug_image
```

---

### 5. Hybrid A* 自主泊车

`agv_parking` 中实现了面向差速 AGV 的 Hybrid A* 泊车规划。

支持：

- 前进
- 倒车
- 转弯
- 原地旋转
- 碰撞检测
- 换向惩罚
- 转弯惩罚
- 终点位置与姿态约束

ROS `map` 坐标系下的最终泊车目标：

```text
x   = -8.357680
y   = -4.166548
yaw = 1.539811
```

---

### 6. Parking Tracker

最终泊车跟踪器版本：

```text
V3.2_SAFE_GEOMETRY
```

特点：

- 根据几何关系推断前进 / 倒车
- 合并重复 XY 的旋转节点
- 低速泊车跟踪
- 最终位置与姿态修正
- 直接输出 `/cmd_vel`

---

## 项目结构

```text
AutoFactory-AGV/
│
├── start_autofactory.sh
├── README.md
├── LICENSE
├── .gitignore
│
├── docs/
│   └── AutoFactory_AGV_项目使用说明_v1.0.docx
│
└── src/
    ├── factory_simulation/
    ├── agv_bringup/
    ├── agv_coverage/
    ├── agv_task_manager/
    ├── agv_perception/
    └── agv_parking/
```

| Package | 功能 |
|---|---|
| `factory_simulation` | Gazebo 工厂世界与模型 |
| `agv_bringup` | 机器人生成、TF、地图、AMCL、Nav2、RPP、RViz |
| `agv_coverage` | 全覆盖规划与执行 |
| `agv_task_manager` | Parking Staging 与完整停车任务编排 |
| `agv_perception` | ArUco 检测与 map 坐标定位 |
| `agv_parking` | Hybrid A* 与 Parking Tracker |

---

## 开发与验证环境

| 项目 | 配置 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Simulator | Gazebo Classic 11 |
| Robot | TurtleBot3 Waffle |
| Python | Python 3.10 |
| Navigation | Nav2 |
| Local Controller | Regulated Pure Pursuit |
| Visualization | RViz2 / rqt_image_view |

当前验证工作空间：

```text
/home/my-ubuntu/autofactory_ws
/home/my-ubuntu/dev_ws
```

> **注意：** v1.0.0 中部分 Shell 脚本仍使用固定绝对路径。首次复现建议保持相同目录结构。后续版本可进一步改造成完全可移植路径。

---

## 编译

```bash
cd /home/my-ubuntu/autofactory_ws

source /opt/ros/humble/setup.bash
source /home/my-ubuntu/dev_ws/install/setup.bash

colcon build \
  --merge-install \
  --symlink-install

source /home/my-ubuntu/autofactory_ws/install/setup.bash
```

检查功能包：

```bash
colcon list
```

预期：

```text
agv_bringup
agv_coverage
agv_parking
agv_perception
agv_task_manager
factory_simulation
```

---

## 快速启动

统一入口：

```text
start_autofactory.sh
```

### 模式 1：Nav2

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh nav2
```

流程：

```text
Factory World
→ Robot
→ TF
→ AMCL
→ Nav2
→ RPP RViz
```

---

### 模式 2：全覆盖巡检

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh coverage
```

流程：

```text
Factory World
→ Robot
→ Nav2
→ Coverage Planning
→ Coverage RViz
→ Full Coverage Execution
```

---

### 模式 3：自主识别泊车

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh parking
```

流程：

```text
Nav2
→ Parking Staging
→ ArUco Recognition
→ Map Localization
→ Hybrid A*
→ Parking Tracker
→ Autonomous Parking
```

---

### 模式 4：完整全自动任务

```bash
cd /home/my-ubuntu/autofactory_ws
./start_autofactory.sh full
```

完整流程：

```text
Factory
→ Robot
→ Nav2
→ Full Coverage
→ Coverage RViz Closed
→ RPP RViz Opened
→ Parking Staging
→ ArUco Recognition
→ Hybrid A*
→ Autonomous Parking
```

正常情况下无需人工执行中间命令。

---

## RViz 阶段化切换

### Coverage 阶段

自动加载：

```text
src/agv_bringup/rviz/autofactory_coverage.rviz
```

主要显示：

- 地图
- 机器人
- Coverage Path
- Coverage Waypoints
- Costmap
- TF

### Parking 阶段

全覆盖完成后：

```text
autofactory_coverage.rviz
```

自动关闭，然后打开：

```text
src/agv_bringup/rviz/autofactory_rpp.rviz
```

主要显示：

- Nav2 `/plan`
- `/parking_staging_path`
- `/parking_staging_path_end`
- `/aruco/marker_pose_map`
- `/parking/hybrid_path`
- Robot / Map / Costmap

ArUco 阶段还会临时打开：

```text
rqt_image_view
```

显示：

```text
/aruco/debug_image
```

---

## Coverage → Parking 自动交接

Coverage 结束后，机器人可能与 Parking Staging 首段路径方向存在较大角度差。

完整模式不会永久修改 Nav2 YAML，而是在运行时临时设置：

```text
FollowPath.use_rotate_to_heading = true
```

随后自动执行：

```text
Clear Local Costmap
→ Clear Global Costmap
→ Wait for Costmap Refresh
→ Start Parking Staging
```

停车任务结束后恢复原始参数值。

---

## 速度控制链

正常 Nav2：

```text
controller_server
        ↓
   /cmd_vel_nav
        ↓
velocity_smoother
        ↓
     /cmd_vel
        ↓
turtlebot3_diff_drive
```

最终泊车：

```text
Parking Tracker PATH_READY
        ↓
velocity_smoother deactivated
        ↓
parking_path_tracker
        ↓
/cmd_vel
        ↓
turtlebot3_diff_drive
```

泊车完成后恢复 Nav2 速度输出。

---

## 重要 ROS 接口

### Topics

| Topic | 用途 |
|---|---|
| `/map` | OccupancyGrid 地图 |
| `/scan` | 激光雷达 |
| `/odom` | 里程计 |
| `/cmd_vel_nav` | Nav2 控制器输出 |
| `/cmd_vel` | 机器人最终速度命令 |
| `/coverage_path` | 全覆盖完整路径 |
| `/coverage_waypoints` | Coverage 关键航点 |
| `/coverage_status` | Coverage 状态 |
| `/coverage_progress` | Coverage 进度 |
| `/parking_staging_path` | 预备停车路径 |
| `/parking_staging_status` | Staging 状态 |
| `/aruco/locked` | ArUco 稳定锁定 |
| `/aruco/marker_pose_map` | map 坐标系中的 Marker 位姿 |
| `/parking/hybrid_path` | Hybrid A* 泊车路径 |
| `/parking/hybrid_status` | Hybrid A* 状态 |
| `/parking/tracking_status` | Parking Tracker 状态 |

### Services

| Service | 用途 |
|---|---|
| `/start_coverage` | 生成 Coverage 路径 |
| `/execute_coverage` | 执行 Coverage |
| `/go_to_parking_staging` | 前往预备停车点 |
| `/cancel_parking_staging` | 取消 Staging |
| `/aruco/enable` | 开关 ArUco 检测 |
| `/clear_hybrid_parking` | 清除旧 Hybrid Path |
| `/plan_hybrid_parking` | 规划 Hybrid A* |
| `/execute_hybrid_parking` | 执行泊车 |
| `/stop_hybrid_parking` | 停止泊车 |

### Actions

```text
/compute_path_to_pose
/compute_path_through_poses
/follow_path
/navigate_to_pose
/navigate_through_poses
/spin
```

---

## 关键配置文件

```text
src/agv_bringup/params/autofactory_nav2_rpp.yaml
src/agv_bringup/behavior_trees/autofactory_navigate_to_pose.xml
src/agv_bringup/behavior_trees/autofactory_navigate_through_poses.xml

src/agv_coverage/config/coverage_params.yaml

src/agv_task_manager/config/parking_staging.yaml

src/agv_perception/config/

src/agv_parking/config/hybrid_astar.yaml

src/agv_bringup/maps/autofactory_navigation.yaml
src/agv_bringup/maps/autofactory_navigation.pgm

src/factory_simulation/worlds/autofactory.world

src/agv_bringup/rviz/autofactory_coverage.rviz
src/agv_bringup/rviz/autofactory_rpp.rviz
```

---

## 常见排查

### Nav2 是否激活

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
```

### RPP 插件

```bash
ros2 param get /controller_server FollowPath.plugin
```

预期：

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

### TF

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

### Coverage 状态

```bash
ros2 topic echo /coverage_status
```

完成状态：

```text
本次覆盖执行完成。
```

### Parking Staging

```bash
ros2 topic echo /parking_staging_status
```

### ArUco

```bash
ros2 topic echo /aruco/locked
ros2 topic echo /aruco/map_status
```

期望：

```text
/aruco/locked = true
/aruco/map_status = TRACKING
```

### Hybrid A*

```bash
ros2 topic echo /parking/hybrid_status
```

### Parking Tracker

```bash
ros2 topic echo /parking/tracking_status
```

---

## 项目文档

完整中文使用说明位于：

```text
docs/AutoFactory_AGV_项目使用说明_v1.0.docx
```

---

## GitHub 开源注意事项

以下目录或文件不建议提交：

```text
build/
install/
log/
__pycache__/
*.pyc
*.bak
*.backup
```

普通运行也不需要 SLAM 继续建图数据：

```text
src/agv_bringup/maps/posegraph/
```

---

## 已知限制

- v1.0.0 主要针对当前 AutoFactory 固定地图与工厂场景验证。
- 部分启动脚本仍包含固定工作空间绝对路径。
- 当前视觉泊车使用 ArUco ID `0`。
- Parking Staging 与最终泊车目标针对当前地图配置。
- 当前机器人模型为 TurtleBot3 Waffle。
- 当前系统面向 ROS 2 Humble。

---

## License

本项目建议采用 **MIT License**。

第三方 Gazebo 模型、TurtleBot3 资源及其他外部资产仍遵循其各自原始许可证。

---

## Citation

如果本项目对你的研究或开发有帮助，欢迎引用本仓库。

相关论文或正式出版信息发布后，可在此补充 BibTeX。

---

## Repository

GitHub:

```text
https://github.com/DreamAblaze/AutoFactory-AGV
```

Maintainer:

```text
DreamAblaze
```

---

## Version

```text
v1.0.0
```

Validated mission:

```text
Full-Coverage Inspection
→ Parking Staging
→ ArUco Recognition
→ Hybrid A*
→ Autonomous Parking
```
