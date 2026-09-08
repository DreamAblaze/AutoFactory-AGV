#!/usr/bin/env bash

# ============================================================
# AutoFactory AGV 一键启动脚本
#
# 功能：
#   1. 清理旧的 Gazebo / Nav2 / RViz / TF 等进程
#   2. 启动 Gazebo 工厂世界
#   3. 生成 AutoFactory Waffle 机器人
#   4. 启动 Robot TF + Camera Optical TF
#   5. 启动 Nav2，控制器使用 RPP
#   6. 自动发布 AMCL 初始位姿
#   7. 启动 RViz，并自动加载 AutoFactory 主配置
#
# 使用：
#   chmod +x start_autofactory_rpp.sh
#   ./start_autofactory_rpp.sh
# ============================================================

set -u

# ============================================================
# 1. 基础路径
# ============================================================

WS="/home/my-ubuntu/autofactory_ws"
DEV_WS="/home/my-ubuntu/dev_ws"

MAP_FILE="${WS}/src/agv_bringup/maps/autofactory_navigation.yaml"

RPP_PARAMS="${WS}/src/agv_bringup/params/autofactory_nav2_rpp.yaml"

RVIZ_CONFIG="${WS}/src/agv_bringup/rviz/autofactory_rpp.rviz"

# ============================================================
# 2. 基础检查
# ============================================================

if ! command -v gnome-terminal >/dev/null 2>&1; then

    echo "[ERROR] 没有找到 gnome-terminal。"
    echo "请先执行："
    echo ""
    echo "sudo apt install gnome-terminal"
    echo ""

    exit 1
fi


if [ ! -f "${MAP_FILE}" ]; then

    echo "[ERROR] 地图文件不存在："
    echo "${MAP_FILE}"

    exit 1
fi


if [ ! -f "${RPP_PARAMS}" ]; then

    echo "[ERROR] Nav2 RPP 参数文件不存在："
    echo "${RPP_PARAMS}"

    exit 1
fi


if [ ! -f "${RVIZ_CONFIG}" ]; then

    echo "[ERROR] RViz 配置文件不存在："
    echo "${RVIZ_CONFIG}"
    echo ""
    echo "请确认已经创建："
    echo "autofactory_rpp.rviz"

    exit 1
fi


if [ ! -f "${DEV_WS}/install/setup.bash" ]; then

    echo "[ERROR] dev_ws setup.bash 不存在："
    echo "${DEV_WS}/install/setup.bash"

    exit 1
fi


if [ ! -f "${WS}/install/setup.bash" ]; then

    echo "[ERROR] autofactory_ws setup.bash 不存在："
    echo "${WS}/install/setup.bash"

    exit 1
fi


# ============================================================
# 3. 所有新终端共用 ROS 环境
# ============================================================

COMMON_SETUP="
cd ${WS}
source /opt/ros/humble/setup.bash
source ${DEV_WS}/install/setup.bash
source ${WS}/install/setup.bash
"


echo "============================================================"
echo " AutoFactory AGV RPP 一键启动"
echo "============================================================"
echo ""
echo "工作空间："
echo "  ${WS}"
echo ""
echo "地图："
echo "  ${MAP_FILE}"
echo ""
echo "Nav2参数："
echo "  ${RPP_PARAMS}"
echo ""
echo "RViz配置："
echo "  ${RVIZ_CONFIG}"
echo ""
echo "============================================================"


# ============================================================
# [1/7] 清理旧进程
# ============================================================

echo "[1/7] 清理旧进程..."


# SLAM / Nav2
pkill -f slam_toolbox 2>/dev/null || true
pkill -f component_container_isolated 2>/dev/null || true

pkill -f amcl 2>/dev/null || true
pkill -f map_server 2>/dev/null || true
pkill -f planner_server 2>/dev/null || true
pkill -f controller_server 2>/dev/null || true
pkill -f bt_navigator 2>/dev/null || true
pkill -f behavior_server 2>/dev/null || true
pkill -f velocity_smoother 2>/dev/null || true
pkill -f waypoint_follower 2>/dev/null || true


# RViz
pkill -f rviz2 2>/dev/null || true


# Robot TF
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f robot_tf.launch.py 2>/dev/null || true
pkill -f camera_rgb_optical_tf 2>/dev/null || true


# Teleop
pkill -f teleop_keyboard 2>/dev/null || true


# Parking Staging
pkill -f parking_staging_navigator 2>/dev/null || true


# ArUco
pkill -f aruco_detector 2>/dev/null || true
pkill -f aruco_map_transformer 2>/dev/null || true


# Coverage
pkill -f coverage_node 2>/dev/null || true


# Gazebo
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true


sleep 2


# ============================================================
# [2/7] 启动 Gazebo 世界
# ============================================================

echo "[2/7] 启动 Gazebo 世界..."


gnome-terminal \
    --title="T1 Gazebo World" \
    -- bash -lc "

${COMMON_SETUP}

echo ''
echo '============================================================'
echo '[T1] Gazebo Factory World'
echo '============================================================'
echo ''

ros2 launch \
factory_simulation \
factory_world.launch.py

exec bash
"


# Gazebo需要时间启动
sleep 8


# ============================================================
# [3/7] 生成 Waffle
# ============================================================

echo "[3/7] 生成 AutoFactory Waffle..."


gnome-terminal \
    --title="T2 Spawn Waffle" \
    -- bash -lc "

${COMMON_SETUP}

echo ''
echo '============================================================'
echo '[T2] Spawn AutoFactory Waffle'
echo '============================================================'
echo ''

ros2 launch \
agv_bringup \
spawn_waffle.launch.py

exec bash
"


sleep 4


# ============================================================
# [4/7] Robot TF + Camera Optical TF
# ============================================================

echo "[4/7] 启动 Robot TF + Camera Optical TF..."


gnome-terminal \
    --title="T3 Robot TF" \
    -- bash -lc "

${COMMON_SETUP}

echo ''
echo '============================================================'
echo '[T3] Robot TF + Camera Optical TF'
echo '============================================================'
echo ''

echo '[T3] TurtleBot3 model: waffle'
echo '[T3] 发布完整机器人 TF'
echo '[T3] 发布 camera_rgb_frame -> camera_rgb_optical_frame'
echo ''

ros2 launch \
agv_bringup \
robot_tf.launch.py \
use_sim_time:=true

exec bash
"


sleep 6


# ============================================================
# [5/7] Nav2 RPP
# ============================================================

echo "[5/7] 启动 Nav2 RPP..."


gnome-terminal \
    --title="T4 Nav2 RPP" \
    -- bash -lc "

${COMMON_SETUP}

echo ''
echo '============================================================'
echo '[T4] Nav2 + Regulated Pure Pursuit'
echo '============================================================'
echo ''

echo '[T4] Map:'
echo '${MAP_FILE}'
echo ''

echo '[T4] Params:'
echo '${RPP_PARAMS}'
echo ''

ros2 launch \
agv_bringup \
navigation.launch.py \
use_sim_time:=true \
autostart:=true \
map:=${MAP_FILE} \
params_file:=${RPP_PARAMS}

exec bash
"


# 等待 Nav2 生命周期节点激活
sleep 10


# ============================================================
# [6/7] 发布 AMCL 初始位姿
# ============================================================

echo "[6/7] 发布机器人初始位姿..."


gnome-terminal \
    --title="T5 Initial Pose" \
    -- bash -lc "

${COMMON_SETUP}

echo ''
echo '============================================================'
echo '[T5] AMCL Initial Pose'
echo '============================================================'
echo ''

echo '[T5] 等待 /initialpose ...'

for i in {1..20}; do

    if ros2 topic list 2>/dev/null \
        | grep -q '^/initialpose$'; then

        echo '[T5] /initialpose 已发现。'
        break
    fi

    echo \"[T5] waiting... \$i/20\"

    sleep 1
done


echo ''
echo '[T5] 发布机器人初始位姿：'
echo '     x   = 8.738'
echo '     y   = 7.489'
echo '     yaw = pi'
echo ''


ros2 topic pub \
    --rate 1 \
    --times 5 \
    /initialpose \
    geometry_msgs/msg/PoseWithCovarianceStamped \
\"{
  header: {
    frame_id: 'map'
  },

  pose: {

    pose: {

      position: {
        x: 8.738,
        y: 7.489,
        z: 0.0
      },

      orientation: {
        x: 0.0,
        y: 0.0,
        z: 1.0,
        w: 0.0
      }
    },

    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    ]
  }
}\"


echo ''
echo '============================================================'
echo '[T5] 初始位姿发布完成'
echo '============================================================'
echo ''

echo '可使用下面命令检查：'
echo ''
echo 'ros2 run tf2_ros tf2_echo map base_footprint'
echo ''

exec bash
"


sleep 3


# ============================================================
# [7/7] 启动 RViz
# ============================================================

echo "[7/7] 启动 RViz，并加载 AutoFactory 主配置..."


gnome-terminal \
    --title="T6 AutoFactory RViz" \
    -- bash -lc "

${COMMON_SETUP}

echo ''
echo '============================================================'
echo '[T6] AutoFactory RViz'
echo '============================================================'
echo ''

echo '[T6] 自动加载配置：'
echo '${RVIZ_CONFIG}'
echo ''

rviz2 \
    -d ${RVIZ_CONFIG} \
    --ros-args \
    -p use_sim_time:=true

exec bash
"


# ============================================================
# 启动完成
# ============================================================

echo ""
echo "============================================================"
echo " AutoFactory RPP 启动命令已全部发出"
echo "============================================================"
echo ""
echo "已启动："
echo ""
echo "  T1  Gazebo Factory World"
echo "  T2  AutoFactory Waffle"
echo "  T3  Robot TF + Camera Optical TF"
echo "  T4  Nav2 + RPP"
echo "  T5  AMCL Initial Pose"
echo "  T6  RViz + autofactory_rpp.rviz"
echo ""
echo "------------------------------------------------------------"
echo "建议检查："
echo "------------------------------------------------------------"
echo ""
echo "1. RPP Controller："
echo ""
echo "   ros2 param get /controller_server FollowPath.plugin"
echo ""
echo "   正确应为："
echo "   nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
echo ""
echo "2. CameraInfo："
echo ""
echo "   ros2 topic echo /camera/camera_info --once"
echo ""
echo "3. Camera Optical Frame："
echo ""
echo "   ros2 run tf2_ros tf2_echo camera_rgb_frame camera_rgb_optical_frame"
echo ""
echo "4. AMCL定位："
echo ""
echo "   ros2 run tf2_ros tf2_echo map base_footprint"
echo ""
echo "5. Nav2 Actions："
echo ""
echo "   ros2 action list | grep -E 'navigate|compute_path|follow_path|spin'"
echo ""
echo "6. RViz配置："
echo ""
echo "   ${RVIZ_CONFIG}"
echo ""
echo "============================================================"
