#!/usr/bin/env bash
# ============================================================
# AutoFactory AGV 牛耕覆盖一键启动脚本
#
# 默认模式：
#   ./start_autofactory_coverage.sh
#   启动 Gazebo + 机器人 + TF + Nav2 RPP + RViz + coverage_node
#   并自动调用 /start_coverage 生成牛耕路径，但不自动执行机器人。
#
# 执行模式：
#   ./start_autofactory_coverage.sh execute
#   在默认模式基础上，自动调用 /execute_coverage。
#   实际执行几批由 coverage_params.yaml 中的 max_batches_to_execute 决定。
#
# 注意：
#   本脚本不会直接发布 /cmd_vel。
#   机器人运动由 coverage_node 调用 Nav2 /navigate_through_poses，
#   再由 Nav2 RPP 控制器输出 /cmd_vel。
# ============================================================

set -u

MODE="${1:-plan}"

WS="/home/my-ubuntu/autofactory_ws"
DEV_WS="/home/my-ubuntu/dev_ws"

MAP_FILE="${WS}/src/agv_bringup/maps/autofactory_navigation.yaml"
RPP_PARAMS="${WS}/src/agv_bringup/params/autofactory_nav2_rpp.yaml"

RVIZ_CONFIG="${WS}/src/agv_bringup/rviz/autofactory_coverage.rviz"
COVERAGE_LAUNCH_FILE="${WS}/src/agv_coverage/launch/coverage.launch.py"

COMMON_SETUP="
cd ${WS}
source /opt/ros/humble/setup.bash
source ${DEV_WS}/install/setup.bash
source ${WS}/install/setup.bash
"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "[ERROR] 没有找到 gnome-terminal。请先安装："
  echo "sudo apt install gnome-terminal"
  exit 1
fi

if [ ! -f "${MAP_FILE}" ]; then
  echo "[ERROR] 地图文件不存在：${MAP_FILE}"
  exit 1
fi

if [ ! -f "${RPP_PARAMS}" ]; then
  echo "[ERROR] RPP 参数文件不存在：${RPP_PARAMS}"
  exit 1
fi

if [ ! -f "${RVIZ_CONFIG}" ]; then
  echo "[ERROR] RViz 配置文件不存在：${RVIZ_CONFIG}"
  exit 1
fi

if [ ! -f "${COVERAGE_LAUNCH_FILE}" ]; then
  echo "[ERROR] coverage.launch.py 不存在：${COVERAGE_LAUNCH_FILE}"
  exit 1
fi

echo "============================================================"
echo "AutoFactory AGV 牛耕覆盖一键启动"
echo "工作空间: ${WS}"
echo "地图文件: ${MAP_FILE}"
echo "RPP参数: ${RPP_PARAMS}"
echo "RViz配置: ${RVIZ_CONFIG}"
echo "Coverage launch: ${COVERAGE_LAUNCH_FILE}"
echo "启动模式: ${MODE}"
echo "============================================================"

echo "[0/9] 清理旧进程..."

pkill -f coverage_node 2>/dev/null || true
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
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f robot_tf.launch.py 2>/dev/null || true
pkill -f camera_rgb_optical_tf 2>/dev/null || true
pkill -f teleop_keyboard 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true

sleep 2

echo "[1/9] 启动 Gazebo 工厂世界..."

gnome-terminal --title="T1 Gazebo World" -- bash -lc "
${COMMON_SETUP}
echo '[T1] 启动 Gazebo 工厂世界...'
ros2 launch factory_simulation factory_world.launch.py
exec bash
"

sleep 8

echo "[2/9] 生成机器人..."

gnome-terminal --title="T2 Spawn Waffle" -- bash -lc "
${COMMON_SETUP}
echo '[T2] 生成 AutoFactory Waffle 机器人...'
ros2 launch agv_bringup spawn_waffle.launch.py
exec bash
"

sleep 4

echo "[3/9] 启动机器人 TF + Camera Optical TF..."

gnome-terminal --title="T3 Robot TF" -- bash -lc "
${COMMON_SETUP}
echo '[T3] 启动 AutoFactory Robot TF...'
echo '[T3] 同时发布 camera_rgb_frame -> camera_rgb_optical_frame'
ros2 launch agv_bringup robot_tf.launch.py use_sim_time:=true
exec bash
"

sleep 6

echo "[4/9] 启动 Nav2 RPP..."

gnome-terminal --title="T4 Nav2 RPP" -- bash -lc "
${COMMON_SETUP}
echo '[T4] 启动 Nav2，控制器使用 RPP...'
ros2 launch agv_bringup navigation.launch.py \
  use_sim_time:=true \
  autostart:=true \
  map:=${MAP_FILE} \
  params_file:=${RPP_PARAMS}
exec bash
"

sleep 12

echo "[5/9] 发布初始位姿..."

gnome-terminal --title="T5 Initial Pose" -- bash -lc "
${COMMON_SETUP}
echo '[T5] 等待 /initialpose 话题...'
for i in {1..30}; do
  if ros2 topic list 2>/dev/null | grep -q '^/initialpose$'; then
    break
  fi
  sleep 1
done

echo '[T5] 发布机器人初始位姿：x=8.738, y=7.489, yaw=pi'
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
echo '[T5] 初始位姿发布完成。'
echo '可检查：ros2 run tf2_ros tf2_echo map base_footprint'
exec bash
"

sleep 3

echo "[6/9] 启动 RViz 指定配置..."

gnome-terminal --title="T6 RViz Coverage" -- bash -lc "
${COMMON_SETUP}
echo '[T6] 打开 RViz 覆盖配置...'
exec rviz2 -d ${RVIZ_CONFIG} --ros-args -p use_sim_time:=true
"

sleep 4

echo "[7/9] 启动 agv_coverage 覆盖节点..."

gnome-terminal --title="T7 Coverage Node" -- bash -lc "
${COMMON_SETUP}
echo '[T7] 启动 agv_coverage coverage.launch.py...'
echo 'Launch file: ${COVERAGE_LAUNCH_FILE}'
ros2 launch ${COVERAGE_LAUNCH_FILE}
exec bash
"

sleep 6

echo "[8/9] 自动生成牛耕覆盖路径..."

gnome-terminal --title="T8 Coverage Plan Execute" -- bash -lc "
${COMMON_SETUP}

echo '[T8] 等待 /start_coverage 服务...'
for i in {1..60}; do
  if ros2 service list 2>/dev/null | grep -q '^/start_coverage$'; then
    break
  fi
  sleep 1
done

echo '[T8] 调用 /start_coverage 生成牛耕路径...'
ros2 service call /start_coverage std_srvs/srv/Trigger {}

echo ''
echo '[T8] 当前 coverage 话题：'
ros2 topic list | grep coverage || true

if [ '${MODE}' = 'execute' ]; then
  echo ''
  echo '[T8] 启动模式为 execute，等待 3 秒后调用 /execute_coverage...'
  sleep 3
  ros2 service call /execute_coverage std_srvs/srv/Trigger {}
else
  echo ''
  echo '[T8] 当前为 plan 模式，只生成路径，不自动执行机器人。'
  echo '[T8] 如需执行当前配置的批次数，请手动运行：'
  echo 'ros2 service call /execute_coverage std_srvs/srv/Trigger {}'
fi

exec bash
"

sleep 2

echo "[9/9] 打开覆盖状态监听终端..."

gnome-terminal --title="T9 Coverage Status" -- bash -lc "
${COMMON_SETUP}
echo '[T9] 监听 /coverage_status...'
echo '如果暂时没有输出，等待 /start_coverage 或 /execute_coverage 发布状态。'
ros2 topic echo /coverage_status
exec bash
"

echo "============================================================"
echo "牛耕覆盖启动命令已全部发出。"
echo ""
echo "默认只生成路径，不执行机器人。"
echo "执行机器人有两种方式："
echo ""
echo "方式一：手动执行当前配置的批次数"
echo "  ros2 service call /execute_coverage std_srvs/srv/Trigger {}"
echo ""
echo "方式二：启动脚本时加 execute 参数"
echo "  ./start_autofactory_coverage.sh execute"
echo ""
echo "建议先保持 coverage_params.yaml 中："
echo "  max_batches_to_execute: 1"
echo "确认第 1 批稳定后，再改成 3、5、全部。"
echo "============================================================"
