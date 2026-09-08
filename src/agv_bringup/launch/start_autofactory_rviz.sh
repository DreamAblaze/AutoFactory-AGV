#!/usr/bin/env bash

set -e

WS="/home/my-ubuntu/autofactory_ws"
DEV_WS="/home/my-ubuntu/dev_ws"

RVIZ_CONFIG="${WS}/src/agv_bringup/rviz/autofactory_rpp.rviz"

source /opt/ros/humble/setup.bash

if [ -f "${DEV_WS}/install/setup.bash" ]; then
    source "${DEV_WS}/install/setup.bash"
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

if [ ! -f "${RVIZ_CONFIG}" ]; then
    echo "[ERROR] RViz config not found:"
    echo "${RVIZ_CONFIG}"
    exit 1
fi

echo "=========================================="
echo " AutoFactory RViz"
echo "=========================================="
echo "Config:"
echo "${RVIZ_CONFIG}"
echo "=========================================="

exec ros2 run rviz2 rviz2 \
    -d "${RVIZ_CONFIG}" \
    --ros-args \
    -p use_sim_time:=true
