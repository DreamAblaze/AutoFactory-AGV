#!/usr/bin/env bash

# ============================================================
# AutoFactory One-Key Parking Mission
#
# Stable Parking Integration V3.2
#
# Flow:
#
#   Base / Nav2
#       ↓
#   Parking Staging
#       ↓
#   Verify Staging XY / Yaw
#       ↓
#   ArUco ID=0
#       ↓
#   ArUco LOCKED + Map TRACKING
#       ↓
#   Hybrid A*
#       ↓
#   Parking Tracker V3.2
#       ↓
#   PATH_READY
#       ↓
#   Velocity ownership handoff
#       ↓
#   Execute parking
#       ↓
#   Final alignment
#       ↓
#   Restore Nav2 velocity_smoother
#
# IMPORTANT:
#
# controller_server remains ACTIVE.
#
# Nav2 chain:
#
#   controller_server
#         ↓
#   /cmd_vel_nav
#         ↓
#   velocity_smoother
#         ↓
#   /cmd_vel
#
# Parking Tracker:
#
#   parking_path_tracker
#         ↓
#   /cmd_vel
#
# During parking:
#   velocity_smoother must NOT be ACTIVE.
#
# Valid parking handoff states:
#
#   inactive
#   unconfigured
#
# ============================================================


set +u


# ============================================================
# Workspace
# ============================================================

WS="/home/my-ubuntu/autofactory_ws"

DEV_WS="/home/my-ubuntu/dev_ws"

BASE_SCRIPT="${WS}/src/agv_bringup/launch/start_autofactory_rpp.sh"

LOG_DIR="/tmp/autofactory_parking"

mkdir -p "${LOG_DIR}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

MISSION_LOG="${LOG_DIR}/mission_${TIMESTAMP}.log"


# ============================================================
# Timeouts
# ============================================================

BASE_TIMEOUT=120

STAGING_TIMEOUT=240

ARUCO_TIMEOUT=60

ARUCO_IMAGE_HOLD_SEC=5

PLANNER_TIMEOUT=60

TRACKER_TIMEOUT=60

PARKING_TIMEOUT=600


# ============================================================
# Frozen known-good values
# ============================================================

STAGING_X=-6.0
STAGING_Y=-4.5
STAGING_YAW=3.1

STAGING_HANDOFF_XY_TOL=0.15
STAGING_HANDOFF_YAW_TOL_DEG=7.0

EXPECTED_TRACKER_VERSION="V3.2_SAFE_GEOMETRY"

EXPECTED_MAP_TOPIC="/global_costmap/costmap"


# ============================================================
# Runtime
# ============================================================

NAV2_PAUSED=0

RQT_PID=""

MISSION_SUCCESS=0


# ============================================================
# Prevent duplicate missions
# ============================================================

LOCK_FILE="/tmp/autofactory_parking_mission.lock"

exec 9>"${LOCK_FILE}"

if ! flock -n 9; then

    echo ""
    echo "============================================================"
    echo " PARKING MISSION ALREADY RUNNING"
    echo "============================================================"
    echo ""
    echo "已有一个 start_parking_mission.sh 正在运行。"
    echo "不要同时启动第二个停车任务。"
    echo ""

    exit 1
fi


# ============================================================
# ROS environment
# ============================================================

source /opt/ros/humble/setup.bash


if [ ! -f "${DEV_WS}/install/setup.bash" ]; then

    echo "[ERROR] Missing:"
    echo "${DEV_WS}/install/setup.bash"

    exit 1
fi


source "${DEV_WS}/install/setup.bash"


if [ ! -f "${WS}/install/setup.bash" ]; then

    echo "[ERROR] Missing:"
    echo "${WS}/install/setup.bash"

    exit 1
fi


source "${WS}/install/setup.bash"


# ============================================================
# Mission log
# ============================================================

exec > >(tee -a "${MISSION_LOG}") 2>&1


COMMON_SETUP="
set +u
cd ${WS}
source /opt/ros/humble/setup.bash
source ${DEV_WS}/install/setup.bash
source ${WS}/install/setup.bash
"


# ============================================================
# Output helper
# ============================================================

section()
{
    echo ""
    echo "============================================================"
    echo "$1"
    echo "============================================================"
}


# ============================================================
# Lifecycle helper
# ============================================================

get_lifecycle_state()
{
    local NODE="$1"

    local OUTPUT

    OUTPUT="$(
        ros2 lifecycle get \
        "${NODE}" \
        2>/dev/null \
        || true
    )"

    echo "${OUTPUT}" \
        | awk 'NR==1 {print $1}'
}


# ============================================================
# Wait node
# ============================================================

wait_for_node()
{
    local NODE="$1"
    local TIMEOUT_SEC="$2"

    echo "[WAIT] Node: ${NODE}"

    for ((i=1; i<=TIMEOUT_SEC; i++)); do

        if ros2 node list 2>/dev/null \
            | grep -qx "${NODE}"; then

            echo "[OK] Node ready: ${NODE}"

            return 0
        fi

        sleep 1
    done

    echo "[ERROR] Node timeout: ${NODE}"

    return 1
}


# ============================================================
# Wait service
# ============================================================

wait_for_service()
{
    local SERVICE="$1"
    local TIMEOUT_SEC="$2"

    echo "[WAIT] Service: ${SERVICE}"

    for ((i=1; i<=TIMEOUT_SEC; i++)); do

        if ros2 service list 2>/dev/null \
            | grep -qx "${SERVICE}"; then

            echo "[OK] Service ready: ${SERVICE}"

            return 0
        fi

        sleep 1
    done

    echo "[ERROR] Service timeout: ${SERVICE}"

    return 1
}


# ============================================================
# Wait topic
# ============================================================

wait_for_topic()
{
    local TOPIC="$1"
    local TIMEOUT_SEC="$2"

    echo "[WAIT] Topic: ${TOPIC}"

    for ((i=1; i<=TIMEOUT_SEC; i++)); do

        if ros2 topic list 2>/dev/null \
            | grep -qx "${TOPIC}"; then

            echo "[OK] Topic ready: ${TOPIC}"

            return 0
        fi

        sleep 1
    done

    echo "[ERROR] Topic timeout: ${TOPIC}"

    return 1
}


# ============================================================
# Wait topic data
# ============================================================

wait_for_topic_data()
{
    local TOPIC="$1"
    local TIMEOUT_SEC="$2"

    echo "[WAIT] Data: ${TOPIC}"

    local START
    START="$(date +%s)"

    while true; do

        if timeout 4 \
            ros2 topic echo \
            "${TOPIC}" \
            --once \
            >/dev/null 2>&1; then

            echo "[OK] Data received: ${TOPIC}"

            return 0
        fi


        local NOW
        NOW="$(date +%s)"


        if [ $((NOW - START)) \
             -ge "${TIMEOUT_SEC}" ]; then

            echo "[ERROR] No data: ${TOPIC}"

            return 1
        fi


        sleep 1
    done
}


# ============================================================
# Wait lifecycle ACTIVE
# ============================================================

wait_lifecycle_active()
{
    local NODE="$1"
    local TIMEOUT_SEC="$2"

    echo "[WAIT] Lifecycle active: ${NODE}"

    for ((i=1; i<=TIMEOUT_SEC; i++)); do

        STATE="$(
            get_lifecycle_state \
            "${NODE}"
        )"


        if [ "${STATE}" = "active" ]; then

            echo "[OK] ${NODE}: active"

            return 0
        fi


        sleep 1
    done


    echo "[ERROR] ${NODE} did not become active"

    return 1
}


# ============================================================
# Wait Bool == true
# ============================================================

wait_bool_true()
{
    local TOPIC="$1"
    local TIMEOUT_SEC="$2"

    echo "[WAIT] ${TOPIC} = true"

    local START
    START="$(date +%s)"


    while true; do

        OUTPUT="$(
            timeout 3 \
            ros2 topic echo \
            "${TOPIC}" \
            --once \
            2>/dev/null \
            || true
        )"


        if echo "${OUTPUT}" \
            | grep -q "data: true"; then

            echo "[OK] ${TOPIC} = true"

            return 0
        fi


        local NOW
        NOW="$(date +%s)"


        if [ $((NOW - START)) \
             -ge "${TIMEOUT_SEC}" ]; then

            echo "[ERROR] Timeout waiting ${TOPIC}=true"

            return 1
        fi


        sleep 0.5
    done
}


# ============================================================
# Wait String
# ============================================================

wait_string_value()
{
    local TOPIC="$1"
    local EXPECTED="$2"
    local TIMEOUT_SEC="$3"

    echo "[WAIT] ${TOPIC} = ${EXPECTED}"

    local START
    START="$(date +%s)"


    while true; do

        OUTPUT="$(
            timeout 3 \
            ros2 topic echo \
            "${TOPIC}" \
            --once \
            2>/dev/null \
            || true
        )"


        if echo "${OUTPUT}" \
            | grep -q "data: ${EXPECTED}"; then

            echo "[OK] ${TOPIC} = ${EXPECTED}"

            return 0
        fi


        local NOW
        NOW="$(date +%s)"


        if [ $((NOW - START)) \
             -ge "${TIMEOUT_SEC}" ]; then

            echo "[ERROR] Timeout waiting:"
            echo "        ${TOPIC} = ${EXPECTED}"

            return 1
        fi


        sleep 0.5
    done
}


# ============================================================
# Wait Parking Staging terminal state
#
# return:
#
# 0  strict success
# 10 final pose just outside strict tolerance
# 1  hard failure
# ============================================================

wait_staging_terminal_state()
{
    local TIMEOUT_SEC="$1"

    echo ""
    echo "[WAIT] Parking Staging terminal state..."


    local START
    START="$(date +%s)"


    local LAST_STATUS=""


    while true; do

        REACHED="$(
            timeout 2 \
            ros2 topic echo \
            /parking_staging_reached \
            --once \
            2>/dev/null \
            || true
        )"


        if echo "${REACHED}" \
            | grep -q "data: true"; then

            echo ""
            echo "[OK] Parking Staging strict success."

            return 0
        fi


        STATUS_OUTPUT="$(
            timeout 2 \
            ros2 topic echo \
            /parking_staging_status \
            --once \
            2>/dev/null \
            || true
        )"


        STATUS="$(
            echo "${STATUS_OUTPUT}" \
            | sed -n 's/^data: //p' \
            | head -n 1
        )"


        if [ -n "${STATUS}" ] \
            && [ "${STATUS}" != "${LAST_STATUS}" ]; then

            echo "[STAGING] ${STATUS}"

            LAST_STATUS="${STATUS}"
        fi


        if [ "${STATUS}" = \
             "FINAL_POSE_OUTSIDE_TOLERANCE" ]; then

            echo ""
            echo "[WARN] Staging finished outside strict tolerance."
            echo "[WARN] Independent TF guard will verify pose."

            return 10
        fi


        case "${STATUS}" in

            FAILED_*|CANCELED|CANCELLED|PLANNING_FAILED|FOLLOW_PATH_FAILED|SPIN_FAILED)

                echo ""
                echo "[ERROR] Parking Staging hard failure:"
                echo "${STATUS}"

                return 1
                ;;

        esac


        local NOW
        NOW="$(date +%s)"


        if [ $((NOW - START)) \
             -ge "${TIMEOUT_SEC}" ]; then

            echo ""
            echo "[ERROR] Parking Staging timeout."
            echo "[ERROR] Last state: ${STATUS}"

            return 1
        fi


        sleep 0.5
    done
}


# ============================================================
# Verify Staging XY
# ============================================================

verify_staging_xy()
{
python3 - <<PY
import math
import sys
import time

import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)


TARGET_X = ${STAGING_X}
TARGET_Y = ${STAGING_Y}

MAX_ERROR = ${STAGING_HANDOFF_XY_TOL}


rclpy.init()


node = Node(
    "parking_staging_xy_guard",
    parameter_overrides=[
        Parameter(
            "use_sim_time",
            Parameter.Type.BOOL,
            True,
        )
    ],
)


tf_buffer = Buffer()

tf_listener = TransformListener(
    tf_buffer,
    node,
)


deadline = time.time() + 10.0

pose = None


while time.time() < deadline:

    rclpy.spin_once(
        node,
        timeout_sec=0.1,
    )

    try:

        tf = tf_buffer.lookup_transform(
            "map",
            "base_footprint",
            Time(),
        )

        pose = (
            tf.transform.translation.x,
            tf.transform.translation.y,
        )

        break

    except TransformException:

        pass


if pose is None:

    node.get_logger().error(
        "Cannot obtain map -> base_footprint TF."
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(2)


x, y = pose


error = math.hypot(
    x - TARGET_X,
    y - TARGET_Y,
)


node.get_logger().info(
    "========================================"
)

node.get_logger().info(
    f"Current staging position: "
    f"x={x:.3f}, y={y:.3f}"
)

node.get_logger().info(
    f"Target staging position: "
    f"x={TARGET_X:.3f}, y={TARGET_Y:.3f}"
)

node.get_logger().info(
    f"XY error = {error:.3f} m"
)

node.get_logger().info(
    f"Task handoff tolerance = "
    f"{MAX_ERROR:.3f} m"
)


if error > MAX_ERROR:

    node.get_logger().error(
        "Staging XY outside handoff tolerance."
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(3)


node.get_logger().info(
    "Staging XY verification PASSED."
)

node.get_logger().info(
    "========================================"
)


node.destroy_node()
rclpy.shutdown()

sys.exit(0)
PY
}


# ============================================================
# Verify / correct Staging Yaw
# ============================================================

verify_and_correct_staging_yaw()
{
python3 - <<PY
import math
import sys
import time

import rclpy

from action_msgs.msg import GoalStatus
from nav2_msgs.action import Spin

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)


TARGET_YAW = ${STAGING_YAW}

TOLERANCE = math.radians(
    ${STAGING_HANDOFF_YAW_TOL_DEG}
)


def normalize(angle):

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def yaw_from_quaternion(q):

    siny_cosp = 2.0 * (
        q.w * q.z
        + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y
        + q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


def read_yaw(
    node,
    tf_buffer,
    timeout_sec=10.0,
):

    deadline = time.time() + timeout_sec


    while time.time() < deadline:

        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )


        try:

            tf = tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
            )


            return yaw_from_quaternion(
                tf.transform.rotation
            )


        except TransformException:

            pass


    raise RuntimeError(
        "Cannot obtain map -> base_footprint TF."
    )


rclpy.init()


node = Node(
    "parking_staging_yaw_guard",
    parameter_overrides=[
        Parameter(
            "use_sim_time",
            Parameter.Type.BOOL,
            True,
        )
    ],
)


tf_buffer = Buffer()

tf_listener = TransformListener(
    tf_buffer,
    node,
)


try:

    current_yaw = read_yaw(
        node,
        tf_buffer,
    )


except Exception as exc:

    node.get_logger().error(
        str(exc)
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(2)


error = normalize(
    TARGET_YAW - current_yaw
)


node.get_logger().info(
    "========================================"
)

node.get_logger().info(
    f"Current staging yaw = "
    f"{current_yaw:.4f} rad "
    f"({math.degrees(current_yaw):.2f} deg)"
)

node.get_logger().info(
    f"Target staging yaw  = "
    f"{TARGET_YAW:.4f} rad "
    f"({math.degrees(TARGET_YAW):.2f} deg)"
)

node.get_logger().info(
    f"Yaw error = "
    f"{math.degrees(error):.2f} deg"
)


if abs(error) <= TOLERANCE:

    node.get_logger().info(
        "Staging heading verification PASSED."
    )

    node.get_logger().info(
        "========================================"
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0)


node.get_logger().warn(
    "Staging heading outside task tolerance."
)

node.get_logger().warn(
    "Sending corrective /spin goal..."
)


spin_client = ActionClient(
    node,
    Spin,
    "/spin",
)


if not spin_client.wait_for_server(
    timeout_sec=8.0
):

    node.get_logger().error(
        "/spin action server unavailable."
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(3)


goal = Spin.Goal()

goal.target_yaw = float(error)

goal.time_allowance.sec = 15
goal.time_allowance.nanosec = 0


future = spin_client.send_goal_async(
    goal
)


rclpy.spin_until_future_complete(
    node,
    future,
    timeout_sec=8.0,
)


goal_handle = future.result()


if (
    goal_handle is None
    or not goal_handle.accepted
):

    node.get_logger().error(
        "Corrective /spin goal rejected."
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(4)


result_future = (
    goal_handle.get_result_async()
)


rclpy.spin_until_future_complete(
    node,
    result_future,
    timeout_sec=20.0,
)


wrapped_result = (
    result_future.result()
)


if wrapped_result is None:

    node.get_logger().error(
        "Corrective /spin result timeout."
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(5)


if (
    wrapped_result.status
    != GoalStatus.STATUS_SUCCEEDED
):

    node.get_logger().error(
        f"Corrective /spin failed. "
        f"status={wrapped_result.status}"
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(6)


time.sleep(1.0)


try:

    final_yaw = read_yaw(
        node,
        tf_buffer,
        timeout_sec=5.0,
    )


except Exception as exc:

    node.get_logger().error(
        str(exc)
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(7)


final_error = normalize(
    TARGET_YAW - final_yaw
)


node.get_logger().info(
    f"Final staging yaw = "
    f"{final_yaw:.4f} rad "
    f"({math.degrees(final_yaw):.2f} deg)"
)

node.get_logger().info(
    f"Final yaw error = "
    f"{math.degrees(final_error):.2f} deg"
)


if abs(final_error) > TOLERANCE:

    node.get_logger().error(
        "Final staging heading outside tolerance."
    )

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(8)


node.get_logger().info(
    "Staging heading verification PASSED."
)

node.get_logger().info(
    "========================================"
)


node.destroy_node()
rclpy.shutdown()

sys.exit(0)
PY
}


# ============================================================
# Close rqt
# ============================================================

close_rqt()
{
    if [ -z "${RQT_PID}" ]; then

        return
    fi


    if kill -0 "${RQT_PID}" \
        >/dev/null 2>&1; then

        kill "${RQT_PID}" \
            >/dev/null 2>&1 \
            || true

        sleep 1
    fi


    RQT_PID=""
}


# ============================================================
# Stop parking
# ============================================================

stop_parking()
{
    if ros2 service list 2>/dev/null \
        | grep -qx \
        "/stop_hybrid_parking"; then

        timeout 5 \
            ros2 service call \
            /stop_hybrid_parking \
            std_srvs/srv/Trigger \
            "{}" \
            >/dev/null 2>&1 \
            || true
    fi
}


# ============================================================
# Prepare velocity_smoother for parking
#
# Valid non-output states:
#
# inactive
# unconfigured
#
# If ACTIVE:
# try deactivate.
#
# IMPORTANT:
#
# Some runs transition:
#
# active -> unconfigured
#
# instead of:
#
# active -> inactive
#
# That is still safe for parking because the smoother is
# no longer actively forwarding velocity to /cmd_vel.
# ============================================================

prepare_parking_velocity_handoff()
{
    section "VELOCITY OWNERSHIP HANDOFF"


    local STATE


    STATE="$(
        get_lifecycle_state \
        /velocity_smoother
    )"


    echo ""
    echo "[HANDOFF] velocity_smoother initial state:"
    echo "${STATE}"


    case "${STATE}" in

        active)

            echo ""
            echo "[HANDOFF] velocity_smoother is ACTIVE."
            echo "[HANDOFF] Requesting deactivate..."


            timeout 10 \
                ros2 lifecycle set \
                /velocity_smoother \
                deactivate \
                >/dev/null 2>&1 \
                || true


            sleep 1


            STATE="$(
                get_lifecycle_state \
                /velocity_smoother
            )"


            echo ""
            echo "[HANDOFF] state after transition:"
            echo "${STATE}"
            ;;


        inactive)

            echo ""
            echo "[HANDOFF] velocity_smoother already INACTIVE."
            ;;


        unconfigured)

            echo ""
            echo "[HANDOFF] velocity_smoother already UNCONFIGURED."
            echo "[HANDOFF] This is safe for direct parking /cmd_vel."
            ;;


        *)

            echo ""
            echo "[ERROR] Unexpected velocity_smoother state:"
            echo "${STATE}"

            return 1
            ;;

    esac


    # ========================================================
    # Final safety decision
    # ========================================================

    if [ "${STATE}" = "active" ]; then

        echo ""
        echo "[ERROR] velocity_smoother is still ACTIVE."

        return 1
    fi


    if [ "${STATE}" != "inactive" ] \
        && [ "${STATE}" != "unconfigured" ]; then

        echo ""
        echo "[ERROR] velocity_smoother state is not safe:"
        echo "${STATE}"

        return 1
    fi


    # ========================================================
    # controller_server intentionally remains ACTIVE
    # ========================================================

    CONTROLLER_STATE="$(
        get_lifecycle_state \
        /controller_server
    )"


    echo ""
    echo "[HANDOFF] controller_server:"
    echo "${CONTROLLER_STATE}"


    if [ "${CONTROLLER_STATE}" != "active" ]; then

        echo ""
        echo "[ERROR] controller_server is unexpectedly not active."

        return 1
    fi


    # ========================================================
    # Confirm tracker is /cmd_vel publisher
    # ========================================================

    CMD_INFO="$(
        ros2 topic info \
        /cmd_vel \
        -v \
        2>/dev/null \
        || true
    )"


    echo ""
    echo "[HANDOFF] /cmd_vel endpoint information:"
    echo "${CMD_INFO}"


    if ! echo "${CMD_INFO}" \
        | grep -q \
        "parking_path_tracker"; then

        echo ""
        echo "[ERROR] parking_path_tracker is not a /cmd_vel publisher."

        return 1
    fi


    NAV2_PAUSED=1


    echo ""
    echo "[OK] Velocity ownership transferred to Parking Tracker."
    echo "[OK] velocity_smoother state: ${STATE}"

    return 0
}


# ============================================================
# Restore Nav2 velocity smoother
#
# Cases:
#
# active
#   -> nothing
#
# inactive
#   -> activate
#
# unconfigured
#   -> configure
#   -> inactive
#   -> activate
#
# controller_server remains active throughout.
# ============================================================

restore_nav2()
{
    if [ "${NAV2_PAUSED}" -ne 1 ]; then

        return
    fi


    section "RESTORE NAV2 VELOCITY OUTPUT"


    local STATE


    STATE="$(
        get_lifecycle_state \
        /velocity_smoother
    )"


    echo ""
    echo "[RESTORE] Initial velocity_smoother state:"
    echo "${STATE}"


    case "${STATE}" in

        active)

            echo "[RESTORE] Already active."
            ;;


        inactive)

            echo "[RESTORE] Activating velocity_smoother..."


            timeout 10 \
                ros2 lifecycle set \
                /velocity_smoother \
                activate \
                >/dev/null 2>&1 \
                || true


            sleep 1
            ;;


        unconfigured)

            echo "[RESTORE] Configuring velocity_smoother..."


            timeout 10 \
                ros2 lifecycle set \
                /velocity_smoother \
                configure \
                >/dev/null 2>&1 \
                || true


            sleep 1


            STATE="$(
                get_lifecycle_state \
                /velocity_smoother
            )"


            echo "[RESTORE] State after configure:"
            echo "${STATE}"


            if [ "${STATE}" = "inactive" ]; then

                echo "[RESTORE] Activating velocity_smoother..."


                timeout 10 \
                    ros2 lifecycle set \
                    /velocity_smoother \
                    activate \
                    >/dev/null 2>&1 \
                    || true


                sleep 1
            fi
            ;;


        *)

            echo ""
            echo "[WARN] Cannot automatically restore state:"
            echo "${STATE}"
            ;;

    esac


    STATE="$(
        get_lifecycle_state \
        /velocity_smoother
    )"


    CONTROLLER_STATE="$(
        get_lifecycle_state \
        /controller_server
    )"


    echo ""
    echo "[RESTORE] Final velocity_smoother:"
    echo "${STATE}"


    echo ""
    echo "[RESTORE] controller_server:"
    echo "${CONTROLLER_STATE}"


    if [ "${STATE}" != "active" ]; then

        echo ""
        echo "[WARN] velocity_smoother did not return to ACTIVE."
    else

        echo ""
        echo "[OK] Nav2 velocity output restored."
    fi


    NAV2_PAUSED=0
}


# ============================================================
# Failure handling
# ============================================================

mission_failed()
{
    local MESSAGE="$1"


    section "PARKING MISSION FAILED"


    echo ""
    echo "${MESSAGE}"
    echo ""


    echo "----- Parking Staging -----"

    timeout 3 \
        ros2 topic echo \
        /parking_staging_status \
        --once \
        2>/dev/null \
        || true


    echo ""
    echo "----- ArUco -----"

    timeout 3 \
        ros2 topic echo \
        /aruco/status \
        --once \
        2>/dev/null \
        || true


    echo ""
    echo "----- ArUco Map -----"

    timeout 3 \
        ros2 topic echo \
        /aruco/map_status \
        --once \
        2>/dev/null \
        || true


    echo ""
    echo "----- Hybrid Planner -----"

    timeout 3 \
        ros2 topic echo \
        /parking/hybrid_status \
        --once \
        2>/dev/null \
        || true


    echo ""
    echo "----- Tracker -----"

    timeout 3 \
        ros2 topic echo \
        /parking/tracking_status \
        --once \
        2>/dev/null \
        || true


    close_rqt

    stop_parking

    restore_nav2


    echo ""
    echo "[LOG]"
    echo "${MISSION_LOG}"
    echo ""


    exit 1
}


# ============================================================
# Exit cleanup
# ============================================================

cleanup()
{
    close_rqt


    if [ "${MISSION_SUCCESS}" -ne 1 ]; then

        stop_parking
    fi


    restore_nav2
}


trap cleanup EXIT


# ============================================================
# Mission header
# ============================================================

section "AutoFactory One-Key Parking Mission"


echo ""
echo "Stable execution chain:"
echo ""
echo "  Parking Staging"
echo "       ↓"
echo "  Staging Pose Guard"
echo "       ↓"
echo "  ArUco ID=0"
echo "       ↓"
echo "  Map Localization"
echo "       ↓"
echo "  Hybrid A*"
echo "       ↓"
echo "  Tracker V3.2"
echo "       ↓"
echo "  Velocity Handoff"
echo "       ↓"
echo "  Automatic Parking"
echo ""


# ============================================================
# [0/8] Clean stale parking nodes
#
# Do NOT kill:
#
# Gazebo
# Nav2
# AMCL
# map_server
# ============================================================

section "[0/8] CLEAN OLD PARKING NODES"


pkill -f parking_staging_navigator \
    2>/dev/null || true


pkill -f aruco_detector \
    2>/dev/null || true


pkill -f aruco_map_transformer \
    2>/dev/null || true


pkill -f parking_planner \
    2>/dev/null || true


pkill -f parking_path_tracker \
    2>/dev/null || true


pkill -f \
    'ros2 service call /stop_hybrid_parking' \
    2>/dev/null || true


sleep 3


# ============================================================
# [1/8] Base / Nav2
# ============================================================

section "[1/8] BASE / NAV2 READINESS"


if ! ros2 node list 2>/dev/null \
    | grep -qx "/controller_server"; then

    echo "[INFO] Base system is not running."

    echo "[INFO] Starting:"
    echo "${BASE_SCRIPT}"


    if [ ! -f "${BASE_SCRIPT}" ]; then

        mission_failed \
            "Base system startup script not found"
    fi


    bash "${BASE_SCRIPT}"
fi


if ! wait_for_node \
    "/controller_server" \
    "${BASE_TIMEOUT}"; then

    mission_failed \
        "controller_server not available"
fi


if ! wait_lifecycle_active \
    "/controller_server" \
    "${BASE_TIMEOUT}"; then

    mission_failed \
        "controller_server not active"
fi


if ! wait_lifecycle_active \
    "/velocity_smoother" \
    "${BASE_TIMEOUT}"; then

    mission_failed \
        "velocity_smoother not active"
fi


if ! wait_for_topic \
    "/global_costmap/costmap" \
    "${BASE_TIMEOUT}"; then

    mission_failed \
        "/global_costmap/costmap unavailable"
fi


if ! wait_for_topic_data \
    "/global_costmap/costmap" \
    30; then

    mission_failed \
        "Global Costmap has no data"
fi


if ! wait_for_topic \
    "/camera/image_raw" \
    "${BASE_TIMEOUT}"; then

    mission_failed \
        "/camera/image_raw unavailable"
fi


echo ""
echo "[OK] Base / Nav2 ready."


# ============================================================
# [2/8] Parking Staging
# ============================================================

section "[2/8] PARKING STAGING"


gnome-terminal \
    --title="Parking Staging" \
    -- bash -lc "

${COMMON_SETUP}

echo '============================================'
echo ' Parking Staging Navigator'
echo '============================================'

ros2 launch \
agv_task_manager \
parking_staging.launch.py \
use_sim_time:=true

exec bash
"


if ! wait_for_service \
    "/go_to_parking_staging" \
    30; then

    mission_failed \
        "Parking Staging service unavailable"
fi


echo ""
echo "[MISSION] Go to Parking Staging..."


STAGING_REQUEST="$(
    timeout 15 \
    ros2 service call \
    /go_to_parking_staging \
    std_srvs/srv/Trigger \
    "{}" \
    2>&1
)"


echo "${STAGING_REQUEST}"


if ! echo "${STAGING_REQUEST}" \
    | grep -Eq \
    "success=True|success: true"; then

    mission_failed \
        "Parking Staging request rejected"
fi


wait_staging_terminal_state \
    "${STAGING_TIMEOUT}"

STAGING_RESULT=$?


if [ "${STAGING_RESULT}" -eq 1 ]; then

    mission_failed \
        "Parking Staging execution failed"
fi


if [ "${STAGING_RESULT}" -eq 10 ]; then

    echo ""
    echo "[WARN] Strict staging tolerance not satisfied."
    echo "[WARN] TF guard will decide whether mission can continue."
fi


# ============================================================
# [3/8] Staging pose guard
# ============================================================

section "[3/8] VERIFY CAMERA STAGING POSE"


echo ""
echo "[CHECK] Staging XY..."


if ! verify_staging_xy; then

    mission_failed \
        "Robot is too far from camera staging position"
fi


echo ""
echo "[CHECK] Staging Yaw..."


if ! verify_and_correct_staging_yaw; then

    mission_failed \
        "Robot could not face ArUco marker"
fi


echo ""
echo "[CHECK] Staging XY after Yaw correction..."


if ! verify_staging_xy; then

    mission_failed \
        "Robot moved too far during final yaw correction"
fi


echo ""
echo "[OK] Staging handoff verified."
echo "[OK] Camera faces ArUco marker."


# ============================================================
# [4/8] ArUco
# ============================================================

section "[4/8] ARUCO ID=0 LOCALIZATION"


gnome-terminal \
    --title="ArUco Localization" \
    -- bash -lc "

${COMMON_SETUP}

echo '============================================'
echo ' ArUco Detection + Map Localization'
echo '============================================'

ros2 launch \
agv_perception \
aruco_localization.launch.py \
use_sim_time:=true

exec bash
"


if ! wait_for_service \
    "/aruco/enable" \
    30; then

    mission_failed \
        "/aruco/enable unavailable"
fi


if ! wait_for_topic \
    "/aruco/debug_image" \
    30; then

    mission_failed \
        "/aruco/debug_image unavailable"
fi


echo ""
echo "[IMAGE] Opening rqt_image_view..."


ros2 run \
    rqt_image_view \
    rqt_image_view \
    /aruco/debug_image \
    >"${LOG_DIR}/rqt_${TIMESTAMP}.log" \
    2>&1 &


RQT_PID=$!


sleep 2


ARUCO_ENABLE_RESULT="$(
    timeout 10 \
    ros2 service call \
    /aruco/enable \
    std_srvs/srv/SetBool \
    "{data: true}" \
    2>&1
)"


echo "${ARUCO_ENABLE_RESULT}"


if ! echo "${ARUCO_ENABLE_RESULT}" \
    | grep -Eq \
    "success=True|success: true"; then

    mission_failed \
        "ArUco detector could not be enabled"
fi


if ! wait_bool_true \
    "/aruco/locked" \
    "${ARUCO_TIMEOUT}"; then

    mission_failed \
        "ArUco ID=0 did not become LOCKED"
fi


echo ""
echo "[OK] ArUco ID=0 LOCKED."


if ! wait_bool_true \
    "/aruco/map_pose_available" \
    30; then

    mission_failed \
        "ArUco map pose unavailable"
fi


if ! wait_string_value \
    "/aruco/map_status" \
    "TRACKING" \
    30; then

    mission_failed \
        "ArUco map transformer did not reach TRACKING"
fi


echo ""
echo "[OK] ArUco marker pose available in map frame."

echo "[IMAGE] Hold image ${ARUCO_IMAGE_HOLD_SEC}s..."


sleep "${ARUCO_IMAGE_HOLD_SEC}"


close_rqt


# ============================================================
# [5/8] Hybrid A*
# ============================================================

section "[5/8] HYBRID A* PLANNING"


gnome-terminal \
    --title="Hybrid A Star Planner" \
    -- bash -lc "

${COMMON_SETUP}

echo '============================================'
echo ' Hybrid A* Parking Planner'
echo '============================================'

ros2 launch \
agv_parking \
hybrid_parking.launch.py \
use_sim_time:=true

exec bash
"


if ! wait_for_node \
    "/parking_planner" \
    "${PLANNER_TIMEOUT}"; then

    mission_failed \
        "parking_planner node unavailable"
fi


if ! wait_for_service \
    "/plan_hybrid_parking" \
    "${PLANNER_TIMEOUT}"; then

    mission_failed \
        "/plan_hybrid_parking unavailable"
fi


if ! wait_for_service \
    "/clear_hybrid_parking" \
    "${PLANNER_TIMEOUT}"; then

    mission_failed \
        "/clear_hybrid_parking unavailable"
fi


MAP_TOPIC="$(
    ros2 param get \
    /parking_planner \
    map_topic \
    2>/dev/null \
    || true
)"


OCCUPIED_THRESHOLD="$(
    ros2 param get \
    /parking_planner \
    occupied_threshold \
    2>/dev/null \
    || true
)"


ROBOT_RADIUS="$(
    ros2 param get \
    /parking_planner \
    robot_radius \
    2>/dev/null \
    || true
)"


SAFETY_MARGIN="$(
    ros2 param get \
    /parking_planner \
    safety_margin \
    2>/dev/null \
    || true
)"


echo ""
echo "[CHECK] Hybrid A* configuration:"
echo "  ${MAP_TOPIC}"
echo "  ${OCCUPIED_THRESHOLD}"
echo "  ${ROBOT_RADIUS}"
echo "  ${SAFETY_MARGIN}"


if ! echo "${MAP_TOPIC}" \
    | grep -q "${EXPECTED_MAP_TOPIC}"; then

    mission_failed \
        "Hybrid A* is not using /global_costmap/costmap"
fi


if ! echo "${OCCUPIED_THRESHOLD}" \
    | grep -Eq "value is: 1"; then

    mission_failed \
        "Hybrid A* occupied_threshold is not 1"
fi


if ! echo "${ROBOT_RADIUS}" \
    | grep -Eq "0\.0"; then

    mission_failed \
        "Hybrid A* robot_radius is not 0.0"
fi


if ! echo "${SAFETY_MARGIN}" \
    | grep -Eq "0\.05"; then

    mission_failed \
        "Hybrid A* safety_margin is not 0.05"
fi


echo ""
echo "[MISSION] Clear previous Hybrid path..."


timeout 10 \
    ros2 service call \
    /clear_hybrid_parking \
    std_srvs/srv/Trigger \
    "{}" \
    >/dev/null 2>&1 \
    || true


sleep 1


echo ""
echo "[MISSION] Planning Hybrid A* path..."


PLAN_RESULT="$(
    timeout 60 \
    ros2 service call \
    /plan_hybrid_parking \
    std_srvs/srv/Trigger \
    "{}" \
    2>&1
)"


echo "${PLAN_RESULT}"


if ! wait_bool_true \
    "/parking/hybrid_path_available" \
    30; then

    mission_failed \
        "Hybrid A* did not generate available path"
fi


if ! wait_string_value \
    "/parking/hybrid_status" \
    "SUCCEEDED" \
    30; then

    mission_failed \
        "Hybrid A* did not reach SUCCEEDED"
fi


PATH_INFO="$(
    ros2 topic info \
    /parking/hybrid_path \
    -v \
    2>/dev/null \
    || true
)"


echo ""
echo "[CHECK] /parking/hybrid_path:"
echo "${PATH_INFO}"


PUBLISHER_COUNT="$(
    echo "${PATH_INFO}" \
    | grep "Publisher count:" \
    | awk '{print $3}'
)"


if [ -n "${PUBLISHER_COUNT}" ] \
    && [ "${PUBLISHER_COUNT}" != "1" ]; then

    mission_failed \
        "/parking/hybrid_path must have one publisher"
fi


if ! wait_for_topic_data \
    "/parking/hybrid_path" \
    15; then

    mission_failed \
        "Hybrid path has no data"
fi


echo ""
echo "[OK] Hybrid A* path generated."


# ============================================================
# [6/8] Tracker
# ============================================================

section "[6/8] PARKING TRACKER V3.2"


gnome-terminal \
    --title="Parking Tracker V3.2" \
    -- bash -lc "

${COMMON_SETUP}

echo '============================================'
echo ' Parking Path Tracker V3.2'
echo '============================================'

ros2 launch \
agv_parking \
parking_tracker.launch.py \
use_sim_time:=true

exec bash
"


if ! wait_for_node \
    "/parking_path_tracker" \
    "${TRACKER_TIMEOUT}"; then

    mission_failed \
        "parking_path_tracker node unavailable"
fi


if ! wait_for_service \
    "/execute_hybrid_parking" \
    "${TRACKER_TIMEOUT}"; then

    mission_failed \
        "/execute_hybrid_parking unavailable"
fi


TRACKER_VERSION="$(
    ros2 param get \
    /parking_path_tracker \
    tracker_version \
    2>/dev/null \
    || true
)"


echo ""
echo "[CHECK] Actual tracker version:"
echo "${TRACKER_VERSION}"


if ! echo "${TRACKER_VERSION}" \
    | grep -q "${EXPECTED_TRACKER_VERSION}"; then

    mission_failed \
        "Parking Tracker is not V3.2_SAFE_GEOMETRY"
fi


if ! wait_string_value \
    "/parking/tracking_status" \
    "PATH_READY" \
    "${TRACKER_TIMEOUT}"; then

    mission_failed \
        "Parking Tracker did not enter PATH_READY"
fi


echo ""
echo "[OK] Parking Tracker PATH_READY."


# ============================================================
# [6.5/8] Velocity handoff
# ============================================================

if ! prepare_parking_velocity_handoff; then

    mission_failed \
        "Velocity ownership handoff failed"
fi


# ============================================================
# [7/8] Execute parking
# ============================================================

section "[7/8] EXECUTE HYBRID A* PARKING PATH"


echo ""
echo "[MISSION] Calling /execute_hybrid_parking..."


EXECUTE_RESULT="$(
    timeout 15 \
    ros2 service call \
    /execute_hybrid_parking \
    std_srvs/srv/Trigger \
    "{}" \
    2>&1
)"


echo "${EXECUTE_RESULT}"


if ! echo "${EXECUTE_RESULT}" \
    | grep -Eq \
    "success=True|success: true"; then

    mission_failed \
        "Parking Tracker rejected execute request"
fi


if ! wait_bool_true \
    "/parking/tracking_active" \
    10; then

    mission_failed \
        "Parking Tracker did not become ACTIVE"
fi


echo ""
echo "[OK] Parking Tracker execution started."
echo ""


# ============================================================
# Monitor parking
# ============================================================

START_TIME="$(date +%s)"

LAST_STATUS=""


while true; do

    REACHED="$(
        timeout 2 \
        ros2 topic echo \
        /parking/tracking_reached \
        --once \
        2>/dev/null \
        || true
    )"


    if echo "${REACHED}" \
        | grep -q "data: true"; then

        MISSION_SUCCESS=1

        echo ""
        echo "[OK] Parking goal reached."

        break
    fi


    STATUS_OUTPUT="$(
        timeout 2 \
        ros2 topic echo \
        /parking/tracking_status \
        --once \
        2>/dev/null \
        || true
    )"


    STATUS="$(
        echo "${STATUS_OUTPUT}" \
        | sed -n 's/^data: //p' \
        | head -n 1
    )"


    if [ -n "${STATUS}" ] \
        && [ "${STATUS}" != "${LAST_STATUS}" ]; then

        echo "[TRACKER] ${STATUS}"

        LAST_STATUS="${STATUS}"
    fi


    case "${STATUS}" in

        FAILED_*)

            mission_failed \
                "Parking Tracker failed: ${STATUS}"
            ;;


        STOPPED)

            mission_failed \
                "Parking Tracker unexpectedly STOPPED"
            ;;

    esac


    NOW="$(date +%s)"


    if [ $((NOW - START_TIME)) \
         -ge "${PARKING_TIMEOUT}" ]; then

        mission_failed \
            "Parking exceeded ${PARKING_TIMEOUT} seconds"
    fi


    sleep 1
done


# ============================================================
# [8/8] Success
# ============================================================

section "[8/8] PARKING MISSION SUCCEEDED"


FINAL_STATUS="$(
    timeout 3 \
    ros2 topic echo \
    /parking/tracking_status \
    --once \
    2>/dev/null \
    || true
)"


echo ""
echo "Final tracker status:"
echo "${FINAL_STATUS}"


echo ""
echo "Parking Staging        : OK"
echo "Camera Staging Pose    : OK"
echo "ArUco ID=0             : OK"
echo "ArUco Map Localization : OK"
echo "Hybrid A* Planning     : OK"
echo "Tracker V3.2           : OK"
echo "Automatic Parking      : OK"
echo ""


restore_nav2


echo ""
echo "============================================================"
echo " FINAL TARGET"
echo "============================================================"
echo ""
echo "Map frame:"
echo ""
echo "x   = -8.357680"
echo "y   = -4.166548"
echo "yaw = 1.539811 rad"
echo ""
echo "Mission log:"
echo ""
echo "${MISSION_LOG}"
echo ""
echo "============================================================"


trap - EXIT

exit 0
