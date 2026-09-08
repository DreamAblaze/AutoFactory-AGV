#!/usr/bin/env bash

# ============================================================
# AutoFactory AGV Unified Launcher
#
# FINAL STAGED-RVIZ VERSION
#
# Modes:
#
#   nav2
#       Factory + Robot + Nav2 + RPP RViz
#
#   coverage
#       Factory + Robot + Nav2
#       + Coverage
#       + Coverage RViz
#
#   parking
#       Existing stable Parking Mission
#
#   full
#       Coverage RViz
#           ↓
#       Full Coverage
#           ↓
#       Close Coverage RViz
#           ↓
#       Open RPP RViz
#           ↓
#       Safe Coverage -> Parking handoff
#           ↓
#       Existing Parking Mission
#
# ============================================================

set +u


# ============================================================
# Workspace
# ============================================================

WS="/home/my-ubuntu/autofactory_ws"

DEV_WS="/home/my-ubuntu/dev_ws"


# ============================================================
# Formal scripts
# ============================================================

RPP_SCRIPT="${WS}/src/agv_bringup/launch/start_autofactory_rpp.sh"

COVERAGE_SCRIPT="${WS}/src/agv_coverage/launch/start_autofactory_coverage.sh"

PARKING_SCRIPT="${WS}/src/agv_task_manager/launch/start_parking_mission.sh"


# ============================================================
# RViz configurations
# ============================================================

COVERAGE_RVIZ_CONFIG="${WS}/src/agv_bringup/rviz/autofactory_coverage.rviz"

RPP_RVIZ_CONFIG="${WS}/src/agv_bringup/rviz/autofactory_rpp.rviz"


# ============================================================
# Mode
# ============================================================

MODE="${1:-help}"


# ============================================================
# ROS environment
# ============================================================

source /opt/ros/humble/setup.bash

if [ -f "${DEV_WS}/install/setup.bash" ]; then
    source "${DEV_WS}/install/setup.bash"
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi


COMMON_SETUP="
set +u
cd ${WS}
source /opt/ros/humble/setup.bash
source ${DEV_WS}/install/setup.bash
source ${WS}/install/setup.bash
"


# ============================================================
# Output
# ============================================================

section()
{
    echo ""
    echo "============================================================"
    echo "$1"
    echo "============================================================"
    echo ""
}


# ============================================================
# File check
# ============================================================

require_file()
{
    local FILE="$1"

    if [ ! -f "${FILE}" ]; then

        echo "[ERROR] Required file missing:"
        echo "${FILE}"

        exit 1
    fi
}


require_file "${RPP_SCRIPT}"

require_file "${COVERAGE_SCRIPT}"

require_file "${PARKING_SCRIPT}"

require_file "${COVERAGE_RVIZ_CONFIG}"

require_file "${RPP_RVIZ_CONFIG}"


# ============================================================
# Wait for service
# ============================================================

wait_for_service()
{
    local SERVICE="$1"

    local TIMEOUT_SEC="$2"


    echo "[WAIT] Service: ${SERVICE}"


    local START

    START="$(date +%s)"


    while true; do

        if ros2 service list \
            2>/dev/null \
            | grep -qx "${SERVICE}"; then

            echo "[OK] Service ready: ${SERVICE}"

            return 0
        fi


        NOW="$(date +%s)"


        if [ $((NOW - START)) \
             -ge "${TIMEOUT_SEC}" ]; then

            echo "[ERROR] Service timeout:"
            echo "${SERVICE}"

            return 1
        fi


        sleep 1
    done
}


# ============================================================
# Wait lifecycle active
# ============================================================

wait_lifecycle_active()
{
    local NODE="$1"

    local TIMEOUT_SEC="$2"


    echo "[WAIT] Lifecycle ACTIVE: ${NODE}"


    local START

    START="$(date +%s)"


    while true; do

        STATE="$(
            ros2 lifecycle get \
            "${NODE}" \
            2>/dev/null \
            || true
        )"


        if echo "${STATE}" \
            | grep -q 'active \[3\]'; then

            echo "[OK] ACTIVE: ${NODE}"

            return 0
        fi


        NOW="$(date +%s)"


        if [ $((NOW - START)) \
             -ge "${TIMEOUT_SEC}" ]; then

            echo "[ERROR] Lifecycle timeout:"
            echo "${NODE}"
            echo "${STATE}"

            return 1
        fi


        sleep 1
    done
}


# ============================================================
# Wait for Coverage completion
# ============================================================

wait_coverage_finished()
{
    local TIMEOUT_SEC="$1"


    section "WAIT FOR FULL COVERAGE COMPLETION"


    echo "[WAIT] /coverage_status"

    echo "[WAIT] Expected terminal message:"
    echo "       本次覆盖执行完成。"


    local START

    START="$(date +%s)"


    local LAST_STATUS=""


    while true; do

        OUTPUT="$(
            timeout 4 \
            ros2 topic echo \
            /coverage_status \
            --once \
            2>/dev/null \
            || true
        )"


        STATUS="$(
            echo "${OUTPUT}" \
            | sed -n 's/^data: //p' \
            | head -n 1
        )"


        if [ -n "${STATUS}" ] \
            && [ "${STATUS}" != "${LAST_STATUS}" ]; then

            echo "[COVERAGE] ${STATUS}"

            LAST_STATUS="${STATUS}"
        fi


        # ====================================================
        # Success
        # ====================================================

        if echo "${STATUS}" \
            | grep -q "本次覆盖执行完成"; then

            echo ""

            echo "[OK] Full Coverage completed."

            return 0
        fi


        # ====================================================
        # Failure
        # ====================================================

        if echo "${STATUS}" \
            | grep -q "执行失败"; then

            echo ""

            echo "[ERROR] Coverage reported failure:"

            echo "${STATUS}"

            return 1
        fi


        NOW="$(date +%s)"


        if [ $((NOW - START)) \
             -ge "${TIMEOUT_SEC}" ]; then

            echo ""

            echo "[ERROR] Coverage completion timeout."

            echo "[ERROR] Last status:"
            echo "${STATUS}"

            return 1
        fi


        sleep 1
    done
}


# ============================================================
# Find RViz PIDs using a specific configuration
#
# This avoids:
#
#   pkill rviz2
#
# which would kill every RViz instance.
# ============================================================

find_rviz_pids()
{
    local CONFIG="$1"


    pgrep -af rviz2 \
        2>/dev/null \
        | grep -F "${CONFIG}" \
        | awk '{print $1}' \
        || true
}


# ============================================================
# Close ONLY Coverage RViz
# ============================================================

close_coverage_rviz()
{
    section "SWITCH RVIZ - CLOSE COVERAGE RVIZ"


    echo "[RVIZ] Closing:"
    echo "${COVERAGE_RVIZ_CONFIG}"
    echo ""


    PIDS="$(
        find_rviz_pids \
        "${COVERAGE_RVIZ_CONFIG}"
    )"


    if [ -z "${PIDS}" ]; then

        echo "[WARN] Coverage RViz process was not found."

        echo "[WARN] It may already be closed."

        return 0
    fi


    echo "[RVIZ] PID(s):"
    echo "${PIDS}"


    for PID in ${PIDS}; do

        kill -TERM "${PID}" \
            >/dev/null 2>&1 \
            || true

    done


    # ========================================================
    # Wait until Coverage RViz actually disappears
    # ========================================================

    for i in {1..15}; do

        REMAINING="$(
            find_rviz_pids \
            "${COVERAGE_RVIZ_CONFIG}"
        )"


        if [ -z "${REMAINING}" ]; then

            echo ""

            echo "[OK] Coverage RViz closed."

            sleep 1

            return 0
        fi


        sleep 1
    done


    # ========================================================
    # Last resort: kill only the matching Coverage RViz
    # ========================================================

    REMAINING="$(
        find_rviz_pids \
        "${COVERAGE_RVIZ_CONFIG}"
    )"


    if [ -n "${REMAINING}" ]; then

        echo "[WARN] Coverage RViz did not exit gracefully."

        echo "[WARN] Force closing matching process..."


        for PID in ${REMAINING}; do

            kill -KILL "${PID}" \
                >/dev/null 2>&1 \
                || true

        done
    fi


    sleep 1


    echo "[OK] Coverage RViz terminated."


    return 0
}


# ============================================================
# Open Parking / RPP RViz
# ============================================================

open_rpp_rviz()
{
    section "SWITCH RVIZ - OPEN RPP RVIZ"


    # ========================================================
    # Prevent duplicate RPP RViz
    # ========================================================

    EXISTING="$(
        find_rviz_pids \
        "${RPP_RVIZ_CONFIG}"
    )"


    if [ -n "${EXISTING}" ]; then

        echo "[INFO] RPP RViz already running."

        echo "[INFO] PID(s):"
        echo "${EXISTING}"

        return 0
    fi


    echo "[RVIZ] Opening:"
    echo "${RPP_RVIZ_CONFIG}"
    echo ""


    gnome-terminal \
        --title="T10 AutoFactory RPP Parking RViz" \
        -- bash -lc "
${COMMON_SETUP}

echo '============================================================'
echo ' AutoFactory RPP / Parking RViz'
echo '============================================================'
echo ''
echo 'Config:'
echo '${RPP_RVIZ_CONFIG}'
echo ''

exec rviz2 \
  -d '${RPP_RVIZ_CONFIG}' \
  --ros-args \
  -p use_sim_time:=true
" >/dev/null 2>&1 &


    # ========================================================
    # Wait for RViz process
    # ========================================================

    for i in {1..20}; do

        PIDS="$(
            find_rviz_pids \
            "${RPP_RVIZ_CONFIG}"
        )"


        if [ -n "${PIDS}" ]; then

            echo "[OK] RPP RViz started."

            echo "[RVIZ] PID(s):"
            echo "${PIDS}"

            sleep 2

            return 0
        fi


        sleep 1
    done


    echo "[ERROR] RPP RViz did not start."

    return 1
}


# ============================================================
# Read current RPP rotate_to_heading
# ============================================================

get_rotate_to_heading()
{
    OUT="$(
        ros2 param get \
        /controller_server \
        FollowPath.use_rotate_to_heading \
        2>/dev/null \
        || true
    )"


    if echo "${OUT}" \
        | grep -qi 'true'; then

        echo "true"

        return 0
    fi


    if echo "${OUT}" \
        | grep -qi 'false'; then

        echo "false"

        return 0
    fi


    return 1
}


# ============================================================
# Full-mode runtime parameter state
# ============================================================

RPP_PARAMETER_CHANGED=0

RPP_ORIGINAL_ROTATE_TO_HEADING=""


# ============================================================
# Restore original RPP value
# ============================================================

restore_rpp_handoff()
{
    if [ "${RPP_PARAMETER_CHANGED}" -ne 1 ]; then

        return 0
    fi


    if ! ros2 node list \
        2>/dev/null \
        | grep -qx '/controller_server'; then

        RPP_PARAMETER_CHANGED=0

        return 0
    fi


    echo ""

    echo "[RESTORE] FollowPath.use_rotate_to_heading"

    echo "          -> ${RPP_ORIGINAL_ROTATE_TO_HEADING}"


    ros2 param set \
        /controller_server \
        FollowPath.use_rotate_to_heading \
        "${RPP_ORIGINAL_ROTATE_TO_HEADING}" \
        >/dev/null 2>&1 \
        || true


    RPP_PARAMETER_CHANGED=0
}


trap restore_rpp_handoff EXIT


# ============================================================
# Coverage -> Parking Nav2 handoff
#
# This is the sequence already manually verified:
#
#   use_rotate_to_heading=true
#       ↓
#   clear local costmap
#       ↓
#   clear global costmap
#       ↓
#   let LaserScan repopulate
#       ↓
#   start original Parking Mission
#
# ============================================================

prepare_parking_handoff()
{
    section "COVERAGE -> PARKING NAV2 HANDOFF"


    wait_lifecycle_active \
        /controller_server \
        30 \
        || return 1


    wait_lifecycle_active \
        /velocity_smoother \
        30 \
        || return 1


    # ========================================================
    # Save current value
    # ========================================================

    RPP_ORIGINAL_ROTATE_TO_HEADING="$(
        get_rotate_to_heading
    )" || {

        echo "[ERROR] Cannot read:"
        echo "FollowPath.use_rotate_to_heading"

        return 1
    }


    echo "[HANDOFF] Original:"
    echo "FollowPath.use_rotate_to_heading=${RPP_ORIGINAL_ROTATE_TO_HEADING}"


    # ========================================================
    # Enable only for the full-mode parking handoff
    # ========================================================

    if [ "${RPP_ORIGINAL_ROTATE_TO_HEADING}" != "true" ]; then

        echo ""

        echo "[HANDOFF] Setting:"
        echo "FollowPath.use_rotate_to_heading=true"


        RESULT="$(
            ros2 param set \
            /controller_server \
            FollowPath.use_rotate_to_heading \
            true \
            2>&1 \
            || true
        )"


        echo "${RESULT}"


        if ! echo "${RESULT}" \
            | grep -q 'Set parameter successful'; then

            echo "[ERROR] Failed to set RPP runtime parameter."

            return 1
        fi


        RPP_PARAMETER_CHANGED=1
    fi


    CURRENT="$(
        get_rotate_to_heading \
        || true
    )"


    if [ "${CURRENT}" != "true" ]; then

        echo "[ERROR] Runtime parameter verification failed."

        return 1
    fi


    echo "[OK] FollowPath.use_rotate_to_heading=true"


    # ========================================================
    # Clear local costmap
    # ========================================================

    echo ""

    echo "[HANDOFF] Clearing local costmap..."


    wait_for_service \
        /local_costmap/clear_entirely_local_costmap \
        20 \
        || return 1


    timeout 15 \
        ros2 service call \
        /local_costmap/clear_entirely_local_costmap \
        nav2_msgs/srv/ClearEntireCostmap \
        "{}" \
        >/dev/null \
        || {

            echo "[ERROR] Local costmap clear failed."

            return 1
        }


    echo "[OK] Local costmap cleared."


    # ========================================================
    # Clear global costmap
    # ========================================================

    echo ""

    echo "[HANDOFF] Clearing global costmap..."


    wait_for_service \
        /global_costmap/clear_entirely_global_costmap \
        20 \
        || return 1


    timeout 15 \
        ros2 service call \
        /global_costmap/clear_entirely_global_costmap \
        nav2_msgs/srv/ClearEntireCostmap \
        "{}" \
        >/dev/null \
        || {

            echo "[ERROR] Global costmap clear failed."

            return 1
        }


    echo "[OK] Global costmap cleared."


    # ========================================================
    # Allow LaserScan to refill local costmap
    # ========================================================

    echo ""

    echo "[HANDOFF] Waiting for costmap refresh..."


    sleep 3


    echo "[OK] Parking handoff ready."


    return 0
}


# ============================================================
# MODE 1
# ============================================================

run_nav2()
{
    section "MODE 1 - NAV2"


    cd "$(dirname "${RPP_SCRIPT}")"


    exec "${RPP_SCRIPT}"
}


# ============================================================
# MODE 2
# ============================================================

run_coverage()
{
    section "MODE 2 - FULL COVERAGE"


    cd "$(dirname "${COVERAGE_SCRIPT}")"


    exec "${COVERAGE_SCRIPT}" execute
}


# ============================================================
# MODE 3
# ============================================================

run_parking()
{
    section "MODE 3 - AUTONOMOUS PARKING"


    cd "$(dirname "${PARKING_SCRIPT}")"


    exec "${PARKING_SCRIPT}"
}


# ============================================================
# MODE 4
# ============================================================

run_full()
{
    section "MODE 4 - COMPLETE AUTOFACTORY MISSION"


    echo "PHASE A"
    echo "  Coverage RViz"
    echo "      ↓"
    echo "  Full Coverage"
    echo ""
    echo "PHASE B"
    echo "  Close Coverage RViz"
    echo "      ↓"
    echo "  Open RPP RViz"
    echo "      ↓"
    echo "  Parking Staging"
    echo "      ↓"
    echo "  ArUco"
    echo "      ↓"
    echo "  Hybrid A*"
    echo "      ↓"
    echo "  Parking Tracker"
    echo ""


    # ========================================================
    # PHASE A
    # ========================================================

    section "PHASE A - START COVERAGE SYSTEM"


    cd "$(dirname "${COVERAGE_SCRIPT}")"


    "${COVERAGE_SCRIPT}" execute


    # ========================================================
    # Wait until Coverage node/service is alive
    # ========================================================

    wait_for_service \
        /execute_coverage \
        120 \
        || {

            section "MISSION FAILED"

            echo "Coverage node did not start."

            return 1
        }


    # ========================================================
    # Do NOT open RPP RViz here.
    #
    # During Coverage:
    #
    #   ONLY autofactory_coverage.rviz is used.
    # ========================================================

    echo ""
    echo "[RVIZ PHASE A]"
    echo "Coverage RViz is active."
    echo ""
    echo "RPP RViz will NOT start until Coverage finishes."


    # ========================================================
    # Wait for all Coverage batches
    # ========================================================

    wait_coverage_finished \
        7200 \
        || {

            section "MISSION FAILED"

            echo "Coverage did not complete."

            echo "Parking phase will NOT start."

            return 1
        }


    # ========================================================
    # Let final NavigateThroughPoses completely settle
    # ========================================================

    echo ""

    echo "[TRANSITION] Coverage completed."

    echo "[TRANSITION] Waiting for final Nav2 action to settle..."


    sleep 3


    # ========================================================
    # PHASE SWITCH:
    #
    # 1. Close Coverage RViz
    # 2. Open RPP RViz
    #
    # ========================================================

    close_coverage_rviz \
        || {

            section "MISSION FAILED"

            echo "Could not close Coverage RViz."

            return 1
        }


    open_rpp_rviz \
        || {

            section "MISSION FAILED"

            echo "Could not open RPP RViz."

            return 1
        }


    # ========================================================
    # Prepare Nav2 for Parking Staging
    #
    # This is done AFTER RPP RViz is visible
    # and BEFORE Parking Staging starts.
    # ========================================================

    prepare_parking_handoff \
        || {

            section "MISSION FAILED"

            echo "Coverage -> Parking Nav2 handoff failed."

            return 1
        }


    # ========================================================
    # PHASE B
    # ========================================================

    section "PHASE B - START EXISTING PARKING MISSION"


    echo "[RVIZ PHASE B]"
    echo "RPP RViz is active."
    echo ""

    echo "The original parking chain now starts:"
    echo ""
    echo "Parking Staging"
    echo "    ↓"
    echo "ArUco"
    echo "    ↓"
    echo "Hybrid A*"
    echo "    ↓"
    echo "Parking Tracker"
    echo ""


    cd "$(dirname "${PARKING_SCRIPT}")"


    "${PARKING_SCRIPT}"


    PARKING_RESULT=$?


    # ========================================================
    # Restore original RPP parameter
    # ========================================================

    restore_rpp_handoff


    # ========================================================
    # Final result
    # ========================================================

    if [ "${PARKING_RESULT}" -ne 0 ]; then

        section "COMPLETE MISSION FAILED"


        echo "Parking Mission returned:"
        echo "${PARKING_RESULT}"


        return "${PARKING_RESULT}"
    fi


    section "COMPLETE AUTOFACTORY MISSION SUCCEEDED"


    echo "Full Coverage       : OK"
    echo "Coverage RViz       : CLOSED"
    echo "RPP RViz            : RUNNING"
    echo "Parking Handoff     : OK"
    echo "Parking Staging     : OK"
    echo "ArUco               : OK"
    echo "Hybrid A*           : OK"
    echo "Parking Tracker     : OK"
    echo "Automatic Parking   : OK"
    echo ""


    return 0
}


# ============================================================
# Help
# ============================================================

show_help()
{
    echo ""
    echo "AutoFactory AGV"
    echo ""
    echo "Usage:"
    echo ""
    echo "  ./start_autofactory.sh nav2"
    echo "  ./start_autofactory.sh coverage"
    echo "  ./start_autofactory.sh parking"
    echo "  ./start_autofactory.sh full"
    echo ""
    echo "Full mode RViz flow:"
    echo ""
    echo "  autofactory_coverage.rviz"
    echo "          ↓"
    echo "  Coverage complete"
    echo "          ↓"
    echo "  close Coverage RViz"
    echo "          ↓"
    echo "  autofactory_rpp.rviz"
    echo "          ↓"
    echo "  Parking Mission"
    echo ""
}


# ============================================================
# Main
# ============================================================

case "${MODE}" in


    nav2)

        run_nav2
        ;;


    coverage)

        run_coverage
        ;;


    parking)

        run_parking
        ;;


    full)

        run_full
        ;;


    help|-h|--help)

        show_help
        ;;


    *)

        echo "[ERROR] Unknown mode:"
        echo "${MODE}"

        show_help

        exit 1
        ;;


esac
