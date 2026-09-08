#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Precise parking staging navigator for AutoFactory AGV.

Workflow:

    PLAN
      ComputePathToPose
      planner_id = StagingGrid

        -> verify path endpoint

    FOLLOW
      FollowPath
      controller_id = FollowPath
      goal_checker_id = staging_goal_checker

        -> verify XY

    ALIGN
      Spin

        -> verify yaw

    SUCCEEDED
"""

from __future__ import annotations

import math

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from nav2_msgs.action import (
    ComputePathToPose,
    FollowPath,
    Spin,
)

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def quaternion_from_yaw(yaw: float):
    """Quaternion with roll=0 and pitch=0."""

    return (
        0.0,
        0.0,
        math.sin(yaw * 0.5),
        math.cos(yaw * 0.5),
    )


def yaw_from_quaternion(q) -> float:
    """Extract planar yaw."""

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


def point_in_robot_frame(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    point_x: float,
    point_y: float,
):
    """Convert a map point to planar robot coordinates."""

    dx = point_x - robot_x
    dy = point_y - robot_y

    forward = (
        math.cos(robot_yaw) * dx
        + math.sin(robot_yaw) * dy
    )

    left = (
        -math.sin(robot_yaw) * dx
        + math.cos(robot_yaw) * dy
    )

    return forward, left


class ParkingStagingNavigator(Node):

    def __init__(self):

        super().__init__(
            "parking_staging_navigator"
        )

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter(
            "map_frame",
            "map",
        )

        self.declare_parameter(
            "base_frame",
            "base_footprint",
        )

        self.declare_parameter(
            "compute_path_action",
            "/compute_path_to_pose",
        )

        self.declare_parameter(
            "follow_path_action",
            "/follow_path",
        )

        self.declare_parameter(
            "spin_action",
            "/spin",
        )

        self.declare_parameter(
            "planner_id",
            "StagingGrid",
        )

        self.declare_parameter(
            "controller_id",
            "FollowPath",
        )

        self.declare_parameter(
            "goal_checker_id",
            "staging_goal_checker",
        )

        self.declare_parameter(
            "action_server_wait_timeout_sec",
            5.0,
        )

        self.declare_parameter(
            "spin_time_allowance_sec",
            10.0,
        )

        # -----------------------------------------------------
        # Staging pose
        # -----------------------------------------------------

        self.declare_parameter(
            "staging_x",
            -6.0,
        )

        self.declare_parameter(
            "staging_y",
            -4.5,
        )

        self.declare_parameter(
            "staging_yaw",
            3.1,
        )

        # -----------------------------------------------------
        # Marker
        # -----------------------------------------------------

        self.declare_parameter(
            "marker_x",
            -7.572620,
        )

        self.declare_parameter(
            "marker_y",
            -4.473930,
        )

        self.declare_parameter(
            "marker_yaw",
            1.570801,
        )

        # -----------------------------------------------------
        # Parking goal
        # -----------------------------------------------------

        self.declare_parameter(
            "parking_goal_x",
            -8.372620,
        )

        self.declare_parameter(
            "parking_goal_y",
            -4.473930,
        )

        self.declare_parameter(
            "parking_goal_yaw",
            0.0,
        )

        # -----------------------------------------------------
        # Tolerances
        # -----------------------------------------------------

        self.declare_parameter(
            "max_path_endpoint_error",
            0.06,
        )

        self.declare_parameter(
            "arrival_xy_tolerance",
            0.08,
        )

        self.declare_parameter(
            "final_yaw_tolerance",
            0.087266,
        )

        self.declare_parameter(
            "max_marker_lateral_offset",
            0.30,
        )

        # =====================================================
        # Read parameters
        # =====================================================

        self.map_frame = str(
            self.get_parameter(
                "map_frame"
            ).value
        )

        self.base_frame = str(
            self.get_parameter(
                "base_frame"
            ).value
        )

        compute_path_action = str(
            self.get_parameter(
                "compute_path_action"
            ).value
        )

        follow_path_action = str(
            self.get_parameter(
                "follow_path_action"
            ).value
        )

        spin_action = str(
            self.get_parameter(
                "spin_action"
            ).value
        )

        self.planner_id = str(
            self.get_parameter(
                "planner_id"
            ).value
        )

        self.controller_id = str(
            self.get_parameter(
                "controller_id"
            ).value
        )

        self.goal_checker_id = str(
            self.get_parameter(
                "goal_checker_id"
            ).value
        )

        self.server_timeout = float(
            self.get_parameter(
                "action_server_wait_timeout_sec"
            ).value
        )

        self.spin_time_allowance = float(
            self.get_parameter(
                "spin_time_allowance_sec"
            ).value
        )

        self.staging_x = float(
            self.get_parameter(
                "staging_x"
            ).value
        )

        self.staging_y = float(
            self.get_parameter(
                "staging_y"
            ).value
        )

        self.staging_yaw = float(
            self.get_parameter(
                "staging_yaw"
            ).value
        )

        self.marker_x = float(
            self.get_parameter(
                "marker_x"
            ).value
        )

        self.marker_y = float(
            self.get_parameter(
                "marker_y"
            ).value
        )

        self.marker_yaw = float(
            self.get_parameter(
                "marker_yaw"
            ).value
        )

        self.parking_goal_x = float(
            self.get_parameter(
                "parking_goal_x"
            ).value
        )

        self.parking_goal_y = float(
            self.get_parameter(
                "parking_goal_y"
            ).value
        )

        self.parking_goal_yaw = float(
            self.get_parameter(
                "parking_goal_yaw"
            ).value
        )

        self.max_path_endpoint_error = float(
            self.get_parameter(
                "max_path_endpoint_error"
            ).value
        )

        self.arrival_xy_tolerance = float(
            self.get_parameter(
                "arrival_xy_tolerance"
            ).value
        )

        self.final_yaw_tolerance = float(
            self.get_parameter(
                "final_yaw_tolerance"
            ).value
        )

        self.max_marker_lateral_offset = float(
            self.get_parameter(
                "max_marker_lateral_offset"
            ).value
        )

        # =====================================================
        # QoS
        # =====================================================

        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # =====================================================
        # Publishers
        # =====================================================

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/parking_staging_pose",
            state_qos,
        )

        self.path_pub = self.create_publisher(
            Path,
            "/parking_staging_path",
            state_qos,
        )

        self.path_end_pub = self.create_publisher(
            PoseStamped,
            "/parking_staging_path_end",
            state_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            "/parking_staging_status",
            state_qos,
        )

        self.reached_pub = self.create_publisher(
            Bool,
            "/parking_staging_reached",
            state_qos,
        )

        # =====================================================
        # Services
        # =====================================================

        self.plan_service = self.create_service(
            Trigger,
            "/plan_parking_staging",
            self.plan_service_callback,
        )

        self.go_service = self.create_service(
            Trigger,
            "/go_to_parking_staging",
            self.go_service_callback,
        )

        self.cancel_service = self.create_service(
            Trigger,
            "/cancel_parking_staging",
            self.cancel_service_callback,
        )

        # =====================================================
        # Nav2 action clients
        # =====================================================

        self.plan_client = ActionClient(
            self,
            ComputePathToPose,
            compute_path_action,
        )

        self.follow_client = ActionClient(
            self,
            FollowPath,
            follow_path_action,
        )

        self.spin_client = ActionClient(
            self,
            Spin,
            spin_action,
        )

        self.plan_goal_handle = None
        self.follow_goal_handle = None
        self.spin_goal_handle = None

        self.busy = False
        self.execute_after_plan = False

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # =====================================================
        # Periodic Staging Pose
        # =====================================================

        self.pose_timer = self.create_timer(
            2.0,
            self.publish_staging_pose,
        )

        self.publish_staging_pose()
        self.publish_status("IDLE")
        self.publish_reached(False)

        self.log_geometry()

        self.get_logger().info(
            "Precise parking staging navigator ready."
        )

    # =========================================================
    # Geometry
    # =========================================================

    def log_geometry(self):

        marker_forward, marker_left = (
            point_in_robot_frame(
                self.staging_x,
                self.staging_y,
                self.staging_yaw,
                self.marker_x,
                self.marker_y,
            )
        )

        goal_forward, goal_left = (
            point_in_robot_frame(
                self.staging_x,
                self.staging_y,
                self.staging_yaw,
                self.parking_goal_x,
                self.parking_goal_y,
            )
        )

        marker_distance = math.hypot(
            marker_forward,
            marker_left,
        )

        marker_bearing = math.degrees(
            math.atan2(
                marker_left,
                marker_forward,
            )
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Parking staging geometry:"
        )

        self.get_logger().info(
            f"Staging: x={self.staging_x:.6f}, "
            f"y={self.staging_y:.6f}, "
            f"yaw={self.staging_yaw:.6f}"
        )

        self.get_logger().info(
            f"Marker: forward={marker_forward:.3f} m, "
            f"left={marker_left:.3f} m, "
            f"distance={marker_distance:.3f} m, "
            f"bearing={marker_bearing:.2f} deg"
        )

        self.get_logger().info(
            f"Parking: forward={goal_forward:.3f} m, "
            f"left={goal_left:.3f} m"
        )

        self.get_logger().info(
            "========================================"
        )

    # =========================================================
    # Pose helpers
    # =========================================================

    def make_staging_pose(self):

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = self.map_frame

        msg.pose.position.x = self.staging_x
        msg.pose.position.y = self.staging_y
        msg.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_yaw(
            self.staging_yaw
        )

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def get_robot_pose(self):

        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
        )

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        yaw = yaw_from_quaternion(
            transform.transform.rotation
        )

        return x, y, yaw

    def publish_staging_pose(self):

        self.pose_pub.publish(
            self.make_staging_pose()
        )

    def publish_status(self, text):

        msg = String()
        msg.data = text

        self.status_pub.publish(msg)

        self.get_logger().info(
            f"STATUS: {text}"
        )

    def publish_reached(self, value):

        msg = Bool()
        msg.data = bool(value)

        self.reached_pub.publish(msg)

    def fail_task(self, status):

        self.publish_status(status)
        self.publish_reached(False)

        self.busy = False
        self.execute_after_plan = False

    # =========================================================
    # Services
    # =========================================================

    def plan_service_callback(
        self,
        request,
        response,
    ):

        del request

        if self.busy:

            response.success = False
            response.message = (
                "Parking staging task is busy."
            )

            return response

        self.execute_after_plan = False

        ok = self.start_planning()

        response.success = ok

        response.message = (
            "Staging planning requested."
            if ok
            else
            "Unable to start staging planning."
        )

        return response

    def go_service_callback(
        self,
        request,
        response,
    ):

        del request

        if self.busy:

            response.success = False
            response.message = (
                "Parking staging task is busy."
            )

            return response

        self.execute_after_plan = True

        ok = self.start_planning()

        response.success = ok

        response.message = (
            "Staging plan-and-execute requested."
            if ok
            else
            "Unable to start staging task."
        )

        return response

    # =========================================================
    # Planning
    # =========================================================

    def start_planning(self):

        if not self.plan_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):

            self.fail_task(
                "PLANNER_SERVER_UNAVAILABLE"
            )

            return False

        self.busy = True

        self.publish_reached(False)

        self.publish_status(
            "PLANNING"
        )

        goal = ComputePathToPose.Goal()

        goal.goal = self.make_staging_pose()

        goal.planner_id = self.planner_id

        goal.use_start = False

        self.get_logger().info(
            "Planning precise staging path:"
        )

        self.get_logger().info(
            f"target x={self.staging_x:.6f}, "
            f"y={self.staging_y:.6f}, "
            f"planner={self.planner_id}"
        )

        future = self.plan_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.plan_goal_response_callback
        )

        return True

    def plan_goal_response_callback(
        self,
        future,
    ):

        try:
            goal_handle = future.result()

        except Exception as exc:

            self.get_logger().error(
                f"Planning goal exception: {exc}"
            )

            self.fail_task(
                "PLAN_GOAL_SEND_FAILED"
            )

            return

        if not goal_handle.accepted:

            self.fail_task(
                "PLAN_GOAL_REJECTED"
            )

            return

        self.plan_goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.plan_result_callback
        )

    def plan_result_callback(
        self,
        future,
    ):

        try:
            wrapped = future.result()

        except Exception as exc:

            self.plan_goal_handle = None

            self.get_logger().error(
                f"Planning result exception: {exc}"
            )

            self.fail_task(
                "PLAN_RESULT_ERROR"
            )

            return

        self.plan_goal_handle = None

        if (
            wrapped.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.fail_task(
                f"PLAN_FAILED_STATUS_{wrapped.status}"
            )

            return

        path = wrapped.result.path

        if len(path.poses) == 0:

            self.fail_task(
                "PLAN_EMPTY"
            )

            return

        self.path_pub.publish(path)

        path_end = path.poses[-1]

        self.path_end_pub.publish(
            path_end
        )

        end_x = path_end.pose.position.x
        end_y = path_end.pose.position.y

        endpoint_error = math.hypot(
            end_x - self.staging_x,
            end_y - self.staging_y,
        )

        self.get_logger().info(
            "----------------------------------------"
        )

        self.get_logger().info(
            "PLANNED PATH ENDPOINT:"
        )

        self.get_logger().info(
            f"x={end_x:.6f}, "
            f"y={end_y:.6f}"
        )

        self.get_logger().info(
            "REQUESTED STAGING:"
        )

        self.get_logger().info(
            f"x={self.staging_x:.6f}, "
            f"y={self.staging_y:.6f}"
        )

        self.get_logger().info(
            f"ENDPOINT ERROR = "
            f"{endpoint_error:.4f} m"
        )

        self.get_logger().info(
            "----------------------------------------"
        )

        if (
            endpoint_error
            > self.max_path_endpoint_error
        ):

            self.get_logger().error(
                "Planner changed the staging endpoint. "
                "Execution is refused."
            )

            self.fail_task(
                "PATH_ENDPOINT_MISMATCH"
            )

            return

        if not self.execute_after_plan:

            self.publish_status(
                "PLAN_SUCCEEDED"
            )

            self.busy = False

            return

        self.start_following(path)

    # =========================================================
    # Follow Path
    # =========================================================

    def start_following(self, path):

        if not self.follow_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):

            self.fail_task(
                "FOLLOW_SERVER_UNAVAILABLE"
            )

            return

        goal = FollowPath.Goal()

        goal.path = path

        goal.controller_id = (
            self.controller_id
        )

        goal.goal_checker_id = (
            self.goal_checker_id
        )

        self.publish_status(
            "FOLLOWING_PATH"
        )

        future = (
            self.follow_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.follow_goal_response_callback
        )

    def follow_goal_response_callback(
        self,
        future,
    ):

        try:
            goal_handle = future.result()

        except Exception as exc:

            self.get_logger().error(
                f"FollowPath send exception: {exc}"
            )

            self.fail_task(
                "FOLLOW_GOAL_SEND_FAILED"
            )

            return

        if not goal_handle.accepted:

            self.fail_task(
                "FOLLOW_GOAL_REJECTED"
            )

            return

        self.follow_goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.follow_result_callback
        )

    def follow_result_callback(
        self,
        future,
    ):

        try:
            wrapped = future.result()

        except Exception as exc:

            self.follow_goal_handle = None

            self.get_logger().error(
                f"FollowPath result exception: {exc}"
            )

            self.fail_task(
                "FOLLOW_RESULT_ERROR"
            )

            return

        self.follow_goal_handle = None

        if (
            wrapped.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.fail_task(
                f"FOLLOW_FAILED_STATUS_{wrapped.status}"
            )

            return

        self.verify_position_before_spin()

    # =========================================================
    # XY verification + Spin
    # =========================================================

    def verify_position_before_spin(self):

        try:
            x, y, yaw = self.get_robot_pose()

        except TransformException as exc:

            self.get_logger().error(
                f"TF unavailable: {exc}"
            )

            self.fail_task(
                "POSITION_TF_ERROR"
            )

            return

        xy_error = math.hypot(
            x - self.staging_x,
            y - self.staging_y,
        )

        self.get_logger().info(
            "Robot after FollowPath:"
        )

        self.get_logger().info(
            f"x={x:.3f}, "
            f"y={y:.3f}, "
            f"yaw={math.degrees(yaw):.2f} deg"
        )

        self.get_logger().info(
            f"XY error={xy_error:.3f} m"
        )

        if (
            xy_error
            > self.arrival_xy_tolerance
        ):

            self.fail_task(
                "POSITION_OUTSIDE_TOLERANCE"
            )

            return

        yaw_error = normalize_angle(
            self.staging_yaw - yaw
        )

        if (
            abs(yaw_error)
            <= self.final_yaw_tolerance
        ):

            self.verify_final_pose()

            return

        self.start_spin(yaw_error)

    # =========================================================
    # Spin
    # =========================================================

    def start_spin(self, yaw_error):

        if not self.spin_client.wait_for_server(
            timeout_sec=self.server_timeout
        ):

            self.fail_task(
                "SPIN_SERVER_UNAVAILABLE"
            )

            return

        goal = Spin.Goal()

        # Relative rotation
        goal.target_yaw = float(
            yaw_error
        )

        allowance = max(
            1.0,
            self.spin_time_allowance,
        )

        sec = int(allowance)

        goal.time_allowance.sec = sec

        goal.time_allowance.nanosec = int(
            (allowance - sec) * 1.0e9
        )

        self.publish_status(
            "ALIGNING_YAW"
        )

        self.get_logger().info(
            "Spin correction: "
            f"{math.degrees(yaw_error):.2f} deg"
        )

        future = (
            self.spin_client.send_goal_async(
                goal
            )
        )

        future.add_done_callback(
            self.spin_goal_response_callback
        )

    def spin_goal_response_callback(
        self,
        future,
    ):

        try:
            goal_handle = future.result()

        except Exception as exc:

            self.get_logger().error(
                f"Spin send exception: {exc}"
            )

            self.fail_task(
                "SPIN_GOAL_SEND_FAILED"
            )

            return

        if not goal_handle.accepted:

            self.fail_task(
                "SPIN_GOAL_REJECTED"
            )

            return

        self.spin_goal_handle = goal_handle

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.spin_result_callback
        )

    def spin_result_callback(
        self,
        future,
    ):

        try:
            wrapped = future.result()

        except Exception as exc:

            self.spin_goal_handle = None

            self.get_logger().error(
                f"Spin result exception: {exc}"
            )

            self.fail_task(
                "SPIN_RESULT_ERROR"
            )

            return

        self.spin_goal_handle = None

        if (
            wrapped.status
            != GoalStatus.STATUS_SUCCEEDED
        ):

            self.fail_task(
                f"SPIN_FAILED_STATUS_{wrapped.status}"
            )

            return

        self.verify_final_pose()

    # =========================================================
    # Final verification
    # =========================================================

    def verify_final_pose(self):

        try:
            x, y, yaw = self.get_robot_pose()

        except TransformException as exc:

            self.get_logger().error(
                f"Final TF unavailable: {exc}"
            )

            self.fail_task(
                "FINAL_TF_ERROR"
            )

            return

        xy_error = math.hypot(
            x - self.staging_x,
            y - self.staging_y,
        )

        yaw_error = abs(
            normalize_angle(
                yaw - self.staging_yaw
            )
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "FINAL PARKING STAGING POSE:"
        )

        self.get_logger().info(
            f"x={x:.3f}, "
            f"y={y:.3f}, "
            f"yaw={yaw:.3f} rad "
            f"({math.degrees(yaw):.2f} deg)"
        )

        self.get_logger().info(
            f"XY error={xy_error:.3f} m"
        )

        self.get_logger().info(
            f"Yaw error="
            f"{math.degrees(yaw_error):.2f} deg"
        )

        self.get_logger().info(
            "========================================"
        )

        if (
            xy_error
            <= self.arrival_xy_tolerance
            and
            yaw_error
            <= self.final_yaw_tolerance
        ):

            self.publish_status(
                "SUCCEEDED"
            )

            self.publish_reached(True)

            self.busy = False
            self.execute_after_plan = False

            self.get_logger().info(
                "Parking staging completed successfully."
            )

        else:

            self.fail_task(
                "FINAL_POSE_OUTSIDE_TOLERANCE"
            )

    # =========================================================
    # Cancel
    # =========================================================

    def cancel_service_callback(
        self,
        request,
        response,
    ):

        del request

        canceled = False

        for handle in (
            self.plan_goal_handle,
            self.follow_goal_handle,
            self.spin_goal_handle,
        ):

            if handle is not None:

                handle.cancel_goal_async()

                canceled = True

        if not canceled:

            response.success = False

            response.message = (
                "No active parking staging action."
            )

            return response

        self.publish_status(
            "CANCEL_REQUESTED"
        )

        response.success = True

        response.message = (
            "Cancel request sent."
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = ParkingStagingNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
