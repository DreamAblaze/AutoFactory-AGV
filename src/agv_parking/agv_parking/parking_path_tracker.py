#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AutoFactory Parking Path Tracker V3.2

目的：
    执行已经规划好的 Hybrid A* 几何路径。

与旧V2最大的区别：
    1. 不订阅 /parking/hybrid_directions
    2. 不等待 directions
    3. 收到 /parking/hybrid_path 后立即 PATH_READY
    4. 将连续相同XY、只改变Yaw的Hybrid A*节点压缩成一个节点
    5. 中途不会逐个执行Hybrid A*的10度spin节点
    6. 根据路径几何自动判断前进/倒车
    7. 最终XY到达后再单独校正最终Yaw
"""

from __future__ import annotations

import math

import rclpy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time

from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformException, TransformListener


TRACKER_VERSION = "V3.2_SAFE_GEOMETRY"


# ============================================================
# Utilities
# ============================================================

def normalize_angle(angle: float) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def yaw_from_quaternion(q) -> float:
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


def clamp(value: float, low: float, high: float) -> float:
    return max(
        low,
        min(high, value),
    )


# ============================================================
# Tracker
# ============================================================

class ParkingPathTracker(Node):

    def __init__(self):

        super().__init__(
            "parking_path_tracker"
        )

        # ====================================================
        # Parameters
        # ====================================================

        self.declare_parameter(
            "tracker_version",
            TRACKER_VERSION,
        )

        self.declare_parameter(
            "map_frame",
            "map",
        )

        self.declare_parameter(
            "base_frame",
            "base_footprint",
        )

        self.declare_parameter(
            "path_topic",
            "/parking/hybrid_path",
        )

        self.declare_parameter(
            "cmd_vel_topic",
            "/cmd_vel",
        )

        self.declare_parameter(
            "control_frequency",
            20.0,
        )

        # 连续路径点XY距离小于此值时，
        # 认为是同一个位置上的姿态变化。
        self.declare_parameter(
            "duplicate_xy_tolerance",
            0.025,
        )

        self.declare_parameter(
            "lookahead_distance",
            0.08,
        )

        self.declare_parameter(
            "waypoint_tolerance",
            0.07,
        )

        self.declare_parameter(
            "pass_lateral_tolerance",
            0.18,
        )

        self.declare_parameter(
            "final_xy_tolerance",
            0.07,
        )

        self.declare_parameter(
            "final_yaw_tolerance_deg",
            3.0,
        )

        self.declare_parameter(
            "max_linear_speed",
            0.08,
        )

        self.declare_parameter(
            "min_linear_speed",
            0.025,
        )

        self.declare_parameter(
            "max_angular_speed",
            0.40,
        )

        self.declare_parameter(
            "min_angular_speed",
            0.07,
        )

        self.declare_parameter(
            "linear_gain",
            0.65,
        )

        self.declare_parameter(
            "heading_gain",
            2.00,
        )

        self.declare_parameter(
            "final_spin_gain",
            1.30,
        )

        self.declare_parameter(
            "rotate_before_drive_deg",
            30.0,
        )

        self.declare_parameter(
            "execution_timeout_sec",
            600.0,
        )

        # 防止再次无限原地旋转
        self.declare_parameter(
            "alignment_timeout_sec",
            25.0,
        )

        # ====================================================
        # Read parameters
        # ====================================================

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

        self.path_topic = str(
            self.get_parameter(
                "path_topic"
            ).value
        )

        self.cmd_vel_topic = str(
            self.get_parameter(
                "cmd_vel_topic"
            ).value
        )

        self.control_frequency = float(
            self.get_parameter(
                "control_frequency"
            ).value
        )

        self.duplicate_xy_tolerance = float(
            self.get_parameter(
                "duplicate_xy_tolerance"
            ).value
        )

        self.lookahead_distance = float(
            self.get_parameter(
                "lookahead_distance"
            ).value
        )

        self.waypoint_tolerance = float(
            self.get_parameter(
                "waypoint_tolerance"
            ).value
        )

        self.pass_lateral_tolerance = float(
            self.get_parameter(
                "pass_lateral_tolerance"
            ).value
        )

        self.final_xy_tolerance = float(
            self.get_parameter(
                "final_xy_tolerance"
            ).value
        )

        self.final_yaw_tolerance = math.radians(
            float(
                self.get_parameter(
                    "final_yaw_tolerance_deg"
                ).value
            )
        )

        self.max_linear_speed = float(
            self.get_parameter(
                "max_linear_speed"
            ).value
        )

        self.min_linear_speed = float(
            self.get_parameter(
                "min_linear_speed"
            ).value
        )

        self.max_angular_speed = float(
            self.get_parameter(
                "max_angular_speed"
            ).value
        )

        self.min_angular_speed = float(
            self.get_parameter(
                "min_angular_speed"
            ).value
        )

        self.linear_gain = float(
            self.get_parameter(
                "linear_gain"
            ).value
        )

        self.heading_gain = float(
            self.get_parameter(
                "heading_gain"
            ).value
        )

        self.final_spin_gain = float(
            self.get_parameter(
                "final_spin_gain"
            ).value
        )

        self.rotate_before_drive = math.radians(
            float(
                self.get_parameter(
                    "rotate_before_drive_deg"
                ).value
            )
        )

        self.execution_timeout_sec = float(
            self.get_parameter(
                "execution_timeout_sec"
            ).value
        )

        self.alignment_timeout_sec = float(
            self.get_parameter(
                "alignment_timeout_sec"
            ).value
        )

        # ====================================================
        # Runtime
        # ====================================================

        self.raw_path = []

        self.path = []

        # directions[i]表示：
        # path[i-1] -> path[i]
        #
        # +1：前进
        # -1：倒车
        self.directions = []

        self.path_ready = False

        self.active = False

        self.reached = False

        self.current_index = 1

        self.execution_start_time = None

        self.alignment_start_time = None

        self.alignment_index = None

        self.last_status = None

        self.last_debug_time = (
            self.get_clock().now()
        )

        # ====================================================
        # TF
        # ====================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # ====================================================
        # QoS
        # ====================================================

        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ====================================================
        # Subscriber
        # ====================================================

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            state_qos,
        )

        # ====================================================
        # Publishers
        # ====================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/parking/tracking_status",
            state_qos,
        )

        self.active_pub = self.create_publisher(
            Bool,
            "/parking/tracking_active",
            state_qos,
        )

        self.reached_pub = self.create_publisher(
            Bool,
            "/parking/tracking_reached",
            state_qos,
        )

        self.progress_pub = self.create_publisher(
            Float32,
            "/parking/tracking_progress",
            state_qos,
        )

        self.target_pub = self.create_publisher(
            PoseStamped,
            "/parking/tracking_target",
            state_qos,
        )

        # ====================================================
        # Services
        # ====================================================

        self.execute_srv = self.create_service(
            Trigger,
            "/execute_hybrid_parking",
            self.execute_callback,
        )

        self.stop_srv = self.create_service(
            Trigger,
            "/stop_hybrid_parking",
            self.stop_callback,
        )

        # ====================================================
        # Control loop
        # ====================================================

        period = 1.0 / max(
            self.control_frequency,
            1.0,
        )

        self.timer = self.create_timer(
            period,
            self.control_loop,
        )

        # ====================================================
        # Initial state
        # ====================================================

        self.publish_status(
            "WAITING_PATH"
        )

        self.publish_active(False)
        self.publish_reached(False)
        self.publish_progress(0.0)

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "AutoFactory Parking Path Tracker V3.2"
        )

        self.get_logger().info(
            f"Tracker version: {TRACKER_VERSION}"
        )

        self.get_logger().info(
            f"Path: {self.path_topic}"
        )

        self.get_logger().info(
            "Directions topic: NOT USED"
        )

        self.get_logger().info(
            f"Velocity: {self.cmd_vel_topic}"
        )

        self.get_logger().info(
            "========================================"
        )

    # ========================================================
    # Publishers
    # ========================================================

    def publish_status(self, status: str):

        msg = String()
        msg.data = status

        self.status_pub.publish(msg)

        if status != self.last_status:

            self.last_status = status

            self.get_logger().info(
                f"STATUS: {status}"
            )

    def publish_active(self, value: bool):

        msg = Bool()
        msg.data = bool(value)

        self.active_pub.publish(msg)

    def publish_reached(self, value: bool):

        msg = Bool()
        msg.data = bool(value)

        self.reached_pub.publish(msg)

    def publish_progress(self, value: float):

        msg = Float32()

        msg.data = float(
            clamp(
                value,
                0.0,
                1.0,
            )
        )

        self.progress_pub.publish(msg)

    def stop_robot(self):

        msg = Twist()

        # 连续发布几次零速度
        for _ in range(3):
            self.cmd_pub.publish(msg)

    # ========================================================
    # Hybrid Path
    # ========================================================

    def path_callback(self, msg: Path):

        if self.active:
            return

        if (
            msg.header.frame_id
            != self.map_frame
        ):

            self.get_logger().error(
                f"Wrong path frame: "
                f"{msg.header.frame_id}"
            )

            return

        if len(msg.poses) < 2:

            self.path_ready = False

            self.publish_status(
                "WAITING_PATH"
            )

            return

        self.raw_path = list(
            msg.poses
        )

        self.preprocess_path()

    def preprocess_path(self):

        """
        Hybrid A*允许原地旋转，因此原始Path可能：

            P1
            P2
            P3(yaw=10)
            P3(yaw=20)
            P3(yaw=30)
            P4

        V2容易逐个执行这些P3姿态而卡住。

        V3.1将其压缩为：

            P1
            P2
            P3(yaw=30)
            P4

        注意：
        不是简单删除所有重复XY点，
        而是保留同一XY位置的最后一个Yaw。
        """

        raw = self.raw_path

        cleaned = [
            raw[0]
        ]

        collapsed = 0

        for pose in raw[1:]:

            previous = cleaned[-1]

            distance = math.hypot(
                pose.pose.position.x
                - previous.pose.position.x,
                pose.pose.position.y
                - previous.pose.position.y,
            )

            if (
                distance
                < self.duplicate_xy_tolerance
            ):

                # 关键：
                # 用最新姿态替换旧姿态，
                # 保留这一串spin完成后的最终Yaw。
                cleaned[-1] = pose

                collapsed += 1

            else:

                cleaned.append(
                    pose
                )

        if len(cleaned) < 2:

            self.path_ready = False

            self.publish_status(
                "PATH_INVALID"
            )

            return

        self.path = cleaned

        # ====================================================
        # Infer translation direction
        # ====================================================

        self.directions = [
            0
            for _ in range(
                len(self.path)
            )
        ]

        for i in range(
            1,
            len(self.path),
        ):

            previous = self.path[
                i - 1
            ]

            current = self.path[i]

            dx = (
                current.pose.position.x
                - previous.pose.position.x
            )

            dy = (
                current.pose.position.y
                - previous.pose.position.y
            )

            motion_heading = math.atan2(
                dy,
                dx,
            )

            source_yaw = yaw_from_quaternion(
                previous.pose.orientation
            )

            relative_heading = normalize_angle(
                motion_heading
                - source_yaw
            )

            if (
                math.cos(
                    relative_heading
                )
                >= 0.0
            ):

                self.directions[i] = +1

            else:

                self.directions[i] = -1

        self.directions[0] = (
            self.directions[1]
        )

        forward_count = sum(
            1
            for d in self.directions[1:]
            if d > 0
        )

        reverse_count = sum(
            1
            for d in self.directions[1:]
            if d < 0
        )

        self.path_ready = True

        self.publish_reached(False)

        self.publish_progress(0.0)

        self.publish_status(
            "PATH_READY"
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Hybrid path ready"
        )

        self.get_logger().info(
            f"Raw poses: {len(raw)}"
        )

        self.get_logger().info(
            f"Tracking poses: {len(cleaned)}"
        )

        self.get_logger().info(
            f"Collapsed spin poses: {collapsed}"
        )

        self.get_logger().info(
            f"Forward segments: {forward_count}"
        )

        self.get_logger().info(
            f"Reverse segments: {reverse_count}"
        )

        self.get_logger().info(
            "========================================"
        )

    # ========================================================
    # TF
    # ========================================================

    def get_robot_pose(self):

        tf = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
        )

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        yaw = yaw_from_quaternion(
            tf.transform.rotation
        )

        return x, y, yaw

    # ========================================================
    # Services
    # ========================================================

    def execute_callback(
        self,
        request,
        response,
    ):

        del request

        if self.active:

            response.success = False
            response.message = (
                "Parking tracker already active."
            )

            return response

        if (
            not self.path_ready
            or len(self.path) < 2
        ):

            response.success = False
            response.message = (
                "Hybrid parking path unavailable."
            )

            return response

        try:

            self.get_robot_pose()

        except TransformException as exc:

            response.success = False
            response.message = (
                f"TF unavailable: {exc}"
            )

            return response

        self.current_index = 1

        self.active = True
        self.reached = False

        self.execution_start_time = (
            self.get_clock().now()
        )

        self.reset_alignment()

        self.publish_active(True)
        self.publish_reached(False)
        self.publish_progress(0.0)

        self.publish_status(
            "EXECUTING"
        )

        response.success = True
        response.message = (
            "Hybrid parking started."
        )

        return response

    def stop_callback(
        self,
        request,
        response,
    ):

        del request

        self.active = False

        self.stop_robot()

        self.publish_active(False)

        self.publish_status(
            "STOPPED"
        )

        response.success = True
        response.message = (
            "Hybrid parking stopped."
        )

        return response

    # ========================================================
    # Path progress
    # ========================================================

    def waypoint_distance(
        self,
        x,
        y,
        index,
    ):

        target = self.path[index]

        return math.hypot(
            target.pose.position.x - x,
            target.pose.position.y - y,
        )

    def waypoint_passed(
        self,
        x,
        y,
        index,
    ):

        if index <= 0:
            return False

        previous = (
            self.path[
                index - 1
            ].pose.position
        )

        target = (
            self.path[
                index
            ].pose.position
        )

        sx = target.x - previous.x
        sy = target.y - previous.y

        length_sq = (
            sx * sx
            + sy * sy
        )

        if length_sq < 1.0e-10:
            return True

        rx = x - target.x
        ry = y - target.y

        passed_dot = (
            rx * sx
            + ry * sy
        )

        if passed_dot <= 0.0:
            return False

        length = math.sqrt(
            length_sq
        )

        lateral_error = abs(
            sx * (previous.y - y)
            - (previous.x - x) * sy
        ) / length

        return (
            lateral_error
            <= self.pass_lateral_tolerance
        )

    def advance_waypoints(
        self,
        x,
        y,
    ):

        while (
            self.current_index
            < len(self.path) - 1
        ):

            distance = (
                self.waypoint_distance(
                    x,
                    y,
                    self.current_index,
                )
            )

            if (
                distance
                <= self.waypoint_tolerance
            ):

                self.get_logger().info(
                    f"Waypoint reached: "
                    f"{self.current_index}"
                )

                self.current_index += 1

                continue

            if self.waypoint_passed(
                x,
                y,
                self.current_index,
            ):

                self.get_logger().info(
                    f"Waypoint passed: "
                    f"{self.current_index}"
                )

                self.current_index += 1

                continue

            break

    def select_lookahead_index(
        self,
        start_index,
    ):

        # Safety-first execution: track the current Hybrid A* waypoint
        # itself. The planner step is 0.10 m; selecting the following
        # waypoint can cut corners and physically enter an inflation zone
        # even when the orange planned polyline is collision-free.
        return start_index

    # ========================================================
    # Alignment timeout
    # ========================================================

    def begin_alignment(
        self,
        index,
    ):

        if (
            self.alignment_index
            == index
        ):
            return

        self.alignment_index = index

        self.alignment_start_time = (
            self.get_clock().now()
        )

    def reset_alignment(self):

        self.alignment_index = None
        self.alignment_start_time = None

    def alignment_timeout(self):

        if (
            self.alignment_start_time
            is None
        ):
            return False

        elapsed = (
            self.get_clock().now()
            - self.alignment_start_time
        ).nanoseconds / 1.0e9

        return (
            elapsed
            > self.alignment_timeout_sec
        )

    # ========================================================
    # Angular control
    # ========================================================

    def angular_command(
        self,
        error,
        gain,
    ):

        value = clamp(
            gain * error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        if (
            abs(error) > 0.01
            and
            abs(value)
            < self.min_angular_speed
        ):

            value = math.copysign(
                self.min_angular_speed,
                value,
            )

        return value

    # ========================================================
    # Debug
    # ========================================================

    def debug_log(
        self,
        x,
        y,
        yaw,
        direction,
        target_distance,
        heading_error,
        final_distance,
    ):

        now = self.get_clock().now()

        elapsed = (
            now - self.last_debug_time
        ).nanoseconds / 1.0e9

        if elapsed < 1.0:
            return

        self.last_debug_time = now

        self.get_logger().info(
            "TRACK | "
            f"idx={self.current_index}/"
            f"{len(self.path)-1} | "
            f"dir={direction:+d} | "
            f"pose=("
            f"{x:.3f}, "
            f"{y:.3f}, "
            f"{math.degrees(yaw):.1f}deg"
            f") | "
            f"target={target_distance:.3f}m | "
            f"heading="
            f"{math.degrees(heading_error):.1f}deg | "
            f"goal={final_distance:.3f}m"
        )

    # ========================================================
    # Control loop
    # ========================================================

    def control_loop(self):

        if not self.active:
            return

        # ----------------------------------------------------
        # Global timeout
        # ----------------------------------------------------

        elapsed = (
            self.get_clock().now()
            - self.execution_start_time
        ).nanoseconds / 1.0e9

        if (
            elapsed
            > self.execution_timeout_sec
        ):

            self.fail(
                "EXECUTION_TIMEOUT"
            )

            return

        # ----------------------------------------------------
        # Current robot pose
        # ----------------------------------------------------

        try:

            x, y, yaw = (
                self.get_robot_pose()
            )

        except TransformException:

            self.stop_robot()
            return

        # ----------------------------------------------------
        # Final goal
        # ----------------------------------------------------

        final_pose = self.path[-1]

        final_x = (
            final_pose.pose.position.x
        )

        final_y = (
            final_pose.pose.position.y
        )

        final_yaw = yaw_from_quaternion(
            final_pose.pose.orientation
        )

        final_distance = math.hypot(
            final_x - x,
            final_y - y,
        )

        # ----------------------------------------------------
        # Final XY reached
        # ----------------------------------------------------

        if (
            final_distance
            <= self.final_xy_tolerance
        ):

            self.reset_alignment()

            yaw_error = normalize_angle(
                final_yaw - yaw
            )

            if (
                abs(yaw_error)
                <= self.final_yaw_tolerance
            ):

                self.finish_success()
                return

            cmd = Twist()

            cmd.linear.x = 0.0

            cmd.angular.z = (
                self.angular_command(
                    yaw_error,
                    self.final_spin_gain,
                )
            )

            self.cmd_pub.publish(cmd)

            self.publish_status(
                "FINAL_ALIGN"
            )

            self.debug_log(
                x,
                y,
                yaw,
                0,
                final_distance,
                yaw_error,
                final_distance,
            )

            return

        # ----------------------------------------------------
        # Move waypoint index forward
        # ----------------------------------------------------

        self.advance_waypoints(
            x,
            y,
        )

        if (
            self.current_index
            >= len(self.path)
        ):

            self.current_index = (
                len(self.path) - 1
            )

        direction = int(
            self.directions[
                self.current_index
            ]
        )

        # ----------------------------------------------------
        # Lookahead target
        # ----------------------------------------------------

        lookahead_index = (
            self.select_lookahead_index(
                self.current_index
            )
        )

        target = (
            self.path[
                lookahead_index
            ]
        )

        self.target_pub.publish(
            target
        )

        target_x = (
            target.pose.position.x
        )

        target_y = (
            target.pose.position.y
        )

        dx = target_x - x
        dy = target_y - y

        target_distance = math.hypot(
            dx,
            dy,
        )

        motion_heading = math.atan2(
            dy,
            dx,
        )

        # ----------------------------------------------------
        # Body heading required for this segment
        # ----------------------------------------------------

        if direction > 0:

            desired_heading = (
                motion_heading
            )

        else:

            # 倒车时车头应指向运动方向的反方向
            desired_heading = normalize_angle(
                motion_heading
                + math.pi
            )

        heading_error = normalize_angle(
            desired_heading
            - yaw
        )

        # ----------------------------------------------------
        # Large angle:
        # align first, do not translate.
        # ----------------------------------------------------

        if (
            abs(heading_error)
            > self.rotate_before_drive
        ):

            self.begin_alignment(
                self.current_index
            )

            if self.alignment_timeout():

                self.fail(
                    "ALIGNMENT_TIMEOUT"
                )

                return

            cmd = Twist()

            cmd.linear.x = 0.0

            cmd.angular.z = (
                self.angular_command(
                    heading_error,
                    self.heading_gain,
                )
            )

            self.cmd_pub.publish(cmd)

            self.publish_status(
                "ALIGNING_PATH"
            )

            self.debug_log(
                x,
                y,
                yaw,
                direction,
                target_distance,
                heading_error,
                final_distance,
            )

            self.publish_current_progress()

            return

        # ----------------------------------------------------
        # Heading acceptable -> translation
        # ----------------------------------------------------

        self.reset_alignment()

        angular = (
            self.heading_gain
            * heading_error
        )

        angular = clamp(
            angular,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        speed = (
            self.linear_gain
            * target_distance
        )

        speed = clamp(
            speed,
            self.min_linear_speed,
            self.max_linear_speed,
        )

        # 接近终点时减速
        if (
            final_distance
            < 0.35
        ):

            speed = min(
                speed,
                0.055,
            )

        # 大角度误差时进一步减速
        heading_scale = max(
            0.45,
            math.cos(
                abs(heading_error)
            ),
        )

        speed *= heading_scale

        cmd = Twist()

        cmd.linear.x = (
            float(direction)
            * speed
        )

        cmd.angular.z = angular

        self.cmd_pub.publish(cmd)

        if direction > 0:

            self.publish_status(
                "TRACKING_FORWARD"
            )

        else:

            self.publish_status(
                "TRACKING_REVERSE"
            )

        self.debug_log(
            x,
            y,
            yaw,
            direction,
            target_distance,
            heading_error,
            final_distance,
        )

        self.publish_current_progress()

    # ========================================================
    # Progress
    # ========================================================

    def publish_current_progress(self):

        if len(self.path) <= 1:

            progress = 0.0

        else:

            progress = (
                self.current_index
                /
                (len(self.path) - 1)
            )

        self.publish_progress(
            progress
        )

    # ========================================================
    # Success / Failure
    # ========================================================

    def finish_success(self):

        self.stop_robot()

        self.active = False
        self.reached = True

        self.publish_active(False)
        self.publish_reached(True)
        self.publish_progress(1.0)

        self.publish_status(
            "SUCCEEDED"
        )

        try:

            x, y, yaw = (
                self.get_robot_pose()
            )

            self.get_logger().info(
                "========================================"
            )

            self.get_logger().info(
                "HYBRID PARKING SUCCESS"
            )

            self.get_logger().info(
                f"x={x:.3f}, "
                f"y={y:.3f}, "
                f"yaw={math.degrees(yaw):.2f} deg"
            )

            self.get_logger().info(
                "========================================"
            )

        except TransformException:

            pass

    def fail(self, reason: str):

        self.stop_robot()

        self.active = False
        self.reached = False

        self.publish_active(False)
        self.publish_reached(False)

        self.publish_status(
            "FAILED_" + reason
        )

        self.get_logger().error(
            "========================================"
        )

        self.get_logger().error(
            f"PARKING FAILED: {reason}"
        )

        self.get_logger().error(
            "========================================"
        )


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ParkingPathTracker()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.stop_robot()

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":

    main()
