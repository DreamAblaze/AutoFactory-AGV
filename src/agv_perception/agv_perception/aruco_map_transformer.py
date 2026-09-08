#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Transform ArUco marker pose from camera optical frame to map frame.

Input:
    /aruco/marker_pose
    /aruco/locked

Output:
    /aruco/marker_pose_map
    /aruco/map_pose_available
    /aruco/map_xy_error
    /aruco/map_status

Important:
Known Gazebo marker coordinates are used ONLY for validation.
They are NOT used to calculate the transformed marker pose.
"""

from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String

from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)


def normalize_quaternion(q):
    """Normalize quaternion [x, y, z, w]."""

    q = np.asarray(
        q,
        dtype=np.float64,
    )

    norm = np.linalg.norm(q)

    if norm < 1.0e-12:
        return np.array(
            [0.0, 0.0, 0.0, 1.0],
            dtype=np.float64,
        )

    return q / norm


def quaternion_multiply(q1, q2):
    """
    Quaternion multiplication.

    q1 and q2:
        [x, y, z, w]

    Result:
        q1 * q2
    """

    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return normalize_quaternion(
        np.array(
            [
                w1 * x2
                + x1 * w2
                + y1 * z2
                - z1 * y2,

                w1 * y2
                - x1 * z2
                + y1 * w2
                + z1 * x2,

                w1 * z2
                + x1 * y2
                - y1 * x2
                + z1 * w2,

                w1 * w2
                - x1 * x2
                - y1 * y2
                - z1 * z2,
            ],
            dtype=np.float64,
        )
    )


def quaternion_to_rotation_matrix(q):
    """
    Quaternion [x, y, z, w]
    to 3x3 rotation matrix.
    """

    x, y, z, w = normalize_quaternion(q)

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


class ArucoMapTransformer(Node):

    def __init__(self):

        super().__init__(
            "aruco_map_transformer"
        )

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter(
            "map_frame",
            "map",
        )

        self.declare_parameter(
            "marker_pose_topic",
            "/aruco/marker_pose",
        )

        self.declare_parameter(
            "locked_topic",
            "/aruco/locked",
        )

        self.declare_parameter(
            "require_locked",
            True,
        )

        # Robot is stationary at Parking Staging when this
        # transformation is used, so latest TF is robust and
        # avoids image/AMCL timestamp extrapolation problems.
        self.declare_parameter(
            "use_latest_tf",
            True,
        )

        self.declare_parameter(
            "tf_timeout_sec",
            0.30,
        )

        # =====================================================
        # Simulation validation only
        # =====================================================

        self.declare_parameter(
            "enable_ground_truth_validation",
            True,
        )

        self.declare_parameter(
            "ground_truth_marker_x",
            -7.572620,
        )

        self.declare_parameter(
            "ground_truth_marker_y",
            -4.473930,
        )

        self.declare_parameter(
            "ground_truth_marker_z",
            0.350,
        )

        self.declare_parameter(
            "validation_xy_tolerance",
            0.15,
        )

        # =====================================================
        # Read parameters
        # =====================================================

        self.map_frame = str(
            self.get_parameter(
                "map_frame"
            ).value
        )

        self.marker_pose_topic = str(
            self.get_parameter(
                "marker_pose_topic"
            ).value
        )

        self.locked_topic = str(
            self.get_parameter(
                "locked_topic"
            ).value
        )

        self.require_locked = bool(
            self.get_parameter(
                "require_locked"
            ).value
        )

        self.use_latest_tf = bool(
            self.get_parameter(
                "use_latest_tf"
            ).value
        )

        self.tf_timeout_sec = float(
            self.get_parameter(
                "tf_timeout_sec"
            ).value
        )

        self.enable_ground_truth_validation = bool(
            self.get_parameter(
                "enable_ground_truth_validation"
            ).value
        )

        self.gt_x = float(
            self.get_parameter(
                "ground_truth_marker_x"
            ).value
        )

        self.gt_y = float(
            self.get_parameter(
                "ground_truth_marker_y"
            ).value
        )

        self.gt_z = float(
            self.get_parameter(
                "ground_truth_marker_z"
            ).value
        )

        self.validation_xy_tolerance = float(
            self.get_parameter(
                "validation_xy_tolerance"
            ).value
        )

        # =====================================================
        # Runtime state
        # =====================================================

        self.locked = False
        self.pose_available = False

        self.current_status = (
            "WAITING_LOCK"
            if self.require_locked
            else "WAITING_MARKER_POSE"
        )

        self.last_log_time_ns = 0

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
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

        self.map_pose_pub = self.create_publisher(
            PoseStamped,
            "/aruco/marker_pose_map",
            10,
        )

        self.available_pub = self.create_publisher(
            Bool,
            "/aruco/map_pose_available",
            state_qos,
        )

        self.error_pub = self.create_publisher(
            Float32,
            "/aruco/map_xy_error",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/aruco/map_status",
            state_qos,
        )

        # =====================================================
        # Subscribers
        # =====================================================

        self.locked_sub = self.create_subscription(
            Bool,
            self.locked_topic,
            self.locked_callback,
            state_qos,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.marker_pose_topic,
            self.marker_pose_callback,
            10,
        )

        # =====================================================
        # Startup
        # =====================================================

        self.publish_available(False)
        self.publish_status(
            self.current_status
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "AutoFactory ArUco Map Transformer"
        )

        self.get_logger().info(
            f"Target frame: {self.map_frame}"
        )

        self.get_logger().info(
            f"Marker pose topic: {self.marker_pose_topic}"
        )

        self.get_logger().info(
            f"Require locked: {self.require_locked}"
        )

        self.get_logger().info(
            f"Use latest TF: {self.use_latest_tf}"
        )

        if self.enable_ground_truth_validation:

            self.get_logger().info(
                "Simulation ground truth:"
            )

            self.get_logger().info(
                f"x={self.gt_x:.6f}, "
                f"y={self.gt_y:.6f}, "
                f"z={self.gt_z:.3f}"
            )

        self.get_logger().info(
            "========================================"
        )

    # =========================================================
    # State
    # =========================================================

    def set_status(self, status):

        if status != self.current_status:

            self.current_status = status

            self.publish_status(status)

            self.get_logger().info(
                f"Map transform status: {status}"
            )

    def publish_status(self, status):

        msg = String()
        msg.data = status

        self.status_pub.publish(msg)

    def publish_available(self, value):

        self.pose_available = bool(value)

        msg = Bool()
        msg.data = self.pose_available

        self.available_pub.publish(msg)

    # =========================================================
    # Locked callback
    # =========================================================

    def locked_callback(self, msg):

        previous = self.locked
        self.locked = bool(msg.data)

        if self.locked:

            # Only the false -> true edge changes state. The detector
            # republishes its state periodically; repeated true messages
            # must not overwrite TRACKING with WAITING_MARKER_POSE.
            if not previous:
                self.get_logger().info(
                    "ArUco LOCKED received."
                )
                self.set_status(
                    "WAITING_MARKER_POSE"
                )

        else:

            self.publish_available(False)

            if self.require_locked:
                self.set_status(
                    "WAITING_LOCK"
                )

    # =========================================================
    # Marker pose callback
    # =========================================================

    def marker_pose_callback(
        self,
        msg: PoseStamped,
    ):

        if (
            self.require_locked
            and not self.locked
        ):
            return

        if not msg.header.frame_id:

            self.set_status(
                "INVALID_SOURCE_FRAME"
            )

            self.publish_available(False)

            return

        # -----------------------------------------------------
        # Select TF time
        # -----------------------------------------------------

        if self.use_latest_tf:

            lookup_time = Time()

        else:

            lookup_time = Time.from_msg(
                msg.header.stamp
            )

        # -----------------------------------------------------
        # map <- camera transform
        # -----------------------------------------------------

        try:

            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                lookup_time,
                timeout=Duration(
                    seconds=self.tf_timeout_sec
                ),
            )

        except TransformException as exc:

            self.publish_available(False)

            self.set_status(
                "TF_UNAVAILABLE"
            )

            self.get_logger().warn(
                "Cannot transform "
                f"{msg.header.frame_id} "
                f"to {self.map_frame}: {exc}"
            )

            return

        # =====================================================
        # Transform translation
        #
        # p_map_marker =
        #     p_map_camera
        #     +
        #     R_map_camera * p_camera_marker
        # =====================================================

        tf_translation = np.array(
            [
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z,
            ],
            dtype=np.float64,
        )

        q_map_camera = np.array(
            [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w,
            ],
            dtype=np.float64,
        )

        p_camera_marker = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            dtype=np.float64,
        )

        rotation_map_camera = (
            quaternion_to_rotation_matrix(
                q_map_camera
            )
        )

        p_map_marker = (
            tf_translation
            + rotation_map_camera.dot(
                p_camera_marker
            )
        )

        # =====================================================
        # Transform orientation
        #
        # q_map_marker =
        #     q_map_camera * q_camera_marker
        #
        # Orientation is published, but parking orientation
        # will NOT use it until marker-frame convention is
        # separately validated.
        # =====================================================

        q_camera_marker = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=np.float64,
        )

        q_map_marker = quaternion_multiply(
            normalize_quaternion(
                q_map_camera
            ),
            normalize_quaternion(
                q_camera_marker
            ),
        )

        # =====================================================
        # Publish map pose
        # =====================================================

        output = PoseStamped()

        output.header.stamp = (
            msg.header.stamp
        )

        output.header.frame_id = (
            self.map_frame
        )

        output.pose.position.x = float(
            p_map_marker[0]
        )

        output.pose.position.y = float(
            p_map_marker[1]
        )

        output.pose.position.z = float(
            p_map_marker[2]
        )

        output.pose.orientation.x = float(
            q_map_marker[0]
        )

        output.pose.orientation.y = float(
            q_map_marker[1]
        )

        output.pose.orientation.z = float(
            q_map_marker[2]
        )

        output.pose.orientation.w = float(
            q_map_marker[3]
        )

        self.map_pose_pub.publish(
            output
        )

        self.publish_available(True)

        # =====================================================
        # Simulation ground-truth validation
        # =====================================================

        xy_error = None

        if self.enable_ground_truth_validation:

            xy_error = math.hypot(
                p_map_marker[0] - self.gt_x,
                p_map_marker[1] - self.gt_y,
            )

            error_msg = Float32()
            error_msg.data = float(
                xy_error
            )

            self.error_pub.publish(
                error_msg
            )

            if (
                xy_error
                <= self.validation_xy_tolerance
            ):

                self.set_status(
                    "TRACKING_OK"
                )

            else:

                self.set_status(
                    "TRACKING_LARGE_ERROR"
                )

        else:

            self.set_status(
                "TRACKING"
            )

        # =====================================================
        # Throttled log: once per second
        # =====================================================

        now_ns = (
            self.get_clock()
            .now()
            .nanoseconds
        )

        if (
            now_ns
            - self.last_log_time_ns
            >= 1_000_000_000
        ):

            self.last_log_time_ns = now_ns

            self.get_logger().info(
                "Map marker pose: "
                f"x={p_map_marker[0]:.3f}, "
                f"y={p_map_marker[1]:.3f}, "
                f"z={p_map_marker[2]:.3f}"
            )

            if xy_error is not None:

                self.get_logger().info(
                    "Marker XY validation error: "
                    f"{xy_error:.3f} m"
                )


def main(args=None):

    rclpy.init(args=args)

    node = ArucoMapTransformer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
