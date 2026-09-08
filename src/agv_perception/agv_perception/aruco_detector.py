#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AutoFactory ArUco perception node.

Responsibilities:
1. Subscribe to camera image and CameraInfo.
2. Stay disabled by default.
3. Enable/disable detection through /aruco/enable.
4. Detect only the configured ArUco marker ID.
5. Estimate marker pose using solvePnP.
6. Publish pose in camera optical frame.
7. Require several consecutive detections before declaring LOCKED.

This node does NOT:
- move the robot;
- modify the world;
- calculate the final parking goal;
- run Hybrid A*;
- transform marker pose into map frame.
"""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


def rotation_matrix_to_quaternion(r):
    """
    Convert a 3x3 rotation matrix to quaternion (x, y, z, w).
    """

    trace = r[0, 0] + r[1, 1] + r[2, 2]

    if trace > 0.0:

        s = math.sqrt(trace + 1.0) * 2.0

        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s

    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:

        s = math.sqrt(
            1.0
            + r[0, 0]
            - r[1, 1]
            - r[2, 2]
        ) * 2.0

        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s

    elif r[1, 1] > r[2, 2]:

        s = math.sqrt(
            1.0
            + r[1, 1]
            - r[0, 0]
            - r[2, 2]
        ) * 2.0

        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s

    else:

        s = math.sqrt(
            1.0
            + r[2, 2]
            - r[0, 0]
            - r[1, 1]
        ) * 2.0

        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s

    norm = math.sqrt(
        qx * qx + qy * qy + qz * qz + qw * qw
    )

    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0

    return (
        qx / norm,
        qy / norm,
        qz / norm,
        qw / norm,
    )


class ArucoDetector(Node):

    def __init__(self):

        super().__init__("aruco_detector")

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter(
            "image_topic",
            "/camera/image_raw",
        )

        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera_info",
        )

        self.declare_parameter(
            "dictionary",
            "DICT_6X6_250",
        )

        self.declare_parameter(
            "target_marker_id",
            0,
        )

        self.declare_parameter(
            "marker_length",
            0.20,
        )

        self.declare_parameter(
            "enabled_on_startup",
            False,
        )

        self.declare_parameter(
            "stable_required_frames",
            5,
        )

        self.declare_parameter(
            "publish_debug_image",
            True,
        )

        self.declare_parameter(
            "draw_axes",
            True,
        )

        self.declare_parameter(
            "axis_length",
            0.10,
        )

        # 这次新增的关键参数
        self.declare_parameter(
            "min_marker_distance_rate",
            0.02,
        )

        # 可选保留，方便以后调试
        self.declare_parameter(
            "min_corner_distance_rate",
            0.05,
        )

        # =====================================================
        # Read parameters
        # =====================================================

        self.image_topic = str(
            self.get_parameter(
                "image_topic"
            ).value
        )

        self.camera_info_topic = str(
            self.get_parameter(
                "camera_info_topic"
            ).value
        )

        self.dictionary_name = str(
            self.get_parameter(
                "dictionary"
            ).value
        )

        self.target_marker_id = int(
            self.get_parameter(
                "target_marker_id"
            ).value
        )

        self.marker_length = float(
            self.get_parameter(
                "marker_length"
            ).value
        )

        self.enabled = bool(
            self.get_parameter(
                "enabled_on_startup"
            ).value
        )

        self.stable_required_frames = max(
            1,
            int(
                self.get_parameter(
                    "stable_required_frames"
                ).value
            ),
        )

        self.publish_debug_image_enabled = bool(
            self.get_parameter(
                "publish_debug_image"
            ).value
        )

        self.draw_axes = bool(
            self.get_parameter(
                "draw_axes"
            ).value
        )

        self.axis_length = float(
            self.get_parameter(
                "axis_length"
            ).value
        )

        self.min_marker_distance_rate = float(
            self.get_parameter(
                "min_marker_distance_rate"
            ).value
        )

        self.min_corner_distance_rate = float(
            self.get_parameter(
                "min_corner_distance_rate"
            ).value
        )

        # =====================================================
        # Validate OpenCV ArUco
        # =====================================================

        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "OpenCV ArUco module is not available."
            )

        if not hasattr(
            cv2.aruco,
            self.dictionary_name,
        ):
            raise RuntimeError(
                "Unknown ArUco dictionary: "
                f"{self.dictionary_name}"
            )

        dictionary_id = getattr(
            cv2.aruco,
            self.dictionary_name,
        )

        if hasattr(
            cv2.aruco,
            "getPredefinedDictionary",
        ):
            self.dictionary = (
                cv2.aruco.getPredefinedDictionary(
                    dictionary_id
                )
            )
        else:
            self.dictionary = (
                cv2.aruco.Dictionary_get(
                    dictionary_id
                )
            )

        if hasattr(
            cv2.aruco,
            "DetectorParameters_create",
        ):
            self.detector_parameters = (
                cv2.aruco.DetectorParameters_create()
            )
        else:
            self.detector_parameters = (
                cv2.aruco.DetectorParameters()
            )

        # =====================================================
        # 关键修复：设置候选Marker距离阈值
        # =====================================================

        if hasattr(
            self.detector_parameters,
            "minMarkerDistanceRate"
        ):
            self.detector_parameters.minMarkerDistanceRate = (
                self.min_marker_distance_rate
            )

        if hasattr(
            self.detector_parameters,
            "minCornerDistanceRate"
        ):
            self.detector_parameters.minCornerDistanceRate = (
                self.min_corner_distance_rate
            )

        # 保持角点细化，增强稳定性
        if hasattr(
            self.detector_parameters,
            "cornerRefinementMethod"
        ) and hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            self.detector_parameters.cornerRefinementMethod = (
                cv2.aruco.CORNER_REFINE_SUBPIX
            )

        if hasattr(
            cv2.aruco,
            "ArucoDetector",
        ):
            self.aruco_detector = (
                cv2.aruco.ArucoDetector(
                    self.dictionary,
                    self.detector_parameters,
                )
            )
        else:
            self.aruco_detector = None

        # =====================================================
        # Marker model points
        #
        # Corner order:
        # top-left
        # top-right
        # bottom-right
        # bottom-left
        # =====================================================

        half = self.marker_length / 2.0

        self.object_points = np.array(
            [
                [-half, +half, 0.0],
                [+half, +half, 0.0],
                [+half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        # =====================================================
        # Camera calibration
        # =====================================================

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_frame = ""

        # =====================================================
        # Runtime state
        # =====================================================

        self.bridge = CvBridge()

        self.consecutive_count = 0
        self.current_detected = False
        self.current_locked = False

        if self.enabled:
            self.current_status = "WAITING_CAMERA_INFO"
        else:
            self.current_status = "DISABLED"

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

        self.detected_pub = self.create_publisher(
            Bool,
            "/aruco/detected",
            state_qos,
        )

        self.locked_pub = self.create_publisher(
            Bool,
            "/aruco/locked",
            state_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            "/aruco/status",
            state_qos,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/aruco/marker_pose",
            10,
        )

        self.distance_pub = self.create_publisher(
            Float32,
            "/aruco/distance",
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            "/aruco/debug_image",
            qos_profile_sensor_data,
        )

        # =====================================================
        # Subscribers
        # =====================================================

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # =====================================================
        # Enable / Disable service
        # =====================================================

        self.enable_service = self.create_service(
            SetBool,
            "/aruco/enable",
            self.enable_callback,
        )

        self.state_timer = self.create_timer(
            1.0,
            self.publish_state,
        )

        # =====================================================
        # Startup logging
        # =====================================================

        self.get_logger().info(
            "========================================"
        )
        self.get_logger().info(
            "AutoFactory ArUco detector"
        )
        self.get_logger().info(
            f"Image topic: {self.image_topic}"
        )
        self.get_logger().info(
            f"CameraInfo topic: {self.camera_info_topic}"
        )
        self.get_logger().info(
            f"Dictionary: {self.dictionary_name}"
        )
        self.get_logger().info(
            f"Target marker ID: {self.target_marker_id}"
        )
        self.get_logger().info(
            f"Marker length: {self.marker_length:.3f} m"
        )
        self.get_logger().info(
            f"Stable frames required: {self.stable_required_frames}"
        )
        self.get_logger().info(
            f"Enabled: {self.enabled}"
        )
        self.get_logger().info(
            f"minMarkerDistanceRate: {self.min_marker_distance_rate:.3f}"
        )
        self.get_logger().info(
            f"minCornerDistanceRate: {self.min_corner_distance_rate:.3f}"
        )
        self.get_logger().info(
            "========================================"
        )

        self.publish_state()

    # =========================================================
    # State helpers
    # =========================================================

    def set_status(self, status: str):

        if status != self.current_status:
            self.current_status = status
            self.get_logger().info(
                f"ArUco status: {status}"
            )

    def publish_state(self):

        detected_msg = Bool()
        detected_msg.data = self.current_detected

        locked_msg = Bool()
        locked_msg.data = self.current_locked

        status_msg = String()
        status_msg.data = self.current_status

        self.detected_pub.publish(detected_msg)
        self.locked_pub.publish(locked_msg)
        self.status_pub.publish(status_msg)

    def reset_detection_state(self):

        self.consecutive_count = 0
        self.current_detected = False
        self.current_locked = False

    # =========================================================
    # Enable service
    # =========================================================

    def enable_callback(self, request, response):

        requested = bool(request.data)

        # Idempotent enable: do not clear a valid mission lock simply
        # because /aruco/enable true is called again.
        if requested == self.enabled:
            response.success = True
            response.message = (
                "ArUco detection already enabled."
                if self.enabled
                else "ArUco detection already disabled."
            )
            self.publish_state()
            return response

        self.enabled = requested

        if self.enabled:

            # A new detection mission starts here.
            self.reset_detection_state()

            if self.camera_matrix is None:
                self.set_status("WAITING_CAMERA_INFO")
            else:
                self.set_status("SEARCHING")

            response.success = True
            response.message = "ArUco detection enabled."

            self.get_logger().info(
                "ArUco detection ENABLED."
            )

        else:

            # Disable explicitly releases the latched lock.
            self.reset_detection_state()
            self.set_status("DISABLED")

            response.success = True
            response.message = "ArUco detection disabled."

            self.get_logger().info(
                "ArUco detection DISABLED."
            )

        self.publish_state()
        return response

    # =========================================================
    # CameraInfo
    # =========================================================

    def camera_info_callback(self, msg: CameraInfo):

        if self.camera_matrix is not None:
            return

        k = np.asarray(msg.k, dtype=np.float64)

        if k.size != 9:
            self.get_logger().error(
                "Invalid CameraInfo K matrix."
            )
            return

        matrix = k.reshape(3, 3)

        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.get_logger().error(
                "Camera focal length is invalid."
            )
            return

        self.camera_matrix = matrix

        self.dist_coeffs = np.asarray(
            msg.d,
            dtype=np.float64,
        ).reshape(-1, 1)

        self.camera_frame = msg.header.frame_id

        self.get_logger().info("CameraInfo received:")
        self.get_logger().info(f"frame={self.camera_frame}")
        self.get_logger().info(
            f"fx={matrix[0, 0]:.3f}, fy={matrix[1, 1]:.3f}"
        )
        self.get_logger().info(
            f"cx={matrix[0, 2]:.3f}, cy={matrix[1, 2]:.3f}"
        )

        if self.enabled:
            self.set_status("SEARCHING")

    # =========================================================
    # Marker detection
    # =========================================================

    def detect_markers(self, gray_image):

        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray_image)

        return cv2.aruco.detectMarkers(
            gray_image,
            self.dictionary,
            parameters=self.detector_parameters,
        )

    # =========================================================
    # Pose estimation
    # =========================================================

    def estimate_pose(self, corners):

        image_points = np.asarray(
            corners,
            dtype=np.float32,
        ).reshape(4, 2)

        if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
            method = cv2.SOLVEPNP_IPPE_SQUARE
        else:
            method = cv2.SOLVEPNP_ITERATIVE

        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=method,
        )

        if not success:
            return None, None

        return rvec, tvec

    # =========================================================
    # Image callback
    # =========================================================

    def image_callback(self, msg: Image):

        if not self.enabled:
            return

        if self.camera_matrix is None:

            self.reset_detection_state()
            self.set_status("WAITING_CAMERA_INFO")
            return

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except CvBridgeError as exc:

            self.get_logger().error(
                f"CvBridge error: {exc}"
            )

            self.reset_detection_state()
            self.set_status("IMAGE_CONVERSION_ERROR")
            return

        debug_image = image.copy()

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY,
        )

        corners, ids, _ = self.detect_markers(gray)

        if (
            ids is not None
            and len(ids) > 0
            and hasattr(cv2.aruco, "drawDetectedMarkers")
        ):
            cv2.aruco.drawDetectedMarkers(
                debug_image,
                corners,
                ids,
            )

        target_index = None

        if ids is not None:

            ids_flat = np.asarray(ids).reshape(-1)

            matches = np.where(
                ids_flat == self.target_marker_id
            )[0]

            if matches.size > 0:
                target_index = int(matches[0])

        if target_index is None:

            # Before LOCKED, a miss breaks the consecutive-frame count.
            # After LOCKED, keep the mission lock latched until detection
            # is explicitly disabled. This prevents a single dropped frame
            # from tearing down Planner/Transformer state.
            self.current_detected = False

            if not self.current_locked:
                self.consecutive_count = 0
                self.set_status("SEARCHING")
            else:
                self.set_status("LOCKED")

            cv2.putText(
                debug_image,
                f"Searching ArUco ID={self.target_marker_id}",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            self.publish_debug_image(debug_image, msg)
            self.publish_state()
            return

        target_corners = corners[target_index]

        rvec, tvec = self.estimate_pose(target_corners)

        if rvec is None or tvec is None:

            self.current_detected = False

            if not self.current_locked:
                self.consecutive_count = 0
                self.set_status("PNP_FAILED")
            else:
                # Keep an already acquired mission lock latched.
                self.set_status("LOCKED")

            self.publish_debug_image(debug_image, msg)
            self.publish_state()
            return

        tvec = np.asarray(
            tvec,
            dtype=np.float64,
        ).reshape(3)

        rvec = np.asarray(
            rvec,
            dtype=np.float64,
        ).reshape(3, 1)

        distance = float(np.linalg.norm(tvec))

        pose_msg = PoseStamped()
        pose_msg.header = msg.header

        if not pose_msg.header.frame_id:
            pose_msg.header.frame_id = self.camera_frame

        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])

        rotation_matrix, _ = cv2.Rodrigues(rvec)

        qx, qy, qz, qw = rotation_matrix_to_quaternion(
            rotation_matrix
        )

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        distance_msg = Float32()
        distance_msg.data = distance
        self.distance_pub.publish(distance_msg)

        self.current_detected = True
        self.consecutive_count += 1

        if self.consecutive_count >= self.stable_required_frames:

            self.current_locked = True
            self.set_status("LOCKED")

        else:

            self.current_locked = False
            self.set_status(
                "DETECTED_"
                f"{self.consecutive_count}"
                "_OF_"
                f"{self.stable_required_frames}"
            )

        if (
            self.draw_axes
            and hasattr(cv2, "drawFrameAxes")
        ):
            cv2.drawFrameAxes(
                debug_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec.reshape(3, 1),
                self.axis_length,
                2,
            )

        cv2.putText(
            debug_image,
            f"ID={self.target_marker_id} LOCKED={self.current_locked}",
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug_image,
            f"x={tvec[0]:+.3f} y={tvec[1]:+.3f} z={tvec[2]:+.3f} m",
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug_image,
            f"distance={distance:.3f} m",
            (30, 135),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        self.publish_debug_image(debug_image, msg)
        self.publish_state()

    # =========================================================
    # Debug image publication
    # =========================================================

    def publish_debug_image(self, image, source_msg):

        if not self.publish_debug_image_enabled:
            return

        try:
            output_msg = self.bridge.cv2_to_imgmsg(
                image,
                encoding="bgr8",
            )

        except CvBridgeError as exc:

            self.get_logger().error(
                f"Debug image conversion failed: {exc}"
            )
            return

        output_msg.header = source_msg.header
        self.debug_image_pub.publish(output_msg)


def main(args=None):

    rclpy.init(args=args)

    node = ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
