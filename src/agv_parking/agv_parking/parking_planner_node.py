#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AutoFactory Hybrid A* parking planner.

Inputs:
    /map
    TF map -> base_footprint
    /aruco/locked

Outputs:
    /parking_goal
    /parking/hybrid_path
    /parking/hybrid_status
    /parking/hybrid_path_available

Services:
    /plan_hybrid_parking
    /clear_hybrid_parking

This node plans only.
It does NOT publish cmd_vel.
"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np

import rclpy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

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

from agv_parking.hybrid_astar import (
    HybridAStarPlanner,
)


def normalize_angle(angle):

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


def quaternion_from_yaw(yaw):

    return (
        0.0,
        0.0,
        math.sin(yaw * 0.5),
        math.cos(yaw * 0.5),
    )


class ParkingPlannerNode(Node):

    def __init__(self):

        super().__init__(
            "parking_planner"
        )

        # =====================================================
        # Frames / topics
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
            "map_topic",
            "/map",
        )

        self.declare_parameter(
            "require_aruco_lock",
            True,
        )

        self.declare_parameter(
            "aruco_locked_topic",
            "/aruco/locked",
        )

        # =====================================================
        # Parking goal in ROS MAP frame
        #
        # Corresponding Gazebo WORLD target:
        #
        # x   = -8.372620
        # y   = -4.473930
        # yaw = 1.570801
        # =====================================================

        self.declare_parameter(
            "goal_x",
            -8.357680,
        )

        self.declare_parameter(
            "goal_y",
            -4.166548,
        )

        self.declare_parameter(
            "goal_yaw",
            1.539811,
        )

        # =====================================================
        # Occupancy map collision
        # =====================================================

        self.declare_parameter(
            "occupied_threshold",
            65,
        )

        self.declare_parameter(
            "unknown_is_obstacle",
            True,
        )

        self.declare_parameter(
            "robot_radius",
            0.22,
        )

        self.declare_parameter(
            "safety_margin",
            0.03,
        )

        # =====================================================
        # Hybrid A*
        # =====================================================

        self.declare_parameter(
            "xy_step",
            0.10,
        )

        self.declare_parameter(
            "yaw_resolution_deg",
            5.0,
        )

        self.declare_parameter(
            "min_turning_radius",
            0.40,
        )

        self.declare_parameter(
            "collision_check_step",
            0.025,
        )

        self.declare_parameter(
            "allow_reverse",
            True,
        )

        self.declare_parameter(
            "allow_in_place_rotation",
            True,
        )

        self.declare_parameter(
            "spin_step_deg",
            10.0,
        )

        self.declare_parameter(
            "reverse_penalty",
            1.30,
        )

        self.declare_parameter(
            "gear_switch_penalty",
            0.40,
        )

        self.declare_parameter(
            "turning_penalty",
            0.15,
        )

        self.declare_parameter(
            "spin_penalty",
            1.20,
        )

        self.declare_parameter(
            "yaw_heuristic_weight",
            1.0,
        )

        self.declare_parameter(
            "heuristic_weight",
            1.0,
        )

        self.declare_parameter(
            "goal_xy_tolerance",
            0.12,
        )

        self.declare_parameter(
            "goal_yaw_tolerance_deg",
            8.0,
        )

        self.declare_parameter(
            "search_margin",
            3.0,
        )

        self.declare_parameter(
            "max_expansions",
            120000,
        )

        # =====================================================
        # Read
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

        self.map_topic = str(
            self.get_parameter(
                "map_topic"
            ).value
        )

        self.require_aruco_lock = bool(
            self.get_parameter(
                "require_aruco_lock"
            ).value
        )

        self.aruco_locked_topic = str(
            self.get_parameter(
                "aruco_locked_topic"
            ).value
        )

        self.goal_x = float(
            self.get_parameter(
                "goal_x"
            ).value
        )

        self.goal_y = float(
            self.get_parameter(
                "goal_y"
            ).value
        )

        self.goal_yaw = float(
            self.get_parameter(
                "goal_yaw"
            ).value
        )

        self.occupied_threshold = int(
            self.get_parameter(
                "occupied_threshold"
            ).value
        )

        self.unknown_is_obstacle = bool(
            self.get_parameter(
                "unknown_is_obstacle"
            ).value
        )

        self.robot_radius = float(
            self.get_parameter(
                "robot_radius"
            ).value
        )

        self.safety_margin = float(
            self.get_parameter(
                "safety_margin"
            ).value
        )

        # =====================================================
        # Runtime
        # =====================================================

        self.map_msg = None
        self.inflated_grid = None

        self.aruco_locked = False

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

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # =====================================================
        # Subscribers
        # =====================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )

        self.lock_sub = self.create_subscription(
            Bool,
            self.aruco_locked_topic,
            self.lock_callback,
            state_qos,
        )

        # =====================================================
        # Publishers
        # =====================================================

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/parking_goal",
            state_qos,
        )

        self.path_pub = self.create_publisher(
            Path,
            "/parking/hybrid_path",
            state_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            "/parking/hybrid_status",
            state_qos,
        )

        self.available_pub = self.create_publisher(
            Bool,
            "/parking/hybrid_path_available",
            state_qos,
        )

        # =====================================================
        # Services
        # =====================================================

        self.plan_srv = self.create_service(
            Trigger,
            "/plan_hybrid_parking",
            self.plan_callback,
        )

        self.clear_srv = self.create_service(
            Trigger,
            "/clear_hybrid_parking",
            self.clear_callback,
        )

        # =====================================================
        # Initial state
        # =====================================================

        self.publish_status(
            "WAITING_MAP"
        )

        self.publish_available(
            False
        )

        self.publish_goal()

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "AutoFactory Hybrid A* Parking Planner"
        )

        self.get_logger().info(
            "Parking goal in map:"
        )

        self.get_logger().info(
            f"x={self.goal_x:.6f}, "
            f"y={self.goal_y:.6f}, "
            f"yaw={self.goal_yaw:.6f}"
        )

        self.get_logger().info(
            f"Robot radius={self.robot_radius:.3f} m"
        )

        self.get_logger().info(
            f"Safety margin={self.safety_margin:.3f} m"
        )

        self.get_logger().info(
            "========================================"
        )

    # =========================================================
    # State publishers
    # =========================================================

    def publish_status(
        self,
        status,
    ):

        msg = String()
        msg.data = status

        self.status_pub.publish(
            msg
        )

        self.get_logger().info(
            f"STATUS: {status}"
        )

    def publish_available(
        self,
        value,
    ):

        msg = Bool()
        msg.data = bool(value)

        self.available_pub.publish(
            msg
        )

    # =========================================================
    # Goal
    # =========================================================

    def make_goal_pose(self):

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            self.map_frame
        )

        msg.pose.position.x = (
            self.goal_x
        )

        msg.pose.position.y = (
            self.goal_y
        )

        msg.pose.position.z = 0.0

        qx, qy, qz, qw = (
            quaternion_from_yaw(
                self.goal_yaw
            )
        )

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def publish_goal(self):

        self.goal_pub.publish(
            self.make_goal_pose()
        )

    # =========================================================
    # ArUco lock
    # =========================================================

    def lock_callback(
        self,
        msg,
    ):

        self.aruco_locked = bool(
            msg.data
        )

    # =========================================================
    # Map
    # =========================================================

    def map_callback(
        self,
        msg,
    ):

        first_map = (
            self.map_msg is None
        )

        self.map_msg = msg

        raw = np.asarray(
            msg.data,
            dtype=np.int16,
        ).reshape(
            msg.info.height,
            msg.info.width,
        )

        occupied = (
            raw
            >= self.occupied_threshold
        )

        if self.unknown_is_obstacle:

            occupied = np.logical_or(
                occupied,
                raw < 0,
            )

        resolution = float(
            msg.info.resolution
        )

        inflate_radius = (
            self.robot_radius
            + self.safety_margin
        )

        radius_cells = int(
            math.ceil(
                inflate_radius
                / resolution
            )
        )

        kernel_size = (
            2 * radius_cells + 1
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                kernel_size,
                kernel_size,
            ),
        )

        inflated = cv2.dilate(
            occupied.astype(np.uint8),
            kernel,
            iterations=1,
        )

        self.inflated_grid = (
            inflated > 0
        )

        if first_map:

            self.get_logger().info(
                "Occupancy map received:"
            )

            self.get_logger().info(
                f"size="
                f"{msg.info.width}x"
                f"{msg.info.height}, "
                f"resolution="
                f"{resolution:.3f} m"
            )

            self.get_logger().info(
                "Collision inflation:"
            )

            self.get_logger().info(
                f"radius="
                f"{inflate_radius:.3f} m "
                f"({radius_cells} cells)"
            )

            if (
                self.require_aruco_lock
                and not self.aruco_locked
            ):
                self.publish_status(
                    "WAITING_ARUCO_LOCK"
                )
            else:
                self.publish_status(
                    "READY"
                )

    # =========================================================
    # Robot pose
    # =========================================================

    def get_robot_pose(self):

        tf_msg = (
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        )

        x = (
            tf_msg.transform.translation.x
        )

        y = (
            tf_msg.transform.translation.y
        )

        yaw = yaw_from_quaternion(
            tf_msg.transform.rotation
        )

        return (
            x,
            y,
            yaw,
        )

    # =========================================================
    # Origin yaw
    # =========================================================

    def map_origin_yaw(self):

        q = (
            self.map_msg
            .info
            .origin
            .orientation
        )

        return yaw_from_quaternion(
            q
        )

    # =========================================================
    # Planner construction
    # =========================================================

    def create_planner(self):

        return HybridAStarPlanner(
            obstacle_grid=(
                self.inflated_grid
            ),

            resolution=float(
                self.map_msg.info.resolution
            ),

            origin_x=float(
                self.map_msg
                .info
                .origin
                .position
                .x
            ),

            origin_y=float(
                self.map_msg
                .info
                .origin
                .position
                .y
            ),

            origin_yaw=(
                self.map_origin_yaw()
            ),

            xy_step=float(
                self.get_parameter(
                    "xy_step"
                ).value
            ),

            yaw_resolution_deg=float(
                self.get_parameter(
                    "yaw_resolution_deg"
                ).value
            ),

            min_turning_radius=float(
                self.get_parameter(
                    "min_turning_radius"
                ).value
            ),

            collision_check_step=float(
                self.get_parameter(
                    "collision_check_step"
                ).value
            ),

            allow_reverse=bool(
                self.get_parameter(
                    "allow_reverse"
                ).value
            ),

            allow_in_place_rotation=bool(
                self.get_parameter(
                    "allow_in_place_rotation"
                ).value
            ),

            spin_step_deg=float(
                self.get_parameter(
                    "spin_step_deg"
                ).value
            ),

            reverse_penalty=float(
                self.get_parameter(
                    "reverse_penalty"
                ).value
            ),

            gear_switch_penalty=float(
                self.get_parameter(
                    "gear_switch_penalty"
                ).value
            ),

            turning_penalty=float(
                self.get_parameter(
                    "turning_penalty"
                ).value
            ),

            spin_penalty=float(
                self.get_parameter(
                    "spin_penalty"
                ).value
            ),

            yaw_heuristic_weight=float(
                self.get_parameter(
                    "yaw_heuristic_weight"
                ).value
            ),

            heuristic_weight=float(
                self.get_parameter(
                    "heuristic_weight"
                ).value
            ),

            goal_xy_tolerance=float(
                self.get_parameter(
                    "goal_xy_tolerance"
                ).value
            ),

            goal_yaw_tolerance_deg=float(
                self.get_parameter(
                    "goal_yaw_tolerance_deg"
                ).value
            ),

            search_margin=float(
                self.get_parameter(
                    "search_margin"
                ).value
            ),

            max_expansions=int(
                self.get_parameter(
                    "max_expansions"
                ).value
            ),
        )

    # =========================================================
    # Plan service
    # =========================================================

    def plan_callback(
        self,
        request,
        response,
    ):

        del request

        self.publish_available(
            False
        )

        if (
            self.map_msg is None
            or self.inflated_grid is None
        ):

            self.publish_status(
                "MAP_UNAVAILABLE"
            )

            response.success = False

            response.message = (
                "Occupancy map unavailable."
            )

            return response

        if (
            self.require_aruco_lock
            and not self.aruco_locked
        ):

            self.publish_status(
                "ARUCO_NOT_LOCKED"
            )

            response.success = False

            response.message = (
                "ArUco ID=0 is not locked."
            )

            return response

        try:

            start = self.get_robot_pose()

        except TransformException as exc:

            self.publish_status(
                "TF_UNAVAILABLE"
            )

            response.success = False

            response.message = (
                f"Robot pose unavailable: {exc}"
            )

            return response

        goal = (
            self.goal_x,
            self.goal_y,
            self.goal_yaw,
        )

        self.publish_goal()

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Hybrid A* planning request"
        )

        self.get_logger().info(
            "START:"
        )

        self.get_logger().info(
            f"x={start[0]:.3f}, "
            f"y={start[1]:.3f}, "
            f"yaw={start[2]:.3f}"
        )

        self.get_logger().info(
            "GOAL:"
        )

        self.get_logger().info(
            f"x={goal[0]:.3f}, "
            f"y={goal[1]:.3f}, "
            f"yaw={goal[2]:.3f}"
        )

        self.get_logger().info(
            "========================================"
        )

        self.publish_status(
            "PLANNING"
        )

        planner = (
            self.create_planner()
        )

        start_time = (
            time.perf_counter()
        )

        result = planner.plan(
            start,
            goal,
        )

        elapsed = (
            time.perf_counter()
            - start_time
        )

        if not result.success:

            status = (
                "FAILED_"
                + result.reason
            )

            self.publish_status(
                status
            )

            self.get_logger().error(
                "Hybrid A* failed:"
            )

            self.get_logger().error(
                f"reason="
                f"{result.reason}"
            )

            self.get_logger().error(
                f"expanded="
                f"{result.expanded_nodes}"
            )

            self.get_logger().error(
                f"time="
                f"{elapsed:.3f} s"
            )

            response.success = False

            response.message = (
                result.reason
            )

            return response

        path_msg = Path()

        path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        path_msg.header.frame_id = (
            self.map_frame
        )

        for (
            x,
            y,
            yaw,
            direction,
        ) in result.path:

            pose = PoseStamped()

            pose.header = (
                path_msg.header
            )

            pose.pose.position.x = float(
                x
            )

            pose.pose.position.y = float(
                y
            )

            pose.pose.position.z = 0.0

            qx, qy, qz, qw = (
                quaternion_from_yaw(
                    yaw
                )
            )

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(
                pose
            )

        self.path_pub.publish(
            path_msg
        )

        self.publish_available(
            True
        )

        self.publish_status(
            "SUCCEEDED"
        )

        # ---------------------------------------------
        # Path length
        # ---------------------------------------------

        path_length = 0.0

        for i in range(
            1,
            len(result.path),
        ):

            x0, y0, _, _ = (
                result.path[i - 1]
            )

            x1, y1, _, _ = (
                result.path[i]
            )

            path_length += math.hypot(
                x1 - x0,
                y1 - y0,
            )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Hybrid A* SUCCESS"
        )

        self.get_logger().info(
            f"path poses="
            f"{len(result.path)}"
        )

        self.get_logger().info(
            f"path length="
            f"{path_length:.3f} m"
        )

        self.get_logger().info(
            f"expanded nodes="
            f"{result.expanded_nodes}"
        )

        self.get_logger().info(
            f"search cost="
            f"{result.cost:.3f}"
        )

        self.get_logger().info(
            f"planning time="
            f"{elapsed:.3f} s"
        )

        self.get_logger().info(
            "========================================"
        )

        response.success = True

        response.message = (
            "Hybrid A* parking path planned."
        )

        return response

    # =========================================================
    # Clear service
    # =========================================================

    def clear_callback(
        self,
        request,
        response,
    ):

        del request

        empty_path = Path()

        empty_path.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        empty_path.header.frame_id = (
            self.map_frame
        )

        self.path_pub.publish(
            empty_path
        )

        self.publish_available(
            False
        )

        self.publish_status(
            "CLEARED"
        )

        response.success = True

        response.message = (
            "Hybrid parking path cleared."
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = ParkingPlannerNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
