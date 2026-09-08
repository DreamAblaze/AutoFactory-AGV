# -*- coding: utf-8 -*-
"""
agv_coverage ROS2 节点。

阶段 5C 功能：
    1. 订阅 /map；
    2. 生成完整 /coverage_path；
    3. 抽稀并发布 /coverage_waypoints；
    4. 提供 /start_coverage 服务生成路径；
    5. 提供 /execute_coverage 服务；
    6. 通过 Nav2 /navigate_through_poses 分批执行覆盖航点。

注意：
    本节点不直接发布 /cmd_vel。
    速度仍然由 Nav2 的 RPP 控制器输出。
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from agv_coverage.grid_coverage_planner import (
    CoveragePlannerConfig,
    GridCoveragePlanner,
)
from agv_coverage.waypoint_utils import (
    choose_path_direction,
    extract_key_waypoints,
    split_waypoint_batches,
)


WorldPoint = Tuple[float, float]


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class CoverageNode(Node):
    """牛耕式全覆盖路径规划与执行节点。"""

    def __init__(self):
        super().__init__("coverage_node")

        # -----------------------------
        # 1. 参数声明
        # -----------------------------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("coverage_path_topic", "/coverage_path")
        self.declare_parameter("coverage_waypoints_topic", "/coverage_waypoints")
        self.declare_parameter("coverage_status_topic", "/coverage_status")
        self.declare_parameter("coverage_progress_topic", "/coverage_progress")

        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("unknown_as_obstacle", True)
        self.declare_parameter("use_largest_connected_region", True)

        self.declare_parameter("obstacle_inflation_radius", 0.25)
        self.declare_parameter("sweep_spacing", 0.40)
        self.declare_parameter("min_segment_length", 0.60)
        self.declare_parameter("sample_step", 0.05)

        self.declare_parameter("use_astar_connection", True)
        self.declare_parameter("astar_max_search_cells", 50000)

        self.declare_parameter("waypoint_spacing", 1.50)
        self.declare_parameter("min_waypoint_distance", 0.80)
        self.declare_parameter("turn_angle_threshold_deg", 135.0)
        self.declare_parameter("max_waypoints_per_batch", 30)

        self.declare_parameter("optimize_start_direction", True)
        self.declare_parameter("start_x", 8.738)
        self.declare_parameter("start_y", 7.489)

        self.declare_parameter("coverage_radius", 0.25)
        self.declare_parameter("target_coverage_rate", 0.90)

        self.declare_parameter("execute_after_plan", False)
        self.declare_parameter("navigate_action_name", "/navigate_through_poses")
        self.declare_parameter("execute_service_name", "/execute_coverage")
        self.declare_parameter("max_batches_to_execute", 1)
        self.declare_parameter("start_batch_index", 0)

        # -----------------------------
        # 2. 参数读取
        # -----------------------------
        self.map_topic = self.get_parameter("map_topic").value
        self.coverage_path_topic = self.get_parameter("coverage_path_topic").value
        self.coverage_waypoints_topic = self.get_parameter("coverage_waypoints_topic").value
        self.coverage_status_topic = self.get_parameter("coverage_status_topic").value
        self.coverage_progress_topic = self.get_parameter("coverage_progress_topic").value

        self.waypoint_spacing = float(self.get_parameter("waypoint_spacing").value)
        self.min_waypoint_distance = float(self.get_parameter("min_waypoint_distance").value)
        self.turn_angle_threshold_deg = float(self.get_parameter("turn_angle_threshold_deg").value)
        self.max_waypoints_per_batch = int(self.get_parameter("max_waypoints_per_batch").value)

        self.optimize_start_direction = bool(self.get_parameter("optimize_start_direction").value)
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)

        self.navigate_action_name = self.get_parameter("navigate_action_name").value
        self.execute_service_name = self.get_parameter("execute_service_name").value
        self.max_batches_to_execute = int(self.get_parameter("max_batches_to_execute").value)
        self.start_batch_index = int(self.get_parameter("start_batch_index").value)
        self.execute_after_plan = bool(self.get_parameter("execute_after_plan").value)

        config = CoveragePlannerConfig(
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
            unknown_as_obstacle=bool(self.get_parameter("unknown_as_obstacle").value),
            use_largest_connected_region=bool(
                self.get_parameter("use_largest_connected_region").value
            ),
            obstacle_inflation_radius=float(
                self.get_parameter("obstacle_inflation_radius").value
            ),
            sweep_spacing=float(self.get_parameter("sweep_spacing").value),
            min_segment_length=float(self.get_parameter("min_segment_length").value),
            sample_step=float(self.get_parameter("sample_step").value),
            use_astar_connection=bool(self.get_parameter("use_astar_connection").value),
            astar_max_search_cells=int(
                self.get_parameter("astar_max_search_cells").value
            ),
            coverage_radius=float(self.get_parameter("coverage_radius").value),
        )

        self.planner = GridCoveragePlanner(config)

        # -----------------------------
        # 3. QoS
        # -----------------------------
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # -----------------------------
        # 4. ROS 通信对象
        # -----------------------------
        self.map_msg: Optional[OccupancyGrid] = None
        self.cached_waypoints: List[WorldPoint] = []
        self.cached_batches: List[List[WorldPoint]] = []
        self.is_executing = False
        self.current_batch_index = 0
        self.final_batch_index_exclusive = 0

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.on_map,
            map_qos,
        )

        self.path_pub = self.create_publisher(
            Path,
            self.coverage_path_topic,
            transient_qos,
        )

        self.waypoints_pub = self.create_publisher(
            PoseArray,
            self.coverage_waypoints_topic,
            transient_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            self.coverage_status_topic,
            transient_qos,
        )

        self.progress_pub = self.create_publisher(
            Float32,
            self.coverage_progress_topic,
            transient_qos,
        )

        self.start_srv = self.create_service(
            Trigger,
            "/start_coverage",
            self.on_start_coverage,
        )

        self.execute_srv = self.create_service(
            Trigger,
            self.execute_service_name,
            self.on_execute_coverage,
        )

        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.navigate_action_name,
        )

        self.get_logger().info("agv_coverage 节点已启动。")
        self.get_logger().info(f"等待地图话题：{self.map_topic}")
        self.get_logger().info(f"Nav2 Action：{self.navigate_action_name}")
        self.get_logger().info(f"执行服务：{self.execute_service_name}")
        self.get_logger().info("先调用 /start_coverage 生成路径，再调用 /execute_coverage 执行。")

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_start_coverage(self, request, response):
        del request

        if self.map_msg is None:
            response.success = False
            response.message = "尚未收到 /map，无法生成覆盖路径。"
            self.publish_status(response.message)
            return response

        self.publish_status("开始生成牛耕式覆盖路径。")
        self.get_logger().info("开始生成牛耕式覆盖路径...")

        try:
            result = self.planner.plan(self.map_msg)
        except Exception as exc:
            response.success = False
            response.message = f"覆盖路径生成失败：{exc}"
            self.publish_status(response.message)
            self.get_logger().error(response.message)
            return response

        if not result.world_path:
            response.success = False
            response.message = result.message
            self.publish_status(response.message)
            return response

        optimized_path = choose_path_direction(
            result.world_path,
            start_x=self.start_x,
            start_y=self.start_y,
            enabled=self.optimize_start_direction,
        )

        self.publish_coverage_path(optimized_path)

        waypoints = extract_key_waypoints(
            optimized_path,
            waypoint_spacing=self.waypoint_spacing,
            min_waypoint_distance=self.min_waypoint_distance,
            turn_angle_threshold_deg=self.turn_angle_threshold_deg,
        )

        batches = split_waypoint_batches(
            waypoints,
            max_waypoints_per_batch=self.max_waypoints_per_batch,
        )

        self.cached_waypoints = waypoints
        self.cached_batches = batches

        self.publish_waypoints(waypoints)

        progress_msg = Float32()
        progress_msg.data = float(result.coverage_rate * 100.0)
        self.progress_pub.publish(progress_msg)

        summary = (
            f"{result.message} "
            f"完整路径点数={len(result.world_path)}, "
            f"抽稀航点数={len(waypoints)}, "
            f"预计批次数={len(batches)}, "
            f"每批最多={self.max_waypoints_per_batch}, "
            f"扫描线段数={result.segment_count}, "
            f"跳过线段={result.skipped_segments}, "
            f"路径长度={result.path_length_m:.2f} m, "
            f"估计覆盖率={result.coverage_rate * 100.0:.1f}%."
        )

        if waypoints:
            summary += (
                f" 起点=({waypoints[0][0]:.2f}, {waypoints[0][1]:.2f}), "
                f"终点=({waypoints[-1][0]:.2f}, {waypoints[-1][1]:.2f})."
            )

        self.publish_status(summary)
        self.get_logger().info(summary)

        response.success = True
        response.message = summary

        if self.execute_after_plan:
            self.get_logger().warn(
                "execute_after_plan=true，自动执行已关闭建议。请手动调用 /execute_coverage。"
            )

        return response

    def on_execute_coverage(self, request, response):
        del request

        if self.is_executing:
            response.success = False
            response.message = "覆盖任务正在执行中，不能重复启动。"
            self.publish_status(response.message)
            return response

        if not self.cached_batches:
            response.success = False
            response.message = "尚未生成覆盖航点，请先调用 /start_coverage。"
            self.publish_status(response.message)
            return response

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = (
                f"Nav2 Action Server 不可用：{self.navigate_action_name}。"
            )
            self.publish_status(response.message)
            return response

        total_batches = len(self.cached_batches)
        start_index = max(0, min(self.start_batch_index, total_batches - 1))

        if self.max_batches_to_execute <= 0:
            end_index = total_batches
        else:
            end_index = min(total_batches, start_index + self.max_batches_to_execute)

        self.current_batch_index = start_index
        self.final_batch_index_exclusive = end_index
        self.is_executing = True

        msg = (
            f"开始执行覆盖任务：从第 {start_index + 1} 批开始，"
            f"执行到第 {end_index} 批，共 {end_index - start_index} 批。"
        )
        self.publish_status(msg)
        self.get_logger().info(msg)

        self.send_current_batch()

        response.success = True
        response.message = msg
        return response

    def send_current_batch(self):
        if self.current_batch_index >= self.final_batch_index_exclusive:
            self.is_executing = False
            msg = "本次覆盖执行完成。"
            self.publish_status(msg)
            self.get_logger().info(msg)
            return

        batch = self.cached_batches[self.current_batch_index]

        if not batch:
            self.current_batch_index += 1
            self.send_current_batch()
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.points_to_pose_stamped_list(batch)

        # Humble 中该字段通常存在，保持为空表示使用默认行为树
        if hasattr(goal_msg, "behavior_tree"):
            goal_msg.behavior_tree = ""

        msg = (
            f"发送第 {self.current_batch_index + 1} 批航点，"
            f"本批 {len(batch)} 个航点。"
        )
        self.publish_status(msg)
        self.get_logger().info(msg)

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.is_executing = False
            msg = f"第 {self.current_batch_index + 1} 批航点被 Nav2 拒绝。"
            self.publish_status(msg)
            self.get_logger().error(msg)
            return

        self.get_logger().info(f"第 {self.current_batch_index + 1} 批航点已被 Nav2 接受。")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = f"第 {self.current_batch_index + 1} 批航点执行成功。"
            self.publish_status(msg)
            self.get_logger().info(msg)

            self.current_batch_index += 1
            self.send_current_batch()
            return

        self.is_executing = False
        msg = (
            f"第 {self.current_batch_index + 1} 批航点执行失败，"
            f"Action status={status}。"
        )
        self.publish_status(msg)
        self.get_logger().error(msg)

    def navigation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # Humble 版本一般有 number_of_poses_remaining 字段
        if hasattr(feedback, "number_of_poses_remaining"):
            remaining = feedback.number_of_poses_remaining
            self.get_logger().info(
                f"当前批次剩余航点数：{remaining}",
                throttle_duration_sec=5.0,
            )

    def points_to_pose_stamped_list(self, points: List[WorldPoint]) -> List[PoseStamped]:
        poses: List[PoseStamped] = []

        now = self.get_clock().now().to_msg()

        for i, point in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0

            yaw = self.compute_yaw(points, i)
            qz, qw = yaw_to_quaternion(yaw)

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            poses.append(pose)

        return poses

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_coverage_path(self, points: List[WorldPoint]):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0

            yaw = self.compute_yaw(points, i)
            qz, qw = yaw_to_quaternion(yaw)

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_waypoints(self, waypoints: List[WorldPoint]):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for i, point in enumerate(waypoints):
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = 0.05

            yaw = self.compute_yaw(waypoints, i)
            qz, qw = yaw_to_quaternion(yaw)

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = qz
            pose.orientation.w = qw

            pose_array.poses.append(pose)

        self.waypoints_pub.publish(pose_array)

    @staticmethod
    def compute_yaw(points: List[WorldPoint], index: int) -> float:
        if len(points) < 2:
            return 0.0

        if index < len(points) - 1:
            x0, y0 = points[index]
            x1, y1 = points[index + 1]
        else:
            x0, y0 = points[index - 1]
            x1, y1 = points[index]

        return math.atan2(y1 - y0, x1 - x0)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
