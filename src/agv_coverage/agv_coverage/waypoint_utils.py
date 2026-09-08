# -*- coding: utf-8 -*-
"""
覆盖路径航点抽稀工具。

完整 /coverage_path 是给 RViz 显示的，点非常密。
真正给 Nav2 执行时，必须抽稀成关键航点。

本文件只做算法处理，不直接依赖 ROS 节点。
"""

from __future__ import annotations

import math
from typing import List, Tuple


WorldPoint = Tuple[float, float]


def distance(a: WorldPoint, b: WorldPoint) -> float:
    """计算两个 map 坐标点之间的欧氏距离。"""

    return math.hypot(a[0] - b[0], a[1] - b[1])


def heading(a: WorldPoint, b: WorldPoint) -> float:
    """计算从 a 指向 b 的航向角。"""

    return math.atan2(b[1] - a[1], b[0] - a[0])


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def choose_path_direction(
    points: List[WorldPoint],
    start_x: float,
    start_y: float,
    enabled: bool,
) -> List[WorldPoint]:
    """
    根据机器人初始位置选择路径方向。

    如果路径末端比路径起点更靠近机器人，就反转整条路径。
    这样机器人可以从更近的一端开始覆盖。
    """

    if not enabled or len(points) < 2:
        return points

    robot_start = (start_x, start_y)

    dist_to_first = distance(robot_start, points[0])
    dist_to_last = distance(robot_start, points[-1])

    if dist_to_last < dist_to_first:
        return list(reversed(points))

    return points


def extract_key_waypoints(
    points: List[WorldPoint],
    waypoint_spacing: float,
    min_waypoint_distance: float,
    turn_angle_threshold_deg: float,
) -> List[WorldPoint]:
    """
    从完整覆盖路径中提取关键航点。

    保留规则：
        1. 第一个点一定保留；
        2. 最后一个点一定保留；
        3. 转角较大的点保留；
        4. 长直线中每隔 waypoint_spacing 保留一个点；
        5. 距离上一个航点太近的点不保留。
    """

    if len(points) <= 2:
        return points.copy()

    turn_threshold = math.radians(turn_angle_threshold_deg)

    waypoints: List[WorldPoint] = [points[0]]
    last_kept = points[0]
    distance_since_last = 0.0

    for i in range(1, len(points) - 1):
        prev_point = points[i - 1]
        current_point = points[i]
        next_point = points[i + 1]

        step_dist = distance(prev_point, current_point)
        distance_since_last += step_dist

        # 当前点距离上一个已保留航点太近，则不保留
        if distance(last_kept, current_point) < min_waypoint_distance:
            continue

        heading_in = heading(prev_point, current_point)
        heading_out = heading(current_point, next_point)
        turn_angle = abs(normalize_angle(heading_out - heading_in))

        is_turn_point = turn_angle >= turn_threshold
        is_spacing_point = distance_since_last >= waypoint_spacing

        if is_turn_point or is_spacing_point:
            waypoints.append(current_point)
            last_kept = current_point
            distance_since_last = 0.0

    # 最后一个点必须保留
    if distance(waypoints[-1], points[-1]) >= min_waypoint_distance:
        waypoints.append(points[-1])
    else:
        waypoints[-1] = points[-1]

    return waypoints


def split_waypoint_batches(
    waypoints: List[WorldPoint],
    max_waypoints_per_batch: int,
) -> List[List[WorldPoint]]:
    """
    将航点分批。

    后续执行 NavigateThroughPoses 时，每批发送一组航点。
    """

    if max_waypoints_per_batch <= 0:
        return [waypoints]

    batches = []
    for i in range(0, len(waypoints), max_waypoints_per_batch):
        batches.append(waypoints[i:i + max_waypoints_per_batch])

    return batches


def compute_path_length(points: List[WorldPoint]) -> float:
    """计算路径长度。"""

    if len(points) < 2:
        return 0.0

    total = 0.0

    for i in range(1, len(points)):
        total += distance(points[i - 1], points[i])

    return total
