# -*- coding: utf-8 -*-
"""
地图工具函数。

本文件只处理 OccupancyGrid 的栅格数据，不直接依赖 ROS 节点。
这样做的好处是：算法和 ROS 通信解耦，后续更容易测试和修改。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import List, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid

# 第一部分：数据结构定义（DTO，数据传输对象）
@dataclass
class GridMapInfo:   # 这是一个 数据容器（DataClass），专门用来存储地图的固定几何参数。
    """保存 OccupancyGrid 的基础几何信息。"""

    width: int    # 像素宽高
    height: int
    resolution: float    # 物理分辨率（m/pixel）
    origin_x: float
    origin_y: float         # 地图原点在物理世界坐标系下的位置

# 第二部分：地图数据预处理（将 ROS 消息转化为数学矩阵）
# 输入：ROS 的 OccupancyGrid 消息
def occupancy_grid_to_free_mask(    
    msg: OccupancyGrid,
    occupied_threshold: int,
    unknown_as_obstacle: bool,
) -> Tuple[np.ndarray, GridMapInfo]:
    """
    将 nav_msgs/OccupancyGrid 转换为自由区域布尔矩阵。

    返回：
        free_mask[y, x] = True  表示该栅格可通行
        free_mask[y, x] = False 表示该栅格不可通行
    """

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    data = np.array(msg.data, dtype=np.int16).reshape((height, width))

    # OccupancyGrid 中：
    # 0 通常表示自由区域
    # 100 通常表示障碍物
    # -1 表示未知区域
    # 根据 unknown_as_obstacle 标志决定如何处理未知区域（-1）
	# 如果为 True，把未知区域看作障碍物（不可通行），以免机器人闯进未建图区域。
	# 如果为 False，把未知区域当作自由区（虽然这是冒险做法，但有时为了路径连通性不得已而为之）。
    if unknown_as_obstacle:
        free_mask = (data >= 0) & (data < occupied_threshold)
    else:
        free_mask = data < occupied_threshold

    info = GridMapInfo(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
    )

# 输出：一个布尔矩阵 free_mask，True 表示“可通行”，False 表示“不可通行”。同时返回 GridMapInfo 元数据。
    return free_mask, info


def dilate_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """
    对布尔矩阵进行圆形膨胀。

    mask=True 的区域会向外扩展 radius_cells 个栅格。
    这里不用 scipy，避免额外依赖。
    """

    if radius_cells <= 0:
        return mask.copy()

    height, width = mask.shape
    result = mask.copy()

    offsets = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                offsets.append((dy, dx))

    for dy, dx in offsets:
        shifted = np.zeros_like(mask, dtype=bool)

        src_y0 = max(0, -dy)
        src_y1 = height - max(0, dy)
        src_x0 = max(0, -dx)
        src_x1 = width - max(0, dx)

        dst_y0 = max(0, dy)
        dst_y1 = height - max(0, -dy)
        dst_x0 = max(0, dx)
        dst_x1 = width - max(0, -dx)

        shifted[dst_y0:dst_y1, dst_x0:dst_x1] = mask[src_y0:src_y1, src_x0:src_x1]
        result |= shifted

    return result


def inflate_obstacles(
    free_mask: np.ndarray,
    resolution: float,
    inflation_radius_m: float,
) -> np.ndarray:
    """
    对障碍物和未知区域进行膨胀，返回安全自由区域。

    输入 free_mask=True 表示自由。
    先取反得到 obstacle_mask，再对 obstacle_mask 膨胀。
    膨胀后的区域不可通行。
    """

    radius_cells = int(math.ceil(inflation_radius_m / resolution))
    obstacle_mask = ~free_mask
    inflated_obstacle = dilate_mask(obstacle_mask, radius_cells)
    safe_free = ~inflated_obstacle
    return safe_free


def largest_connected_component(free_mask: np.ndarray) -> np.ndarray:
    """
    提取最大的自由连通区域。

    这样可以去掉角落中的小碎片自由区，避免生成不可执行的覆盖路径。
    """

    height, width = free_mask.shape
    visited = np.zeros_like(free_mask, dtype=bool)

    best_cells: List[Tuple[int, int]] = []

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for y in range(height):
        for x in range(width):
            if not free_mask[y, x] or visited[y, x]:
                continue

            queue = deque()
            queue.append((y, x))
            visited[y, x] = True
            cells = []

            while queue:
                cy, cx = queue.popleft()
                cells.append((cy, cx))

                for dy, dx in neighbors:
                    ny = cy + dy
                    nx = cx + dx

                    if ny < 0 or ny >= height or nx < 0 or nx >= width:
                        continue
                    if visited[ny, nx] or not free_mask[ny, nx]:
                        continue

                    visited[ny, nx] = True
                    queue.append((ny, nx))

            if len(cells) > len(best_cells):
                best_cells = cells

    result = np.zeros_like(free_mask, dtype=bool)
    for y, x in best_cells:
        result[y, x] = True

    return result


def grid_to_world(info: GridMapInfo, gx: int, gy: int) -> Tuple[float, float]:
    """
    栅格坐标转 map 世界坐标。

    gx 是列索引 x，gy 是行索引 y。
    """

    wx = info.origin_x + (gx + 0.5) * info.resolution
    wy = info.origin_y + (gy + 0.5) * info.resolution
    return wx, wy


def world_to_grid(info: GridMapInfo, wx: float, wy: float) -> Tuple[int, int]:
    """
    map 世界坐标转栅格坐标。
    """

    gx = int(math.floor((wx - info.origin_x) / info.resolution))
    gy = int(math.floor((wy - info.origin_y) / info.resolution))
    return gx, gy


def inside_grid(info: GridMapInfo, gx: int, gy: int) -> bool:
    """判断栅格坐标是否在地图范围内。"""

    return 0 <= gx < info.width and 0 <= gy < info.height


def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """
    Bresenham 直线算法。

    用于检查两个栅格点之间的直线路径是否穿过障碍物。
    返回 [(x, y), ...]
    """

    points = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy
    x = x0
    y = y0

    while True:
        points.append((x, y))

        if x == x1 and y == y1:
            break

        e2 = 2 * err

        if e2 > -dy:
            err -= dy
            x += sx

        if e2 < dx:
            err += dx
            y += sy

    return points


def is_line_free(
    free_mask: np.ndarray,
    start_xy: Tuple[int, int],
    goal_xy: Tuple[int, int],
) -> bool:
    """
    判断两个栅格点之间的直线是否完全位于自由区域。
    """

    height, width = free_mask.shape

    for x, y in bresenham_line(start_xy[0], start_xy[1], goal_xy[0], goal_xy[1]):
        if x < 0 or x >= width or y < 0 or y >= height:
            return False
        if not free_mask[y, x]:
            return False

    return True


def path_length(points_xy: List[Tuple[float, float]]) -> float:
    """计算路径长度，单位 m。"""

    if len(points_xy) < 2:
        return 0.0

    length = 0.0
    for i in range(1, len(points_xy)):
        x0, y0 = points_xy[i - 1]
        x1, y1 = points_xy[i]
        length += math.hypot(x1 - x0, y1 - y0)

    return length
