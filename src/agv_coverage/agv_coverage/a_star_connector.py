# -*- coding: utf-8 -*-
"""
局部 A* 连接器。

作用：
当相邻两条牛耕扫描线之间无法直线连接时，
在膨胀后的自由栅格地图上搜索一条安全连接路径。
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np


GridPoint = Tuple[int, int]


def nearest_free_cell(
    free_mask: np.ndarray,
    point: GridPoint,
    max_radius: int = 10,
) -> Optional[GridPoint]:
    """
    如果输入点恰好落在障碍物或地图外，寻找附近最近的自由栅格。
    """

    height, width = free_mask.shape
    x0, y0 = point

    if 0 <= x0 < width and 0 <= y0 < height and free_mask[y0, x0]:
        return point

    queue = deque()
    visited = set()

    queue.append((x0, y0, 0))
    visited.add((x0, y0))

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while queue:
        x, y, r = queue.popleft()

        if r > max_radius:
            continue

        if 0 <= x < width and 0 <= y < height and free_mask[y, x]:
            return x, y

        for dx, dy in neighbors:
            nx = x + dx
            ny = y + dy

            if (nx, ny) in visited:
                continue

            visited.add((nx, ny))
            queue.append((nx, ny, r + 1))

    return None


def heuristic(a: GridPoint, b: GridPoint) -> float:
    """欧氏距离启发函数。"""

    return math.hypot(a[0] - b[0], a[1] - b[1])


def reconstruct_path(
    came_from: Dict[GridPoint, GridPoint],
    current: GridPoint,
) -> List[GridPoint]:
    """根据 came_from 字典回溯 A* 路径。"""

    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar_grid(
    free_mask: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    max_search_cells: int = 50000,
) -> Optional[List[GridPoint]]:
    """
    在栅格地图上执行 8 邻域 A*。

    free_mask[y, x] = True 表示可通行。
    start, goal 使用 (x, y) 顺序。
    """

    height, width = free_mask.shape

    start_free = nearest_free_cell(free_mask, start)
    goal_free = nearest_free_cell(free_mask, goal)

    if start_free is None or goal_free is None:
        return None

    start = start_free
    goal = goal_free

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))

    came_from: Dict[GridPoint, GridPoint] = {}
    g_score: Dict[GridPoint, float] = {start: 0.0}

    closed = set()
    expanded_count = 0

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        if current == goal:
            return reconstruct_path(came_from, current)

        closed.add(current)
        expanded_count += 1

        if expanded_count > max_search_cells:
            return None

        cx, cy = current

        for dx, dy, cost in neighbors:
            nx = cx + dx
            ny = cy + dy

            if nx < 0 or nx >= width or ny < 0 or ny >= height:
                continue

            if not free_mask[ny, nx]:
                continue

            # 防止斜向穿过障碍物角落
            if dx != 0 and dy != 0:
                if not free_mask[cy, nx] or not free_mask[ny, cx]:
                    continue

            neighbor = (nx, ny)
            tentative_g = g_score[current] + cost

            if tentative_g >= g_score.get(neighbor, float("inf")):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score = tentative_g + heuristic(neighbor, goal)
            heapq.heappush(open_heap, (f_score, neighbor))

    return None
