# -*- coding: utf-8 -*-
"""
牛耕式覆盖路径规划核心。

输入：
    nav_msgs/OccupancyGrid

输出：
    覆盖路径点，坐标系为 map。

算法步骤：
    1. OccupancyGrid 转自由栅格；
    2. 障碍物膨胀；
    3. 最大自由连通区域提取；
    4. PCA 估计自由空间主方向；
    5. 生成平行扫描线；
    6. 提取自由线段；
    7. 蛇形排序；
    8. 用直线或 A* 连接相邻线段；
    9. 计算覆盖率和路径长度。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid

from agv_coverage.a_star_connector import astar_grid
from agv_coverage.map_utils import (
    GridMapInfo,
    bresenham_line,
    dilate_mask,
    grid_to_world,
    inflate_obstacles,
    inside_grid,
    is_line_free,
    largest_connected_component,
    occupancy_grid_to_free_mask,
    path_length,
    world_to_grid,
)


WorldPoint = Tuple[float, float]
GridPoint = Tuple[int, int]


@dataclass
class CoveragePlannerConfig:
    """覆盖规划参数。"""

    occupied_threshold: int
    unknown_as_obstacle: bool
    use_largest_connected_region: bool
    obstacle_inflation_radius: float
    sweep_spacing: float
    min_segment_length: float
    sample_step: float
    use_astar_connection: bool
    astar_max_search_cells: int
    coverage_radius: float


@dataclass
class CoveragePlanResult:
    """覆盖规划结果。"""

    world_path: List[WorldPoint]
    grid_path: List[GridPoint]
    safe_free_mask: np.ndarray
    coverage_rate: float
    path_length_m: float
    skipped_segments: int
    segment_count: int
    message: str


class GridCoveragePlanner:
    """栅格牛耕式覆盖规划器。"""

    def __init__(self, config: CoveragePlannerConfig):
        self.config = config

    def plan(self, map_msg: OccupancyGrid) -> CoveragePlanResult:
        """
        生成完整覆盖路径。
        """

        free_mask, info = occupancy_grid_to_free_mask(
            map_msg,
            occupied_threshold=self.config.occupied_threshold,
            unknown_as_obstacle=self.config.unknown_as_obstacle,
        )

        safe_free = inflate_obstacles(
            free_mask=free_mask,
            resolution=info.resolution,
            inflation_radius_m=self.config.obstacle_inflation_radius,
        )

        if self.config.use_largest_connected_region:
            safe_free = largest_connected_component(safe_free)

        free_y, free_x = np.nonzero(safe_free)

        if len(free_x) < 20:
            return CoveragePlanResult(
                world_path=[],
                grid_path=[],
                safe_free_mask=safe_free,
                coverage_rate=0.0,
                path_length_m=0.0,
                skipped_segments=0,
                segment_count=0,
                message="安全自由区域太少，无法生成覆盖路径。",
            )

        main_dir, side_dir, center = self._estimate_pca_direction(info, free_x, free_y)

        segments = self._generate_sweep_segments(
            info=info,
            safe_free=safe_free,
            main_dir=main_dir,
            side_dir=side_dir,
            center=center,
            free_x=free_x,
            free_y=free_y,
        )

        if not segments:
            return CoveragePlanResult(
                world_path=[],
                grid_path=[],
                safe_free_mask=safe_free,
                coverage_rate=0.0,
                path_length_m=0.0,
                skipped_segments=0,
                segment_count=0,
                message="没有生成有效扫描线段，请检查扫描间距或膨胀半径。",
            )

        ordered_segments = self._snake_order_segments(segments)

        grid_path, skipped = self._connect_segments(
            safe_free=safe_free,
            ordered_segments=ordered_segments,
        )

        world_path = [grid_to_world(info, gx, gy) for gx, gy in grid_path]

        coverage_rate = self._estimate_coverage_rate(
            safe_free=safe_free,
            grid_path=grid_path,
            info=info,
        )

        length_m = path_length(world_path)

        return CoveragePlanResult(
            world_path=world_path,
            grid_path=grid_path,
            safe_free_mask=safe_free,
            coverage_rate=coverage_rate,
            path_length_m=length_m,
            skipped_segments=skipped,
            segment_count=len(ordered_segments),
            message="覆盖路径生成成功。",
        )

    def _estimate_pca_direction(
        self,
        info: GridMapInfo,
        free_x: np.ndarray,
        free_y: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        使用 PCA 估计自由区域主方向。
        """

        wx = info.origin_x + (free_x.astype(float) + 0.5) * info.resolution
        wy = info.origin_y + (free_y.astype(float) + 0.5) * info.resolution

        points = np.stack([wx, wy], axis=1)
        center = np.mean(points, axis=0)

        centered = points - center
        cov = np.cov(centered.T)

        eigen_values, eigen_vectors = np.linalg.eigh(cov)
        main_dir = eigen_vectors[:, int(np.argmax(eigen_values))]

        # 统一方向，避免每次启动方向随机反过来
        if main_dir[0] < 0:
            main_dir = -main_dir

        main_dir = main_dir / np.linalg.norm(main_dir)

        # 垂直于主方向的方向，用于扫描线逐条平移
        side_dir = np.array([-main_dir[1], main_dir[0]])
        side_dir = side_dir / np.linalg.norm(side_dir)

        return main_dir, side_dir, center

    def _generate_sweep_segments(
        self,
        info: GridMapInfo,
        safe_free: np.ndarray,
        main_dir: np.ndarray,
        side_dir: np.ndarray,
        center: np.ndarray,
        free_x: np.ndarray,
        free_y: np.ndarray,
    ) -> List[List[List[GridPoint]]]:
        """
        生成扫描线段。

        返回结构：
            [
                [segment1, segment2],   # 第 1 条扫描线上的多个自由线段
                [segment3],             # 第 2 条扫描线
                ...
            ]
        每个 segment 是 [(gx, gy), ...]
        """

        wx = info.origin_x + (free_x.astype(float) + 0.5) * info.resolution
        wy = info.origin_y + (free_y.astype(float) + 0.5) * info.resolution
        points = np.stack([wx, wy], axis=1)

        centered = points - center
        u_values = centered @ main_dir
        v_values = centered @ side_dir

        min_u = float(np.min(u_values))
        max_u = float(np.max(u_values))
        min_v = float(np.min(v_values))
        max_v = float(np.max(v_values))

        sweep_lines: List[List[List[GridPoint]]] = []

        v = min_v
        while v <= max_v:
            line_segments: List[List[GridPoint]] = []
            current_segment: List[GridPoint] = []

            u = min_u
            while u <= max_u:
                world = center + u * main_dir + v * side_dir
                gx, gy = world_to_grid(info, float(world[0]), float(world[1]))

                is_free = False
                if inside_grid(info, gx, gy):
                    is_free = bool(safe_free[gy, gx])

                if is_free:
                    if not current_segment or current_segment[-1] != (gx, gy):
                        current_segment.append((gx, gy))
                else:
                    if self._segment_length_cells(current_segment, info) >= self.config.min_segment_length:
                        line_segments.append(current_segment)

                    current_segment = []

                u += self.config.sample_step

            if self._segment_length_cells(current_segment, info) >= self.config.min_segment_length:
                line_segments.append(current_segment)

            if line_segments:
                sweep_lines.append(line_segments)

            v += self.config.sweep_spacing

        return sweep_lines

    @staticmethod
    def _segment_length_cells(segment: List[GridPoint], info: GridMapInfo) -> float:
        """估算一个扫描线段的长度，单位 m。"""

        if len(segment) < 2:
            return 0.0

        x0, y0 = segment[0]
        x1, y1 = segment[-1]
        return math.hypot(x1 - x0, y1 - y0) * info.resolution

    @staticmethod
    def _snake_order_segments(
        sweep_lines: List[List[List[GridPoint]]],
    ) -> List[List[GridPoint]]:
        """
        对扫描线进行蛇形排序。

        偶数行从左到右，奇数行从右到左。
        """

        ordered: List[List[GridPoint]] = []

        for line_index, line_segments in enumerate(sweep_lines):
            if line_index % 2 == 0:
                for segment in line_segments:
                    ordered.append(segment)
            else:
                for segment in reversed(line_segments):
                    ordered.append(list(reversed(segment)))

        return ordered

    def _connect_segments(
        self,
        safe_free: np.ndarray,
        ordered_segments: List[List[GridPoint]],
    ) -> Tuple[List[GridPoint], int]:
        """
        将蛇形扫描线段连接成完整路径。

        优先直线连接；
        直线不安全时使用 A*；
        A* 失败则跳过该线段。
        """

        full_path: List[GridPoint] = []
        skipped_segments = 0

        for segment in ordered_segments:
            if not segment:
                continue

            if not full_path:
                full_path.extend(segment)
                continue

            start = full_path[-1]
            goal = segment[0]

            connector: List[GridPoint] = []

            if is_line_free(safe_free, start, goal):
                connector = bresenham_line(start[0], start[1], goal[0], goal[1])
            elif self.config.use_astar_connection:
                astar_path = astar_grid(
                    free_mask=safe_free,
                    start=start,
                    goal=goal,
                    max_search_cells=self.config.astar_max_search_cells,
                )
                if astar_path is not None:
                    connector = astar_path

            if not connector:
                skipped_segments += 1
                continue

            # 去掉重复起点
            full_path.extend(connector[1:])

            # 如果连接器已经到达 segment 的第一个点，则去掉重复点
            if full_path and full_path[-1] == segment[0]:
                full_path.extend(segment[1:])
            else:
                full_path.extend(segment)

        # 删除连续重复点
        deduped: List[GridPoint] = []
        for p in full_path:
            if not deduped or deduped[-1] != p:
                deduped.append(p)

        return deduped, skipped_segments

    def _estimate_coverage_rate(
        self,
        safe_free: np.ndarray,
        grid_path: List[GridPoint],
        info: GridMapInfo,
    ) -> float:
        """
        粗略估计覆盖率。

        方法：
        把机器人路径中心线按照 coverage_radius 膨胀，
        看它覆盖了多少安全自由栅格。
        """

        if not grid_path:
            return 0.0

        path_mask = np.zeros_like(safe_free, dtype=bool)

        for gx, gy in grid_path:
            if 0 <= gy < safe_free.shape[0] and 0 <= gx < safe_free.shape[1]:
                path_mask[gy, gx] = True

        radius_cells = int(math.ceil(self.config.coverage_radius / info.resolution))
        covered_mask = dilate_mask(path_mask, radius_cells)
        covered_free = covered_mask & safe_free

        total_free = int(np.count_nonzero(safe_free))
        if total_free == 0:
            return 0.0

        return float(np.count_nonzero(covered_free)) / float(total_free)
