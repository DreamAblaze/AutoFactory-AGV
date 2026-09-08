#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Differential-drive Hybrid A* planner.

State:
    (x, y, yaw, direction)

Motion primitives:
    forward-left
    forward-straight
    forward-right

    reverse-left
    reverse-straight
    reverse-right

    rotate-left
    rotate-right

The occupancy grid supplied to this planner should already be
inflated by the robot radius.
"""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


def normalize_angle(angle: float) -> float:

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def angle_difference(
    target: float,
    current: float,
) -> float:

    return normalize_angle(
        target - current
    )


@dataclass
class SearchNode:

    x: float
    y: float
    yaw: float

    # +1 forward
    # -1 reverse
    #  0 in-place rotation / initial state
    direction: int

    g: float

    parent_key: Optional[Tuple[int, int, int, int]]


@dataclass
class PlanResult:

    success: bool

    path: List[Tuple[float, float, float, int]]

    expanded_nodes: int

    cost: float

    reason: str


class HybridAStarPlanner:

    def __init__(
        self,
        obstacle_grid: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        xy_step: float = 0.10,
        yaw_resolution_deg: float = 5.0,
        min_turning_radius: float = 0.40,
        collision_check_step: float = 0.025,
        allow_reverse: bool = True,
        allow_in_place_rotation: bool = True,
        spin_step_deg: float = 10.0,
        reverse_penalty: float = 1.30,
        gear_switch_penalty: float = 0.40,
        turning_penalty: float = 0.15,
        spin_penalty: float = 1.20,
        yaw_heuristic_weight: float = 1.0,
        heuristic_weight: float = 1.0,
        goal_xy_tolerance: float = 0.12,
        goal_yaw_tolerance_deg: float = 8.0,
        search_margin: float = 3.0,
        max_expansions: int = 120000,
    ):

        self.grid = obstacle_grid.astype(
            np.bool_
        )

        self.height = int(
            self.grid.shape[0]
        )

        self.width = int(
            self.grid.shape[1]
        )

        self.resolution = float(
            resolution
        )

        self.origin_x = float(
            origin_x
        )

        self.origin_y = float(
            origin_y
        )

        self.origin_yaw = float(
            origin_yaw
        )

        self.xy_step = float(
            xy_step
        )

        self.yaw_resolution = math.radians(
            yaw_resolution_deg
        )

        self.yaw_bins = max(
            1,
            int(
                round(
                    2.0
                    * math.pi
                    / self.yaw_resolution
                )
            ),
        )

        self.min_turning_radius = float(
            min_turning_radius
        )

        self.max_curvature = (
            1.0
            / self.min_turning_radius
        )

        self.collision_check_step = float(
            collision_check_step
        )

        self.allow_reverse = bool(
            allow_reverse
        )

        self.allow_in_place_rotation = bool(
            allow_in_place_rotation
        )

        self.spin_step = math.radians(
            spin_step_deg
        )

        self.reverse_penalty = float(
            reverse_penalty
        )

        self.gear_switch_penalty = float(
            gear_switch_penalty
        )

        self.turning_penalty = float(
            turning_penalty
        )

        self.spin_penalty = float(
            spin_penalty
        )

        self.yaw_heuristic_weight = float(
            yaw_heuristic_weight
        )

        self.heuristic_weight = float(
            heuristic_weight
        )

        self.goal_xy_tolerance = float(
            goal_xy_tolerance
        )

        self.goal_yaw_tolerance = math.radians(
            goal_yaw_tolerance_deg
        )

        self.search_margin = float(
            search_margin
        )

        self.max_expansions = int(
            max_expansions
        )

    # =========================================================
    # Map conversion
    # =========================================================

    def world_to_grid(
        self,
        x: float,
        y: float,
    ) -> Tuple[int, int]:

        dx = x - self.origin_x
        dy = y - self.origin_y

        c = math.cos(
            self.origin_yaw
        )

        s = math.sin(
            self.origin_yaw
        )

        local_x = (
            c * dx
            + s * dy
        )

        local_y = (
            -s * dx
            + c * dy
        )

        gx = int(
            math.floor(
                local_x / self.resolution
            )
        )

        gy = int(
            math.floor(
                local_y / self.resolution
            )
        )

        return gx, gy

    def inside_map(
        self,
        x: float,
        y: float,
    ) -> bool:

        gx, gy = self.world_to_grid(
            x,
            y,
        )

        return (
            0 <= gx < self.width
            and
            0 <= gy < self.height
        )

    def collision(
        self,
        x: float,
        y: float,
    ) -> bool:

        gx, gy = self.world_to_grid(
            x,
            y,
        )

        if (
            gx < 0
            or gx >= self.width
            or gy < 0
            or gy >= self.height
        ):
            return True

        return bool(
            self.grid[gy, gx]
        )

    # =========================================================
    # State discretization
    # =========================================================

    def yaw_bin(
        self,
        yaw: float,
    ) -> int:

        wrapped = (
            normalize_angle(yaw)
            + math.pi
        )

        index = int(
            round(
                wrapped
                / self.yaw_resolution
            )
        )

        return index % self.yaw_bins

    def state_key(
        self,
        x: float,
        y: float,
        yaw: float,
        direction: int,
    ):

        gx, gy = self.world_to_grid(
            x,
            y,
        )

        return (
            gx,
            gy,
            self.yaw_bin(yaw),
            int(direction),
        )

    # =========================================================
    # Goal / heuristic
    # =========================================================

    def goal_reached(
        self,
        x: float,
        y: float,
        yaw: float,
        goal,
    ) -> bool:

        gx, gy, gyaw = goal

        distance = math.hypot(
            x - gx,
            y - gy,
        )

        yaw_error = abs(
            angle_difference(
                gyaw,
                yaw,
            )
        )

        return (
            distance
            <= self.goal_xy_tolerance
            and
            yaw_error
            <= self.goal_yaw_tolerance
        )

    def heuristic(
        self,
        x: float,
        y: float,
        yaw: float,
        goal,
    ) -> float:

        gx, gy, gyaw = goal

        distance = math.hypot(
            gx - x,
            gy - y,
        )

        yaw_error = abs(
            angle_difference(
                gyaw,
                yaw,
            )
        )

        yaw_cost = (
            self.min_turning_radius
            * yaw_error
        )

        return (
            distance
            +
            self.yaw_heuristic_weight
            * yaw_cost
        )

    # =========================================================
    # Search window
    # =========================================================

    def create_search_window(
        self,
        start,
        goal,
    ):

        sx, sy, _ = start
        gx, gy, _ = goal

        return (
            min(sx, gx)
            - self.search_margin,

            max(sx, gx)
            + self.search_margin,

            min(sy, gy)
            - self.search_margin,

            max(sy, gy)
            + self.search_margin,
        )

    def inside_search_window(
        self,
        x,
        y,
        window,
    ):

        xmin, xmax, ymin, ymax = window

        return (
            xmin <= x <= xmax
            and
            ymin <= y <= ymax
        )

    # =========================================================
    # Motion simulation
    # =========================================================

    def simulate_translation(
        self,
        node: SearchNode,
        direction: int,
        curvature: float,
    ):

        count = max(
            2,
            int(
                math.ceil(
                    self.xy_step
                    / self.collision_check_step
                )
            ),
        )

        final_state = None

        for i in range(
            1,
            count + 1,
        ):

            distance = (
                direction
                * self.xy_step
                * i
                / count
            )

            if abs(curvature) < 1.0e-9:

                x = (
                    node.x
                    + distance
                    * math.cos(node.yaw)
                )

                y = (
                    node.y
                    + distance
                    * math.sin(node.yaw)
                )

                yaw = node.yaw

            else:

                yaw = (
                    node.yaw
                    + distance
                    * curvature
                )

                x = (
                    node.x
                    +
                    (
                        math.sin(yaw)
                        - math.sin(node.yaw)
                    )
                    / curvature
                )

                y = (
                    node.y
                    -
                    (
                        math.cos(yaw)
                        - math.cos(node.yaw)
                    )
                    / curvature
                )

            yaw = normalize_angle(
                yaw
            )

            if self.collision(
                x,
                y,
            ):
                return None

            final_state = (
                x,
                y,
                yaw,
            )

        return final_state

    def simulate_spin(
        self,
        node: SearchNode,
        delta_yaw: float,
    ):

        # Circular robot footprint:
        # rotation in place does not change collision position.

        if self.collision(
            node.x,
            node.y,
        ):
            return None

        return (
            node.x,
            node.y,
            normalize_angle(
                node.yaw
                + delta_yaw
            ),
        )

    # =========================================================
    # Exact-goal connection check
    # =========================================================

    def can_connect_goal(
        self,
        state,
        goal,
    ):

        x0, y0, _ = state
        x1, y1, _ = goal

        distance = math.hypot(
            x1 - x0,
            y1 - y0,
        )

        samples = max(
            2,
            int(
                math.ceil(
                    distance
                    / self.collision_check_step
                )
            ),
        )

        for i in range(
            1,
            samples + 1,
        ):

            ratio = i / samples

            x = (
                x0
                + ratio
                * (x1 - x0)
            )

            y = (
                y0
                + ratio
                * (y1 - y0)
            )

            if self.collision(
                x,
                y,
            ):
                return False

        return True

    # =========================================================
    # Path reconstruction
    # =========================================================

    def reconstruct(
        self,
        records,
        key,
    ):

        path = []

        while key is not None:

            node = records[key]

            path.append(
                (
                    node.x,
                    node.y,
                    node.yaw,
                    node.direction,
                )
            )

            key = node.parent_key

        path.reverse()

        return path

    # =========================================================
    # Main planning
    # =========================================================

    def plan(
        self,
        start,
        goal,
    ) -> PlanResult:

        sx, sy, syaw = start
        gx, gy, gyaw = goal

        syaw = normalize_angle(
            syaw
        )

        gyaw = normalize_angle(
            gyaw
        )

        start = (
            sx,
            sy,
            syaw,
        )

        goal = (
            gx,
            gy,
            gyaw,
        )

        if self.collision(
            sx,
            sy,
        ):

            return PlanResult(
                False,
                [],
                0,
                math.inf,
                "START_IN_COLLISION",
            )

        if self.collision(
            gx,
            gy,
        ):

            return PlanResult(
                False,
                [],
                0,
                math.inf,
                "GOAL_IN_COLLISION",
            )

        search_window = (
            self.create_search_window(
                start,
                goal,
            )
        )

        start_node = SearchNode(
            x=sx,
            y=sy,
            yaw=syaw,
            direction=0,
            g=0.0,
            parent_key=None,
        )

        start_key = self.state_key(
            sx,
            sy,
            syaw,
            0,
        )

        records: Dict[
            Tuple[int, int, int, int],
            SearchNode,
        ] = {
            start_key: start_node
        }

        g_score = {
            start_key: 0.0
        }

        counter = itertools.count()

        open_heap = []

        h0 = self.heuristic(
            sx,
            sy,
            syaw,
            goal,
        )

        heapq.heappush(
            open_heap,
            (
                self.heuristic_weight
                * h0,

                next(counter),

                start_key,

                0.0,
            ),
        )

        expanded = 0

        directions = [1]

        if self.allow_reverse:
            directions.append(-1)

        curvatures = [
            -self.max_curvature,
            0.0,
            +self.max_curvature,
        ]

        while open_heap:

            _, _, current_key, heap_g = (
                heapq.heappop(
                    open_heap
                )
            )

            best_g = g_score.get(
                current_key
            )

            if best_g is None:
                continue

            if heap_g > best_g + 1.0e-9:
                continue

            current = records[
                current_key
            ]

            expanded += 1

            if expanded > self.max_expansions:

                return PlanResult(
                    False,
                    [],
                    expanded,
                    current.g,
                    "MAX_EXPANSIONS_REACHED",
                )

            # ---------------------------------------------
            # Goal reached
            # ---------------------------------------------

            if self.goal_reached(
                current.x,
                current.y,
                current.yaw,
                goal,
            ):

                if self.can_connect_goal(
                    (
                        current.x,
                        current.y,
                        current.yaw,
                    ),
                    goal,
                ):

                    path = self.reconstruct(
                        records,
                        current_key,
                    )

                    # Append exact requested goal.
                    path.append(
                        (
                            gx,
                            gy,
                            gyaw,
                            current.direction,
                        )
                    )

                    return PlanResult(
                        True,
                        path,
                        expanded,
                        current.g,
                        "SUCCESS",
                    )

            # ---------------------------------------------
            # Translation primitives
            # ---------------------------------------------

            for direction in directions:

                for curvature in curvatures:

                    result = (
                        self.simulate_translation(
                            current,
                            direction,
                            curvature,
                        )
                    )

                    if result is None:
                        continue

                    nx, ny, nyaw = result

                    if not self.inside_search_window(
                        nx,
                        ny,
                        search_window,
                    ):
                        continue

                    turn_ratio = (
                        abs(curvature)
                        / self.max_curvature
                    )

                    primitive_cost = (
                        self.xy_step
                    )

                    if direction < 0:

                        primitive_cost *= (
                            self.reverse_penalty
                        )

                    primitive_cost *= (
                        1.0
                        +
                        self.turning_penalty
                        * turn_ratio
                    )

                    if (
                        current.direction != 0
                        and
                        current.direction != direction
                    ):

                        primitive_cost += (
                            self.gear_switch_penalty
                        )

                    new_g = (
                        current.g
                        + primitive_cost
                    )

                    new_key = self.state_key(
                        nx,
                        ny,
                        nyaw,
                        direction,
                    )

                    old_g = g_score.get(
                        new_key,
                        math.inf,
                    )

                    if new_g + 1.0e-9 >= old_g:
                        continue

                    new_node = SearchNode(
                        x=nx,
                        y=ny,
                        yaw=nyaw,
                        direction=direction,
                        g=new_g,
                        parent_key=current_key,
                    )

                    g_score[new_key] = new_g

                    records[new_key] = (
                        new_node
                    )

                    h = self.heuristic(
                        nx,
                        ny,
                        nyaw,
                        goal,
                    )

                    f = (
                        new_g
                        +
                        self.heuristic_weight
                        * h
                    )

                    heapq.heappush(
                        open_heap,
                        (
                            f,
                            next(counter),
                            new_key,
                            new_g,
                        ),
                    )

            # ---------------------------------------------
            # In-place rotation primitives
            # ---------------------------------------------

            if self.allow_in_place_rotation:

                for delta_yaw in (
                    -self.spin_step,
                    +self.spin_step,
                ):

                    result = (
                        self.simulate_spin(
                            current,
                            delta_yaw,
                        )
                    )

                    if result is None:
                        continue

                    nx, ny, nyaw = result

                    new_direction = 0

                    spin_cost = (
                        self.spin_penalty
                        *
                        self.min_turning_radius
                        *
                        abs(delta_yaw)
                    )

                    new_g = (
                        current.g
                        + spin_cost
                    )

                    new_key = self.state_key(
                        nx,
                        ny,
                        nyaw,
                        new_direction,
                    )

                    old_g = g_score.get(
                        new_key,
                        math.inf,
                    )

                    if new_g + 1.0e-9 >= old_g:
                        continue

                    new_node = SearchNode(
                        x=nx,
                        y=ny,
                        yaw=nyaw,
                        direction=0,
                        g=new_g,
                        parent_key=current_key,
                    )

                    g_score[new_key] = new_g

                    records[new_key] = (
                        new_node
                    )

                    h = self.heuristic(
                        nx,
                        ny,
                        nyaw,
                        goal,
                    )

                    f = (
                        new_g
                        +
                        self.heuristic_weight
                        * h
                    )

                    heapq.heappush(
                        open_heap,
                        (
                            f,
                            next(counter),
                            new_key,
                            new_g,
                        ),
                    )

        return PlanResult(
            False,
            [],
            expanded,
            math.inf,
            "OPEN_SET_EMPTY",
        )
