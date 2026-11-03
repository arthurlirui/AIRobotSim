"""
Navigation reference implementations.

* :class:`PotentialFieldNav` — attractive force toward the goal, repulsive
  forces from forward lidar hits; outputs unicycle ``(v, omega)`` for wheeled
  robots or ``(vx, vy, vz)`` for UAVs.
* :class:`AStarDwaNav` — global A* on a coarse occupancy grid (built online
  from lidar) + Dynamic Window Approach for local velocity selection.
"""

from __future__ import annotations

import heapq
import numpy as np
from typing import TYPE_CHECKING

from algorithms.base import NavigationBase, AlgorithmOutput
from simlab.registry import register
from robots.mobile_base import WheeledRobot, UAV

if TYPE_CHECKING:
    from simlab.world import World


def _goal_reached(pos: np.ndarray, goal: np.ndarray, radius: float) -> bool:
    return float(np.linalg.norm(goal[:2] - pos[:2])) <= radius


class PotentialFieldNav(NavigationBase):
    """Artificial potential field navigation."""

    name = "potential_field"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.robot_id: str = self.cfg.get("robot_id", "r0")
        self.goal = np.asarray(self.cfg.get("goal", [0.0, 0.0, 0.0]), float)
        self.k_att = float(self.cfg.get("k_att", 1.0))
        self.k_rep = float(self.cfg.get("k_rep", 2.0))
        self.rep_range = float(self.cfg.get("rep_range", 3.0))
        self.reach_radius = float(self.cfg.get("reach_radius", 0.5))
        self.reached = False
        self.collisions = 0
        self.path_len = 0.0
        self._prev_pos: np.ndarray | None = None
        self.steps = 0

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        self.steps += 1
        obs = observations.get(self.robot_id, {})
        pos = np.asarray(obs.get("pose_truth", [0, 0, 0]), float)
        lidar = obs.get("lidar")
        robot = world.robots[self.robot_id]

        # attractive force
        to_goal = self.goal[:2] - pos[:2]
        dist_goal = float(np.linalg.norm(to_goal))
        if dist_goal <= self.reach_radius:
            self.reached = True
            return AlgorithmOutput(actions={self.robot_id: np.zeros(3)},
                                   data={"reached": True})
        f_att = self.k_att * to_goal / max(dist_goal, 1e-6)

        # repulsive force from lidar hits
        f_rep = np.zeros(2)
        if lidar is not None and len(lidar):
            yaw = pos[2] if pos.size == 3 else 0.0
            for ang, rng in lidar:
                if rng >= self.rep_range:
                    continue
                world_ang = float(ang) + yaw
                hx = pos[0] + rng * np.cos(world_ang)
                hy = pos[1] + rng * np.sin(world_ang)
                d = np.array([hx - pos[0], hy - pos[1]])
                nd = float(np.linalg.norm(d))
                if nd < 1e-6:
                    continue
                mag = self.k_rep * (1.0 / nd - 1.0 / self.rep_range) / (nd ** 2)
                f_rep -= mag * d / nd

        force = f_att + f_rep
        fnorm = float(np.linalg.norm(force))
        if fnorm < 1e-6:
            force = f_att
            fnorm = float(np.linalg.norm(force)) or 1.0
        desired = force / fnorm

        if isinstance(robot, WheeledRobot):
            v = min(fnorm, robot.max_speed)
            heading = np.array([np.cos(pos[2]), np.sin(pos[2])])
            cross = heading[0] * desired[1] - heading[1] * desired[0]
            omega = np.clip(2.0 * cross, -robot.max_omega, robot.max_omega)
            action = np.array([v, omega])
        else:  # UAV or generic velocity-driven base
            action = np.array([desired[0], desired[1], 0.0])

        # bookkeeping
        if self._prev_pos is not None:
            self.path_len += float(np.linalg.norm(pos[:2] - self._prev_pos[:2]))
            if robot.check_collision(world.env):
                self.collisions += 1
        self._prev_pos = pos.copy()

        return AlgorithmOutput(actions={self.robot_id: action},
                               data={"reached": self.reached})

    def metrics(self) -> dict:
        return {
            "nav.reach_rate": 1.0 if self.reached else 0.0,
            "nav.path_length": self.path_len,
            "nav.collisions": self.collisions,
            "nav.steps": self.steps,
        }


@register("navigation", "potential_field")
class _RegisteredPotentialFieldNav(PotentialFieldNav):
    pass


class AStarDwaNav(NavigationBase):
    """
    A* global planner over a coarse occupancy grid built from accumulated
    lidar, paired with a Dynamic Window Approach local planner that picks
    the best (v, omega) within the robot's dynamic limits.
    """

    name = "astar_dwa"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.robot_id: str = self.cfg.get("robot_id", "r0")
        self.goal = np.asarray(self.cfg.get("goal", [0.0, 0.0, 0.0]), float)
        self.reach_radius = float(self.cfg.get("reach_radius", 0.5))
        self.res = float(self.cfg.get("grid_res", 0.5))
        bounds = self.cfg.get("bounds", (-10, 10, -10, 10))
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.nx = int((self.x_max - self.x_min) / self.res)
        self.ny = int((self.y_max - self.y_min) / self.res)
        self.grid = np.zeros((self.nx, self.ny), dtype=np.int8)  # 0 free, 1 occ
        self.reached = False
        self.collisions = 0
        self.path_len = 0.0
        self.steps = 0
        self._prev_pos: np.ndarray | None = None
        self._path: list[tuple[int, int]] = []

    def _to_cell(self, x: float, y: float) -> tuple[int, int]:
        ix = int(np.clip((x - self.x_min) / self.res, 0, self.nx - 1))
        iy = int(np.clip((y - self.y_min) / self.res, 0, self.ny - 1))
        return ix, iy

    def _update_grid(self, pos: np.ndarray, lidar: np.ndarray, max_range: float):
        if lidar is None or len(lidar) == 0:
            return
        yaw = pos[2] if pos.size == 3 else 0.0
        ox, oy = self._to_cell(pos[0], pos[1])
        for ang, rng in lidar:
            r = float(min(rng, max_range))
            hx = pos[0] + r * np.cos(float(ang) + yaw)
            hy = pos[1] + r * np.sin(float(ang) + yaw)
            ex, ey = self._to_cell(hx, hy)
            if rng < max_range:
                self.grid[ex, ey] = 1
            # mark along-ray cells free (simple line)
            n = max(abs(ex - ox), abs(ey - oy), 1)
            for k in range(1, n):
                cx = int(ox + (ex - ox) * k / n)
                cy = int(oy + (ey - oy) * k / n)
                if self.grid[cx, cy] == 0:
                    self.grid[cx, cy] = 0

    def _astar(self, start, goal) -> list[tuple[int, int]]:
        if self.grid[goal] == 1:
            return []
        open_h = [(0, start)]
        came = {start: None}
        g = {start: 0}

        def h(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        while open_h:
            _, cur = heapq.heappop(open_h)
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came[cur]
                return path[::-1]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nb = (cur[0] + dx, cur[1] + dy)
                if not (0 <= nb[0] < self.nx and 0 <= nb[1] < self.ny):
                    continue
                if self.grid[nb] == 1:
                    continue
                step = 1 if dx == 0 or dy == 0 else 1.414
                ng = g[cur] + step
                if ng < g.get(nb, 1e9):
                    g[nb] = ng
                    came[nb] = cur
                    heapq.heappush(open_h, (ng + h(nb, goal), nb))
        return []

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        self.steps += 1
        obs = observations.get(self.robot_id, {})
        pos = np.asarray(obs.get("pose_truth", [0, 0, 0]), float)
        lidar = obs.get("lidar")
        max_range = float(obs.get("lidar_max_range", 5.0))
        self._update_grid(pos, lidar, max_range)
        robot = world.robots[self.robot_id]

        if _goal_reached(pos, self.goal, self.reach_radius):
            self.reached = True
            return AlgorithmOutput(actions={self.robot_id: np.zeros(3)},
                                   data={"reached": True})

        start = self._to_cell(pos[0], pos[1])
        gcell = self._to_cell(self.goal[0], self.goal[1])
        self._path = self._astar(start, gcell)

        # DWA: sample (v, omega) and score by progress along path + clearance.
        if isinstance(robot, WheeledRobot):
            best_v, best_w = 0.0, 0.0
            best_score = -1e9
            for v in np.linspace(0, robot.max_speed, 7):
                for w in np.linspace(-robot.max_omega, robot.max_omega, 11):
                    score = v - 0.3 * abs(w)
                    if self._path and len(self._path) > 1:
                        nx, ny = self._path[1]
                        tx = self.x_min + (nx + 0.5) * self.res
                        ty = self.y_min + (ny + 0.5) * self.res
                        heading = np.array([np.cos(pos[2] + w * world.cfg.dt),
                                            np.sin(pos[2] + w * world.cfg.dt)])
                        to_next = np.array([tx - pos[0], ty - pos[1]])
                        to_next /= max(np.linalg.norm(to_next), 1e-6)
                        score += 2.0 * float(heading @ to_next)
                    # clearance: penalize heading into occupied cell
                    hx = pos[0] + 0.5 * np.cos(pos[2] + w * world.cfg.dt)
                    hy = pos[1] + 0.5 * np.sin(pos[2] + w * world.cfg.dt)
                    cx, cy = self._to_cell(hx, hy)
                    if self.grid[cx, cy] == 1:
                        score -= 5.0
                    if score > best_score:
                        best_score = score
                        best_v, best_w = float(v), float(w)
            action = np.array([best_v, best_w])
        else:
            # UAV: head toward next path waypoint
            if self._path and len(self._path) > 1:
                nx, ny = self._path[1]
                tx = self.x_min + (nx + 0.5) * self.res
                ty = self.y_min + (ny + 0.5) * self.res
                d = np.array([tx - pos[0], ty - pos[1]])
                d /= max(np.linalg.norm(d), 1e-6)
                action = np.array([d[0], d[1], 0.0])
            else:
                d = self.goal[:2] - pos[:2]
                d /= max(np.linalg.norm(d), 1e-6)
                action = np.array([d[0], d[1], 0.0])

        if self._prev_pos is not None:
            self.path_len += float(np.linalg.norm(pos[:2] - self._prev_pos[:2]))
            if robot.check_collision(world.env):
                self.collisions += 1
        self._prev_pos = pos.copy()

        return AlgorithmOutput(actions={self.robot_id: action},
                               data={"reached": self.reached,
                                     "path_cells": len(self._path)})

    def metrics(self) -> dict:
        return {
            "nav.reach_rate": 1.0 if self.reached else 0.0,
            "nav.path_length": self.path_len,
            "nav.collisions": self.collisions,
            "nav.steps": self.steps,
        }


@register("navigation", "astar_dwa")
class _RegisteredAStarDwaNav(AStarDwaNav):
    pass
