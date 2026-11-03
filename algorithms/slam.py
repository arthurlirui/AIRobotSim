"""
SLAM reference implementations over the 3DGS world.

* :class:`OccupancyGridSLAM` — a 2-D occupancy grid updated from a lidar scan
  (hit cells occupied, traversed cells free), with a dead-reckoning pose
  estimate from odometry. Lightweight, numpy-only, good enough to validate
  the SLAM metrics (map IoU, ATE/RPE).
* :class:`PoseGraphSLAM` — wraps an :class:`OccupancyGridSLAM` and additionally
  maintains a pose graph (nodes = poses, edges = odometry constraints) with a
  trivial loop-closure hook: when the robot revisits a cell, add an edge to
  the earlier node. No solver is run (kept dependency-free); the graph is
  exposed for downstream optimizers.
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from algorithms.base import SLAMBase, AlgorithmOutput
from simlab.registry import register

if TYPE_CHECKING:
    from simlab.world import World


class OccupancyGridSLAM(SLAMBase):
    """
    Bayesian-ish occupancy grid SLAM.

    The grid covers ``[x_min, x_max] × [y_min, y_max]`` at ``resolution``
    meters/cell. Each lidar ray marks the endpoint cell as occupied (log-odd
    + ``occ_inc``) and the cells along the ray as free (log-odd - ``free_inc``).
    A cell is "occupied" when its log-odd > 0.

    Pose is estimated by integrating odometry deltas (a stand-in for scan
    matching); the ground-truth pose is only used for evaluation.
    """

    name = "occupancy_grid"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        c = self.cfg
        self.robot_id: str = c.get("robot_id", "r0")
        bounds = c.get("bounds", (-10.0, 10.0, -10.0, 10.0))
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.res: float = c.get("resolution", 0.2)
        self.nx = int((self.x_max - self.x_min) / self.res)
        self.ny = int((self.y_max - self.y_min) / self.res)
        self.log_odds = np.zeros((self.nx, self.ny), dtype=float)
        self.occ_inc = c.get("occ_inc", 0.85)
        self.free_inc = c.get("free_inc", 0.4)
        self.pose_est = np.zeros(3)
        self._ate: list[float] = []
        self._rpe: list[float] = []
        self._prev_truth: np.ndarray | None = None
        self._prev_est: np.ndarray | None = None

    # -- grid helpers ---------------------------------------------------
    def _to_cell(self, x: float, y: float) -> tuple[int, int]:
        ix = int((x - self.x_min) / self.res)
        iy = int((y - self.y_min) / self.res)
        return np.clip(ix, 0, self.nx - 1), np.clip(iy, 0, self.ny - 1)

    def _bresenham(self, x0: int, y0: int, x1: int, y1: int):
        """Yield cells along the line (x0,y0)->(x1,y1), endpoints inclusive."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        cx, cy = x0, y0
        while True:
            yield cx, cy
            if cx == x1 and cy == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

    def _integrate_scan(self, origin_xy: np.ndarray, lidar: np.ndarray,
                        max_range: float) -> None:
        ox, oy = self._to_cell(origin_xy[0], origin_xy[1])
        for ang, rng in lidar:
            r = float(min(rng, max_range))
            hx = origin_xy[0] + r * np.cos(ang)
            hy = origin_xy[1] + r * np.sin(ang)
            ex, ey = self._to_cell(hx, hy)
            for cx, cy in self._bresenham(ox, oy, ex, ey):
                if (cx, cy) == (ex, ey) and rng < max_range:
                    self.log_odds[cx, cy] += self.occ_inc
                else:
                    self.log_odds[cx, cy] -= self.free_inc
        np.clip(self.log_odds, -5.0, 5.0, out=self.log_odds)

    # -- lifecycle ------------------------------------------------------
    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        obs = observations.get(self.robot_id, {})
        truth = np.asarray(obs.get("pose_truth", self.pose_est), dtype=float)
        odom = obs.get("odometry")
        if odom is not None:
            self.pose_est = self.pose_est + np.asarray(odom, dtype=float)
        else:
            # fall back to truth when no odometry is provided (eval mode)
            self.pose_est = truth.copy()

        lidar = obs.get("lidar")
        max_range = float(obs.get("lidar_max_range", 5.0))
        if lidar is not None and len(lidar) > 0:
            self._integrate_scan(self.pose_est[:2], lidar, max_range)

        # evaluation metrics
        self._ate.append(float(np.linalg.norm(truth[:2] - self.pose_est[:2])))
        if self._prev_truth is not None and self._prev_est is not None:
            dt = truth[:2] - self._prev_truth[:2]
            de = self.pose_est[:2] - self._prev_est[:2]
            self._rpe.append(float(np.linalg.norm(dt - de)))
        self._prev_truth = truth.copy()
        self._prev_est = self.pose_est.copy()

        return AlgorithmOutput(data={
            "map": self.occupied_mask(),
            "pose_estimate": self.pose_est.copy(),
        })

    def occupied_mask(self) -> np.ndarray:
        return self.log_odds > 0.0

    def metrics(self) -> dict:
        ate = float(np.mean(self._ate)) if self._ate else 0.0
        rpe = float(np.mean(self._rpe)) if self._rpe else 0.0
        return {
            "slam.ate_mean": ate,
            "slam.rpe_mean": rpe,
            "slam.n_steps": len(self._ate),
            "slam.occupied_cells": int(self.occupied_mask().sum()),
        }


@register("slam", "occupancy_grid")
class _RegisteredOccGridSLAM(OccupancyGridSLAM):
    pass


class PoseGraphSLAM(OccupancyGridSLAM):
    """
    Occupancy-grid SLAM plus a pose graph with a naive loop-closure hook.

    The pose graph is a list of (pose, node_index) and an edge list of
    ``(i, j, constraint)`` where ``constraint`` is the relative pose. On each
    step a new node is appended with an odometry edge to the previous node.
    When the robot's current cell was first visited ≥ ``loop_gap`` steps ago,
    a loop-closure edge is added back to that earlier node (no optimization
    is performed — the graph is exposed for a downstream solver).
    """

    name = "pose_graph"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.loop_gap: int = self.cfg.get("loop_gap", 20)
        self.nodes: list[np.ndarray] = []      # poses
        self.edges: list[tuple[int, int, np.ndarray]] = []
        self._first_visit: dict[tuple[int, int], int] = {}
        self._node_step: int = 0

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        out = super().step(world, observations)
        idx = len(self.nodes)
        self.nodes.append(self.pose_est.copy())
        if idx > 0:
            rel = self.pose_est - self.nodes[idx - 1]
            self.edges.append((idx - 1, idx, rel))
        cell = self._to_cell(self.pose_est[0], self.pose_est[1])
        if cell not in self._first_visit:
            self._first_visit[cell] = self._node_step
        else:
            prev = self._first_visit[cell]
            if self._node_step - prev >= self.loop_gap:
                self.edges.append((prev, idx, self.pose_est - self.nodes[prev]))
        self._node_step += 1
        out.data["pose_graph"] = {"nodes": len(self.nodes),
                                  "edges": len(self.edges)}
        return out

    def metrics(self) -> dict:
        m = super().metrics()
        m["slam.pose_graph_nodes"] = len(self.nodes)
        m["slam.pose_graph_edges"] = len(self.edges)
        return m


@register("slam", "pose_graph")
class _RegisteredPoseGraphSLAM(PoseGraphSLAM):
    pass
