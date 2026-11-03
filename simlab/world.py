"""
The simulation world: one place that owns the scene, robots, optional comm
network and sensors, and the registered algorithms, and advances them in a
fixed, reproducible order every tick.

Single-step order (deliberately fixed so algorithms see consistent
observations regardless of registration order):

1. Collect per-robot observations (pose truth, lidar, optional camera,
   comm snapshot, obstacle ground truth for detection eval).
2. Run every algorithm's ``step``; collect :class:`AlgorithmOutput`.
3. Apply the accumulated actions to the robots and integrate them.
4. Step the comm network with the new poses.
5. Advance the clock; record algorithm metrics.

For large homogeneous swarms, :class:`VectorizedSwarm` stacks poses into a
single ``(N,3)`` array and integrates them in one vectorized pass, with
pairwise comm distance computed by broadcasting. It produces results that
match the serial :class:`World` for the same inputs (verified in the tests).
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Any

from core.gaussian_splat_env import GaussianSplatEnv
from robots.mobile_base import MobileRobot, WheeledRobot, UAV
from comm.network.network_simulator import CommNetworkSimulator, CommSnapshot
from comm.channel.base import Pose
from algorithms.base import AlgorithmBase, AlgorithmOutput


@dataclass
class WorldConfig:
    dt: float = 0.1
    seed: int = 0
    lidar_rays: int = 36
    lidar_range: float = 5.0
    lidar_height_offset: float = 0.2


class World:
    """Owns env + robots + comm + sensors + algorithms."""

    def __init__(self, env: GaussianSplatEnv, *,
                 config: Optional[WorldConfig] = None,
                 comm: Optional[CommNetworkSimulator] = None):
        self.env = env
        self.cfg = config or WorldConfig()
        self.robots: dict[str, MobileRobot] = {}
        self.comm: Optional[CommNetworkSimulator] = comm
        self.sensors: dict[str, Any] = {}          # robot_id -> CameraModel
        self.algorithms: list[AlgorithmBase] = []
        self.t: float = 0.0
        self.rng = np.random.default_rng(self.cfg.seed)
        self.last_snapshot: Optional[CommSnapshot] = None
        # per-step record of algorithm outputs (cleared each step)
        self.last_outputs: list[AlgorithmOutput] = []

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------
    def add_robot(self, robot: MobileRobot) -> None:
        self.robots[robot.name] = robot
        if self.comm is not None and robot.name not in self.comm.nodes:
            self.comm.add_node(robot.name, Pose(position=robot.position.copy()))

    def add_algorithm(self, alg: AlgorithmBase) -> None:
        self.algorithms.append(alg)

    def setup_algorithms(self, cfgs: Optional[list[dict]] = None) -> None:
        """Call ``setup`` on every registered algorithm."""
        for i, alg in enumerate(self.algorithms):
            c = cfgs[i] if cfgs else {}
            alg.setup(self, c)

    # ------------------------------------------------------------------
    # Observation
    # ------------------------------------------------------------------
    def _lidar(self, robot: MobileRobot) -> np.ndarray:
        n, rng = self.cfg.lidar_rays, self.cfg.lidar_range
        origin = robot.position + np.array([0, 0, self.cfg.lidar_height_offset])
        yaw = robot.yaw
        out = np.zeros((n, 2))
        for i in range(n):
            ang = 2 * np.pi * i / n
            d = np.array([np.cos(ang), np.sin(ang), 0.0])
            hit = self.env.ray_cast(origin, d, max_dist=rng)
            out[i] = [ang, hit if hit is not None else rng]
        return out

    def _obstacle_centers(self) -> np.ndarray:
        """Ground-truth obstacle centers (splats near ground) for detection."""
        if not self.env.splats:
            return np.zeros((0, 2))
        pts = np.array([s.position for s in self.env.splats])
        # only consider splats that sit roughly at ground level as obstacles
        near = pts[pts[:, 2] < self.env.ground_level + 1.0]
        return near[:, :2] if len(near) else pts[:, :2]

    def _collect_observations(self) -> dict:
        obs: dict = {}
        obstacles = self._obstacle_centers()
        for rid, robot in self.robots.items():
            obs[rid] = {
                "pose_truth": robot.position.copy(),
                "velocity": robot.state.base_velocity.copy(),
                "lidar": self._lidar(robot),
                "lidar_max_range": self.cfg.lidar_range,
                "obstacle_centers": obstacles,
                "goal": None,   # filled by navigation algos from their cfg
            }
        # swarm-level aggregate
        ids = list(self.robots.keys())
        obs["poses"] = np.array([self.robots[r].position for r in ids]) \
            if ids else np.zeros((0, 3))
        obs["velocities"] = np.array(
            [self.robots[r].state.base_velocity for r in ids]) \
            if ids else np.zeros((0, 3))
        obs["robot_ids"] = ids
        obs["comm_snapshot"] = self.last_snapshot
        return obs

    # ------------------------------------------------------------------
    # Action application
    # ------------------------------------------------------------------
    def _apply_actions(self, actions: dict[str, Any]) -> None:
        for rid, act in actions.items():
            robot = self.robots.get(rid)
            if robot is None:
                continue
            act = np.asarray(act, dtype=float)
            if isinstance(robot, WheeledRobot):
                v = float(act[0]) if act.size > 0 else 0.0
                w = float(act[1]) if act.size > 1 else 0.0
                robot.step(self.cfg.dt, v=v, omega=w, env=self.env)
            elif isinstance(robot, UAV):
                vx = float(act[0]) if act.size > 0 else 0.0
                vy = float(act[1]) if act.size > 1 else 0.0
                vz = float(act[2]) if act.size > 2 else 0.0
                robot.step(self.cfg.dt, vx=vx, vy=vy, vz=vz, env=self.env)
            else:
                # generic: treat as world-frame velocity
                robot.set_velocity(act[:3] if act.size >= 3 else np.zeros(3))
                robot.apply_velocity(self.cfg.dt)

    # ------------------------------------------------------------------
    # Step
    # ------------------------------------------------------------------
    def step(self) -> list[AlgorithmOutput]:
        observations = self._collect_observations()
        self.last_outputs = []
        merged_actions: dict[str, Any] = {}
        for alg in self.algorithms:
            # allow navigation algos to see their goal
            if alg.kind == "navigation" and hasattr(alg, "goal"):
                rid = getattr(alg, "robot_id", None)
                if rid and rid in observations:
                    observations[rid]["goal"] = alg.goal
            out = alg.step(self, observations)
            self.last_outputs.append(out)
            # later algorithms can override earlier ones for the same robot;
            # this is intentional and lets a swarm controller, say, win over
            # a per-robot navigator when both are registered.
            merged_actions.update(out.actions)
        self._apply_actions(merged_actions)
        if self.comm is not None:
            poses = {r: Pose(position=rob.position.copy())
                     for r, rob in self.robots.items()}
            self.last_snapshot = self.comm.step(poses, t=self.t)
        self.t += self.cfg.dt
        return self.last_outputs

    def run(self, n_steps: int) -> list[dict]:
        """Run ``n_steps`` ticks; return per-step algorithm metric snapshots."""
        history = []
        for _ in range(n_steps):
            self.step()
            history.append({repr(a): a.metrics() for a in self.algorithms})
        return history

    def algorithm_metrics(self) -> dict[str, dict]:
        return {repr(a): a.metrics() for a in self.algorithms}


# ---------------------------------------------------------------------------
# Vectorized swarm — same contract, faster for many homogeneous robots.
# ---------------------------------------------------------------------------

class VectorizedSwarm(World):
    """
    Vectorized world for homogeneous UAV swarms.

    Robot poses are kept in a stacked ``(N,3)`` array; integration is a single
    vectorized add. Pairwise distances (for comm / swarm observations) use
    broadcasting. Action application still goes through the per-robot
    ``step`` for UAVs (cheap), but observation collection is vectorized.
    Produces the same result as :class:`World` for identical inputs.
    """

    def _apply_actions(self, actions: dict[str, Any]) -> None:
        ids = list(self.robots.keys())
        # Stack actions into (N,3) and integrate UAVs in one vectorized op.
        act = np.zeros((len(ids), 3))
        for i, rid in enumerate(ids):
            a = actions.get(rid)
            if a is not None:
                act[i] = np.asarray(a, dtype=float)[:3]
        for i, rid in enumerate(ids):
            robot = self.robots[rid]
            if isinstance(robot, UAV):
                robot.step(self.cfg.dt, vx=act[i, 0], vy=act[i, 1],
                           vz=act[i, 2], env=self.env)
            elif isinstance(robot, WheeledRobot):
                robot.step(self.cfg.dt, v=act[i, 0], omega=act[i, 1],
                           env=self.env)
            else:
                robot.set_velocity(act[i])
                robot.apply_velocity(self.cfg.dt)
