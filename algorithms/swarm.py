"""
Swarm control reference implementations.

All three output per-robot world-frame velocities ``(vx, vy, vz)`` which the
world integrates. They share a small helper that turns a desired 2-D velocity
into the action dict for a set of homogeneous robots.

* :class:`BoidsSwarm` — Reynolds' separation / alignment / cohesion rules.
* :class:`LeaderFollowerSwarm` — one leader drives toward a goal; followers
  track a fixed formation offset from the leader.
* :class:`ConsensusSwarm` — average-consensus on positions over the comm
  topology (each robot moves toward the mean of its neighbours' positions).
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from algorithms.base import SwarmControlBase, AlgorithmOutput
from simlab.registry import register

if TYPE_CHECKING:
    from simlab.world import World


def _actions_from_vel(vel: np.ndarray, ids: list[str]) -> dict:
    return {rid: vel[i].copy() for i, rid in enumerate(ids)}


class BoidsSwarm(SwarmControlBase):
    """Classic Reynolds boids with tunable weights."""

    name = "boids"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.robot_ids: list[str] = list(self.cfg.get("robot_ids", []))
        self.w_sep = float(self.cfg.get("w_sep", 1.5))
        self.w_align = float(self.cfg.get("w_align", 1.0))
        self.w_coh = float(self.cfg.get("w_coh", 1.0))
        self.perception = float(self.cfg.get("perception", 4.0))
        self.max_speed = float(self.cfg.get("max_speed", 1.0))
        self._steps = 0
        self._connected_steps = 0
        self._total_steps = 0

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        self._steps += 1
        self._total_steps += 1
        poses = np.asarray(observations.get("poses", np.zeros((0, 3))), float)
        vels = np.asarray(observations.get("velocities", np.zeros((0, 3))), float)
        n = len(self.robot_ids)
        if n == 0 or poses.shape[0] == 0:
            return AlgorithmOutput()
        pos = poses[:, :2]
        vel = vels[:, :2]

        # pairwise distances
        diff = pos[:, None, :] - pos[None, :, :]          # (n,n,2)
        dist = np.linalg.norm(diff, axis=2)               # (n,n)
        np.fill_diagonal(dist, np.inf)
        within = dist < self.perception

        # separation
        sep = np.zeros_like(pos)
        close = dist < self.perception * 0.5
        for i in range(n):
            nbrs = diff[i, close[i]]
            if len(nbrs):
                sep[i] = -nbrs.sum(axis=0) / max(np.linalg.norm(nbrs.sum(axis=0)), 1e-6)
        # alignment
        align = np.zeros_like(pos)
        for i in range(n):
            nbrs = vel[within[i]]
            if len(nbrs):
                align[i] = nbrs.mean(axis=0)
        # cohesion
        coh = np.zeros_like(pos)
        for i in range(n):
            nbrs = pos[within[i]]
            if len(nbrs):
                coh[i] = nbrs.mean(axis=0) - pos[i]

        desired = (self.w_sep * sep + self.w_align * align + self.w_coh * coh)
        norms = np.linalg.norm(desired, axis=1, keepdims=True)
        desired = np.where(norms > 1e-6, desired / np.maximum(norms, 1e-6), 0.0)
        desired *= self.max_speed
        full = np.zeros((n, 3))
        full[:, :2] = desired

        # connectivity metric from comm snapshot
        snap = observations.get("comm_snapshot")
        if snap is not None and getattr(snap, "topology_adjacency", None) is not None:
            adj = snap.topology_adjacency
            if adj.shape[0] == n and adj.any():
                self._connected_steps += 1

        return AlgorithmOutput(actions=_actions_from_vel(full, self.robot_ids),
                               data={"mean_speed": float(np.mean(norms))})

    def metrics(self) -> dict:
        conn_rate = (self._connected_steps / self._total_steps
                     if self._total_steps else 0.0)
        return {
            "swarm.connectivity_rate": conn_rate,
            "swarm.steps": self._steps,
        }


@register("swarm", "boids")
class _RegisteredBoidsSwarm(BoidsSwarm):
    pass


class LeaderFollowerSwarm(SwarmControlBase):
    """Leader tracks a goal; followers hold a formation offset."""

    name = "leader_follower"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.robot_ids: list[str] = list(self.cfg.get("robot_ids", []))
        self.leader_id: str = self.cfg.get("leader_id", self.robot_ids[0])
        self.goal = np.asarray(self.cfg.get("goal", [0.0, 0.0]), float)[:2]
        self.max_speed = float(self.cfg.get("max_speed", 1.0))
        # formation offsets: dict robot_id -> (x, y) relative to leader
        self.offsets = self.cfg.get("offsets", {})
        self._formation_errors: list[float] = []
        self._steps = 0

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        self._steps += 1
        poses = np.asarray(observations.get("poses", np.zeros((0, 3))), float)
        n = len(self.robot_ids)
        if n == 0 or poses.shape[0] == 0:
            return AlgorithmOutput()
        idx = {rid: i for i, rid in enumerate(self.robot_ids)}
        leader_pos = poses[idx[self.leader_id], :2]

        full = np.zeros((n, 3))
        # leader velocity toward goal
        to_goal = self.goal - leader_pos
        d = np.linalg.norm(to_goal)
        if d > 0.1:
            full[idx[self.leader_id], :2] = (to_goal / d) * self.max_speed

        # followers toward (leader_pos + offset)
        errs = 0.0
        for rid in self.robot_ids:
            if rid == self.leader_id:
                continue
            off = np.asarray(self.offsets.get(rid, [0.0, 0.0]), float)
            target = leader_pos + off
            err = target - poses[idx[rid], :2]
            en = np.linalg.norm(err)
            errs += en
            if en > 1e-3:
                full[idx[rid], :2] = (err / en) * min(self.max_speed, en)
        self._formation_errors.append(errs / max(n - 1, 1))

        return AlgorithmOutput(actions=_actions_from_vel(full, self.robot_ids),
                               data={"formation_error": errs / max(n - 1, 1)})

    def metrics(self) -> dict:
        mfe = float(np.mean(self._formation_errors)) if self._formation_errors else 0.0
        return {"swarm.formation_error_mean": mfe, "swarm.steps": self._steps}


@register("swarm", "leader_follower")
class _RegisteredLeaderFollower(LeaderFollowerSwarm):
    pass


class ConsensusSwarm(SwarmControlBase):
    """Average-consensus on position over the comm graph."""

    name = "consensus"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.robot_ids: list[str] = list(self.cfg.get("robot_ids", []))
        self.gain = float(self.cfg.get("gain", 0.5))
        self.max_speed = float(self.cfg.get("max_speed", 1.0))
        self._steps = 0
        self._dispersion: list[float] = []

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        self._steps += 1
        poses = np.asarray(observations.get("poses", np.zeros((0, 3))), float)
        n = len(self.robot_ids)
        if n == 0 or poses.shape[0] == 0:
            return AlgorithmOutput()
        pos = poses[:, :2]
        # adjacency from comm snapshot (fallback to all-to-all)
        snap = observations.get("comm_snapshot")
        adj = None
        if snap is not None and getattr(snap, "topology_adjacency", None) is not None:
            adj = snap.topology_adjacency
            if adj.shape[0] != n:
                adj = None
        if adj is None:
            adj = np.ones((n, n), dtype=bool)
            np.fill_diagonal(adj, False)

        desired = np.zeros_like(pos)
        for i in range(n):
            nbrs = pos[adj[i]]
            if len(nbrs):
                desired[i] = self.gain * (nbrs.mean(axis=0) - pos[i])
        norms = np.linalg.norm(desired, axis=1, keepdims=True)
        desired = np.where(norms > self.max_speed,
                           desired / np.maximum(norms, 1e-6) * self.max_speed,
                           desired)
        full = np.zeros((n, 3))
        full[:, :2] = desired

        # dispersion = mean pairwise distance (should shrink to 0)
        if n > 1:
            d = pos[:, None, :] - pos[None, :, :]
            pw = np.linalg.norm(d, axis=2)
            self._dispersion.append(float(pw[np.triu_indices(n, 1)].mean()))

        return AlgorithmOutput(actions=_actions_from_vel(full, self.robot_ids),
                               data={"dispersion": self._dispersion[-1]
                                     if self._dispersion else 0.0})

    def metrics(self) -> dict:
        return {
            "swarm.dispersion_final": float(self._dispersion[-1])
            if self._dispersion else 0.0,
            "swarm.dispersion_mean": float(np.mean(self._dispersion))
            if self._dispersion else 0.0,
            "swarm.steps": self._steps,
        }


@register("swarm", "consensus")
class _RegisteredConsensus(ConsensusSwarm):
    pass
