"""
Abstract base classes for all pluggable algorithms.

The shared lifecycle lets :class:`~simlab.world.World` schedule SLAM,
navigation, swarm control and detection side by side in one simulation loop
without knowing their internals:

1. ``setup(world, cfg)``      — once per episode, inject world + config.
2. ``step(world, obs) -> out``— every tick, consume observations, return an
   :class:`AlgorithmOutput` (actions to apply + algorithm-specific results).
3. ``teardown(world)``        — once per episode, return summary data.
4. ``metrics()``              — current cumulative metric dict.

Sub-bases only narrow ``kind`` and document the expected observation /
action contract; they add no new required methods, so implementing a new
algorithm is a small subclass.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    from simlab.world import World


@dataclass
class AlgorithmOutput:
    """
    What an algorithm returns from one ``step``.

    ``actions`` maps ``robot_id -> control`` and is applied to the robots by
    the world after all algorithms have stepped (so algorithm order does not
    bias who sees whose action first).  ``data`` carries algorithm-specific
    results (e.g. the occupancy grid, detected boxes) that the world records
    into the episode result and that :mod:`simlab.metrics` reads.
    """
    actions: dict[str, Any] = field(default_factory=dict)
    data: dict[str, Any] = field(default_factory=dict)


class AlgorithmBase(ABC):
    """Common lifecycle for every algorithm family."""

    #: Family identifier — one of ``slam | navigation | swarm | detection``.
    kind: str = "algorithm"
    #: Short, unique-within-family name used by the registry.
    name: str = "base"

    def setup(self, world: "World", cfg: Optional[dict] = None) -> None:
        """Called once at episode start. Default: stash config."""
        self.cfg = cfg or {}
        self.world = world

    @abstractmethod
    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        """Advance one simulation tick."""

    def teardown(self, world: "World") -> dict:
        """Called once at episode end. Default: return metrics."""
        return self.metrics()

    def metrics(self) -> dict:
        """Cumulative metrics for this algorithm so far."""
        return {}

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} kind={self.kind!r} name={self.name!r}>"


# ---------------------------------------------------------------------------
# Family sub-bases — narrow the contract, add no required methods.
# ---------------------------------------------------------------------------

class SLAMBase(AlgorithmBase):
    """
    Simultaneous localization and mapping.

    Observation contract (per step, keyed by ``robot_id``):
        ``pose_truth`` (3,)   — ground-truth base position (for eval only).
        ``lidar``      (n,2)  — (angle, range) scan from the robot.
        ``odometry``   (3,)   — delta pose since last step (optional).

    Output ``data`` convention:
        ``map``        — the current map (implementation-defined).
        ``pose_estimate`` (3,) — estimated base position.
    """
    kind = "slam"


class NavigationBase(AlgorithmBase):
    """
    Single-robot goal-directed navigation.

    Observation contract:
        ``pose_truth`` (3,), ``lidar`` (n,2), ``goal`` (3,).

    Output ``actions[robot_id]`` convention:
        ``WheeledRobot`` -> ``(v, omega)``;
        ``UAV``          -> ``(vx, vy, vz)``.
    """
    kind = "navigation"


class SwarmControlBase(AlgorithmBase):
    """
    Multi-robot coordination.

    Observation contract:
        ``poses``  (N,3)  — all robots' positions.
        ``velocities`` (N,3).
        ``comm_snapshot`` — latest :class:`~comm.network.network_simulator.
                            CommSnapshot` or None.

    Output ``actions[robot_id]`` convention: world-frame velocity ``(vx,vy,vz)``
    (the world integrates it for the robot).
    """
    kind = "swarm"


class DetectionBase(AlgorithmBase):
    """
    Target / object detection.

    Observation contract (per robot):
        ``lidar``  (n,2)   or  ``camera_depth`` (H,W).

    Output ``data`` convention:
        ``detections`` — list of dicts ``{"center": (3,), "bbox": [...],
                         "label": str, "score": float}``.
    """
    kind = "detection"
