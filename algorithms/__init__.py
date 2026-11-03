"""
Algorithm package: pluggable robot algorithms tested inside the 3DGS world.

Four algorithm families share a common lifecycle (``setup → step → teardown
→ metrics``) so they can be scheduled uniformly by :mod:`simlab.world`:

* **SLAM** (:mod:`algorithms.slam`) — build a map and localize within it.
* **Navigation** (:mod:`algorithms.navigation`) — drive a robot to a goal.
* **Swarm control** (:mod:`algorithms.swarm`) — coordinate many robots.
* **Detection** (:mod:`algorithms.detection`) — perceive targets.

Every family has an abstract base in :mod:`algorithms.base` and at least one
reference implementation. New algorithms register themselves through
:data:`algorithms.registry` (a module-level :class:`~simlab.registry.
AlgorithmRegistry` re-exported here) so scenarios / benchmarks can find them
by ``(kind, name)`` without importing the implementing module explicitly.
"""

from algorithms.base import (
    AlgorithmBase,
    SLAMBase,
    NavigationBase,
    SwarmControlBase,
    DetectionBase,
    AlgorithmOutput,
)
from algorithms.slam import OccupancyGridSLAM, PoseGraphSLAM
from algorithms.navigation import PotentialFieldNav, AStarDwaNav
from algorithms.swarm import BoidsSwarm, LeaderFollowerSwarm, ConsensusSwarm
from algorithms.detection import EuclideanClusterDetector, RayEnvelopeDetector

# Importing the modules above has the side effect of registering every
# implementation; expose the registry afterwards so callers can query it.
from simlab.registry import registry, register  # noqa: E402

__all__ = [
    "AlgorithmBase", "SLAMBase", "NavigationBase", "SwarmControlBase",
    "DetectionBase", "AlgorithmOutput",
    "OccupancyGridSLAM", "PoseGraphSLAM",
    "PotentialFieldNav", "AStarDwaNav",
    "BoidsSwarm", "LeaderFollowerSwarm", "ConsensusSwarm",
    "EuclideanClusterDetector", "RayEnvelopeDetector",
    "registry", "register",
]
