"""
Algorithm design platform: a scalable, extensible, high-performance simulation
harness for testing SLAM, navigation, swarm control and detection algorithms
inside 3D Gaussian Splatting (3DGS) scenes, built on the existing AIRobotSim
framework.

Two new top-level packages cooperate:

* :mod:`algorithms` — the four algorithm families and their reference
  implementations, all sharing a common ``setup/step/teardown/metrics``
  lifecycle.
* :mod:`platform` — the simulation kernel that owns the scene, robots, comm
  network and sensors, schedules the algorithms every tick, drives
  configuration-driven :class:`Scenario` experiments, runs
  :class:`Benchmark` sweeps, and (optionally) fans sweeps out across
  processes via :class:`Orchestrator`.

Typical use::

    from simlab.scenario import Scenario
    sc = Scenario.from_dict({
        "env": {"procedural": {"n_obstacles": 6, "size": 10}},
        "robots": [{"name": "r0", "type": "wheeled",
                    "position": [-4, 0, 0]}],
        "algorithms": [
            {"kind": "navigation", "name": "potential_field",
             "config": {"robot_id": "r0", "goal": [4, 0, 0]}},
        ],
        "steps": 200,
    })
    result = sc.run()
    print(result.final_metrics)
"""

from simlab.registry import AlgorithmRegistry, registry, register
from simlab.world import World, WorldConfig, VectorizedSwarm
from simlab.scenario import Scenario, EpisodeResult
from simlab.benchmark import Benchmark, BenchmarkReport
from simlab.orchestrator import Orchestrator
from simlab.metrics import collect_episode_metrics, aggregate, compare

__all__ = [
    "AlgorithmRegistry", "registry", "register",
    "World", "WorldConfig", "VectorizedSwarm",
    "Scenario", "EpisodeResult",
    "Benchmark", "BenchmarkReport",
    "Orchestrator",
    "collect_episode_metrics", "aggregate", "compare",
]
