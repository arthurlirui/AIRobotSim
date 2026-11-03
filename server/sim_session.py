"""
One live WebSocket simulation session: a :class:`~simlab.world.World` built
from a scenario config dict, kept alive so the client can step it frame by
frame.

The env/robot/comm/algorithm construction mirrors
:meth:`simlab.scenario.Scenario.run` but stops short of running the episode,
exposing the ``World`` for interactive stepping.
"""

from __future__ import annotations

from typing import Any

from simlab.world import World, WorldConfig
from simlab.scenario import Scenario


class SimSession:
    """A build-once, step-many simulation world."""

    def __init__(self, config: dict):
        self.config = config
        sc = Scenario("live", config)
        self.env = sc._build_env()
        self.comm = sc._build_comm(self.env)
        wc = config.get("world", {})
        self.world = World(self.env, config=WorldConfig(
            dt=wc.get("dt", 0.1),
            seed=wc.get("seed", 0),
            lidar_rays=wc.get("lidar_rays", 36),
            lidar_range=wc.get("lidar_range", 5.0),
        ), comm=self.comm)
        for name, robot in sc._build_robots().items():
            self.world.add_robot(robot)
        for alg, cfg in sc._build_algorithms():
            self.world.add_algorithm(alg)
            alg.setup(self.world, cfg)
        self.step_index = 0

    def describe(self) -> dict[str, Any]:
        """Static world description sent to the client right after create."""
        return {
            "robots": [
                {
                    "id": rid,
                    "kind": type(robot).__name__,
                    "position": [float(x) for x in robot.position],
                }
                for rid, robot in self.world.robots.items()
            ],
            "algorithms": [repr(a) for a in self.world.algorithms],
            "dt": self.world.cfg.dt,
            "has_comm": self.comm is not None,
        }
