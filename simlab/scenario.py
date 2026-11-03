"""
Configuration-driven single experiment.

A :class:`Scenario` is a fully serializable description of one episode: which
3DGS scene, which robots at which start poses, optional comm channel,
optional sensors, and which algorithms to run with what configs. ``run``
builds a :class:`~simlab.world.World`, drives it for ``steps`` ticks and
returns an :class:`EpisodeResult` with the per-algorithm final metrics plus
the full metric history.

Scenarios are the unit that :class:`~simlab.benchmark.Benchmark` sweeps and
that :class:`~simlab.orchestrator.Orchestrator` parallelizes.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Any

from core.gaussian_splat_env import GaussianSplatEnv, GaussianSplat
from core.terrain_generator import TerrainGenerator
from robots.mobile_base import WheeledRobot, UAV
from comm.network.network_simulator import CommNetworkSimulator
from comm.channel.statistical import FreeSpaceChannel
from simlab.world import World, WorldConfig
from simlab.registry import registry
from simlab.metrics import collect_episode_metrics


@dataclass
class EpisodeResult:
    scenario_name: str
    steps: int
    final_metrics: dict[str, float]
    history: list[dict[str, dict[str, float]]]
    config: dict

    def metric(self, key: str) -> float:
        return self.final_metrics.get(key, 0.0)


class Scenario:
    """One configurable experiment."""

    def __init__(self, name: str, config: dict):
        self.name = name
        self.config = config

    # -- serialization --------------------------------------------------
    @classmethod
    def from_dict(cls, d: dict) -> "Scenario":
        return cls(d.get("name", "scenario"), d)

    def to_dict(self) -> dict:
        return {"name": self.name, **self.config}

    # -- builders -------------------------------------------------------
    def _build_env(self) -> GaussianSplatEnv:
        ec = self.config.get("env", {})
        if ec.get("terrain"):
            gen = TerrainGenerator()
            splats = gen.generate(ec["terrain"], **ec.get("terrain_kwargs", {}))
            return GaussianSplatEnv(splats, ground_level=ec.get("ground_level", 0.0))
        if ec.get("procedural"):
            return self._procedural_env(ec["procedural"])
        # default: a small synthetic scene with a few obstacle splats
        return self._procedural_env({"n_obstacles": 6, "size": 10.0})

    @staticmethod
    def _procedural_env(cfg: dict) -> GaussianSplatEnv:
        rng = np.random.default_rng(0)
        n = int(cfg.get("n_obstacles", 6))
        size = float(cfg.get("size", 10.0))
        splats: list[GaussianSplat] = []
        # ground plane (a few flat splats)
        for i in range(4):
            for j in range(4):
                splats.append(GaussianSplat(
                    position=np.array([(i - 1.5) * size / 3, (j - 1.5) * size / 3, 0.0]),
                    scale=np.array([size / 3, size / 3, 0.05]),
                    rotation=np.eye(3),
                    opacity=1.0,
                ))
        # obstacles near ground level (so detection eval finds them)
        for _ in range(n):
            x = rng.uniform(-size / 2, size / 2)
            y = rng.uniform(-size / 2, size / 2)
            splats.append(GaussianSplat(
                position=np.array([x, y, 0.5]),
                scale=np.array([0.4, 0.4, 0.5]),
                rotation=np.eye(3),
                opacity=1.0,
            ))
        return GaussianSplatEnv(splats, ground_level=0.0)

    def _build_robots(self) -> dict:
        out = {}
        for rc in self.config.get("robots", []):
            rtype = rc.get("type", "wheeled")
            pos = np.asarray(rc.get("position", [0, 0, 0]), float)
            name = rc.get("name", f"r{len(out)}")
            if rtype == "uav":
                out[name] = UAV(name=name, base_position=pos,
                                max_speed=rc.get("max_speed", 5.0))
            else:
                out[name] = WheeledRobot(name=name, base_position=pos,
                                         max_speed=rc.get("max_speed", 2.0))
        return out

    def _build_comm(self, env) -> Optional[CommNetworkSimulator]:
        cc = self.config.get("comm")
        if cc is None:
            return None
        ch = None
        if cc.get("channel") == "free_space":
            ch = FreeSpaceChannel(carrier_hz=cc.get("carrier_hz", 2.4e9))
        return CommNetworkSimulator(channel_model=ch, env=env)

    def _build_algorithms(self):
        algos = []
        for ac in self.config.get("algorithms", []):
            kind, name = ac["kind"], ac["name"]
            cfg = ac.get("config", {})
            alg = registry.create(kind, name)
            algos.append((alg, cfg))
        return algos

    # -- run ------------------------------------------------------------
    def run(self) -> EpisodeResult:
        env = self._build_env()
        comm = self._build_comm(env)
        wc = self.config.get("world", {})
        world = World(env, config=WorldConfig(
            dt=wc.get("dt", 0.1),
            seed=wc.get("seed", 0),
            lidar_rays=wc.get("lidar_rays", 36),
            lidar_range=wc.get("lidar_range", 5.0),
        ), comm=comm)
        for name, robot in self._build_robots().items():
            world.add_robot(robot)
        algos = self._build_algorithms()
        for alg, cfg in algos:
            world.add_algorithm(alg)
            alg.setup(world, cfg)

        steps = int(self.config.get("steps", 100))
        history = world.run(steps)
        final = collect_episode_metrics(world)
        # teardown
        for alg in world.algorithms:
            try:
                alg.teardown(world)
            except Exception:
                pass
        return EpisodeResult(
            scenario_name=self.name,
            steps=steps,
            final_metrics=final,
            history=[{k: v for k, v in h.items()} for h in history],
            config=self.to_dict(),
        )
