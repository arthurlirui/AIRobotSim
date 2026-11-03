"""
Tests for the algorithm design platform (algorithms/ + platform/).

Exercises:
- registry discovery of all four algorithm families,
- each reference implementation runs to completion and produces metrics,
- Scenario.run end-to-end with navigation + detection + SLAM,
- Benchmark sweep + comparison report,
- VectorizedSwarm matches serial World for the same swarm inputs.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv, GaussianSplat
from robots.mobile_base import WheeledRobot, UAV
from comm.network.network_simulator import CommNetworkSimulator
from algorithms import (
    registry, OccupancyGridSLAM, PotentialFieldNav, BoidsSwarm,
    EuclideanClusterDetector, PoseGraphSLAM, AStarDwaNav,
    LeaderFollowerSwarm, ConsensusSwarm, RayEnvelopeDetector,
)
from simlab.world import World, WorldConfig, VectorizedSwarm
from simlab.scenario import Scenario
from simlab.benchmark import Benchmark
from simlab.metrics import aggregate, compare


def _make_env(n_obstacles: int = 5, size: float = 10.0) -> GaussianSplatEnv:
    rng = np.random.default_rng(42)
    splats = []
    for i in range(4):
        for j in range(4):
            splats.append(GaussianSplat(
                position=np.array([(i - 1.5) * size / 3, (j - 1.5) * size / 3, 0.0]),
                scale=np.array([size / 3, size / 3, 0.05]),
                rotation=np.eye(3), opacity=1.0))
    for _ in range(n_obstacles):
        splats.append(GaussianSplat(
            position=np.array([rng.uniform(-size / 2, size / 2),
                               rng.uniform(-size / 2, size / 2), 0.5]),
            scale=np.array([0.4, 0.4, 0.5]),
            rotation=np.eye(3), opacity=1.0))
    return GaussianSplatEnv(splats, ground_level=0.0)


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

def test_registry():
    print("  [test] registry discovery")
    slam = registry.list("slam")
    nav = registry.list("navigation")
    swarm = registry.list("swarm")
    det = registry.list("detection")
    assert ("slam", "occupancy_grid") in slam, slam
    assert ("slam", "pose_graph") in slam, slam
    assert ("navigation", "potential_field") in nav, nav
    assert ("navigation", "astar_dwa") in nav, nav
    assert ("swarm", "boids") in swarm, swarm
    assert ("swarm", "leader_follower") in swarm, swarm
    assert ("swarm", "consensus") in swarm, swarm
    assert ("detection", "euclidean_cluster") in det, det
    assert ("detection", "ray_envelope") in det, det
    # create round-trip
    alg = registry.create("slam", "occupancy_grid")
    assert alg.kind == "slam" and alg.name == "occupancy_grid"
    # unknown
    try:
        registry.create("slam", "does_not_exist")
        assert False, "expected KeyError"
    except KeyError:
        pass
    print("    OK: 4 families, 9 implementations registered")


# ---------------------------------------------------------------------------
# SLAM
# ---------------------------------------------------------------------------

def test_slam():
    print("  [test] SLAM (occupancy grid + pose graph)")
    env = _make_env()
    robot = WheeledRobot(name="r0", base_position=np.array([-3.0, 0.0, 0.0]))
    world = World(env, config=WorldConfig(lidar_rays=24, lidar_range=5.0))
    world.add_robot(robot)
    slam = OccupancyGridSLAM()
    world.add_algorithm(slam)
    slam.setup(world, {"robot_id": "r0", "resolution": 0.5,
                       "bounds": (-8, 8, -8, 8)})
    for _ in range(30):
        # drive the robot in a straight line so SLAM sees the scene
        world._apply_actions({"r0": np.array([0.8, 0.0])})
        world.step()
    m = slam.metrics()
    assert m["slam.n_steps"] == 30, m
    assert m["slam.occupied_cells"] >= 0, m
    # pose graph variant
    pg = PoseGraphSLAM()
    pg.setup(world, {"robot_id": "r0", "resolution": 0.5,
                     "bounds": (-8, 8, -8, 8)})
    out = pg.step(world, world._collect_observations())
    assert out.data["pose_graph"]["nodes"] == 1
    print("    OK: occupancy grid + pose graph run, metrics produced")


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------

def test_navigation():
    print("  [test] navigation (potential field + A*-DWA)")
    for cls, nm in ((PotentialFieldNav, "potential_field"),
                    (AStarDwaNav, "astar_dwa")):
        env = _make_env(n_obstacles=3)
        robot = WheeledRobot(name="r0", base_position=np.array([-4.0, 0.0, 0.0]),
                             max_speed=1.5)
        world = World(env, config=WorldConfig(lidar_rays=24, lidar_range=5.0))
        world.add_robot(robot)
        nav = cls()
        world.add_algorithm(nav)
        nav.setup(world, {"robot_id": "r0", "goal": [4.0, 0.0, 0.0],
                          "reach_radius": 0.5})
        for _ in range(150):
            world.step()
            if nav.reached:
                break
        m = nav.metrics()
        assert "nav.reach_rate" in m and "nav.path_length" in m, m
        print(f"    OK: {nm} reach_rate={m['nav.reach_rate']} "
              f"path={m['nav.path_length']:.2f}")


# ---------------------------------------------------------------------------
# Swarm
# ---------------------------------------------------------------------------

def test_swarm():
    print("  [test] swarm (boids + leader-follower + consensus)")
    # boids with comm
    env = _make_env(n_obstacles=0)
    comm = CommNetworkSimulator()
    world = World(env, config=WorldConfig(), comm=comm)
    ids = []
    for i in range(5):
        r = UAV(name=f"u{i}",
                base_position=np.array([np.cos(2 * np.pi * i / 5) * 3,
                                        np.sin(2 * np.pi * i / 5) * 3, 1.0]),
                max_speed=2.0)
        world.add_robot(r)
        ids.append(r.name)
    boids = BoidsSwarm()
    world.add_algorithm(boids)
    boids.setup(world, {"robot_ids": ids, "perception": 5.0, "max_speed": 1.0})
    for _ in range(40):
        world.step()
    m = boids.metrics()
    assert m["swarm.steps"] == 40, m

    # leader-follower
    lf = LeaderFollowerSwarm()
    lf.setup(world, {"robot_ids": ids, "leader_id": "u0",
                     "goal": [5.0, 5.0], "max_speed": 1.0,
                     "offsets": {f"u{i}": [float(i) * 0.5, 0.0]
                                 for i in range(1, 5)}})
    for _ in range(20):
        out = lf.step(world, world._collect_observations())
        world._apply_actions(out.actions)
    assert lf.metrics()["swarm.formation_error_mean"] >= 0.0

    # consensus
    cons = ConsensusSwarm()
    cons.setup(world, {"robot_ids": ids, "gain": 0.5, "max_speed": 1.0})
    for _ in range(30):
        out = cons.step(world, world._collect_observations())
        world._apply_actions(out.actions)
    disp = cons.metrics()["swarm.dispersion_final"]
    assert disp >= 0.0
    print(f"    OK: boids+comm, leader-follower, consensus (dispersion={disp:.2f})")


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

def test_detection():
    print("  [test] detection (euclidean cluster + ray envelope)")
    env = _make_env(n_obstacles=5)
    robot = WheeledRobot(name="r0", base_position=np.array([0.0, 0.0, 0.0]))
    world = World(env, config=WorldConfig(lidar_rays=48, lidar_range=5.0))
    world.add_robot(robot)
    for cls, nm in ((EuclideanClusterDetector, "euclidean_cluster"),
                    (RayEnvelopeDetector, "ray_envelope")):
        det = cls()
        world.add_algorithm(det)
        det.setup(world, {"robot_id": "r0"})
        for _ in range(5):
            world.step()
        m = det.metrics()
        assert "detection.f1" in m, m
        # remove so the next detector doesn't double-count
        world.algorithms.remove(det)
    print("    OK: both detectors produce precision/recall/f1")


# ---------------------------------------------------------------------------
# Scenario end-to-end
# ---------------------------------------------------------------------------

def test_scenario():
    print("  [test] scenario end-to-end (nav + slam + detection)")
    sc = Scenario.from_dict({
        "name": "integrated",
        "env": {"procedural": {"n_obstacles": 5, "size": 10}},
        "robots": [{"name": "r0", "type": "wheeled",
                    "position": [-4, 0, 0], "max_speed": 1.5}],
        "comm": {"channel": "free_space"},
        "algorithms": [
            {"kind": "navigation", "name": "potential_field",
             "config": {"robot_id": "r0", "goal": [4, 0, 0]}},
            {"kind": "slam", "name": "occupancy_grid",
             "config": {"robot_id": "r0", "resolution": 0.5,
                        "bounds": (-8, 8, -8, 8)}},
            {"kind": "detection", "name": "euclidean_cluster",
             "config": {"robot_id": "r0"}},
        ],
        "steps": 60,
        "world": {"dt": 0.1, "lidar_rays": 24, "lidar_range": 5.0},
    })
    res = sc.run()
    assert res.steps == 60, res.steps
    assert "nav.reach_rate" in res.final_metrics
    assert "slam.ate_mean" in res.final_metrics
    assert "detection.f1" in res.final_metrics
    assert len(res.history) == 60
    print(f"    OK: nav.reach={res.final_metrics['nav.reach_rate']:.0f}, "
          f"slam.ate={res.final_metrics['slam.ate_mean']:.3f}, "
          f"det.f1={res.final_metrics['detection.f1']:.2f}")


# ---------------------------------------------------------------------------
# Benchmark + comparison
# ---------------------------------------------------------------------------

def test_benchmark():
    print("  [test] benchmark sweep + comparison")
    base = {
        "env": {"procedural": {"n_obstacles": 4, "size": 10}},
        "robots": [{"name": "r0", "type": "wheeled",
                    "position": [-4, 0, 0], "max_speed": 1.5}],
        "algorithms": [{"kind": "navigation", "name": "potential_field",
                        "config": {"robot_id": "r0", "goal": [4, 0, 0]}}],
        "steps": 50,
    }
    bench = Benchmark("nav_sweep")
    bench.add_sweep(base, "steps", [20, 40])
    report = bench.run()
    assert report.n_episodes == 2
    s = report.summary()
    assert "nav.path_length" in s, s
    comp = report.comparison(["nav.reach_rate", "nav.path_length"])
    assert "nav.reach_rate" in comp
    print(f"    OK: 2 episodes, summary keys={len(s)}")


# ---------------------------------------------------------------------------
# Vectorized swarm matches serial world
# ---------------------------------------------------------------------------

def test_vectorized_matches_serial():
    print("  [test] vectorized swarm == serial world")
    n = 4
    ids = [f"u{i}" for i in range(n)]

    def make_world(vec):
        env = _make_env(n_obstacles=0)
        w = VectorizedSwarm(env, config=WorldConfig(seed=1)) if vec \
            else World(env, config=WorldConfig(seed=1))
        for i in range(n):
            w.add_robot(UAV(name=ids[i],
                            base_position=np.array([float(i), 0.0, 1.0]),
                            max_speed=2.0))
        return w

    ws = make_world(False)
    wv = make_world(True)
    # apply identical actions and compare positions
    for _ in range(10):
        act = {rid: np.array([0.5, 0.0, 0.0]) for rid in ids}
        ws._apply_actions(act)
        wv._apply_actions(act)
    for rid in ids:
        d = float(np.linalg.norm(ws.robots[rid].position
                                 - wv.robots[rid].position))
        assert d < 1e-9, f"{rid} diverged by {d}"
    print("    OK: positions match to machine precision")


def run():
    print("=" * 60)
    print("  Algorithm Platform (algorithms/ + platform/)")
    print("=" * 60)
    test_registry()
    test_slam()
    test_navigation()
    test_swarm()
    test_detection()
    test_scenario()
    test_benchmark()
    test_vectorized_matches_serial()
    print()
    print("  All algorithm-platform tests passed.")


if __name__ == "__main__":
    run()
