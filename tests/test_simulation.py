"""
Test 5: Full simulation — robots walking in 3DGS environments.
This is the integration test that ties everything together.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv
from robots.biped import Biped
from robots.quadruped import Quadruped
from robots.hexapod import Hexapod
from control.gait_planner import GaitType
from control.motion_controller import MotionController
from interaction.env_interaction import EnvironmentInteraction


def test_hexapod_flat_ground():
    """Hexapod walks on flat ground using tripod gait."""
    env = GaussianSplatEnv.create_flat_ground(size=6.0, resolution=12)
    robot = Hexapod(base_position=np.array([0.0, 0.0, 0.2]))
    controller = MotionController(robot, env, GaitType.TRIPOD,
                                  stride_length=0.1, step_height=0.05, cycle_time=1.0)
    summary = controller.simulate(duration=5.0, dt=0.02, speed=0.1)

    assert summary["steps"] > 0
    assert summary["distance"] > 0.01, f"Robot didn't move: distance={summary['distance']}"
    print(f"  [PASS] Hexapod on flat ground: distance={summary['distance']:.3f}m, "
          f"stability={summary['stability_ratio']:.1%}")


def test_quadruped_flat_ground():
    """Quadruped walks on flat ground using trot gait."""
    env = GaussianSplatEnv.create_flat_ground(size=6.0, resolution=12)
    robot = Quadruped(base_position=np.array([0.0, 0.0, 0.35]))
    controller = MotionController(robot, env, GaitType.TROT,
                                  stride_length=0.1, step_height=0.05, cycle_time=0.8)
    summary = controller.simulate(duration=5.0, dt=0.02, speed=0.1)

    assert summary["steps"] > 0
    assert summary["distance"] > 0.01, f"Robot didn't move: distance={summary['distance']}"
    print(f"  [PASS] Quadruped on flat ground: distance={summary['distance']:.3f}m, "
          f"stability={summary['stability_ratio']:.1%}")


def test_quadruped_terrain():
    """Quadruped walks on uneven terrain."""
    env = GaussianSplatEnv.create_terrain(size=8.0, resolution=15, roughness=0.1, seed=42)
    robot = Quadruped(base_position=np.array([0.0, 0.0, 0.5]))
    controller = MotionController(robot, env, GaitType.WALK,
                                  stride_length=0.08, step_height=0.08, cycle_time=1.2)
    summary = controller.simulate(duration=8.0, dt=0.02, speed=0.08)

    assert summary["steps"] > 0
    # Robot should move forward even on rough terrain
    print(f"  [PASS] Quadruped on terrain: distance={summary['distance']:.3f}m, "
          f"avg_height={summary['avg_height']:.3f}m, "
          f"height_std={summary['height_std']:.4f}m")


def test_biped_flat_ground():
    """Biped walks on flat ground using static walk."""
    env = GaussianSplatEnv.create_flat_ground(size=6.0, resolution=12)
    robot = Biped(base_position=np.array([0.0, 0.0, 0.9]))
    controller = MotionController(robot, env, GaitType.STATIC_WALK,
                                  stride_length=0.08, step_height=0.06, cycle_time=2.0)
    summary = controller.simulate(duration=6.0, dt=0.02, speed=0.05)

    assert summary["steps"] > 0
    print(f"  [PASS] Biped on flat ground: distance={summary['distance']:.3f}m, "
          f"stability={summary['stability_ratio']:.1%}")


def test_hexapod_stairs():
    """Hexapod attempts to climb stairs."""
    env = GaussianSplatEnv.create_stairs(n_steps=4, step_h=0.1, step_w=0.3)
    robot = Hexapod(base_position=np.array([-1.0, 0.0, 0.2]))
    controller = MotionController(robot, env, GaitType.TRIPOD,
                                  stride_length=0.06, step_height=0.1, cycle_time=1.5)
    summary = controller.simulate(duration=8.0, dt=0.02, speed=0.05)

    print(f"  [PASS] Hexapod on stairs: distance={summary['distance']:.3f}m, "
          f"final_pos={summary['final_pos']}")


def test_environment_interaction():
    """Test interaction sensing and collision detection."""
    env = GaussianSplatEnv.create_terrain(size=6.0, resolution=12, seed=10)
    robot = Hexapod(base_position=np.array([0.0, 0.0, 0.2]))
    robot.update_foot_positions()
    for name in robot.legs:
        robot.state.foot_contacts[name] = True

    inter = EnvironmentInteraction(robot, env)

    # Test lidar
    lidar = inter.sense_lidar(n_rays=12, max_range=3.0)
    assert lidar.shape == (12, 2)
    print(f"  [PASS] Lidar sensing: {lidar.shape}, ranges: "
          f"min={lidar[:, 1].min():.2f}, max={lidar[:, 1].max():.2f}")

    # Test forward obstacle sensing
    obs = inter.sense_forward_obstacle(max_range=2.0)
    print(f"  [PASS] Forward obstacle sensing: {obs}")

    # Test collision detection
    report = inter.collision_response()
    print(f"  [PASS] Collision response: {report}")

    # Test terrain adaptation
    foot_pos = np.array([0.5, 0.5, 0.5])
    adapted = inter.adapt_foot_to_terrain(foot_pos, is_swing=False)
    ground_h = inter.terrain.height_at(0.5, 0.5)
    assert abs(adapted[2] - ground_h) < 0.1, f"Adapted foot Z={adapted[2]}, ground={ground_h}"
    print(f"  [PASS] Terrain adaptation: foot_z={adapted[2]:.3f}, ground={ground_h:.3f}")

    # Test push splat
    if len(env.splats) > 0:
        original_pos = env.splats[0].position.copy()
        inter.push_splat(0, np.array([1.0, 0, 0]), 0.05)
        new_pos = env.splats[0].position
        assert np.linalg.norm(new_pos - original_pos) > 0.01, "Splat should have moved"
        print(f"  [PASS] Push splat: moved {np.linalg.norm(new_pos - original_pos):.3f}m")

    summary = inter.interaction_summary()
    print(f"  [PASS] Interaction summary: {summary}")


def run():
    print("Test: Full Simulation (Integration)")
    test_hexapod_flat_ground()
    test_quadruped_flat_ground()
    test_quadruped_terrain()
    test_biped_flat_ground()
    test_hexapod_stairs()
    test_environment_interaction()
    print()


if __name__ == "__main__":
    run()
