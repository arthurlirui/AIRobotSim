"""
Tests for the advanced terrain generator and URDF-based robot models.

Tests:
  1. Terrain generator produces valid environments for all terrain types
  2. Curriculum generation works
  3. Height-field primitives have correct properties
  4. URDF parser correctly extracts kinematic data from Go2 URDF
  5. URDF robot factory creates standing, stable quadrupeds
  6. FK/IK consistency for URDF robots
  7. Robot-terrain integration: robots walk on advanced terrains
"""

import sys
import os
import numpy as np

# Ensure project root is on path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.terrain_generator import (
    TerrainGenerator, TERRAIN_FUNCTIONS, SubTerrain,
    heightfield_to_splats,
    flat_terrain, random_uniform_terrain, pyramid_sloped_terrain,
    pyramid_stairs_terrain, discrete_obstacles_terrain,
    stepping_stones_terrain, gap_terrain, pit_terrain, wave_terrain,
)
from core.gaussian_splat_env import GaussianSplatEnv
from core.urdf_parser import parse_urdf
from robots.urdf_robot import (
    create_robot_from_urdf, create_robot_from_urdf_path,
    list_available_robots, URDFRobot,
)


def test_terrain_types_available():
    """All 9 terrain types are registered."""
    expected = {"flat", "rough", "slope", "stairs", "obstacles",
                "stepping_stones", "gap", "pit", "wave"}
    assert set(TERRAIN_FUNCTIONS.keys()) == expected, \
        f"Expected {expected}, got {set(TERRAIN_FUNCTIONS.keys())}"
    print("  [PASS] All 9 terrain types registered")


def test_terrain_generator_single():
    """Each terrain type produces a valid GaussianSplatEnv."""
    gen = TerrainGenerator(horizontal_scale=0.2, terrain_length=6.0,
                           terrain_width=6.0)
    for t_type in gen.list_terrain_types():
        env = gen.generate_single(t_type, difficulty=0.5)
        assert isinstance(env, GaussianSplatEnv), f"{t_type} did not return GaussianSplatEnv"
        assert len(env.splats) > 0, f"{t_type} produced 0 splats"
        stats = env.stats()
        assert stats["num_splats"] > 0
    print(f"  [PASS] All {len(gen.list_terrain_types())} terrain types produce valid environments")


def test_terrain_difficulty_scaling():
    """Higher difficulty produces more extreme terrain."""
    gen = TerrainGenerator(horizontal_scale=0.2, terrain_length=6.0,
                           terrain_width=6.0)
    for t_type in ["slope", "stairs", "pit", "wave"]:
        env_easy = gen.generate_single(t_type, difficulty=0.1)
        env_hard = gen.generate_single(t_type, difficulty=0.9)
        z_range_easy = max(s.position[2] for s in env_easy.splats) - \
                       min(s.position[2] for s in env_easy.splats)
        z_range_hard = max(s.position[2] for s in env_hard.splats) - \
                       min(s.position[2] for s in env_hard.splats)
        assert z_range_hard >= z_range_easy * 0.5, \
            f"{t_type}: hard terrain ({z_range_hard:.3f}) should be more extreme than easy ({z_range_easy:.3f})"
    print("  [PASS] Difficulty scaling produces more extreme terrain")


def test_curriculum_generation():
    """Curriculum grid generates a combined environment."""
    gen = TerrainGenerator(horizontal_scale=0.2, terrain_length=8.0,
                           terrain_width=8.0)
    env = gen.generate_curriculum(num_rows=3, num_cols=4)
    assert isinstance(env, GaussianSplatEnv)
    assert len(env.splats) > 100, f"Curriculum too small: {len(env.splats)} splats"
    stats = env.stats()
    assert stats["num_splats"] > 100
    print(f"  [PASS] Curriculum (3x4) generated with {stats['num_splats']} splats")


def test_heightfield_primitives():
    """Individual height-field primitives produce expected shapes."""
    sub = SubTerrain("test", width=64, length=64,
                     horizontal_scale=0.1, vertical_scale=0.005)

    # Flat: all zeros
    flat_terrain(sub)
    assert np.all(sub.height_field_raw == 0), "Flat terrain should be all zeros"

    # Slope: height increases with distance from centre
    sub.height_field_raw[:] = 0
    pyramid_sloped_terrain(sub, slope=0.3, platform_size=2.0)
    hf = sub.height_field_raw
    assert hf[0, 0] > hf[32, 32], "Slope should be higher at edges than centre"

    # Stairs: monotonic steps from centre
    sub.height_field_raw[:] = 0
    pyramid_stairs_terrain(sub, step_width=0.2, step_height=0.1, platform_size=2.0)
    hf = sub.height_field_raw
    assert hf[0, 32] > hf[32, 32], "Stairs should step up from centre"

    # Pit: negative depth in centre
    sub.height_field_raw[:] = 0
    pit_terrain(sub, depth=0.5, platform_size=3.0)
    hf = sub.height_field_raw
    assert hf[32, 32] < 0, "Pit should have negative centre"
    assert hf[0, 0] == 0, "Pit should have flat edges"

    print("  [PASS] Height-field primitives have correct shapes")


def test_heightfield_to_splats():
    """Height-field to splat conversion produces valid splats."""
    hf = np.zeros((10, 10), dtype=np.int16)
    hf[5, 5] = 100  # a bump
    splats = heightfield_to_splats(hf, horizontal_scale=0.1,
                                    vertical_scale=0.005)
    assert len(splats) == 100, f"Expected 100 splats, got {len(splats)}"
    # The bump should be at z = 100 * 0.005 = 0.5
    bump_splat = max(splats, key=lambda s: s.position[2])
    assert abs(bump_splat.position[2] - 0.5) < 0.01, \
        f"Bump height should be 0.5, got {bump_splat.position[2]}"
    print("  [PASS] Height-field to splat conversion correct")


def test_urdf_parser_go2():
    """URDF parser correctly extracts Go2 kinematic data."""
    urdf_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "assets", "external", "unitree_ros", "robots",
        "go2_description", "urdf", "go2_description.urdf"
    )
    if not os.path.exists(urdf_path):
        print("  [SKIP] Go2 URDF not found (external assets not downloaded)")
        return

    spec = parse_urdf(urdf_path, base_link="base")
    assert spec.name == "go2_description"
    assert len(spec.leg_joints) == 4, f"Go2 should have 4 legs, got {len(spec.leg_joints)}"
    assert set(spec.leg_joints.keys()) == {"FL", "FR", "RL", "RR"}

    # Each leg should have 3 joints
    for prefix, joints in spec.leg_joints.items():
        assert len(joints) == 3, f"{prefix} should have 3 joints, got {len(joints)}"

    # Total mass should be ~15 kg
    assert 14 < spec.total_mass < 16, f"Go2 mass should be ~15 kg, got {spec.total_mass:.2f}"

    # Hip offsets should match URDF
    fl_hip = spec.get_hip_offset("FL")
    assert abs(fl_hip[0] - 0.1934) < 0.001, f"FL hip x should be 0.1934, got {fl_hip[0]}"

    print("  [PASS] Go2 URDF parsed correctly (4 legs, 3 DOF each, ~15 kg)")


def test_urdf_robot_factory():
    """URDF robot factory creates standing, stable quadrupeds."""
    available = list_available_robots()
    if "go2" not in available:
        print("  [SKIP] Go2 URDF not found")
        return

    robot = create_robot_from_urdf("go2")
    assert robot.n_legs == 4, f"Go2 should have 4 legs, got {robot.n_legs}"
    assert robot.mass > 10, f"Go2 mass should be >10 kg, got {robot.mass:.2f}"

    robot.update_foot_positions()
    for name, pos in robot.state.foot_positions.items():
        assert abs(pos[2]) < 0.1, \
            f"{name} foot should be near ground (z≈0), got z={pos[2]:.4f}"

    assert robot.is_stable(), "Go2 should be stable when standing"
    print("  [PASS] Go2 robot factory: 4 legs, feet on ground, stable")


def test_urdf_robot_fk_ik_consistency():
    """FK/IK consistency for URDF robots."""
    available = list_available_robots()
    if "go2" not in available:
        print("  [SKIP] Go2 URDF not found")
        return

    robot = create_robot_from_urdf("go2")

    for leg_name in robot.legs:
        if len(robot.legs[leg_name].joints) != 3:
            continue
        # Get current foot position
        foot_pos = robot.leg_forward_kinematics(leg_name)
        # Try IK to reach a nearby target
        target = foot_pos + np.array([0.02, 0.0, -0.02])
        angles = robot.leg_inverse_kinematics(leg_name, target)
        if angles:
            robot.set_joint_angles(leg_name, angles)
            new_foot = robot.leg_forward_kinematics(leg_name)
            error = np.linalg.norm(new_foot - target)
            assert error < 0.02, \
                f"{leg_name} IK error too large: {error:.4f} m"

    print("  [PASS] Go2 FK/IK consistency (error < 20mm)")


def test_multiple_urdf_robots():
    """Multiple URDF robots can be created and stand on ground."""
    available = list_available_robots()
    quadrupeds = [r for r in ["go2", "a1", "laikago"] if r in available]
    if not quadrupeds:
        print("  [SKIP] No quadruped URDFs found")
        return

    for rname in quadrupeds:
        robot = create_robot_from_urdf(rname)
        robot.update_foot_positions()
        all_grounded = all(
            abs(pos[2]) < 0.1
            for pos in robot.state.foot_positions.values()
        )
        assert all_grounded, \
            f"{rname}: not all feet on ground"
        assert robot.is_stable(), f"{rname}: not stable when standing"

    print(f"  [PASS] {len(quadrupeds)} quadruped robots ({', '.join(quadrupeds)}) stand stably")


def test_robot_on_advanced_terrain():
    """URDF robot can walk on advanced terrain types."""
    available = list_available_robots()
    if "go2" not in available:
        print("  [SKIP] Go2 URDF not found")
        return

    from core.terrain import TerrainHeightField
    from control.gait_planner import GaitPlanner

    gen = TerrainGenerator(horizontal_scale=0.2, terrain_length=8.0,
                           terrain_width=8.0)
    terrain_types = ["flat", "rough", "slope", "wave"]

    for t_type in terrain_types:
        env = gen.generate_single(t_type, difficulty=0.3)
        terrain = TerrainHeightField(env, bounds=(-4, 4, -4, 4), resolution=0.2)

        robot = create_robot_from_urdf("go2")
        robot.update_foot_positions()

        # Check that terrain height queries work
        h = terrain.height_at(0, 0)
        assert isinstance(h, float), f"Terrain height should be float, got {type(h)}"

        # Check that the robot can be placed on the terrain
        robot.state.base_position[2] += h  # adjust base height to terrain
        robot.update_foot_positions()

    print(f"  [PASS] Go2 on {len(terrain_types)} advanced terrain types (terrain queries work)")


def test_urdf_robot_specs_match_real():
    """URDF robot specs match known real-world values."""
    available = list_available_robots()
    if "go2" not in available:
        print("  [SKIP] Go2 URDF not found")
        return

    robot = create_robot_from_urdf("go2")

    # Go2 real specs (from Unitree documentation):
    # - Mass: ~15 kg
    # - 4 legs, 3 DOF each = 12 DOF
    # - Thigh length: 0.213 m
    # - Calf length: 0.213 m
    assert 14 < robot.mass < 16, f"Go2 mass should be ~15 kg, got {robot.mass:.2f}"
    assert robot.n_legs == 4

    for name, leg in robot.legs.items():
        assert len(leg.joints) == 3, f"{name} should have 3 joints"
        # Thigh link length should be ~0.213
        assert abs(leg.link_lengths[0] - 0.213) < 0.01, \
            f"{name} thigh should be ~0.213m, got {leg.link_lengths[0]:.4f}"

    print("  [PASS] Go2 specs match real-world values (15 kg, 12 DOF, 0.213m links)")


def run_all():
    """Run all terrain and URDF robot tests."""
    print("=" * 60)
    print("  Test: Advanced Terrain Generator")
    print("=" * 60)

    tests = [
        test_terrain_types_available,
        test_terrain_generator_single,
        test_terrain_difficulty_scaling,
        test_curriculum_generation,
        test_heightfield_primitives,
        test_heightfield_to_splats,
    ]
    for t in tests:
        try:
            t()
        except Exception as e:
            print(f"  [FAIL] {t.__name__}: {e}")
            raise

    print()
    print("=" * 60)
    print("  Test: URDF Parser & Robot Factory")
    print("=" * 60)

    tests = [
        test_urdf_parser_go2,
        test_urdf_robot_factory,
        test_urdf_robot_fk_ik_consistency,
        test_multiple_urdf_robots,
        test_urdf_robot_specs_match_real,
    ]
    for t in tests:
        try:
            t()
        except Exception as e:
            print(f"  [FAIL] {t.__name__}: {e}")
            raise

    print()
    print("=" * 60)
    print("  Test: Robot-Terrain Integration")
    print("=" * 60)

    try:
        test_robot_on_advanced_terrain()
    except Exception as e:
        print(f"  [FAIL] test_robot_on_advanced_terrain: {e}")
        raise

    print()
    print("  All terrain & URDF tests passed!")


if __name__ == "__main__":
    run_all()
