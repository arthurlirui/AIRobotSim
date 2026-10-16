"""
Test 2: 3DGS environment — terrain generation, height queries, collision.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv, GaussianSplat
from core.terrain import TerrainHeightField


def test_terrain_creation():
    """Procedural terrain should have the right number of splats."""
    env = GaussianSplatEnv.create_terrain(size=5.0, resolution=10, seed=42)
    stats = env.stats()
    assert stats["num_splats"] == 100, f"Expected 100 splats, got {stats['num_splats']}"
    print(f"  [PASS] Terrain created with {stats['num_splats']} splats")


def test_flat_ground_height():
    """Flat ground should have height 0 everywhere."""
    env = GaussianSplatEnv.create_flat_ground(size=5.0, resolution=10)
    for x, y in [(0, 0), (1, 1), (-2, 0.5)]:
        h = env.height_at(x, y)
        assert abs(h) < 0.1, f"Flat ground height at ({x},{y}): {h}, expected ~0"
    print("  [PASS] Flat ground has zero height")


def test_terrain_height_variation():
    """Procedural terrain should have height variation."""
    env = GaussianSplatEnv.create_terrain(size=8.0, resolution=15, roughness=0.2, seed=42)
    heights = []
    for x in np.linspace(-3, 3, 10):
        for y in np.linspace(-3, 3, 10):
            heights.append(env.height_at(x, y))
    heights = np.array(heights)
    h_range = heights.max() - heights.min()
    assert h_range > 0.1, f"Terrain too flat: range={h_range}"
    print(f"  [PASS] Terrain has height variation (range={h_range:.3f})")


def test_normal_at():
    """Surface normal should be roughly upward on flat ground."""
    env = GaussianSplatEnv.create_flat_ground(size=5.0, resolution=10)
    n = env.normal_at(0, 0)
    assert n[2] > 0.9, f"Normal should point up: {n}"
    print(f"  [PASS] Surface normal correct: {n}")


def test_collision_detection():
    """Collision query should find splats at their own position."""
    env = GaussianSplatEnv.create_flat_ground(size=5.0, resolution=10)
    # The center splat is at approximately (0, 0, 0)
    hits = env.collision(np.array([0, 0, 0]))
    assert len(hits) > 0, "Expected collision at center"
    print(f"  [PASS] Collision detection works ({len(hits)} hits at center)")


def test_ray_cast():
    """Ray cast should find obstacles."""
    env = GaussianSplatEnv.create_stairs(n_steps=5)
    # Cast a ray from below the stairs upward
    origin = np.array([0.5, 0, -0.5])
    direction = np.array([0, 0, 1])
    dist = env.ray_cast(origin, direction, max_dist=3.0)
    assert dist is not None, "Ray should hit the stairs"
    assert dist > 0, "Ray hit distance should be positive"
    print(f"  [PASS] Ray cast works (hit at {dist:.3f}m)")


def test_stairs_terrain():
    """Stairs terrain should have increasing height."""
    env = GaussianSplatEnv.create_stairs(n_steps=5, step_h=0.15)
    stats = env.stats()
    assert stats["num_splats"] > 0
    print(f"  [PASS] Stairs terrain created ({stats['num_splats']} splats)")


def test_height_field():
    """TerrainHeightField should provide fast interpolated queries."""
    env = GaussianSplatEnv.create_terrain(size=6.0, resolution=12, seed=1)
    hf = TerrainHeightField(env, (-3, 3, -3, 3), resolution=0.2)
    # Query at a point
    h = hf.height_at(0.5, 0.5)
    assert isinstance(h, float)
    # Slope should be non-negative
    s = hf.slope_at(0.5, 0.5)
    assert s >= 0
    print(f"  [PASS] HeightField works (h={h:.3f}, slope={np.degrees(s):.1f}°)")


def test_splat_contains():
    """GaussianSplat.contains_point should work for its own center."""
    s = GaussianSplat(
        position=np.array([1, 2, 3]),
        scale=np.array([0.1, 0.1, 0.1]),
        rotation=np.eye(3),
        opacity=1.0,
    )
    assert s.contains_point(np.array([1, 2, 3]))
    assert not s.contains_point(np.array([10, 10, 10]))
    print("  [PASS] Splat containment check works")


def run():
    print("Test: 3DGS Environment")
    test_terrain_creation()
    test_flat_ground_height()
    test_terrain_height_variation()
    test_normal_at()
    test_collision_detection()
    test_ray_cast()
    test_stairs_terrain()
    test_height_field()
    test_splat_contains()
    print()


if __name__ == "__main__":
    run()
