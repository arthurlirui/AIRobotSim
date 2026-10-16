"""
Advanced terrain generator for legged robot testing.

Implements procedurally generated height-field terrains inspired by the
legged_gym / Isaac Gym terrain_utils library and the GenTe benchmark:
  - Flat ground
  - Pyramid slopes (varying difficulty)
  - Pyramid stairs (ascending/descending)
  - Discrete obstacles (random rectangular blocks)
  - Stepping stones
  - Gaps (crevasses)
  - Pits
  - Rough / uneven ground (random uniform noise)
  - Wave field (rolling hills)

Each terrain type is generated as a 2-D NumPy height-field.  The field can
then be converted to a ``GaussianSplatEnv`` for use in the simulation, or
queried directly via a ``TerrainHeightField``-like interface.

The generator is **self-contained** — it does not depend on Isaac Gym or any
external physics engine, only NumPy.
"""

from __future__ import annotations

import numpy as np
from typing import Optional

from .gaussian_splat_env import GaussianSplat, GaussianSplatEnv
from .math_utils import rot_axis_angle


# ---------------------------------------------------------------------------
# Height-field terrain primitives
# ---------------------------------------------------------------------------

class SubTerrain:
    """A single sub-terrain tile with a raw int16 height-field."""

    def __init__(self, name: str, width: int = 128, length: int = 128,
                 horizontal_scale: float = 0.1, vertical_scale: float = 0.005):
        self.name = name
        self.width = width
        self.length = length
        self.horizontal_scale = horizontal_scale
        self.vertical_scale = vertical_scale
        self.height_field_raw = np.zeros((length, width), dtype=np.int16)

    @property
    def height_meters(self) -> np.ndarray:
        """Height-field in metres (float32)."""
        return self.height_field_raw.astype(np.float32) * self.vertical_scale


# -- individual terrain functions (each mutates *terrain* in place) ---------

def flat_terrain(terrain: SubTerrain, step_height: float = 0.0):
    """Perfectly flat (optionally raised by *step_height*)."""
    terrain.height_field_raw[:] = int(step_height / terrain.vertical_scale)


def random_uniform_terrain(terrain: SubTerrain, min_height: float = -0.05,
                           max_height: float = 0.05, step: float = 0.005,
                           downsampled_scale: float = 0.2):
    """Random uniform rough ground."""
    h = int(min_height / terrain.vertical_scale)
    H = int(max_height / terrain.vertical_scale)
    s = max(1, int(step / terrain.vertical_scale))
    ds = max(1, int(downsampled_scale / terrain.horizontal_scale))
    rng = np.random.default_rng()
    # Generate at low resolution then upsample
    low_l = terrain.length // ds
    low_w = terrain.width // ds
    low = rng.integers(h // s, H // s + 1, size=(low_l, low_w)) * s
    # Nearest-neighbour upsample
    row_idx = (np.arange(terrain.length) * low_l // terrain.length)
    col_idx = (np.arange(terrain.width) * low_w // terrain.width)
    terrain.height_field_raw = low[row_idx[:, None], col_idx[None, :]].astype(np.int16)


def pyramid_sloped_terrain(terrain: SubTerrain, slope: float = 0.3,
                           platform_size: float = 3.0):
    """A pyramid-shaped slope: four ramps meeting at a central platform."""
    slope = int(slope / terrain.vertical_scale)
    platform_size = int(platform_size / terrain.horizontal_scale)
    center_x = terrain.length // 2
    center_y = terrain.width // 2
    half_platform = platform_size // 2

    x = np.arange(terrain.length)
    y = np.arange(terrain.width)
    X, Y = np.meshgrid(x, y, indexing='ij')

    dist_x = np.maximum(np.abs(X - center_x) - half_platform, 0)
    dist_y = np.maximum(np.abs(Y - center_y) - half_platform, 0)
    # Slope is along X; the ramp height increases with distance from platform
    terrain.height_field_raw = (slope * dist_x).astype(np.int16)


def pyramid_stairs_terrain(terrain: SubTerrain, step_width: float = 0.31,
                           step_height: float = 0.1, platform_size: float = 3.0):
    """Pyramid staircase: steps ascending from each side to a central platform."""
    step_width_px = max(1, int(step_width / terrain.horizontal_scale))
    step_height_px = int(step_height / terrain.vertical_scale)
    platform_size_px = int(platform_size / terrain.horizontal_scale)
    center_x = terrain.length // 2
    center_y = terrain.width // 2

    x = np.arange(terrain.length)
    # Distance from centre along X
    dist_x = np.abs(x - center_x) - platform_size_px // 2
    dist_x = np.maximum(dist_x, 0)
    # Number of steps from centre
    n_steps = dist_x // step_width_px
    terrain.height_field_raw = (n_steps * step_height_px).astype(np.int16)
    # Broadcast across Y
    terrain.height_field_raw = np.broadcast_to(
        terrain.height_field_raw[:, None], (terrain.length, terrain.width)
    ).copy().astype(np.int16)


def discrete_obstacles_terrain(terrain: SubTerrain, max_height: float = 0.15,
                               min_size: float = 1.0, max_size: float = 2.0,
                               num_rectangles: int = 20, platform_size: float = 3.0):
    """Random rectangular obstacles scattered on a flat field."""
    height_px = int(max_height / terrain.vertical_scale)
    platform_size_px = int(platform_size / terrain.horizontal_scale)
    center_x = terrain.length // 2
    center_y = terrain.width // 2
    rng = np.random.default_rng()

    for _ in range(num_rectangles):
        w = int(rng.uniform(min_size, max_size) / terrain.horizontal_scale)
        l = int(rng.uniform(min_size, max_size) / terrain.horizontal_scale)
        x0 = rng.integers(0, terrain.length - l)
        y0 = rng.integers(0, terrain.width - w)
        h = int(rng.uniform(0.3, 1.0) * height_px)
        terrain.height_field_raw[x0:x0 + l, y0:y0 + w] = h

    # Clear a central platform
    x1 = center_x - platform_size_px // 2
    x2 = center_x + platform_size_px // 2
    y1 = center_y - platform_size_px // 2
    y2 = center_y + platform_size_px // 2
    terrain.height_field_raw[x1:x2, y1:y2] = 0


def stepping_stones_terrain(terrain: SubTerrain, stone_size: float = 1.0,
                            stone_distance: float = 0.1, max_height: float = 0.0,
                            platform_size: float = 4.0):
    """A grid of raised stepping stones with gaps between them."""
    stone_size_px = max(1, int(stone_size / terrain.horizontal_scale))
    stone_distance_px = max(1, int(stone_distance / terrain.horizontal_scale))
    max_height_px = int(max_height / terrain.vertical_scale)
    platform_size_px = int(platform_size / terrain.horizontal_scale)
    center_x = terrain.length // 2
    center_y = terrain.width // 2

    # Fill with a negative "pit" value, then place stones
    terrain.height_field_raw[:] = -1000

    step = stone_size_px + stone_distance_px
    rng = np.random.default_rng()
    for x0 in range(0, terrain.length, step):
        for y0 in range(0, terrain.width, step):
            h = int(rng.uniform(0, max_height_px))
            x1 = min(x0 + stone_size_px, terrain.length)
            y1 = min(y0 + stone_size_px, terrain.width)
            terrain.height_field_raw[x0:x1, y0:y1] = h

    # Clear central platform
    x1 = center_x - platform_size_px // 2
    x2 = center_x + platform_size_px // 2
    y1 = center_y - platform_size_px // 2
    y2 = center_y + platform_size_px // 2
    terrain.height_field_raw[x1:x2, y1:y2] = 0


def gap_terrain(terrain: SubTerrain, gap_size: float = 0.5,
                platform_size: float = 3.0):
    """A gap (crevasse) running across the terrain."""
    gap_px = int(gap_size / terrain.horizontal_scale)
    platform_px = int(platform_size / terrain.horizontal_scale)
    center_x = terrain.length // 2
    center_y = terrain.width // 2
    x1 = (terrain.length - platform_px) // 2
    x2 = x1 + gap_px

    terrain.height_field_raw[center_x - x2: center_x + x2,
                             center_y - x2: center_y + x2] = -1000
    terrain.height_field_raw[center_x - x1: center_x + x1,
                             center_y - x1: center_y + x1] = 0


def pit_terrain(terrain: SubTerrain, depth: float = 0.5,
                platform_size: float = 4.0):
    """A pit in the centre of the terrain."""
    depth_px = int(depth / terrain.vertical_scale)
    platform_px = int(platform_size / terrain.horizontal_scale / 2)
    center_x = terrain.length // 2
    center_y = terrain.width // 2
    terrain.height_field_raw[
        center_x - platform_px: center_x + platform_px,
        center_y - platform_px: center_y + platform_px
    ] = -depth_px


def wave_terrain(terrain: SubTerrain, amplitude: float = 0.15,
                 frequency: float = 0.5, platform_size: float = 3.0):
    """Rolling sinusoidal hills."""
    amp_px = int(amplitude / terrain.vertical_scale)
    platform_px = int(platform_size / terrain.horizontal_scale)
    center_x = terrain.length // 2
    center_y = terrain.width // 2

    x = np.arange(terrain.length) * terrain.horizontal_scale
    y = np.arange(terrain.width) * terrain.horizontal_scale
    X, Y = np.meshgrid(x, y, indexing='ij')
    h = amp_px * (np.sin(frequency * X) * np.cos(frequency * Y))
    terrain.height_field_raw = h.astype(np.int16)

    # Flat central platform
    x1 = center_x - platform_px // 2
    x2 = center_x + platform_px // 2
    y1 = center_y - platform_px // 2
    y2 = center_y + platform_px // 2
    terrain.height_field_raw[x1:x2, y1:y2] = 0


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

TERRAIN_FUNCTIONS = {
    "flat": flat_terrain,
    "rough": random_uniform_terrain,
    "slope": pyramid_sloped_terrain,
    "stairs": pyramid_stairs_terrain,
    "obstacles": discrete_obstacles_terrain,
    "stepping_stones": stepping_stones_terrain,
    "gap": gap_terrain,
    "pit": pit_terrain,
    "wave": wave_terrain,
}


# ---------------------------------------------------------------------------
# Height-field → 3DGS conversion
# ---------------------------------------------------------------------------

def heightfield_to_splats(height_field: np.ndarray,
                          horizontal_scale: float = 0.1,
                          vertical_scale: float = 0.005,
                          x_origin: float = 0.0,
                          y_origin: float = 0.0,
                          color: np.ndarray | None = None
                          ) -> list[GaussianSplat]:
    """Convert a raw int16 height-field to a list of GaussianSplat objects.

    Each pixel becomes a flat splat oriented along the local surface normal
    (estimated from the height gradient).
    """
    if color is None:
        color = np.array([0.3, 0.6, 0.2])

    length, width = height_field.shape
    splats: list[GaussianSplat] = []
    h_meters = height_field.astype(np.float32) * vertical_scale

    # Compute gradients for normal estimation
    grad_x = np.zeros_like(h_meters)
    grad_y = np.zeros_like(h_meters)
    grad_x[1:-1, :] = (h_meters[2:, :] - h_meters[:-2, :]) / (2 * horizontal_scale)
    grad_y[:, 1:-1] = (h_meters[:, 2:] - h_meters[:, :-2]) / (2 * horizontal_scale)

    for i in range(length):
        for j in range(width):
            x = x_origin + i * horizontal_scale
            y = y_origin + j * horizontal_scale
            z = h_meters[i, j]

            # Skip "void" cells (used by gap / stepping-stone terrains)
            if height_field[i, j] <= -500:
                continue

            # Surface normal from gradient: n = (-dh/dx, -dh/dy, 1) normalised
            nx = -grad_x[i, j]
            ny = -grad_y[i, j]
            n = np.array([nx, ny, 1.0])
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                n = np.array([0.0, 0.0, 1.0])
            else:
                n = n / norm

            # Build rotation matrix with Z axis = normal
            up = np.array([0.0, 0.0, 1.0])
            if abs(np.dot(n, up)) > 0.999:
                R = np.eye(3)
            else:
                axis = np.cross(up, n)
                axis = axis / (np.linalg.norm(axis) + 1e-12)
                angle = np.arccos(np.clip(np.dot(up, n), -1, 1))
                R = rot_axis_angle(axis, angle)

            splats.append(GaussianSplat(
                position=np.array([x, y, z]),
                scale=np.array([horizontal_scale * 0.7,
                                horizontal_scale * 0.7,
                                0.03]),
                rotation=R,
                opacity=0.9,
                color=color.copy(),
            ))

    return splats


# ---------------------------------------------------------------------------
# High-level TerrainGenerator
# ---------------------------------------------------------------------------

class TerrainGenerator:
    """
    Generate advanced test terrains for legged robot locomotion.

    Can produce either a single terrain type or a curriculum grid (rows =
    difficulty levels, columns = terrain types) similar to legged_gym.

    The output is a ``GaussianSplatEnv`` ready for simulation, plus a
    ``TerrainHeightField`` for O(1) ground queries.
    """

    def __init__(self, horizontal_scale: float = 0.1,
                 vertical_scale: float = 0.005,
                 terrain_length: float = 8.0,
                 terrain_width: float = 8.0):
        self.h_scale = horizontal_scale
        self.v_scale = vertical_scale
        self.length = terrain_length
        self.width = terrain_width
        self.length_px = int(terrain_length / horizontal_scale)
        self.width_px = int(terrain_width / horizontal_scale)

    def generate_single(self, terrain_type: str, difficulty: float = 0.5,
                        **kwargs) -> GaussianSplatEnv:
        """Generate a single terrain type at a given difficulty (0–1)."""
        if terrain_type not in TERRAIN_FUNCTIONS:
            raise ValueError(f"Unknown terrain type '{terrain_type}'. "
                             f"Available: {list(TERRAIN_FUNCTIONS.keys())}")

        sub = SubTerrain(
            name=terrain_type,
            width=self.width_px,
            length=self.length_px,
            horizontal_scale=self.h_scale,
            vertical_scale=self.v_scale,
        )

        func = TERRAIN_FUNCTIONS[terrain_type]

        if terrain_type == "flat":
            func(sub)
        elif terrain_type == "rough":
            func(sub, min_height=-0.05 * difficulty, max_height=0.05 * difficulty)
        elif terrain_type == "slope":
            func(sub, slope=0.4 * difficulty, platform_size=3.0)
        elif terrain_type == "stairs":
            func(sub, step_width=0.31,
                 step_height=0.05 + 0.18 * difficulty, platform_size=3.0)
        elif terrain_type == "obstacles":
            func(sub, max_height=0.05 + 0.2 * difficulty,
                 num_rectangles=20, platform_size=3.0)
        elif terrain_type == "stepping_stones":
            func(sub, stone_size=1.5 * (1.05 - difficulty),
                 stone_distance=0.05 if difficulty == 0 else 0.1,
                 max_height=0.0, platform_size=4.0)
        elif terrain_type == "gap":
            func(sub, gap_size=1.0 * difficulty, platform_size=3.0)
        elif terrain_type == "pit":
            func(sub, depth=1.0 * difficulty, platform_size=4.0)
        elif terrain_type == "wave":
            func(sub, amplitude=0.15 * difficulty, frequency=0.5,
                 platform_size=3.0)
        else:
            func(sub, **kwargs)

        # Choose a colour per terrain type
        colours = {
            "flat": np.array([0.4, 0.4, 0.4]),
            "rough": np.array([0.5, 0.4, 0.3]),
            "slope": np.array([0.6, 0.5, 0.3]),
            "stairs": np.array([0.6, 0.5, 0.3]),
            "obstacles": np.array([0.3, 0.5, 0.3]),
            "stepping_stones": np.array([0.4, 0.3, 0.5]),
            "gap": np.array([0.3, 0.3, 0.5]),
            "pit": np.array([0.3, 0.3, 0.4]),
            "wave": np.array([0.3, 0.6, 0.2]),
        }

        splats = heightfield_to_splats(
            sub.height_field_raw,
            horizontal_scale=self.h_scale,
            vertical_scale=self.v_scale,
            x_origin=-self.length / 2,
            y_origin=-self.width / 2,
            color=colours.get(terrain_type, np.array([0.3, 0.6, 0.2])),
        )
        return GaussianSplatEnv(splats=splats, ground_level=0.0)

    def generate_curriculum(self, num_rows: int = 5, num_cols: int = 8,
                            terrain_proportions: list[float] | None = None
                            ) -> GaussianSplatEnv:
        """
        Generate a curriculum grid of terrains.

        Rows  = difficulty levels (0 at top, 1 at bottom).
        Cols  = terrain types sampled according to *terrain_proportions*.
        """
        if terrain_proportions is None:
            terrain_proportions = [0.1, 0.1, 0.2, 0.2, 0.15, 0.1, 0.1, 0.05]
        terrain_types = ["slope", "rough", "stairs", "stairs",
                         "obstacles", "stepping_stones", "gap", "pit"]
        cum = np.cumsum(terrain_proportions)

        all_splats: list[GaussianSplat] = []
        sub_len_px = self.length_px // num_rows
        sub_wid_px = self.width_px // num_cols

        for i in range(num_rows):
            difficulty = i / num_rows
            for j in range(num_cols):
                choice = j / num_cols + 0.001
                idx = int(np.searchsorted(cum, choice))
                idx = min(idx, len(terrain_types) - 1)
                t_type = terrain_types[idx]

                sub = SubTerrain(
                    name=f"{t_type}_{i}_{j}",
                    width=sub_wid_px,
                    length=sub_len_px,
                    horizontal_scale=self.h_scale,
                    vertical_scale=self.v_scale,
                )
                func = TERRAIN_FUNCTIONS[t_type]

                if t_type == "slope":
                    func(sub, slope=0.4 * difficulty, platform_size=2.0)
                elif t_type == "stairs":
                    func(sub, step_width=0.31,
                         step_height=0.05 + 0.18 * difficulty, platform_size=2.0)
                elif t_type == "rough":
                    func(sub, min_height=-0.05 * difficulty,
                         max_height=0.05 * difficulty)
                elif t_type == "obstacles":
                    func(sub, max_height=0.05 + 0.2 * difficulty,
                         num_rectangles=10, platform_size=2.0)
                elif t_type == "stepping_stones":
                    func(sub, stone_size=1.0 * (1.1 - difficulty),
                         stone_distance=0.1, platform_size=2.0)
                elif t_type == "gap":
                    func(sub, gap_size=0.8 * difficulty, platform_size=2.0)
                elif t_type == "pit":
                    func(sub, depth=0.8 * difficulty, platform_size=2.0)
                else:
                    func(sub)

                x0 = -self.length / 2 + i * (self.length / num_rows)
                y0 = -self.width / 2 + j * (self.width / num_cols)
                colours = {
                    "slope": np.array([0.6, 0.5, 0.3]),
                    "rough": np.array([0.5, 0.4, 0.3]),
                    "stairs": np.array([0.6, 0.5, 0.3]),
                    "obstacles": np.array([0.3, 0.5, 0.3]),
                    "stepping_stones": np.array([0.4, 0.3, 0.5]),
                    "gap": np.array([0.3, 0.3, 0.5]),
                    "pit": np.array([0.3, 0.3, 0.4]),
                }
                splats = heightfield_to_splats(
                    sub.height_field_raw,
                    horizontal_scale=self.h_scale,
                    vertical_scale=self.v_scale,
                    x_origin=x0,
                    y_origin=y0,
                    color=colours.get(t_type, np.array([0.3, 0.6, 0.2])),
                )
                all_splats.extend(splats)

        return GaussianSplatEnv(splats=all_splats, ground_level=0.0)

    def list_terrain_types(self) -> list[str]:
        """Return all available terrain type names."""
        return list(TERRAIN_FUNCTIONS.keys())
