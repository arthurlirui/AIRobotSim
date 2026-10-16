"""
3D Gaussian Splatting (3DGS) environment representation.

A 3DGS scene is a collection of Gaussians, each defined by:
  - position  (mean)       : (3,)
  - scale     (3D ellipsoid): (3,)
  - rotation  (quaternion)  : (4,)  [w, x, y, z]
  - opacity                 : float
  - color (SH or RGB)       : (3,)

For robot simulation we don't render the splats (that requires a rasterizer);
instead we use the Gaussian ellipsoids as a **volumetric occupancy model** for:
  - terrain height queries (projection onto ground plane)
  - collision detection (robot foot/body vs. Gaussians)
  - surface normal estimation

If no real .ply 3DGS file is available, we generate a synthetic procedural
terrain scene so the full pipeline can be tested end-to-end.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from .math_utils import Transform, euler_to_matrix


@dataclass
class GaussianSplat:
    """A single 3D Gaussian primitive."""
    position: np.ndarray   # (3,)
    scale: np.ndarray      # (3,) — semi-axes of the ellipsoid
    rotation: np.ndarray   # (3,3) — rotation matrix
    opacity: float
    color: np.ndarray = field(default_factory=lambda: np.ones(3))

    def covariance(self) -> np.ndarray:
        """3x3 covariance matrix: Σ = R diag(s²) Rᵀ."""
        S = np.diag(self.scale ** 2)
        return self.rotation @ S @ self.rotation.T

    def contains_point(self, point: np.ndarray, n_sigma: float = 2.0) -> bool:
        """Check if a point is inside the n-sigma ellipsoid."""
        d = point - self.position
        Sigma_inv = np.linalg.inv(self.covariance() + 1e-9 * np.eye(3))
        mahala = d @ Sigma_inv @ d
        return mahala <= n_sigma ** 2


class GaussianSplatEnv:
    """
    3DGS environment for robot simulation.

    Stores a set of Gaussian splats and provides spatial queries:
      - height_at(x, y)  → ground elevation
      - normal_at(x, y)  → surface normal
      - collision(point) → list of splats overlapping a point
      - ray_cast(origin, dir) → first hit distance
    """

    def __init__(self, splats: Optional[list[GaussianSplat]] = None,
                 ground_level: float = 0.0):
        self.splats: list[GaussianSplat] = splats or []
        self.ground_level = ground_level
        # Build a spatial hash for fast queries (grid-based)
        self._grid: dict[tuple[int, int, int], list[int]] = {}
        self._grid_cell = 0.5  # meters
        if self.splats:
            self._rebuild_grid()

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def add_splat(self, splat: GaussianSplat):
        self.splats.append(splat)
        self._index_splat(len(self.splats) - 1, splat)

    def _rebuild_grid(self):
        self._grid.clear()
        for i, s in enumerate(self.splats):
            self._index_splat(i, s)

    def _index_splat(self, idx: int, splat: GaussianSplat):
        # Index the splat's center cell and neighbors (simplified)
        c = splat.position / self._grid_cell
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    key = (int(c[0]) + dx, int(c[1]) + dy, int(c[2]) + dz)
                    self._grid.setdefault(key, []).append(idx)

    # ------------------------------------------------------------------
    # Spatial queries
    # ------------------------------------------------------------------

    def _nearby_indices(self, point: np.ndarray) -> list[int]:
        c = point / self._grid_cell
        key = (int(c[0]), int(c[1]), int(c[2]))
        return self._grid.get(key, [])

    def height_at(self, x: float, y: float) -> float:
        """
        Estimate ground height at (x, y) by finding the highest splat
        whose horizontal projection covers (x, y) and whose center is
        below a reasonable height threshold.

        Falls back to ground_level if no splat found.
        """
        best_z = self.ground_level
        point_xy = np.array([x, y, 0.0])
        # Search in a vertical column — check all splats near (x, y)
        cx, cy = int(x / self._grid_cell), int(y / self._grid_cell)
        candidates: set[int] = set()
        for dz in range(-5, 6):
            key = (cx, cy, dz)
            if key in self._grid:
                candidates.update(self._grid[key])

        for idx in candidates:
            s = self.splats[idx]
            # Project splat onto XY plane
            dx = x - s.position[0]
            dy = y - s.position[1]
            # Approximate horizontal extent
            horiz_scale = max(s.scale[0], s.scale[1])
            if dx * dx + dy * dy <= (horiz_scale * 2) ** 2:
                # This splat is under (x, y); use its top as height
                top_z = s.position[2] + s.scale[2]
                if top_z > best_z and s.position[2] < 3.0:
                    best_z = top_z
        return best_z

    def normal_at(self, x: float, y: float) -> np.ndarray:
        """
        Estimate surface normal at (x, y) using local splat orientations.
        """
        cx, cy = int(x / self._grid_cell), int(y / self._grid_cell)
        candidates: set[int] = set()
        for dz in range(-5, 6):
            key = (cx, cy, dz)
            if key in self._grid:
                candidates.update(self._grid[key])

        if not candidates:
            return np.array([0.0, 0.0, 1.0])

        normals = []
        weights = []
        for idx in candidates:
            s = self.splats[idx]
            dx = x - s.position[0]
            dy = y - s.position[1]
            d2 = dx * dx + dy * dy
            if d2 < 1e-6:
                continue
            horiz_scale = max(s.scale[0], s.scale[1])
            if d2 <= (horiz_scale * 2) ** 2:
                # The splat's local Z axis (3rd column of rotation)
                n = s.rotation[:, 2]
                if n[2] < 0:
                    n = -n  # ensure pointing up
                normals.append(n)
                weights.append(1.0 / (d2 + 1e-6))

        if not normals:
            return np.array([0.0, 0.0, 1.0])

        weights = np.array(weights)
        normals = np.array(normals)
        avg = (normals * weights[:, None]).sum(axis=0)
        norm = np.linalg.norm(avg)
        if norm < 1e-9:
            return np.array([0.0, 0.0, 1.0])
        return avg / norm

    def collision(self, point: np.ndarray, radius: float = 0.1) -> list[int]:
        """Return indices of splats that collide with a sphere at *point*."""
        results = []
        for idx in self._nearby_indices(point):
            s = self.splats[idx]
            if s.contains_point(point, n_sigma=2.0):
                results.append(idx)
        # Also check by distance for safety
        if not results:
            for idx in self._nearby_indices(point):
                s = self.splats[idx]
                d = np.linalg.norm(point - s.position)
                if d < radius + max(s.scale):
                    results.append(idx)
        return results

    def ray_cast(self, origin: np.ndarray, direction: np.ndarray,
                 max_dist: float = 10.0) -> Optional[float]:
        """
        Simple ray cast: march along the ray and check splat containment.
        Returns the distance to the first hit, or None.
        """
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        step = 0.05
        t = 0.0
        while t < max_dist:
            p = origin + direction * t
            if self.collision(p):
                return t
            t += step
        return None

    # ------------------------------------------------------------------
    # Procedural scene generation (for testing without real 3DGS data)
    # ------------------------------------------------------------------

    @classmethod
    def create_terrain(cls, size: float = 10.0, resolution: int = 20,
                       roughness: float = 0.15, seed: int = 42) -> "GaussianSplatEnv":
        """
        Generate a procedural terrain scene with rolling hills, suitable for
        testing robot locomotion. Each terrain patch is represented as a flat
        Gaussian splat.
        """
        rng = np.random.default_rng(seed)
        splats: list[GaussianSplat] = []
        step = size / resolution

        for i in range(resolution):
            for j in range(resolution):
                x = (i + 0.5) * step - size / 2
                y = (j + 0.5) * step - size / 2
                # Heightmap: sum of sinusoids + noise
                z = (0.5 * np.sin(0.5 * x) * np.cos(0.5 * y)
                     + 0.3 * np.sin(1.2 * x + 0.3) * np.cos(1.1 * y)
                     + rng.normal(0, roughness))
                # Estimate local normal from height gradient
                h_x = 0.5 * 0.5 * np.cos(0.5 * x) * np.cos(0.5 * y) + 0.3 * 1.2 * np.cos(1.2 * x + 0.3) * np.cos(1.1 * y)
                h_y = -0.5 * np.sin(0.5 * x) * 0.5 * np.sin(0.5 * y) - 0.3 * np.sin(1.2 * x + 0.3) * 1.1 * np.sin(1.1 * y)
                # Normal = (-dh/dx, -dh/dy, 1) normalized
                n = np.array([-h_x, -h_y, 1.0])
                n = n / (np.linalg.norm(n) + 1e-12)

                # Build rotation matrix with Z axis = normal
                # Use a simple method: align Z to normal
                up = np.array([0, 0, 1])
                if np.abs(np.dot(n, up)) > 0.999:
                    R = np.eye(3)
                else:
                    axis = np.cross(up, n)
                    axis = axis / (np.linalg.norm(axis) + 1e-12)
                    angle = np.arccos(np.clip(np.dot(up, n), -1, 1))
                    from .math_utils import rot_axis_angle
                    R = rot_axis_angle(axis, angle)

                splat = GaussianSplat(
                    position=np.array([x, y, z]),
                    scale=np.array([step * 0.7, step * 0.7, 0.05]),
                    rotation=R,
                    opacity=0.9,
                    color=np.array([0.3, 0.6, 0.2]),
                )
                splats.append(splat)

        env = cls(splats=splats, ground_level=0.0)
        return env

    @classmethod
    def create_flat_ground(cls, size: float = 10.0,
                           resolution: int = 10) -> "GaussianSplatEnv":
        """Flat ground plane — simplest test scene."""
        splats: list[GaussianSplat] = []
        step = size / resolution
        for i in range(resolution):
            for j in range(resolution):
                x = (i + 0.5) * step - size / 2
                y = (j + 0.5) * step - size / 2
                splats.append(GaussianSplat(
                    position=np.array([x, y, 0.0]),
                    scale=np.array([step * 0.7, step * 0.7, 0.02]),
                    rotation=np.eye(3),
                    opacity=0.9,
                    color=np.array([0.4, 0.4, 0.4]),
                ))
        return cls(splats=splats)

    @classmethod
    def create_stairs(cls, n_steps: int = 5, step_h: float = 0.15,
                      step_w: float = 0.4) -> "GaussianSplatEnv":
        """Staircase scene for testing climbing."""
        splats: list[GaussianSplat] = []
        for i in range(n_steps):
            z = i * step_h
            x_center = i * step_w
            # Each step is a slab
            for px in np.arange(x_center - step_w / 2, x_center + step_w / 2, 0.15):
                for py in np.arange(-1.0, 1.0, 0.15):
                    splats.append(GaussianSplat(
                        position=np.array([px, py, z]),
                        scale=np.array([0.08, 0.08, 0.02]),
                        rotation=np.eye(3),
                        opacity=0.95,
                        color=np.array([0.6, 0.5, 0.3]),
                    ))
        # Add ground before the stairs
        for px in np.arange(-3.0, -step_w / 2, 0.2):
            for py in np.arange(-1.0, 1.0, 0.2):
                splats.append(GaussianSplat(
                    position=np.array([px, py, 0.0]),
                    scale=np.array([0.1, 0.1, 0.02]),
                    rotation=np.eye(3),
                    opacity=0.9,
                    color=np.array([0.4, 0.4, 0.4]),
                ))
        return cls(splats=splats, ground_level=0.0)

    def stats(self) -> dict:
        """Return summary statistics about the environment."""
        positions = np.array([s.position for s in self.splats])
        return {
            "num_splats": len(self.splats),
            "bounds_min": positions.min(axis=0) if len(positions) else np.zeros(3),
            "bounds_max": positions.max(axis=0) if len(positions) else np.zeros(3),
            "grid_cells": len(self._grid),
        }

    def __repr__(self):
        return f"GaussianSplatEnv({len(self.splats)} splats)"
