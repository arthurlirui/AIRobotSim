"""
Terrain wrapper: extracts a height-field from a GaussianSplatEnv for
efficient ground queries during locomotion.
"""

from __future__ import annotations

import numpy as np
from .gaussian_splat_env import GaussianSplatEnv


class TerrainHeightField:
    """
    Pre-computed height-field sampled from a GaussianSplatEnv.
    Enables fast O(1) ground height and normal lookups via bilinear interpolation.
    """

    def __init__(self, env: GaussianSplatEnv, bounds: tuple[float, float, float, float],
                 resolution: float = 0.1):
        """
        Args:
            env: the 3DGS environment
            bounds: (x_min, x_max, y_min, y_max) in meters
            resolution: grid spacing in meters
        """
        self.env = env
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.res = resolution
        self.nx = int((self.x_max - self.x_min) / resolution) + 1
        self.ny = int((self.y_max - self.y_min) / resolution) + 1

        # Sample heights
        self.heights = np.zeros((self.nx, self.ny))
        self.normals = np.zeros((self.nx, self.ny, 3))
        for i in range(self.nx):
            for j in range(self.ny):
                x = self.x_min + i * resolution
                y = self.y_min + j * resolution
                self.heights[i, j] = env.height_at(x, y)
                self.normals[i, j] = env.normal_at(x, y)

    def height_at(self, x: float, y: float) -> float:
        """Bilinear interpolated height at (x, y)."""
        fi = (x - self.x_min) / self.res
        fj = (y - self.y_min) / self.res
        i0 = int(np.clip(np.floor(fi), 0, self.nx - 2))
        j0 = int(np.clip(np.floor(fj), 0, self.ny - 2))
        di = np.clip(fi - i0, 0, 1)
        dj = np.clip(fj - j0, 0, 1)

        h00 = self.heights[i0, j0]
        h10 = self.heights[i0 + 1, j0]
        h01 = self.heights[i0, j0 + 1]
        h11 = self.heights[i0 + 1, j0 + 1]

        return (h00 * (1 - di) * (1 - dj) + h10 * di * (1 - dj) +
                h01 * (1 - di) * dj + h11 * di * dj)

    def normal_at(self, x: float, y: float) -> np.ndarray:
        """Bilinear interpolated normal at (x, y)."""
        fi = (x - self.x_min) / self.res
        fj = (y - self.y_min) / self.res
        i0 = int(np.clip(np.floor(fi), 0, self.nx - 2))
        j0 = int(np.clip(np.floor(fj), 0, self.ny - 2))
        di = np.clip(fi - i0, 0, 1)
        dj = np.clip(fj - j0, 0, 1)

        n = ((self.normals[i0, j0] * (1 - di) * (1 - dj) +
              self.normals[i0 + 1, j0] * di * (1 - dj) +
              self.normals[i0, j0 + 1] * (1 - di) * dj +
              self.normals[i0 + 1, j0 + 1] * di * dj))
        nrm = np.linalg.norm(n)
        return n / (nrm + 1e-12)

    def slope_at(self, x: float, y: float) -> float:
        """Return the slope angle (radians) at (x, y)."""
        n = self.normal_at(x, y)
        return np.arccos(np.clip(n[2], -1, 1))
