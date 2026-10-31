"""
Shared camera geometry: intrinsics, extrinsics, pinhole projection and the
abstract :class:`CameraModel` that every sensor type implements.
"""

from __future__ import annotations

import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, TYPE_CHECKING

from core.math_utils import euler_to_matrix

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


# ---------------------------------------------------------------------------
# Intrinsics — the pinhole model shared by every camera type
# ---------------------------------------------------------------------------

@dataclass
class CameraIntrinsics:
    """Pinhole intrinsics (Brown-Conrady distortion on demand)."""

    width: int = 640
    height: int = 480
    fx: float = 500.0            # focal length in pixels
    fy: float = 500.0
    cx: float = 320.0            # principal point in pixels
    cy: float = 240.0
    distortion: tuple = ()       # (k1, k2, p1, p2, k3) rad-tan, optional

    @classmethod
    def from_fov(cls, width: int, height: int, fov_deg: float) -> "CameraIntrinsics":
        """Build intrinsics from a horizontal field of view."""
        fx = 0.5 * width / np.tan(np.deg2rad(fov_deg) / 2)
        return cls(width=width, height=height, fx=fx, fy=fx,
                   cx=(width - 1) / 2, cy=(height - 1) / 2)

    @property
    def fov_h_deg(self) -> float:
        return float(np.rad2deg(2 * np.arctan(0.5 * self.width / self.fx)))

    @property
    def fov_v_deg(self) -> float:
        return float(np.rad2deg(2 * np.arctan(0.5 * self.height / self.fy)))

    def project(self, points_cam: np.ndarray) -> np.ndarray:
        """
        Project N×3 camera-frame points to N×2 pixel coordinates (u, v).
        Points with z ≤ 0 project to NaN.
        """
        pts = np.atleast_2d(np.asarray(points_cam, dtype=float))
        z = pts[:, 2]
        valid = z > 1e-9
        uv = np.full((pts.shape[0], 2), np.nan)
        with np.errstate(divide="ignore", invalid="ignore"):
            u = pts[:, 0] / z * self.fx + self.cx
            v = pts[:, 1] / z * self.fy + self.cy
        uv[valid, 0] = u[valid]
        uv[valid, 1] = v[valid]
        return uv

    def pixel_rays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Unit ray directions (camera frame) for every pixel, raster order.

        Returns (dirs, us, vs) where dirs is (H·W, 3), us / vs are the
        matching pixel coordinates.
        """
        us, vs = np.meshgrid(np.arange(self.width), np.arange(self.height))
        us = us.ravel()
        vs = vs.ravel()
        x = (us - self.cx) / self.fx
        y = (vs - self.cy) / self.fy
        dirs = np.stack([x, y, np.ones_like(x)], axis=1)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        return dirs, us, vs


# ---------------------------------------------------------------------------
# Extrinsics — camera mounting pose on the robot
# ---------------------------------------------------------------------------

@dataclass
class CameraExtrinsics:
    """
    Camera pose relative to the robot base (translation + ZYX Euler).

    Camera convention: +X right, +Y down, +Z forward (optical frame).
    """

    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw

    def rotation(self) -> np.ndarray:
        return euler_to_matrix(*self.euler)

    def world_transform(self, base_position: np.ndarray,
                        base_orientation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Camera-to-world (R, t) given the robot base pose.

        Returns rotation R (3×3) and translation t (3,) such that a
        camera-frame point p maps to world as R @ p + t.
        """
        R_base = euler_to_matrix(*base_orientation)
        R_cam = R_base @ self.rotation()
        t_cam = base_position + R_base @ self.translation
        return R_cam, t_cam


# ---------------------------------------------------------------------------
# CameraFrame — one rendered frame, shared across sensor types
# ---------------------------------------------------------------------------

@dataclass
class CameraFrame:
    """
    A single rendered frame. Channels are populated per sensor type:

    - MonocularCamera: ``rgb`` (H, W, 3) float [0, 1]
    - StereoCamera:    ``rgb`` + ``depth`` (H, W) meters
    - ITofCamera:      ``depth`` (H, W) meters, ``confidence`` (H, W)
    - ThermalCamera:   ``thermal`` (H, W) °C apparent temperature
    - NearInfraredCamera: ``nir`` (H, W) [0, 1]
    """
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    confidence: Optional[np.ndarray] = None
    thermal: Optional[np.ndarray] = None
    nir: Optional[np.ndarray] = None
    timestamp: float = 0.0
    metadata: dict = field(default_factory=dict)

    @property
    def height(self) -> int:
        for arr in (self.rgb, self.depth, self.thermal, self.nir):
            if arr is not None:
                return arr.shape[0]
        return 0

    @property
    def width(self) -> int:
        for arr in (self.rgb, self.depth, self.thermal, self.nir):
            if arr is not None:
                return arr.shape[1]
        return 0


# ---------------------------------------------------------------------------
# CameraModel — abstract base
# ---------------------------------------------------------------------------

class CameraModel(ABC):
    """
    Abstract camera: pinhole geometry + per-type measurement model.

    Subclasses implement :meth:`render` which turns a robot pose (or an
    explicit extrinsic world transform) plus the environment into a
    :class:`CameraFrame`.  Rendering casts one ray per (sub-sampled) pixel
    against the Gaussian-splat environment.
    """

    def __init__(self, intrinsics: Optional[CameraIntrinsics] = None,
                 extrinsics: Optional[CameraExtrinsics] = None,
                 name: str = "camera"):
        self.intrinsics = intrinsics or CameraIntrinsics()
        self.extrinsics = extrinsics or CameraExtrinsics()
        self.name = name
        self._rng = np.random.default_rng()

    def seed(self, seed: int):
        """Seed the sensor noise RNG (reproducible experiments)."""
        self._rng = np.random.default_rng(seed)

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def world_camera_transform(self, base_position: np.ndarray,
                               base_orientation: np.ndarray
                               ) -> tuple[np.ndarray, np.ndarray]:
        return self.extrinsics.world_transform(base_position, base_orientation)

    def _pixel_rays_world(self, R_cam: np.ndarray,
                          stride: int = 1) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        World-frame ray directions + pixel coords for the (optionally
        strided) pixel grid. Used by every renderer.
        """
        dirs, us, vs = self.intrinsics.pixel_rays()
        if stride > 1:
            mask = ((us % stride) == 0) & ((vs % stride) == 0)
            dirs, us, vs = dirs[mask], us[mask], vs[mask]
        dirs_world = dirs @ R_cam.T
        return dirs_world, us, vs

    def _hit_batch(self, env: "GaussianSplatEnv",
                   origin: np.ndarray, dirs: np.ndarray,
                   max_dist: float) -> np.ndarray:
        """
        Cast N rays; return distances (np.inf where no hit).
        """
        dists = np.full(dirs.shape[0], np.inf)
        for i in range(dirs.shape[0]):
            d = env.ray_cast(origin, dirs[i], max_dist=max_dist)
            dists[i] = d if d is not None else np.inf
        return dists

    # ------------------------------------------------------------------
    # Abstract measurement model
    # ------------------------------------------------------------------

    @abstractmethod
    def render(self, env: "GaussianSplatEnv",
               base_position: np.ndarray, base_orientation: np.ndarray,
               t: float = 0.0, stride: int = 1) -> CameraFrame:
        """Render one frame from the current robot pose."""
        ...

    def __repr__(self):
        return (f"{self.__class__.__name__}(name={self.name!r}, "
                f"{self.intrinsics.width}x{self.intrinsics.height}, "
                f"fov_h={self.intrinsics.fov_h_deg:.1f}°)")
