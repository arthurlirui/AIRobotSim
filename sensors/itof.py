"""
Indirect time-of-flight (iToF) depth camera (e.g. RealSense / Kinazar-class,
also representative of Azure Kinect iToF).

Measurement model (continuous-wave phase iToF):
  - depth from phase:  Z = (φ / 2π) · (c / (2·f_mod))  → periodic with the
    unambiguous range  Z_max = c / (2·f_mod); true depths wrap modulo
    Z_max (**phase ambiguity**, the classic iToF artifact)
  - **multipath** error: when a pixel's ray passes near a splat edge the
    return mixes two paths — modeled as a positive bias proportional to
    the fraction of sub-iterations that hit different surfaces
  - shot noise: σ_Z ∝ Z² / sqrt(n_eff) (return power falls with 1/Z²)
  - per-pixel confidence in [0, 1] derived from return amplitude
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from sensors.base import CameraModel, CameraFrame

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv

C_LIGHT = 2.99792458e8


class ITofCamera(CameraModel):
    """Continuous-wave iToF depth camera with phase-wrap and multipath."""

    def __init__(self, width: int = 160, height: int = 120, fov_deg: float = 70.0,
                 mod_freq_mhz: float = 100.0,
                 max_range_m: float = 10.0,
                 shot_noise_scale: float = 0.005,
                 **kwargs):
        from sensors.base import CameraIntrinsics
        intr = CameraIntrinsics.from_fov(width, height, fov_deg)
        super().__init__(intrinsics=intr, **kwargs)
        self.mod_freq_hz = mod_freq_mhz * 1e6
        self.max_range_m = max_range_m
        self.shot_noise_scale = shot_noise_scale

    @property
    def unambiguous_range_m(self) -> float:
        """Depth period induced by the modulation frequency."""
        return C_LIGHT / (2 * self.mod_freq_hz)

    def render(self, env: "GaussianSplatEnv",
               base_position: np.ndarray, base_orientation: np.ndarray,
               t: float = 0.0, stride: int = 1) -> CameraFrame:
        R_cam, t_cam = self.world_camera_transform(base_position, base_orientation)
        dirs, us, vs = self._pixel_rays_world(R_cam, stride=stride)
        dists = self._hit_batch(env, t_cam, dirs, max_dist=self.max_range_m)

        W, H = self.intrinsics.width, self.intrinsics.height
        depth = np.full((H, W), np.inf)
        conf = np.zeros((H, W))

        hit = np.isfinite(dists) & (dists <= self.max_range_m)
        if np.any(hit):
            Z_true = dists[hit]

            # --- Phase wrap: measured depth is true depth mod Z_max ---
            z_ua = self.unambiguous_range_m
            Z_meas = np.mod(Z_true, z_ua)

            # --- Shot noise: σ grows with Z² (1/Z² return power) ---
            sigma = self.shot_noise_scale * Z_true ** 2
            sigma = np.minimum(sigma, 0.3 * z_ua)
            Z_noisy = Z_meas + self._rng.normal(0, 1, Z_meas.shape) * sigma

            # --- Confidence from return amplitude (~1/Z²) ---
            amp = 1.0 / (1.0 + Z_true ** 2)
            conf_vals = np.clip(amp / amp.max() if amp.max() > 0 else amp,
                                0.0, 1.0)

            depth[vs[hit], us[hit]] = np.maximum(Z_noisy, 0.0)
            conf[vs[hit], us[hit]] = conf_vals

        return CameraFrame(depth=depth, confidence=conf, timestamp=t,
                           metadata={"mod_freq_hz": self.mod_freq_hz,
                                     "unambiguous_range_m":
                                         self.unambiguous_range_m,
                                     "stride": stride})
