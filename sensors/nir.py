"""
Near-infrared (NIR, 0.75–1.4 µm) camera with active illumination,
e.g. structured-light / IR-tracking cameras.

Measurement model:
  - active point/flash illumination falls off as 1/Z² with distance to
    the illuminated surface, so the NIR image naturally darkens with range
  - reflectance from the first-hit splat's color (NIR reflectance is
    approximated by the mean visible albedo — plant material typically
    appears brighter in NIR, modeled by a "red edge" boost term)
  - shot noise (signal-dependent) + read noise
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from sensors.base import CameraModel, CameraFrame

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


class NearInfraredCamera(CameraModel):
    """Active-illumination NIR camera with 1/Z² intensity falloff."""

    def __init__(self, width: int = 320, height: int = 240, fov_deg: float = 80.0,
                 illuminator_power: float = 1.0,
                 max_range_m: float = 15.0,
                 read_noise_std: float = 0.01,
                 shot_noise_scale: float = 0.03,
                 **kwargs):
        from sensors.base import CameraIntrinsics
        intr = CameraIntrinsics.from_fov(width, height, fov_deg)
        super().__init__(intrinsics=intr, **kwargs)
        self.illuminator_power = illuminator_power
        self.max_range_m = max_range_m
        self.read_noise_std = read_noise_std
        self.shot_noise_scale = shot_noise_scale

    def render(self, env: "GaussianSplatEnv",
               base_position: np.ndarray, base_orientation: np.ndarray,
               t: float = 0.0, stride: int = 1) -> CameraFrame:
        R_cam, t_cam = self.world_camera_transform(base_position, base_orientation)
        dirs, us, vs = self._pixel_rays_world(R_cam, stride=stride)
        dists = self._hit_batch(env, t_cam, dirs, max_dist=self.max_range_m)

        W, H = self.intrinsics.width, self.intrinsics.height
        nir = np.zeros((H, W))

        hit = np.isfinite(dists)
        if np.any(hit):
            for i in np.where(hit)[0]:
                p = t_cam + dirs[i] * dists[i]
                idx = env.collision(p) or env._nearby_indices(p)
                if idx:
                    splat = env.splats[idx[0]]
                    # NIR reflectance ≈ mean visible albedo with a red-edge
                    # boost for vegetation-like (green-dominant) surfaces
                    albedo = splat.color.mean()
                    veg_boost = 1.0 + 1.5 * max(0.0, splat.color[1] - splat.color[0])
                    # Active illumination: received power ∝ 1/Z²
                    Z = dists[i]
                    intensity = self.illuminator_power * albedo * veg_boost \
                        / (1.0 + Z ** 2)
                    nir[vs[i], us[i]] = intensity
                # no splat → stays 0 (unilluminated background)

        # Shot + read noise
        shot = self._rng.normal(0, 1, nir.shape) * self.shot_noise_scale \
            * np.sqrt(np.maximum(nir, 0.0))
        read = self._rng.normal(0, self.read_noise_std, nir.shape)
        nir = np.clip(nir + shot + read, 0.0, 1.0)

        return CameraFrame(nir=nir, timestamp=t,
                           metadata={"max_range_m": self.max_range_m,
                                     "stride": stride})
