"""
Thermal (long-wave infrared, LWIR 8–14 µm) camera, e.g. FLIR Boson-class.

Measurement model:
  - apparent temperature of the first-hit surface: the splat's
    ``temperature`` attribute if present, else an ambient default
    (optionally modulated by a spatial temperature field)
  - noise: NETD (noise-equivalent temperature difference, ~50 mK) as
    Gaussian noise on the apparent temperature, plus a small
    fixed-pattern offset per pixel
  - thermal cameras see through visual occluders only partially; here the
    geometry ray cast is the same as other sensors (geometry-faithful)
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from sensors.base import CameraModel, CameraFrame

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


class ThermalCamera(CameraModel):
    """LWIR thermal camera measuring apparent surface temperature."""

    def __init__(self, width: int = 160, height: int = 120, fov_deg: float = 60.0,
                 netd_mk: float = 50.0,            # noise-equivalent ΔT (mK)
                 ambient_temperature_c: float = 20.0,
                 fpn_std_mk: float = 10.0,         # fixed-pattern noise
                 max_range_m: float = 100.0,
                 **kwargs):
        from sensors.base import CameraIntrinsics
        intr = CameraIntrinsics.from_fov(width, height, fov_deg)
        super().__init__(intrinsics=intr, **kwargs)
        self.netd_mk = netd_mk
        self.ambient_temperature_c = ambient_temperature_c
        self.fpn_std_mk = fpn_std_mk
        self.max_range_m = max_range_m
        # Per-pixel fixed-pattern offset, frozen for the camera's lifetime
        self._fpm = None

    def _fixed_pattern(self, shape) -> np.ndarray:
        if self._fpm is None or self._fpm.shape != shape:
            self._fpm = self._rng.normal(0, self.fpn_std_mk * 1e-3, shape)
        return self._fpm

    def render(self, env: "GaussianSplatEnv",
               base_position: np.ndarray, base_orientation: np.ndarray,
               t: float = 0.0, stride: int = 1) -> CameraFrame:
        R_cam, t_cam = self.world_camera_transform(base_position, base_orientation)
        dirs, us, vs = self._pixel_rays_world(R_cam, stride=stride)
        dists = self._hit_batch(env, t_cam, dirs, max_dist=self.max_range_m)

        W, H = self.intrinsics.width, self.intrinsics.height
        thermal = np.full((H, W), np.nan)  # NaN = sky / no thermal return

        hit = np.isfinite(dists)
        if np.any(hit):
            for i in np.where(hit)[0]:
                p = t_cam + dirs[i] * dists[i]
                idx = env.collision(p) or env._nearby_indices(p)
                if idx:
                    splat = env.splats[idx[0]]
                    temp = getattr(splat, "temperature", None)
                    if temp is None:
                        # Splat color → crude thermal signature: bright
                        # (visually hot) surfaces a bit above ambient
                        temp = self.ambient_temperature_c + 2.0 * splat.color.mean()
                    thermal[vs[i], us[i]] = temp
                else:
                    thermal[vs[i], us[i]] = self.ambient_temperature_c

        # NETD temporal noise + fixed-pattern noise
        valid = np.isfinite(thermal)
        noise = self._rng.normal(0, self.netd_mk * 1e-3, thermal.shape)
        thermal = thermal + noise + self._fixed_pattern((H, W))
        thermal[~valid] = np.nan

        return CameraFrame(thermal=thermal, timestamp=t,
                           metadata={"netd_mk": self.netd_mk,
                                     "stride": stride})
