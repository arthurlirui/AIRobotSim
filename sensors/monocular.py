"""
Monocular RGB camera (visible band, 380–750 nm).

Measurement model:
  - depth per pixel via environment ray cast (geometry)
  - color from the first-hit splat's RGB, distance-attenuated
  - photon shot noise (Poisson-like, scales with sqrt of signal) and
    Gaussian read noise; exposure control via gain.
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from sensors.base import CameraModel, CameraFrame

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


class MonocularCamera(CameraModel):
    """Visible-band RGB camera with shot + read noise."""

    def __init__(self, width: int = 320, height: int = 240, fov_deg: float = 90.0,
                 read_noise_std: float = 0.01, shot_noise_scale: float = 0.02,
                 **kwargs):
        from sensors.base import CameraIntrinsics
        intr = CameraIntrinsics.from_fov(width, height, fov_deg)
        super().__init__(intrinsics=intr, **kwargs)
        self.read_noise_std = read_noise_std
        self.shot_noise_scale = shot_noise_scale

    def render(self, env: "GaussianSplatEnv",
               base_position: np.ndarray, base_orientation: np.ndarray,
               t: float = 0.0, stride: int = 1) -> CameraFrame:
        R_cam, t_cam = self.world_camera_transform(base_position, base_orientation)
        dirs, us, vs = self._pixel_rays_world(R_cam, stride=stride)
        dists = self._hit_batch(env, t_cam, dirs, max_dist=60.0)

        W, H = self.intrinsics.width, self.intrinsics.height
        rgb = np.zeros((H, W, 3))
        depth = np.full((H, W), np.inf)

        hit = np.isfinite(dists)
        if np.any(hit):
            # Color: sample the splat color at the hit point, attenuate
            # with distance (crash atmospheric / illumination falloff).
            for i in np.where(hit)[0]:
                p = t_cam + dirs[i] * dists[i]
                d = np.linalg.norm(p - t_cam)
                atten = 1.0 / (1.0 + 0.05 * d)
                # Nearest splat color (approximation of the radiance)
                cands = env.collision(p)
                if cands:
                    col = env.splats[cands[0]].color
                else:
                    # Between ray-cast steps: use nearest splat by center
                    idx = env._nearby_indices(p)
                    col = env.splats[idx[0]].color if idx else np.array([0.5, 0.5, 0.5])
                rgb[vs[i], us[i]] = np.clip(col * atten, 0, 1)
            depth[vs[hit], us[hit]] = dists[hit]

        # Sensor noise: shot (signal-dependent) + read (Gaussian)
        signal = rgb
        shot = self._rng.normal(0, 1, rgb.shape) * self.shot_noise_scale \
            * np.sqrt(np.maximum(signal, 0.0))
        read = self._rng.normal(0, self.read_noise_std, rgb.shape)
        rgb = np.clip(signal + shot + read, 0.0, 1.0)

        return CameraFrame(rgb=rgb, depth=depth, timestamp=t,
                           metadata={"stride": stride})
