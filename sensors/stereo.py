"""
Stereo camera (e.g. RealSense D435-class): two monocular imagers with a
known baseline; depth from disparity.

Measurement model:
  disparity d = f·B / Z  (f: focal px, B: baseline m, Z: depth m)
  depth noise grows quadratically with range:
      σ_Z = Z² / (f·B) · σ_d        (σ_d: disparity matching noise in px)
  A minimum disparity (maximum measurable range) is enforced — beyond it
  the depth is invalid (inf), reproducing the hard range limit of real
  passive stereo.
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from sensors.base import CameraModel, CameraFrame, CameraExtrinsics
from sensors.monocular import MonocularCamera

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


class StereoCamera(CameraModel):
    """Passive stereo depth camera with quadratic range-dependent noise."""

    def __init__(self, width: int = 320, height: int = 240, fov_deg: float = 87.0,
                 baseline_m: float = 0.05, disparity_noise_px: float = 0.2,
                 min_disparity_px: float = 0.5, **kwargs):
        from sensors.base import CameraIntrinsics
        intr = CameraIntrinsics.from_fov(width, height, fov_deg)
        super().__init__(intrinsics=intr, **kwargs)
        self.baseline_m = baseline_m
        self.disparity_noise_px = disparity_noise_px
        self.min_disparity_px = min_disparity_px
        # Left imager as the reference monocular camera
        self._left = MonocularCamera(width=width, height=height, fov_deg=fov_deg,
                                     extrinsics=CameraExtrinsics())

    @property
    def max_range_m(self) -> float:
        """Maximum measurable depth, set by the minimum disparity."""
        return self.intrinsics.fx * self.baseline_m / self.min_disparity_px

    def render(self, env: "GaussianSplatEnv",
               base_position: np.ndarray, base_orientation: np.ndarray,
               t: float = 0.0, stride: int = 1) -> CameraFrame:
        # Left imager shares the stereo rig's mounting pose
        self._left.extrinsics = self.extrinsics
        # Left-image RGB from the reference imager (same extrinsics)
        frame = self._left.render(env, base_position, base_orientation,
                                  t=t, stride=stride)
        depth_true = frame.depth

        W, H = self.intrinsics.width, self.intrinsics.height
        f = self.intrinsics.fx
        depth = np.full((H, W), np.inf)

        valid = np.isfinite(depth_true)
        if np.any(valid):
            Z = depth_true[valid]
            # Depth beyond stereo's disparity limit is unmeasurable
            meas = Z < self.max_range_m
            Zm = Z[meas]
            # σ_Z = Z² / (f·B) · σ_d, clipped to avoid absurd values
            sigma = np.minimum(Zm ** 2 / (f * self.baseline_m)
                               * self.disparity_noise_px, 0.5 * Zm)
            Z_noisy = Zm + self._rng.normal(0, 1, Zm.shape) * sigma
            out = np.full(Z.shape, np.inf)
            out[meas] = np.maximum(Z_noisy, 1e-3)
            depth[valid] = out

        return CameraFrame(rgb=frame.rgb, depth=depth, timestamp=t,
                           metadata={"baseline_m": self.baseline_m,
                                     "max_range_m": self.max_range_m,
                                     "stride": stride})
