"""
Test 11: Camera optical models (sensors package).

Verifies for each camera type (monocular / stereo / iToF / thermal / NIR):
  - the pinhole geometry: a wall straight ahead yields finite returns in
    the image center
  - per-type physics signatures:
      · stereo depth noise grows with range (quadratic)
      · iToF phase wrap: beyond the unambiguous range the reported depth
        wraps (aliasing) — the classic iToF artifact
      · thermal measures apparent temperature around ambient
      · NIR intensity falls off with distance (1/Z² active illumination)
  - reproducibility: same seed → identical frames

**Control-group assertion** (platform test philosophy): iToF must report
a *wrapped* depth (≈ Z − n·Z_max) where monocular/stereo report the true
depth — the sensors must be distinguishable, not interchangeable.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv, GaussianSplat
from sensors import (
    CameraIntrinsics, MonocularCamera, StereoCamera,
    ITofCamera, ThermalCamera, NearInfraredCamera,
)


def _pose_facing_wall(wall_x: float = 5.0):
    """
    Flat env + a wall at x=wall_x facing a robot at the origin.

    The wall is a lattice of small splats (the env's spatial grid indexes
    each splat only into its center cell ± 1 neighbor, so one big splat
    would be invisible to rays passing far from its center).
    """
    env = GaussianSplatEnv.create_flat_ground(size=12.0, resolution=4)
    for y in np.linspace(-1.5, 1.5, 7):
        for z in np.linspace(0.3, 2.7, 7):
            env.add_splat(GaussianSplat(
                position=np.array([wall_x, y, z]),
                scale=np.array([0.15, 0.25, 0.25]),
                rotation=np.eye(3),
                opacity=1.0,
                color=np.array([0.8, 0.2, 0.2]),
            ))
    return env


def _cam_pose(x=0.0, z=1.0):
    # Camera looking along +X: optical frame is X-right/Y-down/Z-forward,
    # so pitch the identity mounting by -90° about Y to align +Z with +X.
    from sensors import CameraExtrinsics
    return np.array([x, 0.0, z]), np.zeros(3), \
        CameraExtrinsics(euler=np.array([0.0, np.pi / 2, 0.0]))


def run():
    # ------------------------------------------------------------------
    print("  [1] Pinhole geometry: intrinsics from FOV")
    intr = CameraIntrinsics.from_fov(640, 480, 90.0)
    assert abs(intr.fov_h_deg - 90.0) < 1e-6
    pts = np.array([[0.0, 0.0, 2.0]])  # on-axis point at 2 m
    uv = intr.project(pts)
    assert np.allclose(uv, [[319.5, 239.5]], atol=0.5), uv
    behind = intr.project(np.array([[0.0, 0.0, -1.0]]))
    assert np.isnan(behind[0, 0]), "points behind camera must be NaN"
    print("      OK — projection & FOV consistent")

    # ------------------------------------------------------------------
    print("  [2] Monocular: wall visible in image center")
    env = _pose_facing_wall(5.0)
    pos, orn, ext = _cam_pose()
    cam = MonocularCamera(extrinsics=ext)
    cam.seed(0)
    f = cam.render(env, pos, orn, stride=8)
    assert f.rgb is not None and f.rgb.shape == (240, 320, 3)
    center = f.depth[110:130, 150:170]
    n_hits = int(np.isfinite(center).sum())
    assert n_hits >= 6, f"wall 5m ahead must be visible (got {n_hits} strided px)"
    med = float(np.median(center[np.isfinite(center)]))
    assert abs(med - 5.0) < 0.6, f"depth should be ~5 m, got {med:.2f}"
    print(f"      OK — wall depth ~{med:.2f} m ({n_hits} strided px visible)")

    # ------------------------------------------------------------------
    print("  [3] Stereo: noise grows with range (control group)")
    from sensors import CameraExtrinsics
    ext = CameraExtrinsics(euler=np.array([0.0, np.pi / 2, 0.0]))
    errs = {}
    for wall_x in (2.0, 8.0):
        env = _pose_facing_wall(wall_x)
        ext = CameraExtrinsics(euler=np.array([0.0, np.pi / 2, 0.0]))
        cam = StereoCamera(extrinsics=ext, baseline_m=0.05,
                           disparity_noise_px=0.3)
        cam.seed(1)
        pos = np.array([0.0, 0.0, 1.0])
        f = cam.render(env, pos, np.zeros(3), stride=8)
        d = f.depth[110:130, 150:170]
        fin = np.isfinite(d)
        assert fin.sum() >= 6, f"wall at {wall_x} m in stereo range"
        errs[wall_x] = float(np.std(d[fin] - wall_x))
    assert errs[8.0] > errs[2.0], \
        f"stereo σ must grow with range: {errs}"
    print(f"      OK — σ(Z=2m)={errs[2.0]:.4f} < σ(Z=8m)={errs[8.0]:.4f}")

    # ------------------------------------------------------------------
    print("  [4] iToF: phase wrap beyond unambiguous range")
    # mod 20 MHz → unambiguous range = c/2f = 7.5 m
    ext = CameraExtrinsics(euler=np.array([0.0, np.pi / 2, 0.0]))
    tof = ITofCamera(extrinsics=ext, mod_freq_mhz=20.0, max_range_m=30.0,
                     shot_noise_scale=0.0)
    tof.seed(2)
    env = _pose_facing_wall(9.0)  # 9 m > 7.5 m unambiguous → wraps to 1.5 m
    f = tof.render(env, np.array([0.0, 0.0, 1.0]), np.zeros(3), stride=8)
    # iToF image is 160x120 → center window (rows 52:68, cols 72:88)
    d = f.depth[52:68, 72:88]
    fin = np.isfinite(d)
    assert fin.sum() >= 3
    wrapped = float(np.median(d[fin]))
    assert abs(wrapped - 1.5) < 0.3, \
        f"9 m must wrap to 1.5 m, got {wrapped:.2f}"
    print(f"      OK — 9 m aliased to {wrapped:.2f} m (Z_max=7.5 m)")

    # Control group: monocular reports the TRUE depth at the same pose
    mono = MonocularCamera(extrinsics=ext)
    mono.seed(2)
    fm = mono.render(env, np.array([0.0, 0.0, 1.0]), np.zeros(3), stride=8)
    dm = fm.depth[110:130, 150:170]  # monocular is 320x240
    finm = np.isfinite(dm)
    assert finm.sum() >= 6
    assert abs(float(np.median(dm[finm])) - 9.0) < 0.4
    assert abs(wrapped - float(np.median(dm[finm]))) > 5.0, \
        "iToF and monocular must disagree here (wrap vs true depth)"
    print("      OK — control group: monocular reports true 9 m")

    # ------------------------------------------------------------------
    print("  [5] Thermal: ambient temperature + NETD noise")
    env = _pose_facing_wall(5.0)
    ext = CameraExtrinsics(euler=np.array([0.0, np.pi / 2, 0.0]))
    tc = ThermalCamera(extrinsics=ext, ambient_temperature_c=20.0,
                       netd_mk=50.0)
    tc.seed(3)
    f = tc.render(env, np.array([0.0, 0.0, 1.0]), np.zeros(3), stride=8)
    # thermal image is 160x120 → center window
    t = f.thermal[52:68, 72:88]
    fin = np.isfinite(t)
    assert fin.sum() >= 3
    med = float(np.nanmedian(t[fin]))
    assert 15.0 < med < 30.0, f"apparent temp ≈ ambient, got {med:.1f}"
    print(f"      OK — apparent temperature {med:.1f} °C ≈ ambient 20 °C")

    # ------------------------------------------------------------------
    print("  [6] NIR: 1/Z² active-illumination falloff")
    ext = CameraExtrinsics(euler=np.array([0.0, np.pi / 2, 0.0]))
    means = {}
    for wall_x in (2.0, 6.0):
        env = _pose_facing_wall(wall_x)
        cam = NearInfraredCamera(extrinsics=ext, shot_noise_scale=0.0,
                                 read_noise_std=0.0, max_range_m=20.0)
        cam.seed(4)
        f = cam.render(env, np.array([0.0, 0.0, 1.0]), np.zeros(3), stride=8)
        v = f.nir[110:130, 150:170]
        means[wall_x] = float(np.mean(v))
    assert means[2.0] > means[6.0] * 2, \
        f"NIR must darken with range: {means}"
    print(f"      OK — NIR at 2 m: {means[2.0]:.3f} > 2× at 6 m: {means[6.0]:.3f}")

    # ------------------------------------------------------------------
    print("  [7] Reproducibility: same seed → identical frame")
    env = _pose_facing_wall(5.0)
    ext = CameraExtrinsics(euler=np.array([0.0, np.pi / 2, 0.0]))
    c1 = MonocularCamera(extrinsics=ext); c1.seed(42)
    c2 = MonocularCamera(extrinsics=ext); c2.seed(42)
    f1 = c1.render(env, np.array([0.0, 0.0, 1.0]), np.zeros(3), stride=8)
    f2 = c2.render(env, np.array([0.0, 0.0, 1.0]), np.zeros(3), stride=8)
    assert np.array_equal(f1.rgb, f2.rgb), "seeded noise must be identical"
    print("      OK — deterministic under fixed seed")

    print("\n  All camera-model tests passed.")


if __name__ == "__main__":
    run()
