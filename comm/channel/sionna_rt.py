"""
Layer 2 channel: Sionna RT ray tracing (NVIDIA Sionna ≥ 1.0 / 2.x).

This adapter bridges the platform's unified ``ChannelModel`` interface to
`Sionna RT <https://nvlabs.github.io/sionna/>`_, NVIDIA's differentiable
ray-tracing wireless simulator built on Mitsuba 3.  While Layer 0/1
models estimate path loss statistically (+ geometric occlusion), this
layer **computes** the channel from scene geometry: specular reflections,
diffuse scattering, diffraction and the full channel impulse response.

Design (adapter pattern, same as every other channel):

    SionnaRTChannel(env_or_scene).generate(tx_pose, rx_pose, t)
        → ChannelState(path_gain_db=…, cir=…, los=…, delay_ms=…)

Scene sources, in priority order:
  1. A Sionna ``Scene`` passed directly (advanced users — load any
     Mitsuba XML / Sionna built-in scene).
  2. The platform's :class:`GaussianSplatEnv` — the splat set is
     converted to an axis-aligned-box Mitsuba scene (one box per splat,
     scaled to 1σ) so ray tracing runs on the *same reconstruction* the
     robots walk on.
  3. A flat-ground fallback scene (no env, no scene).

Because a Mitsuba scene rebuild is expensive, the adapter caches the
built scene and reuses it across ``generate`` calls; only the transmitter
/ receiver positions move.

Sionna is an optional dependency: importing this module without sionna
installed raises a helpful ImportError.  The rest of the comm stack
(Layer 0/1, THz, FSO, fiber) works without it.
"""

from __future__ import annotations

import numpy as np
from typing import Optional, TYPE_CHECKING

from comm.channel.base import ChannelModel, ChannelState, Pose

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv

try:
    import sionna
    from sionna.rt import load_scene, PathSolver, PlanarArray, Transmitter, Receiver
    import drjit as dr
    SIONNA_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only without sionna
    sionna = None
    SIONNA_AVAILABLE = False


class SionnaRTChannel(ChannelModel):
    """
    Ray-traced channel backed by Sionna RT (Fidelity Level 2).

    Args:
        env: optional :class:`GaussianSplatEnv` — its splats become
            Mitsuba radio boxes (1σ extents, ITU material by default).
        scene: optional prebuilt Sionna ``Scene`` (overrides ``env``).
        carrier_hz: carrier frequency (Hz), default 3.5 GHz (5G NR n78).
        max_depth: maximum number of interactions per path
            (reflection / diffraction).
        scattering: enable diffuse scattering.
        diffraction: enable edge diffraction (wedge diffraction on
            Sionna ≥ 1.0).
    """

    FIDELITY_LEVEL = 2

    def __init__(self, env: Optional["GaussianSplatEnv"] = None,
                 scene=None, carrier_hz: float = 3.5e9,
                 max_depth: int = 2,
                 scattering: bool = False,
                 diffraction: bool = False,
                 **kwargs):
        if not SIONNA_AVAILABLE:
            raise ImportError(
                "Sionna is not installed. Install it with "
                "`pip install sionna` (requires Python ≥ 3.10) to use "
                "SionnaRTChannel. The Layer-0/1 channel models do not "
                "need it.")
        super().__init__(**kwargs)

        self.carrier_hz = float(carrier_hz)
        self.max_depth = int(max_depth)
        self.scattering = bool(scattering)
        self.diffraction = bool(diffraction)

        self._scene, self._mesh_dir = self._build_scene(env) \
            if scene is None else (scene, None)
        self._configure_scene()
        self._solver = PathSolver()

    # ------------------------------------------------------------------
    # Scene construction
    # ------------------------------------------------------------------

    @staticmethod
    def _write_box_ply(path: str, pos: np.ndarray, half: np.ndarray):
        """Write an axis-aligned box mesh as a binary PLY file."""
        hx, hy, hz = half
        cx, cy, cz = pos
        v = np.array([
            [cx-hx, cy-hy, cz-hz], [cx+hx, cy-hy, cz-hz],
            [cx+hx, cy+hy, cz-hz], [cx-hx, cy+hy, cz-hz],
            [cx-hx, cy-hy, cz+hz], [cx+hx, cy-hy, cz+hz],
            [cx+hx, cy+hy, cz+hz], [cx-hx, cy+hy, cz+hz],
        ], dtype=np.float32)
        f = np.array([
            [0,1,2], [0,2,3],   # bottom (-z)
            [4,6,5], [4,7,6],   # top (+z)
            [0,4,5], [0,5,1],   # front (-y)
            [2,6,7], [2,7,3],   # back (+y)
            [1,5,6], [1,6,2],   # right (+x)
            [3,7,4], [3,4,0],   # left (-x)
        ], dtype=np.int32)

        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {len(v)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            f"element face {len(f)}\n"
            "property list uchar int vertex_indices\n"
            "end_header\n"
        ).encode("ascii")
        with open(path, "wb") as fh:
            fh.write(header)
            fh.write(v.tobytes())
            # Per-face: uint8 count + 3×int32 indices (packed, no upcast)
            face_rec = np.empty(len(f), dtype=[("n", "u1"),
                                               ("idx", "<i4", (3,))])
            face_rec["n"] = 3
            face_rec["idx"] = f
            fh.write(face_rec.tobytes())

    def _build_scene(self, env: Optional["GaussianSplatEnv"]):
        """Build a Sionna scene from the 3DGS environment (or flat ground)."""
        import tempfile
        import os

        tmpdir = tempfile.mkdtemp(prefix="airobotsim_sionna_")

        def shape_xml(mesh_path: str) -> str:
            mesh_id = os.path.basename(mesh_path).replace(".", "_")
            return (
                f'<shape type="ply" id="{mesh_id}">'
                f'<string name="filename" value="{mesh_path}"/>'
                f'<boolean name="face_normals" value="true"/>'
                f'<ref id="mat-itu" name="bsdf"/>'
                f'</shape>'
            )

        shapes: list[str] = []

        if env is not None and len(env.splats) > 0:
            for i, s in enumerate(env.splats):
                # 1σ half-extents (rotation ignored — axis-aligned approx)
                half = np.maximum(np.abs(s.scale) * 0.5, 1e-3)
                mesh = os.path.join(tmpdir, f"splat_{i:05d}.ply")
                self._write_box_ply(mesh, s.position, half)
                shapes.append(shape_xml(mesh))

        # Ground plane (always present — ray tracing needs a floor)
        ground = os.path.join(tmpdir, "ground_plane.ply")
        self._write_box_ply(
            ground,
            np.array([0.0, 0.0,
                      (env.ground_level if env else 0.0) - 0.5]),
            np.array([1000.0, 1000.0, 0.5]),
        )
        shapes.append(shape_xml(ground))

        xml = (
            '<scene version="2.1.0">'
            '<bsdf type="itu-radio-material" id="mat-itu">'
            '<string name="type" value="brick"/>'
            '<float name="thickness" value="0.3"/>'
            '</bsdf>'
            + "".join(shapes) +
            '</scene>'
        )
        from sionna.rt import load_scene_from_string
        return load_scene_from_string(xml), tmpdir

    def _configure_scene(self):
        arr_tx = PlanarArray(num_rows=1, num_cols=1,
                             pattern="iso", polarization="V")
        arr_rx = PlanarArray(num_rows=1, num_cols=1,
                             pattern="iso", polarization="V")
        self._scene.tx_array = arr_tx
        self._scene.rx_array = arr_rx
        self._scene.frequency = self.carrier_hz

        self._tx = Transmitter(name="_adapter_tx",
                               position=[0.0, 0.0, 1.0])
        self._rx = Receiver(name="_adapter_rx",
                             position=[10.0, 0.0, 1.0])
        self._scene.add(self._tx)
        self._scene.add(self._rx)

    # ------------------------------------------------------------------
    # ChannelModel interface
    # ------------------------------------------------------------------

    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float = 0.0,
                 env: Optional["GaussianSplatEnv"] = None,
                 tx_id: str = "tx", rx_id: str = "rx") -> ChannelState:
        # Move the radio devices (mitsuba Point3f accepts numpy lists)
        self._tx.position = np.asarray(tx_pose.position, dtype=float)
        self._rx.position = np.asarray(rx_pose.position, dtype=float)

        paths = self._solver(
            self._scene,
            max_depth=self.max_depth,
            los=True,
            specular_reflection=True,
            diffuse_reflection=self.scattering,
            diffraction=self.diffraction,
            edge_diffraction=self.diffraction,
        )
        a, tau = paths.cir()
        a0 = np.asarray(a[0])[0, 0, 0, 0, :, 0]   # (n_paths,)
        tau0 = np.asarray(tau[0])[0, :]           # (n_paths,)

        dist = self._distance(tx_pose, rx_pose)
        doppler = self._doppler(tx_pose, rx_pose, self.carrier_hz)

        # Aggregate power over all propagation paths (linear → dB)
        total_power = float(np.sum(np.abs(a0) ** 2))
        if total_power <= 0 or not np.isfinite(total_power):
            path_gain_db = -np.inf
        else:
            path_gain_db = 10.0 * np.log10(total_power)

        # LOS flag: the shortest path delay ≈ d/c → direct path exists
        c = 2.99792458e8
        los = bool(len(tau0) > 0 and tau0[0] * c < dist * 1.02)

        # CIR: power-delay profile, zero-padded to a fixed grid
        cir = None
        if len(tau0) > 0:
            dt = 1e-9  # 1 ns resolution
            n = int(np.ceil(tau0.max() / dt)) + 1
            cir = np.zeros(n)
            for gain, delay in zip(np.abs(a0) ** 2, tau0):
                cir[int(round(float(delay) / dt))] += float(gain)

        from comm.phy.link_budget import fill_link_metrics
        state = ChannelState(
            tx_id=tx_id, rx_id=rx_id, timestamp=t,
            path_gain_db=path_gain_db,
            los=los,
            distance_m=dist,
            doppler_hz=doppler,
            delay_ms=float(tau0[0] * 1e3) if len(tau0) else 0.0,
            bandwidth_hz=self.radio.bandwidth_hz,
            cir=cir,
            metadata={"n_paths": int(len(tau0)),
                      "solver": "sionna-rt",
                      "max_depth": self.max_depth},
        )
        return fill_link_metrics(state, self.radio)
