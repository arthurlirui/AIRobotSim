"""
Free-Space Optical (FSO) channel model.

FSO uses a narrow laser beam (typically 850 nm or 1550 nm) between two
nodes that must maintain line-of-sight alignment.  The dominant impairments
in a mobile-robot scenario are:

  * **Atmospheric turbulence** — scintillation modelled by the Gamma-Gamma
    distribution, parameterised by the Rytov variance σ_R² which depends on
    the refractive-index structure constant C_n², wavelength, and distance.
  * **Pointing error** — the receiver aperture is finite; robot motion and
    gait-induced vibration cause the beam spot to drift off-centre.  This is
    modelled by the Farid-Uysal pointing-error model: a radial Gaussian
    displacement with std σ_p, received fraction = erf(v) where v depends on
    the beam waist and aperture ratio.

The combined fading distribution (turbulence × pointing) is complex; for a
simulation-level model we compute the **mean** received power and a
**scintillation fade margin** rather than drawing per-sample random
fading.  This gives a deterministic, reproducible link budget that captures
the key physics (turbulence loss grows with distance^3, pointing loss grows
with jitter) without requiring a Monte-Carlo fading engine.

Path-gain model (dB):

    PG = 10·log10( (λ / 4πd)² )           -- free-space (Friis)
         - turbulence_margin(d, λ, Cn2)
         - pointing_margin(σ_p, w_z, a)

where ``turbulence_margin`` ≈ 1.33 · σ_R² / ln(2) (3 dB scintillation
approximation) and ``pointing_margin`` is derived from the Farid-Uysal
collection fraction.
"""

from __future__ import annotations

import math
from typing import Optional, TYPE_CHECKING

import numpy as np

from comm.channel.base import ChannelModel, ChannelState, Pose, RadioParameters
from comm.phy.link_budget import fill_link_metrics

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv

_SPEED_OF_LIGHT = 2.99792458e8


def _rytov_variance(distance_m: float, wavelength_m: float,
                    cn2: float) -> float:
    """
    Plane-wave Rytov variance:

        σ_R² = 1.23 · C_n² · k^(7/6) · d^(11/6)

    where k = 2π / λ.  This is the standard weak-turbulence metric.
    """
    k = 2.0 * math.pi / wavelength_m
    return 1.23 * cn2 * k ** (7.0 / 6.0) * distance_m ** (11.0 / 6.0)


def _pointing_collection(sigma_p: float, beam_waist_z: float,
                         aperture_radius: float) -> float:
    """
    Farid-Uysal pointing-error collection fraction (simplified).

    Returns the fraction of optical power collected by a circular aperture
    of radius ``a`` when the beam centre is displaced by a zero-mean
    Gaussian random variable with std ``σ_p``.  The beam intensity profile
    at the receiver is Gaussian with beam waist ``w_z`` (1/e² radius).

    The exact expression involves the Marcum Q-function; we use the
    common approximation:

        A_0 = erf(v) · erf(v)  where  v = sqrt(π) · a / (sqrt(2) · w_z)

    and then the average collected fraction, accounting for pointing
    jitter, is approximately:

        η_p ≈ A_0 · w_z² / (w_z² + 2·σ_p²)

    which captures the key behaviour: as σ_p → 0, η_p → A_0; as σ_p grows,
    η_p drops quadratically.
    """
    w = beam_waist_z
    if w <= 0.0:
        return 0.0
    v = math.sqrt(math.pi) * aperture_radius / (math.sqrt(2.0) * w)
    a0 = math.erf(v) ** 2
    eta_p = a0 * w ** 2 / (w ** 2 + 2.0 * sigma_p ** 2)
    return max(0.0, min(1.0, eta_p))


class FSOChannel(ChannelModel):
    """
    Free-space optical link.

    Args:
        wavelength_m: laser wavelength (default 1550 nm = 1.55e-6).
        cn2: refractive-index structure constant C_n² (m^(-2/3)).
            Typical: 1e-15 (weak), 1e-14 (moderate), 1e-13 (strong).
        pointing_jitter_m: std of beam-centre displacement at the receiver
            due to robot motion / vibration (m).
        beam_waist_m: 1/e² beam radius at the receiver (m).
        aperture_radius_m: receiver aperture radius (m).
        tx_power_dbm: optical transmit power (dBm).
    """

    FIDELITY_LEVEL = 0

    def __init__(self,
                 wavelength_m: float = 1.55e-6,
                 cn2: float = 1e-14,
                 pointing_jitter_m: float = 0.005,
                 beam_waist_m: float = 0.01,
                 aperture_radius_m: float = 0.005,
                 radio: Optional[RadioParameters] = None):
        if radio is None:
            radio = RadioParameters(
                tx_power_dbm=10.0,     # 10 mW optical
                tx_gain_dbi=0.0,
                rx_gain_dbi=0.0,
                noise_figure_db=3.0,
                bandwidth_hz=1e9,      # 1 GHz optical baseband
                modulation="OOK",
                temperature_k=290.0,
                max_range_m=2000.0,
            )
        super().__init__(radio)
        self.wavelength_m = wavelength_m
        self.cn2 = cn2
        self.pointing_jitter_m = pointing_jitter_m
        self.beam_waist_m = beam_waist_m
        self.aperture_radius_m = aperture_radius_m

    def _path_gain_db(self, distance_m: float) -> float:
        if distance_m <= 0.0:
            return 0.0
        wavelength = self.wavelength_m
        # Free-space (Friis)
        fspl_db = 20.0 * math.log10(4.0 * math.pi * distance_m / wavelength)

        # Turbulence scintillation fade (3 dB approximation)
        sigma_r2 = _rytov_variance(distance_m, wavelength, self.cn2)
        # Clamp to weak/moderate regime; strong turbulence saturates.
        sigma_r2 = min(sigma_r2, 1.0)
        turbulence_margin_db = 1.33 * sigma_r2 / math.log(2.0) * 3.0  # ≈ 3 dB per σ_R²

        # Pointing loss
        eta_p = _pointing_collection(
            self.pointing_jitter_m, self.beam_waist_m, self.aperture_radius_m)
        pointing_db = -10.0 * math.log10(eta_p + 1e-30)

        pg = -(fspl_db + turbulence_margin_db + pointing_db)
        return pg

    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float = 0.0,
                 env: Optional["GaussianSplatEnv"] = None,
                 tx_id: str = "tx", rx_id: str = "rx") -> ChannelState:
        d = self._distance(tx_pose, rx_pose)
        pg = self._path_gain_db(d)

        sigma_r2 = min(_rytov_variance(d, self.wavelength_m, self.cn2), 1.0)
        eta_p = _pointing_collection(
            self.pointing_jitter_m, self.beam_waist_m, self.aperture_radius_m)

        state = ChannelState(
            tx_id=tx_id, rx_id=rx_id, timestamp=t,
            path_gain_db=pg,
            los=True,  # FSO requires LoS; occlusion is checked by Layer-1
            distance_m=d,
            delay_ms=d / _SPEED_OF_LIGHT * 1e3,
            doppler_hz=self._doppler(tx_pose, rx_pose,
                                     _SPEED_OF_LIGHT / self.wavelength_m),
            metadata={
                "model": "fso",
                "wavelength_m": self.wavelength_m,
                "rytov_variance": sigma_r2,
                "pointing_collection": eta_p,
                "turbulence_margin_db": 1.33 * sigma_r2 / math.log(2.0) * 3.0,
                "pointing_loss_db": -10.0 * math.log10(eta_p + 1e-30),
            },
        )
        if d > self.radio.max_range_m:
            state.path_gain_db = float("-inf")
        fill_link_metrics(state, self.radio)
        return state
