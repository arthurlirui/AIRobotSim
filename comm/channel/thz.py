"""
Terahertz (THz) channel model, 100 GHz – 1 THz.

THz links are characterised by:
  * Molecular absorption loss from water vapour / oxygen, modelled by the
    Beer-Lambert law: A(f, d) = α(f) · d  (dB), where α is the frequency-
    dependent absorption coefficient.
  * Very directional LoS-dominant propagation; NLOS multipath is minimal
    because surfaces are extremely rough relative to the wavelength.
  * Short usable range (metres to tens of metres indoors).

This implementation uses a simplified absorption model: a piecewise set of
absorption coefficients (dB/m) at representative frequencies in the THz
windows, with linear interpolation in log-frequency space.  For higher
fidelity the ``hapi`` library (HITRAN) can be plugged in later — the
interface stays the same.

Path loss model (LoS):

    PL_total = FSPL(d, f) + α(f) · d   (all in dB)

where FSPL is the free-space term and α(f)·d is the molecular absorption.
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

# Representative molecular absorption coefficients (dB/m) at THz window
# centres.  Values are order-of-magnitude approximations for standard
# humidity (≈ 50 % RH, 20 °C).  Sourced from ITU-R P.676 and THz channel
# literature (Akyildiz et al.).
_THZ_ABSORPTION_DB_PER_M = {
    100e9: 0.02,
    200e9: 0.08,
    300e9: 0.30,
    350e9: 5.0,     # water line
    410e9: 0.50,
    670e9: 40.0,    # strong water line
    850e9: 1.2,
    1.0e12: 10.0,
}


class THzChannel(ChannelModel):
    """
    Terahertz channel with molecular absorption.

    Args:
        carrier_hz: centre frequency in the 100 GHz – 1 THz range.
        absorption_db_per_m: override the interpolated absorption
            coefficient.  If None, it is looked up / interpolated from the
            internal table.
        shadowing_std: log-normal shadowing std (dB); 0 disables.
    """

    FIDELITY_LEVEL = 0

    def __init__(self, carrier_hz: float = 300e9,
                 absorption_db_per_m: Optional[float] = None,
                 shadowing_std: float = 1.0,
                 radio: Optional[RadioParameters] = None):
        super().__init__(radio)
        self.carrier_hz = carrier_hz
        self.shadowing_std = shadowing_std
        if absorption_db_per_m is not None:
            self.absorption_db_per_m = absorption_db_per_m
        else:
            self.absorption_db_per_m = self._lookup_absorption(carrier_hz)

    @staticmethod
    def _lookup_absorption(freq: float) -> float:
        """Linear interpolation in log-freq space from the table."""
        freqs = sorted(_THZ_ABSORPTION_DB_PER_M.keys())
        if freq <= freqs[0]:
            return _THZ_ABSORPTION_DB_PER_M[freqs[0]]
        if freq >= freqs[-1]:
            return _THZ_ABSORPTION_DB_PER_M[freqs[-1]]
        for i in range(len(freqs) - 1):
            f0, f1 = freqs[i], freqs[i + 1]
            if f0 <= freq <= f1:
                a0 = _THZ_ABSORPTION_DB_PER_M[f0]
                a1 = _THZ_ABSORPTION_DB_PER_M[f1]
                # Linear in log-freq
                t = (math.log10(freq) - math.log10(f0)) / \
                    (math.log10(f1) - math.log10(f0))
                return a0 * (1 - t) + a1 * t
        return 0.1  # fallback

    def _path_gain_db(self, distance_m: float) -> float:
        if distance_m <= 0.0:
            return 0.0
        wavelength = _SPEED_OF_LIGHT / self.carrier_hz
        fspl_db = 20.0 * math.log10(4.0 * math.pi * distance_m / wavelength)
        absorption_db = self.absorption_db_per_m * distance_m
        pg = -(fspl_db + absorption_db)
        if self.shadowing_std > 0.0:
            pg += np.random.normal(0.0, self.shadowing_std)
        return pg

    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float = 0.0,
                 env: Optional["GaussianSplatEnv"] = None,
                 tx_id: str = "tx", rx_id: str = "rx") -> ChannelState:
        d = self._distance(tx_pose, rx_pose)
        pg = self._path_gain_db(d)

        state = ChannelState(
            tx_id=tx_id, rx_id=rx_id, timestamp=t,
            path_gain_db=pg,
            los=True,  # THz is LoS-dominant by assumption
            distance_m=d,
            delay_ms=d / _SPEED_OF_LIGHT * 1e3,
            doppler_hz=self._doppler(tx_pose, rx_pose, self.carrier_hz),
            metadata={
                "model": "thz",
                "carrier_hz": self.carrier_hz,
                "absorption_db_per_m": self.absorption_db_per_m,
                "absorption_loss_db": self.absorption_db_per_m * d,
            },
        )
        if d > self.radio.max_range_m:
            state.path_gain_db = float("-inf")
        fill_link_metrics(state, self.radio)
        return state
