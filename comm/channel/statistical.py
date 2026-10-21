"""
Layer-0 statistical channel models.

These models compute path gain from distance only, using closed-form path-loss
equations.  No 3D-geometry queries are performed; geometric occlusion (Layer 1)
and ray tracing (Layer 2) are layered on top in later phases.

Implemented technologies:

  * :class:`FreeSpaceChannel` — Friis free-space path loss.  Base class for
    every wireless model; also used directly for short-range LoS links.
  * :class:`LogDistanceChannel` — log-distance path loss with a configurable
    path-loss exponent; the workhorse for 3G/4G cellular.
  * :class:`CellularChannel` — convenience presets for 3G / 4G bands.
  * :class:`NRChannel` — 5G NR sub-6 using a CI-style (close-in) model aligned
    with 3GPP TR 38.901.
  * :class:`MmWaveChannel` — mmWave (28/60/73 GHz) with oxygen absorption and
    severe blockage sensitivity (NLOS penalty).

All models return :class:`~comm.channel.base.ChannelState`; link-budget
metrics (SNR / capacity / BER) are filled by :mod:`comm.phy.link_budget`.
"""

from __future__ import annotations

import math
from typing import Optional, TYPE_CHECKING

import numpy as np

from comm.channel.base import (
    ChannelModel, ChannelState, Pose, RadioParameters,
)
from comm.phy.link_budget import fill_link_metrics

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv

_SPEED_OF_LIGHT = 2.99792458e8


# ---------------------------------------------------------------------------
# Free-space (Friis)
# ---------------------------------------------------------------------------

class FreeSpaceChannel(ChannelModel):
    """
    Free-space path loss (Friis).

    PL(dB) = 20 log10(4π d f / c)
    Path gain returned is the negative of PL (so a smaller number is worse).
    """

    FIDELITY_LEVEL = 0

    def __init__(self, carrier_hz: float, radio: Optional[RadioParameters] = None):
        super().__init__(radio)
        self.carrier_hz = carrier_hz

    def _path_gain_db(self, distance_m: float) -> float:
        if distance_m <= 0.0:
            return 0.0
        wavelength = _SPEED_OF_LIGHT / self.carrier_hz
        pl_db = 20.0 * math.log10(4.0 * math.pi * distance_m / wavelength)
        return -pl_db

    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float = 0.0,
                 env: Optional["GaussianSplatEnv"] = None,
                 tx_id: str = "tx", rx_id: str = "rx") -> ChannelState:
        d = self._distance(tx_pose, rx_pose)
        pg = self._path_gain_db(d)

        state = ChannelState(
            tx_id=tx_id, rx_id=rx_id, timestamp=t,
            path_gain_db=pg,
            los=True,
            distance_m=d,
            delay_ms=d / _SPEED_OF_LIGHT * 1e3,
            doppler_hz=self._doppler(tx_pose, rx_pose, self.carrier_hz),
            metadata={"model": "free_space", "carrier_hz": self.carrier_hz},
        )
        if d > self.radio.max_range_m:
            state.path_gain_db = float("-inf")
        fill_link_metrics(state, self.radio)
        return state


# ---------------------------------------------------------------------------
# Log-distance (general purpose)
# ---------------------------------------------------------------------------

class LogDistanceChannel(FreeSpaceChannel):
    """
    Log-distance path loss model:

        PL(d) = PL(d0) + 10 * n * log10(d / d0)  + Xσ

    where ``n`` is the path-loss exponent, ``d0`` the reference distance,
    and ``Xσ`` an optional zero-mean Gaussian shadowing (dB) with std
    ``shadowing_std``.

    ``PL(d0)`` itself comes from the parent free-space (Friis) model at the
    reference distance, so the log-distance curve is anchored to a physical
    near-field value rather than an arbitrary constant.
    """

    FIDELITY_LEVEL = 0

    def __init__(self, carrier_hz: float,
                 d0_m: float = 1.0,
                 exponent: float = 3.0,
                 shadowing_std: float = 0.0,
                 nlos_penalty_db: float = 20.0,
                 radio: Optional[RadioParameters] = None):
        super().__init__(carrier_hz, radio)
        # reference distance for the log-distance model
        self.d0_m = d0_m
        # path-loss exponent (2 = free space, 3-4 typical urban)
        self.exponent = exponent
        # shadow-fading std (dB); 0 disables
        self.shadowing_std = shadowing_std
        # NLOS penalty (dB) added when a higher layer flags the link NLOS;
        # Layer 0 always assumes LoS, so this is a hook for Layer 1.
        self.nlos_penalty_db = nlos_penalty_db

    def _path_gain_db(self, distance_m: float) -> float:
        if distance_m <= 0.0:
            return 0.0
        # PL(d0) from the free-space (Friis) parent, as a positive number.
        pl_d0 = -super()._path_gain_db(self.d0_m)
        pl_db = pl_d0 + 10.0 * self.exponent * math.log10(distance_m / self.d0_m)
        if self.shadowing_std > 0.0:
            pl_db += np.random.normal(0.0, self.shadowing_std)
        return -pl_db


# ---------------------------------------------------------------------------
# Cellular (3G / 4G)
# ---------------------------------------------------------------------------

# Common cellular bands (downlink centre frequency, Hz).
_CELLULAR_BANDS = {
    "3g_900":  0.94e9,
    "3g_1900": 1.95e9,
    "4g_700":  0.74e9,
    "4g_1800": 1.84e9,
    "4g_2600": 2.60e9,
}


class CellularChannel(LogDistanceChannel):
    """
    3G / 4G cellular channel with band presets.

    Uses a log-distance model (exponent ~3.5 urban, shadowing optional).
    Pass ``band`` as one of the keys in :data:`_CELLULAR_BANDS` or set
    ``carrier_hz`` directly.
    """

    FIDELITY_LEVEL = 0

    def __init__(self, band: str = "4g_2600",
                 carrier_hz: Optional[float] = None,
                 exponent: float = 3.5,
                 shadowing_std: float = 6.0,
                 radio: Optional[RadioParameters] = None):
        if carrier_hz is None:
            if band not in _CELLULAR_BANDS:
                raise KeyError(
                    f"Unknown cellular band '{band}'. "
                    f"Known: {sorted(_CELLULAR_BANDS)}")
            carrier_hz = _CELLULAR_BANDS[band]
        super().__init__(
            carrier_hz=carrier_hz,
            d0_m=1.0,
            exponent=exponent,
            shadowing_std=shadowing_std,
            nlos_penalty_db=20.0,
            radio=radio,
        )
        self.band = band


# ---------------------------------------------------------------------------
# 5G NR sub-6 (CI model aligned with 3GPP TR 38.901)
# ---------------------------------------------------------------------------

class NRChannel(FreeSpaceChannel):
    """
    5G NR sub-6 GHz using the close-in (CI) reference-distance model:

        PL_CI(d) = 32.4 + 20*n*log10(d [m]) + 20*log10(f [GHz])

    with n = 2.0 (LoS) for UMi/UMa.  Shadowing and an NLOS exponent can be
    configured for Layer-1 extension.
    """

    FIDELITY_LEVEL = 0

    def __init__(self, carrier_hz: float = 3.5e9,
                 exponent: float = 2.0,
                 shadowing_std: float = 4.0,
                 radio: Optional[RadioParameters] = None):
        super().__init__(carrier_hz, radio)
        self.exponent = exponent
        self.shadowing_std = shadowing_std

    def _path_gain_db(self, distance_m: float) -> float:
        if distance_m <= 0.0:
            return 0.0
        f_ghz = self.carrier_hz / 1e9
        pl_db = 32.4 + 20.0 * self.exponent * math.log10(distance_m) \
            + 20.0 * math.log10(f_ghz)
        if self.shadowing_std > 0.0:
            pl_db += np.random.normal(0.0, self.shadowing_std)
        return -pl_db


# ---------------------------------------------------------------------------
# mmWave
# ---------------------------------------------------------------------------

# Oxygen absorption (dB/km) near common mmWave windows.
# 28 GHz ≈ 0.1, 60 GHz peak ≈ 15, 73 GHz ≈ 0.3.
_MMWAVE_O2_ABSORPTION = {
    28e9: 0.1,
    60e9: 15.0,
    73e9: 0.3,
}


class MmWaveChannel(FreeSpaceChannel):
    """
    mmWave channel (28 / 60 / 73 GHz).

    Path loss: CI model + oxygen absorption + optional severe NLOS penalty.
    Because mmWave is extremely blockage-sensitive, :meth:`generate` flags
    the link as ``los=True`` only in Layer 0; when a Layer-1 geometry query
    reports NLOS the caller can set ``state.los = False`` and recompute —
    this class leaves an ``nlos_penalty_db`` hook for that.
    """

    FIDELITY_LEVEL = 0

    def __init__(self, carrier_hz: float = 28e9,
                 exponent: float = 2.0,
                 shadowing_std: float = 5.2,
                 nlos_penalty_db: float = 30.0,
                 radio: Optional[RadioParameters] = None):
        super().__init__(carrier_hz, radio)
        self.exponent = exponent
        self.shadowing_std = shadowing_std
        self.nlos_penalty_db = nlos_penalty_db
        # Oxygen absorption (dB/km); look up closest band, default small.
        self.o2_absorption_db_per_km = min(
            _MMWAVE_O2_ABSORPTION.items(),
            key=lambda kv: abs(kv[0] - carrier_hz),
        )[1]

    def _path_gain_db(self, distance_m: float) -> float:
        if distance_m <= 0.0:
            return 0.0
        f_ghz = self.carrier_hz / 1e9
        pl_db = 32.4 + 20.0 * self.exponent * math.log10(distance_m) \
            + 20.0 * math.log10(f_ghz)
        # Oxygen absorption over this distance.
        pl_db += self.o2_absorption_db_per_km * (distance_m / 1000.0)
        if self.shadowing_std > 0.0:
            pl_db += np.random.normal(0.0, self.shadowing_std)
        return -pl_db
