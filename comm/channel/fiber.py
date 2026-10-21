"""
Fiber-optic channel model — an approximately lossless fixed-capacity pipe.

On a robot platform fiber is used for short internal interconnects (compute
unit ↔ sensor module) where dispersion and attenuation are negligible.  This
model therefore treats the link as a deterministic, near-zero-latency, fixed-
capacity channel whose "path gain" is a configurable constant.

For free-space optical (FSO) links between robots — which *do* need
turbulence and pointing-error modeling — a dedicated ``FSOChannel`` ships
in a later phase (P2).
"""

from __future__ import annotations

import math
from typing import Optional, TYPE_CHECKING

from comm.channel.base import (
    ChannelModel, ChannelState, Pose, RadioParameters,
)
from comm.phy.link_budget import fill_link_metrics

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv

# A generous fiber attenuation: 0.2 dB/km at 1550 nm.  On a robot-scale link
# (metres) this is effectively zero.
_FIBER_ATTENUATION_DB_PER_KM = 0.2
# Propagation speed in fiber (~2/3 c).
_FIBER_PROPAGATION_SPEED = 2.0e8


class FiberChannel(ChannelModel):
    """
    Wired fiber link.

    ``capacity_bps`` is the configured fixed line rate (e.g. 10 GbE = 1e10).
    Latency is dominated by propagation over the cable length; we model the
    cable as following the straight-line tx-rx distance by default, but a
    caller may override via ``cable_length_m``.
    """

    FIDELITY_LEVEL = 0

    def __init__(self, capacity_bps: float = 1e10,
                 cable_length_m: Optional[float] = None,
                 attenuation_db_per_km: float = _FIBER_ATTENUATION_DB_PER_KM,
                 radio: Optional[RadioParameters] = None):
        # Fiber doesn't really use the RF radio parameters, but we keep a
        # RadioParameters so that the network simulator can ask every node
        # for one uniformly.  We synthesise a radio whose bandwidth reflects
        # the fiber line rate.
        if radio is None:
            radio = RadioParameters(
                tx_power_dbm=0.0,
                tx_gain_dbi=0.0,
                rx_gain_dbi=0.0,
                noise_figure_db=0.0,
                bandwidth_hz=capacity_bps,  # used only for capacity display
                modulation="BPSK",
                max_range_m=1e6,
            )
        super().__init__(radio)
        self.capacity_bps = capacity_bps
        self.cable_length_m = cable_length_m
        self.attenuation_db_per_km = attenuation_db_per_km

    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float = 0.0,
                 env: Optional["GaussianSplatEnv"] = None,
                 tx_id: str = "tx", rx_id: str = "rx") -> ChannelState:
        if self.cable_length_m is not None:
            d = self.cable_length_m
        else:
            d = self._distance(tx_pose, rx_pose)

        # Path gain: negative of cable attenuation.
        loss_db = self.attenuation_db_per_km * (d / 1000.0)
        pg_db = -loss_db

        state = ChannelState(
            tx_id=tx_id, rx_id=rx_id, timestamp=t,
            path_gain_db=pg_db,
            los=True,                  # wired: always "up" structurally
            distance_m=d,
            delay_ms=d / _FIBER_PROPAGATION_SPEED * 1e3,
            doppler_hz=0.0,
            bandwidth_hz=self.capacity_bps,  # line rate
            metadata={
                "model": "fiber",
                "capacity_bps": self.capacity_bps,
                "cable_length_m": d,
            },
        )
        # Fill SNR/BER from the radio config; fiber is essentially noise-free,
        # so SNR will be very high and BER ≈ 0.
        fill_link_metrics(state, self.radio)
        # Override capacity to the configured line rate — Shannon will give a
        # huge number for a noiseless link, but the physical line rate is the
        # real cap.
        state.capacity_bps = self.capacity_bps if math.isfinite(state.snr_db) else 0.0
        return state
