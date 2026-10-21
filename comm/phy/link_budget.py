"""
Link-budget utilities — turn a path-gain (dB) plus a radio configuration into
SNR, Shannon capacity, and BER.

These functions are the shared "back half" of every Layer-0 channel model:
the technology-specific code computes path gain from geometry, and this
module turns that gain into the link-quality metrics that the network
simulator and control experiments consume.
"""

from __future__ import annotations

import math
from typing import Optional

from comm.channel.base import ChannelState, RadioParameters
from comm.phy.modulations import ber_for, get_modulation

# Boltzmann constant
_K_BOLTZMANN = 1.380649e-23


def thermal_noise_dbm(radio: RadioParameters) -> float:
    """
    Receiver thermal noise power in dBm.

    N = k * T * B  (Watts) → dBm
    """
    noise_w = _K_BOLTZMANN * radio.temperature_k * radio.bandwidth_hz
    noise_dbm = 10.0 * math.log10(noise_w / 1e-3) + radio.noise_figure_db
    return noise_dbm


def snr_db(path_gain_db: float, radio: RadioParameters) -> float:
    """
    Received SNR in dB.

    SNR = Pt + Gt + Gr - PL - N
    """
    rx_power_dbm = (radio.tx_power_dbm + radio.tx_gain_dbi + radio.rx_gain_dbi
                    + path_gain_db)
    return rx_power_dbm - thermal_noise_dbm(radio)


def shannon_capacity_bps(snr_db: float, bandwidth_hz: float) -> float:
    """Shannon-Hartley capacity (bits/s). Clamped at snr >= -inf."""
    if not math.isfinite(snr_db):
        return 0.0
    snr_lin = 10.0 ** (snr_db / 10.0)
    if snr_lin <= 0.0:
        return 0.0
    return bandwidth_hz * math.log2(1.0 + snr_lin)


def fill_link_metrics(state: ChannelState, radio: RadioParameters) -> ChannelState:
    """
    Populate ``snr_db``, ``capacity_bps`` and ``ber`` on an existing
    :class:`ChannelState` whose ``path_gain_db`` and ``bandwidth_hz`` are
    already set.  Returns the same object for chaining.
    """
    state.bandwidth_hz = radio.bandwidth_hz
    if not state.is_up:
        state.snr_db = float("-inf")
        state.capacity_bps = 0.0
        state.ber = 0.5
        return state

    state.snr_db = snr_db(state.path_gain_db, radio)
    state.capacity_bps = shannon_capacity_bps(state.snr_db, radio.bandwidth_hz)
    state.ber = ber_for(radio.modulation, state.snr_db)
    return state
