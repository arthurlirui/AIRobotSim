"""PHY layer: modulation library and link-budget utilities."""

from comm.phy.modulations import (
    ModulationSpec,
    BPSK, QPSK, QAM16, QAM64, QAM256,
    get_modulation, list_modulations, ber_for,
)
from comm.phy.link_budget import (
    thermal_noise_dbm, snr_db, shannon_capacity_bps, fill_link_metrics,
)

__all__ = [
    "ModulationSpec",
    "BPSK", "QPSK", "QAM16", "QAM64", "QAM256",
    "get_modulation", "list_modulations", "ber_for",
    "thermal_noise_dbm", "snr_db", "shannon_capacity_bps", "fill_link_metrics",
]
