"""
Modulation library — BER vs SNR curves for common schemes.

Closed-form (or tight approximation) BER expressions are used so that the
Layer-0 link budget can compute a bit-error-rate without pulling in a full
communications library.  For M-QAM the standard square-QAM approximation is
used; for BPSK/QPSK the exact expression is used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class ModulationSpec:
    """Description of one modulation scheme."""
    name: str
    bits_per_symbol: int
    # function: snr_linear -> ber
    ber_func: callable

    @property
    def spectral_efficiency(self) -> float:
        """bits/s/Hz (uncoded, Nyquist)."""
        return float(self.bits_per_symbol)


def _erfc(x: float) -> float:
    return math.erfc(x)


def _bpsk_ber(snr_lin: float) -> float:
    """Exact BER for coherent BPSK: 0.5 * erfc(sqrt(2*Eb/N0))."""
    if snr_lin <= 0.0:
        return 0.5
    return 0.5 * _erfc(math.sqrt(2.0 * snr_lin))


def _qpsk_ber(snr_lin: float) -> float:
    """QPSK has the same BER as BPSK per bit (Gray-coded)."""
    return _bpsk_ber(snr_lin)


def _mqam_ber(snr_lin: float, m: int) -> float:
    """
    Square M-QAM approximate BER (Gray coding), per bit.

    BER ≈ (4/log2(M)) * (1 - 1/sqrt(M)) * Q(sqrt(3*log2(M)/(M-1) * snr_lin))

    where Q(x) = 0.5 * erfc(x / sqrt(2)).
    """
    if snr_lin <= 0.0:
        return 0.5
    k = math.log2(m)
    c = 3.0 * k / (m - 1) * snr_lin
    q = 0.5 * _erfc(math.sqrt(c / 2.0))
    return (4.0 / k) * (1.0 - 1.0 / math.sqrt(m)) * q


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

_REGISTRY: dict[str, ModulationSpec] = {}


def _register(spec: ModulationSpec) -> ModulationSpec:
    _REGISTRY[spec.name.upper()] = spec
    return spec


BPSK = _register(ModulationSpec("BPSK", 1, _bpsk_ber))
OOK = _register(ModulationSpec("OOK", 1, _bpsk_ber))  # OOK ≈ BPSK per-bit BER
QPSK = _register(ModulationSpec("QPSK", 2, _qpsk_ber))
QAM16 = _register(ModulationSpec(
    "16QAM", 4, lambda s: _mqam_ber(s, 16)))
QAM64 = _register(ModulationSpec(
    "64QAM", 6, lambda s: _mqam_ber(s, 64)))
QAM256 = _register(ModulationSpec(
    "256QAM", 8, lambda s: _mqam_ber(s, 256)))


def get_modulation(name: str) -> ModulationSpec:
    """Look up a modulation by name (case-insensitive)."""
    key = name.upper()
    if key not in _REGISTRY:
        raise KeyError(
            f"Unknown modulation '{name}'. Known: {sorted(_REGISTRY)}")
    return _REGISTRY[key]


def list_modulations() -> list[str]:
    return sorted(_REGISTRY.keys())


def ber_for(modulation: str, snr_db: float) -> float:
    """Convenience: compute BER given modulation name and SNR in dB."""
    spec = get_modulation(modulation)
    snr_lin = 10.0 ** (snr_db / 10.0)
    return max(0.0, min(0.5, spec.ber_func(snr_lin)))
