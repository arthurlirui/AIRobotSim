"""
Base classes for the communication channel module.

Defines the unified ``ChannelState`` returned by every radio technology and the
abstract ``ChannelModel`` that each technology (3G/4G/5G/mmWave/THz/FSO/fiber)
implements.  The design follows an adapter pattern: the simulation core only
sees ``generate(tx_pose, rx_pose, t) -> ChannelState`` while the physics
details of each standard are encapsulated inside the concrete model.

Layer 0 (statistical baseline) and the fiber model ship in P0 with **no
extra dependencies** beyond numpy.  Higher-fidelity layers (geometric
occlusion, Sionna ray tracing, ns-3 bridge) plug into the same interface in
later phases.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


# ---------------------------------------------------------------------------
# Pose — lightweight carrier for a node's position + orientation
# ---------------------------------------------------------------------------

@dataclass
class Pose:
    """Position (m, world frame) + orientation as ZYX Euler angles (rad)."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))     # for Doppler

    def copy(self) -> "Pose":
        return Pose(
            position=self.position.copy(),
            orientation=self.orientation.copy(),
            velocity=self.velocity.copy(),
        )


# ---------------------------------------------------------------------------
# ChannelState — the single object every channel model produces per link
# ---------------------------------------------------------------------------

@dataclass
class ChannelState:
    """
    Snapshot of one tx→rx link at time ``t``.

    Fields are intentionally a superset so that both the low-resolution
    (capacity-level) and high-resolution (CIR-level) modes can populate the
    same structure; fields unused by a given fidelity level stay at their
    defaults.
    """
    tx_id: str
    rx_id: str
    timestamp: float = 0.0

    # Link-quality scalars (always populated)
    path_gain_db: float = 0.0       # path gain (dB); -inf => link down
    los: bool = True                # line-of-sight clear?
    distance_m: float = 0.0         # tx-rx 3D distance
    delay_ms: float = 0.0           # propagation + processing delay
    doppler_hz: float = 0.0         # Doppler shift

    # PHY-derived metrics (populated by link_budget when a radio params set
    # is available)
    bandwidth_hz: float = 0.0
    snr_db: float = 0.0
    capacity_bps: float = 0.0       # Shannon capacity estimate
    ber: float = 0.0                # bit error rate for current modulation

    # High-fidelity only (Layer 2+)
    cir: Optional[np.ndarray] = None    # channel impulse response, shape (N,)

    # Free-form per-technology metadata
    metadata: dict = field(default_factory=dict)

    @property
    def is_up(self) -> bool:
        """Link is usable (path gain finite and above -inf)."""
        return np.isfinite(self.path_gain_db)


# ---------------------------------------------------------------------------
# RadioParameters — technology-agnostic radio configuration
# ---------------------------------------------------------------------------

@dataclass
class RadioParameters:
    """
    Transmit / receive radio configuration shared across technologies.

    Concrete channel models may add technology-specific fields via
    ``metadata``; the fields below are enough for the Layer-0 link budget.
    """
    tx_power_dbm: float = 23.0      # transmit power (dBm)
    tx_gain_dbi: float = 0.0        # tx antenna gain (dBi)
    rx_gain_dbi: float = 0.0        # rx antenna gain (dBi)
    noise_figure_db: float = 7.0    # receiver noise figure (dB)
    bandwidth_hz: float = 20e6      # signal bandwidth (Hz)
    modulation: str = "QPSK"        # modulation name (see phy.modulations)
    temperature_k: float = 290.0    # noise temperature (K)
    max_range_m: float = 1e4        # beyond this the link is considered down


# ---------------------------------------------------------------------------
# ChannelModel — abstract base
# ---------------------------------------------------------------------------

class ChannelModel(ABC):
    """
    Abstract base for all radio channel models.

    Subclasses implement :meth:`generate`, which turns a pair of poses at
    time ``t`` into a :class:`ChannelState`.  The optional ``env`` argument
    (a :class:`~core.gaussian_splat_env.GaussianSplatEnv`) lets geometric
    layers query 3D occlusion; Layer-0 statistical models ignore it.
    """

    #: 0 = statistical, 1 = geometric occlusion, 2 = ray tracing, 3 = protocol
    FIDELITY_LEVEL: int = 0

    def __init__(self, radio: Optional[RadioParameters] = None):
        self.radio = radio if radio is not None else RadioParameters()

    @property
    def fidelity_level(self) -> int:
        return self.FIDELITY_LEVEL

    @abstractmethod
    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float = 0.0,
                 env: Optional["GaussianSplatEnv"] = None,
                 tx_id: str = "tx", rx_id: str = "rx") -> ChannelState:
        """Compute the channel state between two poses at time ``t``."""
        ...

    # ------------------------------------------------------------------
    # Shared helpers used by concrete Layer-0 models
    # ------------------------------------------------------------------

    @staticmethod
    def _distance(tx_pose: Pose, rx_pose: Pose) -> float:
        return float(np.linalg.norm(tx_pose.position - rx_pose.position))

    @staticmethod
    def _doppler(tx_pose: Pose, rx_pose: Pose, carrier_hz: float) -> float:
        """
        Doppler shift from relative radial velocity.

        f_d = (v_rel · r_hat) * f_c / c
        """
        delta = rx_pose.position - tx_pose.position
        dist = np.linalg.norm(delta)
        if dist < 1e-9:
            return 0.0
        r_hat = delta / dist
        v_rel = rx_pose.velocity - tx_pose.velocity
        radial = float(np.dot(v_rel, r_hat))
        return radial * carrier_hz / 2.99792458e8
