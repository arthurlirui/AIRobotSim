"""
AIRobotSim communication simulation module.

Provides a unified, technology-agnostic channel interface and a cluster-level
network simulator for studying swarm communication, inter-robot relay, and
mobility-driven link variation in 3D-reconstructed scenes.

P0 (this release) ships:
  * Channel base classes (:class:`ChannelModel`, :class:`ChannelState`,
    :class:`Pose`, :class:`RadioParameters`).
  * Layer-0 statistical channel models: free-space, log-distance, 3G/4G
    cellular, 5G NR sub-6, mmWave.
  * A fiber-optic wired link model.
  * A PHY link-budget helper (SNR / Shannon capacity / BER).
  * A cluster :class:`CommNetworkSimulator` with dynamic topology,
    multi-hop Dijkstra routing, link-up/down events and snapshots.

Higher-fidelity layers (geometric occlusion reusing
``GaussianSplatEnv.ray_cast``, Sionna RT ray tracing, ns-3 protocol bridge)
plug into the same ``ChannelModel.generate`` interface in later phases.
"""

from __future__ import annotations

from comm.channel.base import (
    ChannelModel, ChannelState, Pose, RadioParameters,
)
from comm.channel.statistical import (
    FreeSpaceChannel, LogDistanceChannel,
    CellularChannel, NRChannel, MmWaveChannel,
)
from comm.channel.geometric_los import GeometricLOSChannel
from comm.channel.thz import THzChannel
from comm.channel.fso import FSOChannel
from comm.channel.fiber import FiberChannel
from comm.channel import create_channel, list_channels
from comm.network import (
    CommNode, Topology, LinkEntry,
    CommNetworkSimulator, CommSnapshot, CommEvent, LinkSnapshot,
    DeliveryResult,
)
from comm.phy import (
    get_modulation, list_modulations, ber_for,
    thermal_noise_dbm, snr_db, shannon_capacity_bps, fill_link_metrics,
)

__all__ = [
    # base
    "ChannelModel", "ChannelState", "Pose", "RadioParameters",
    # channels
    "FreeSpaceChannel", "LogDistanceChannel",
    "CellularChannel", "NRChannel", "MmWaveChannel",
    "GeometricLOSChannel", "THzChannel", "FSOChannel", "FiberChannel",
    "create_channel", "list_channels",
    # network
    "CommNode", "Topology", "LinkEntry",
    "CommNetworkSimulator", "CommSnapshot", "CommEvent", "LinkSnapshot",
    "DeliveryResult",
    # phy
    "get_modulation", "list_modulations", "ber_for",
    "thermal_noise_dbm", "snr_db", "shannon_capacity_bps", "fill_link_metrics",
]

__version__ = "0.1.0"
