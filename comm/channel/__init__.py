"""
Channel-model registry and convenience constructors.

Public API:
    ChannelModel, ChannelState, Pose, RadioParameters  (from base)
    FreeSpaceChannel, LogDistanceChannel,
    CellularChannel, NRChannel, MmWaveChannel          (from statistical)
    FiberChannel                                        (from fiber)
    create_channel(name, **kwargs)                      (factory)
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

# Layer 2 — Sionna RT ray tracing (optional heavy dependency)
try:
    from comm.channel.sionna_rt import SionnaRTChannel, SIONNA_AVAILABLE
except ImportError:
    SionnaRTChannel = None
    SIONNA_AVAILABLE = False

# Factory table — name -> class.  Used by ``create_channel``.
_CHANNEL_REGISTRY = {
    "free_space": FreeSpaceChannel,
    "log_distance": LogDistanceChannel,
    "cellular": CellularChannel,
    "nr": NRChannel,
    "mmwave": MmWaveChannel,
    "thz": THzChannel,
    "fso": FSOChannel,
    "geometric_los": GeometricLOSChannel,
    "fiber": FiberChannel,
}
if SIONNA_AVAILABLE:
    _CHANNEL_REGISTRY["sionna_rt"] = SionnaRTChannel

# Friendly aliases mapping common technology names to the above.
_ALIASES = {
    "3g": "cellular",
    "4g": "cellular",
    "5g": "nr",
    "5g_nr": "nr",
    "sub6": "nr",
    "mmwave": "mmwave",
    "mm_wave": "mmwave",
    "terahertz": "thz",
    "optical_fiber": "fiber",
    "fibre": "fiber",
    "los": "geometric_los",
    "geometric": "geometric_los",
    "ray_tracing": "sionna_rt",
    "rt": "sionna_rt",
}


def create_channel(name: str, **kwargs) -> ChannelModel:
    """
    Construct a channel model by name.

    Examples:
        create_channel("4g", band="4g_2600")
        create_channel("5g", carrier_hz=3.5e9)
        create_channel("mmwave", carrier_hz=28e9)
        create_channel("fiber", capacity_bps=10e9)
        create_channel("free_space", carrier_hz=2.4e9)
    """
    key = name.lower().strip()
    key = _ALIASES.get(key, key)
    if key not in _CHANNEL_REGISTRY:
        raise KeyError(
            f"Unknown channel '{name}'. Known: {sorted(_CHANNEL_REGISTRY)} "
            f"(aliases: {sorted(_ALIASES)})")
    return _CHANNEL_REGISTRY[key](**kwargs)


def list_channels() -> list[str]:
    return sorted(_CHANNEL_REGISTRY)


__all__ = [
    "ChannelModel", "ChannelState", "Pose", "RadioParameters",
    "FreeSpaceChannel", "LogDistanceChannel",
    "CellularChannel", "NRChannel", "MmWaveChannel",
    "FiberChannel", "SIONNA_AVAILABLE",
    "create_channel", "list_channels",
]
