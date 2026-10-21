"""
Communication node — binds a robot identity to a radio and a current pose.

A :class:`CommNode` is the network-layer abstraction of one robot's radio.
It owns the node id, its current :class:`~comm.channel.base.Pose`, and a
reference to the :class:`~comm.channel.base.ChannelModel` it uses.  The
network simulator moves poses and asks the channel model for link states.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from comm.channel.base import ChannelModel, Pose, RadioParameters


@dataclass
class CommNode:
    """One robot's communication endpoint."""
    node_id: str
    pose: Pose = field(default_factory=Pose)
    channel: Optional[ChannelModel] = None
    # Optional human-readable label (e.g. robot name).
    label: str = ""

    @property
    def radio(self) -> RadioParameters:
        return self.channel.radio if self.channel is not None else RadioParameters()

    def set_pose(self, position=None, orientation=None, velocity=None):
        """Update this node's pose in place (copies arrays)."""
        if position is not None:
            self.pose.position = position.copy() if hasattr(position, "copy") else position
        if orientation is not None:
            self.pose.orientation = orientation.copy() if hasattr(orientation, "copy") else orientation
        if velocity is not None:
            self.pose.velocity = velocity.copy() if hasattr(velocity, "copy") else velocity
