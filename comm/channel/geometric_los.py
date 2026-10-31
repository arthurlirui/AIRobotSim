"""
Layer-1 geometric line-of-sight channel.

Wraps a Layer-0 statistical channel model and adds 3D-geometry occlusion
checking by reusing :meth:`GaussianSplatEnv.ray_cast`.  When the ray from
tx to rx hits a Gaussian splat before reaching the receiver, the link is
flagged NLOS and a configurable penalty is applied to the path gain
(mmWave/THz/FSO are extremely blockage-sensitive, so the penalty can be
set high enough to drop the link entirely).

This is the core differentiator of the platform: link quality is driven by
the actual 3D reconstructed scene geometry, not just distance.
"""

from __future__ import annotations

import math
from typing import Optional, TYPE_CHECKING

import numpy as np

from comm.channel.base import ChannelModel, ChannelState, Pose
from comm.phy.link_budget import fill_link_metrics

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


class GeometricLOSChannel(ChannelModel):
    """
    Geometric occlusion wrapper around any Layer-0 channel model.

    Args:
        base_channel: the underlying statistical model (e.g. FreeSpaceChannel,
            MmWaveChannel) that provides distance-based path gain.
        nlos_penalty_db: path-gain penalty (dB, positive) applied when the
            tx→rx ray is blocked.  A large value (e.g. 40 dB) effectively
            drops mmWave/THz links; a smaller value models partial
            penetration for sub-6 GHz.
        hard_block: if True, NLOS links are set to path_gain_db = -inf
            (fully blocked), ignoring ``nlos_penalty_db``.
        ray_step: override for the ray-march step size (m).  If None, uses
            the env's default.
    """

    FIDELITY_LEVEL = 1

    def __init__(self, base_channel: ChannelModel,
                 nlos_penalty_db: float = 30.0,
                 hard_block: bool = False,
                 ray_step: Optional[float] = None):
        # Share the radio params from the base channel.
        super().__init__(base_channel.radio)
        self.base = base_channel
        self.nlos_penalty_db = nlos_penalty_db
        self.hard_block = hard_block
        self.ray_step = ray_step

    @property
    def carrier_hz(self) -> float:
        return getattr(self.base, "carrier_hz", 2.4e9)

    def _check_los(self, tx_pose: Pose, rx_pose: Pose,
                   env: "GaussianSplatEnv") -> bool:
        """
        Return True if the line of sight is clear (no splat intersection).

        Uses ``env.ray_cast`` with max_dist set to the tx-rx distance so
        that only occluders *between* the two nodes count.
        """
        delta = rx_pose.position - tx_pose.position
        dist = float(np.linalg.norm(delta))
        if dist < 1e-6:
            return True
        direction = delta / dist
        # Ray-cast with max_dist just below the full distance so a hit
        # means something is strictly between tx and rx.
        hit_dist = env.ray_cast(tx_pose.position, direction,
                                max_dist=dist - 0.01)
        return hit_dist is None

    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float = 0.0,
                 env: Optional["GaussianSplatEnv"] = None,
                 tx_id: str = "tx", rx_id: str = "rx") -> ChannelState:
        # Delegate the distance-based path gain to the base model.
        state = self.base.generate(tx_pose, rx_pose, t=t, env=None,
                                   tx_id=tx_id, rx_id=rx_id)

        if env is not None and state.is_up:
            los = self._check_los(tx_pose, rx_pose, env)
            state.los = los
            if not los:
                if self.hard_block:
                    state.path_gain_db = float("-inf")
                else:
                    state.path_gain_db -= self.nlos_penalty_db
                state.metadata["occluded"] = True
                # Re-derive link-budget metrics after the penalty.
                fill_link_metrics(state, self.radio)
            else:
                state.metadata["occluded"] = False
        else:
            state.metadata["occluded"] = None  # no env → not checked

        return state
