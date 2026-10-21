"""
Cluster-level network simulator.

Every simulation step the :class:`CommNetworkSimulator` recomputes the
channel state for every ordered pair of nodes, updates the
:class:`~comm.network.topology.Topology`, and emits a
:class:`CommSnapshot` describing the instantaneous state of the whole
swarm.  It also implements :meth:`send` which, given a source/destination
pair and a payload, finds a multi-hop route and reports delivery
outcome (delay, packet-error probability, drop reason).

This is the Layer-0 / P0 implementation: it uses whatever
:class:`~comm.channel.base.ChannelModel` each node is configured with,
but does not yet call into geometric occlusion or ray tracing — those
higher-fidelity layers will subclass or wrap this simulator in later
phases.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Iterator, TYPE_CHECKING

import numpy as np

from comm.channel.base import ChannelModel, ChannelState, Pose
from comm.network.node import CommNode
from comm.network.topology import Topology, LinkEntry

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


# ---------------------------------------------------------------------------
# Snapshot & events
# ---------------------------------------------------------------------------

@dataclass
class LinkSnapshot:
    """A directional link as seen in one step's snapshot."""
    tx_id: str
    rx_id: str
    up: bool
    path_gain_db: float
    snr_db: float
    delay_ms: float
    capacity_bps: float
    ber: float
    distance_m: float
    los: bool


@dataclass
class CommEvent:
    """A topology event emitted during a step."""
    type: str               # "link_up", "link_down", "quality_change"
    tx_id: str
    rx_id: str
    timestamp: float
    details: dict = field(default_factory=dict)


@dataclass
class CommSnapshot:
    """Full swarm communication state at one timestep."""
    timestamp: float
    n_nodes: int
    links: list[LinkSnapshot]
    events: list[CommEvent]
    adjacency: np.ndarray   # bool matrix, node-index-ordered
    components: list[list[str]]


@dataclass
class DeliveryResult:
    """Outcome of a single :meth:`send` attempt."""
    delivered: bool
    path: list[str]
    end_to_end_delay_ms: float
    bottleneck_bps: float
    path_ber: float
    drop_reason: str = ""  # "" if delivered, else "no_route" / "link_down" / ...


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------

class CommNetworkSimulator:
    """
    Drives the communication state of a robot swarm.

    Typical use::

        sim = CommNetworkSimulator(channel_model)
        sim.add_node("r1", pose1)
        sim.add_node("r2", pose2)
        snap = sim.step({"r1": pose1_new, "r2": pose2_new}, t=0.1)
    """

    def __init__(self, channel_model: Optional[ChannelModel] = None,
                 env: Optional["GaussianSplatEnv"] = None,
                 link_up_gain_db: Optional[float] = None):
        """
        Args:
            channel_model: default channel model used when a node has none
                of its own.  If ``None``, a FreeSpaceChannel (2.4 GHz) is
                used.
            env: optional 3DGS environment; passed to channel models that
                support geometric queries (Layer 1+).  Layer 0 ignores it.
            link_up_gain_db: override the topology's link-up threshold.
        """
        from comm.channel.statistical import FreeSpaceChannel
        if channel_model is None:
            channel_model = FreeSpaceChannel(carrier_hz=2.4e9)
        self.default_channel = channel_model
        self.env = env
        self.nodes: dict[str, CommNode] = {}
        self.topology: Optional[Topology] = None
        self._link_up_gain_db = link_up_gain_db
        self._prev_link_state: dict[tuple[str, str], bool] = {}
        self.last_snapshot: Optional[CommSnapshot] = None

    # ------------------------------------------------------------------
    # Node management
    # ------------------------------------------------------------------

    def add_node(self, node_id: str, pose: Optional[Pose] = None,
                 channel: Optional[ChannelModel] = None,
                 label: str = "") -> CommNode:
        if node_id in self.nodes:
            raise KeyError(f"Node '{node_id}' already exists")
        node = CommNode(
            node_id=node_id,
            pose=pose if pose is not None else Pose(),
            channel=channel if channel is not None else self.default_channel,
            label=label or node_id,
        )
        self.nodes[node_id] = node
        self._rebuild_topology()
        return node

    def remove_node(self, node_id: str) -> None:
        self.nodes.pop(node_id, None)
        self._rebuild_topology()

    def _rebuild_topology(self) -> None:
        ids = list(self.nodes.keys())
        self.topology = Topology(ids, link_up_gain_db=self._link_up_gain_db)

    # ------------------------------------------------------------------
    # Step — recompute every link for the current poses
    # ------------------------------------------------------------------

    def step(self, poses: Optional[dict[str, Pose]] = None,
             t: float = 0.0) -> CommSnapshot:
        """
        Advance the network by one timestep.

        Args:
            poses: mapping node_id -> new pose.  Nodes not present keep
                their previous pose.  ``None`` keeps all poses.
            t: simulation time (s).
        """
        if poses is not None:
            for nid, p in poses.items():
                if nid in self.nodes:
                    self.nodes[nid].pose = p.copy()
        if self.topology is None:
            self._rebuild_topology()
            if self.topology is None:
                raise RuntimeError("Topology not initialised")

        states: list[ChannelState] = []
        ids = list(self.nodes.keys())
        for tx_id in ids:
            for rx_id in ids:
                if tx_id == rx_id:
                    continue
                tx = self.nodes[tx_id]
                rx = self.nodes[rx_id]
                st = tx.channel.generate(
                    tx.pose, rx.pose, t=t, env=self.env,
                    tx_id=tx_id, rx_id=rx_id)
                states.append(st)

        # Update topology.
        self.topology.update(states)

        # Detect events (link up/down transitions).
        events: list[CommEvent] = []
        new_state: dict[tuple[str, str], bool] = {}
        for st in states:
            up = self.topology.is_linked(st.tx_id, st.rx_id)
            new_state[(st.tx_id, st.rx_id)] = up
            prev = self._prev_link_state.get((st.tx_id, st.rx_id))
            if prev is None:
                if up:
                    events.append(CommEvent("link_up", st.tx_id, st.rx_id, t))
            elif prev != up:
                events.append(CommEvent(
                    "link_up" if up else "link_down",
                    st.tx_id, st.rx_id, t,
                    details={"path_gain_db": st.path_gain_db}))
        self._prev_link_state = new_state

        # Build link snapshots.
        link_snaps = [
            LinkSnapshot(
                tx_id=st.tx_id, rx_id=st.rx_id,
                up=self.topology.is_linked(st.tx_id, st.rx_id),
                path_gain_db=st.path_gain_db,
                snr_db=st.snr_db,
                delay_ms=st.delay_ms,
                capacity_bps=st.capacity_bps,
                ber=st.ber,
                distance_m=st.distance_m,
                los=st.los,
            )
            for st in states
        ]

        self.last_snapshot = CommSnapshot(
            timestamp=t,
            n_nodes=len(ids),
            links=link_snaps,
            events=events,
            adjacency=self.topology.adjacency.copy(),
            components=self.topology.connected_components(),
        )
        return self.last_snapshot

    # ------------------------------------------------------------------
    # Send — multi-hop delivery
    # ------------------------------------------------------------------

    def send(self, src_id: str, dst_id: str, payload_size_bits: int = 1000,
             max_hops: int = 16) -> DeliveryResult:
        """
        Attempt delivery from ``src_id`` to ``dst_id`` over the current
        topology.  Uses Dijkstra by delay; reports end-to-end delay,
        bottleneck capacity and product BER along the chosen path.
        """
        if self.topology is None:
            return DeliveryResult(False, [], math.inf, 0.0, 1.0,
                                  drop_reason="no_topology")
        if src_id not in self.nodes or dst_id not in self.nodes:
            return DeliveryResult(False, [], math.inf, 0.0, 1.0,
                                  drop_reason="unknown_node")

        path = self.topology.route(src_id, dst_id, max_hops=max_hops)
        if path is None:
            return DeliveryResult(False, [], math.inf, 0.0, 1.0,
                                  drop_reason="no_route")

        q = self.topology.route_quality(path)
        # Simple per-packet delivery probability: 1 - path_ber, sampled.
        # For deterministic reporting we flag delivered = path_ber < 0.5.
        delivered = q["path_ber"] < 0.5
        return DeliveryResult(
            delivered=delivered,
            path=path,
            end_to_end_delay_ms=q["delay_ms"]
                + payload_size_bits / max(q["bottleneck_bps"], 1.0) * 1e3,
            bottleneck_bps=q["bottleneck_bps"],
            path_ber=q["path_ber"],
            drop_reason="" if delivered else "high_ber",
        )

    # ------------------------------------------------------------------
    # Convenience iterators
    # ------------------------------------------------------------------

    def iter_links(self) -> Iterator[LinkEntry]:
        """Yield all directional links currently in the topology table."""
        if self.topology is None:
            return iter(())
        return iter(self.topology.links.values())

    def summary(self) -> dict:
        """Compact summary string for logging."""
        if self.last_snapshot is None:
            return {"n_nodes": len(self.nodes), "n_links": 0, "n_events": 0}
        snap = self.last_snapshot
        n_up = sum(1 for lk in snap.links if lk.up)
        return {
            "n_nodes": snap.n_nodes,
            "n_links": len(snap.links),
            "n_links_up": n_up,
            "n_events": len(snap.events),
            "n_components": len(snap.components),
            "timestamp": snap.timestamp,
        }
