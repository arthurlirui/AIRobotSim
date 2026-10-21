"""
Dynamic topology — adjacency matrix, link-state table, and multi-hop relay
routing for the robot swarm.

The topology is recomputed every simulation step from the latest
:class:`~comm.channel.base.ChannelState` for each pair of nodes.  A link is
considered "up" when its path gain exceeds a configurable threshold
(:data:`Topology.LINK_UP_GAIN_DB`) — equivalently when the SNR supports at
least a minimal modulation.  Multi-hop routes use Dijkstra over the up-link
graph with edge weights derived from link delay (so relay paths prefer
low-latency hops).
"""

from __future__ import annotations

import math
import heapq
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from comm.channel.base import ChannelState


@dataclass
class LinkEntry:
    """One directional link in the topology table."""
    tx_id: str
    rx_id: str
    up: bool
    path_gain_db: float
    delay_ms: float
    capacity_bps: float
    ber: float
    snr_db: float


class Topology:
    """
    Manages the swarm's connectivity graph.

    The class is stateless across steps except for the current link table;
    callers feed it the per-step :class:`ChannelState` list and ask for the
    adjacency matrix or a route.
    """

    #: Minimum path gain (dB) for a link to be considered up.
    LINK_UP_GAIN_DB: float = -120.0

    def __init__(self, node_ids: list[str],
                 link_up_gain_db: Optional[float] = None):
        self.node_ids = list(node_ids)
        self.index = {nid: i for i, nid in enumerate(self.node_ids)}
        self.n = len(node_ids)
        if link_up_gain_db is not None:
            self.LINK_UP_GAIN_DB = link_up_gain_db
        # id -> id -> LinkEntry (directional)
        self.links: dict[tuple[str, str], LinkEntry] = {}
        # Adjacency matrix (bool), updated on each step.
        self.adjacency = np.zeros((self.n, self.n), dtype=bool)

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def update(self, states: list[ChannelState]) -> None:
        """Rebuild the link table and adjacency from a batch of link states."""
        self.links.clear()
        self.adjacency[:] = False
        for st in states:
            if st.tx_id not in self.index or st.rx_id not in self.index:
                continue
            up = st.is_up and st.path_gain_db >= self.LINK_UP_GAIN_DB
            entry = LinkEntry(
                tx_id=st.tx_id, rx_id=st.rx_id, up=up,
                path_gain_db=st.path_gain_db, delay_ms=st.delay_ms,
                capacity_bps=st.capacity_bps, ber=st.ber, snr_db=st.snr_db,
            )
            self.links[(st.tx_id, st.rx_id)] = entry
            if up:
                i, j = self.index[st.tx_id], self.index[st.rx_id]
                self.adjacency[i, j] = True

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def is_linked(self, tx_id: str, rx_id: str) -> bool:
        e = self.links.get((tx_id, rx_id))
        return e.up if e is not None else False

    def link(self, tx_id: str, rx_id: str) -> Optional[LinkEntry]:
        return self.links.get((tx_id, rx_id))

    def neighbors(self, node_id: str) -> list[str]:
        i = self.index.get(node_id)
        if i is None:
            return []
        return [self.node_ids[j] for j in range(self.n)
                if self.adjacency[i, j]]

    def connected_components(self) -> list[list[str]]:
        """Undirected connected components (symmetric adjacency)."""
        seen = [False] * self.n
        comps = []
        sym = self.adjacency | self.adjacency.T
        for start in range(self.n):
            if seen[start]:
                continue
            stack = [start]
            seen[start] = True
            comp = []
            while stack:
                k = stack.pop()
                comp.append(self.node_ids[k])
                for j in range(self.n):
                    if sym[k, j] and not seen[j]:
                        seen[j] = True
                        stack.append(j)
            comps.append(comp)
        return comps

    # ------------------------------------------------------------------
    # Routing — Dijkstra over up-links with delay weights
    # ------------------------------------------------------------------

    def route(self, src_id: str, dst_id: str,
              max_hops: int = 16) -> Optional[list[str]]:
        """
        Shortest-delay path from ``src_id`` to ``dst_id`` over up-links.

        Returns the list of node ids along the path (inclusive of both
        endpoints), or ``None`` if unreachable within ``max_hops``.
        """
        if src_id not in self.index or dst_id not in self.index:
            return None
        if src_id == dst_id:
            return [src_id]

        # Build adjacency from directional links that are up.
        graph: dict[str, list[tuple[str, float]]] = {
            nid: [] for nid in self.node_ids}
        for (tx, rx), e in self.links.items():
            if e.up and e.delay_ms > 0:
                graph[tx].append((rx, e.delay_ms))
            elif e.up:
                graph[tx].append((rx, 1e-3))  # tiny floor weight

        dist = {nid: math.inf for nid in self.node_ids}
        prev: dict[str, Optional[str]] = {nid: None for nid in self.node_ids}
        dist[src_id] = 0.0
        pq = [(0.0, 0, src_id)]  # (cost, hops, node)
        while pq:
            cost, hops, u = heapq.heappop(pq)
            if cost > dist[u]:
                continue
            if u == dst_id:
                break
            if hops >= max_hops:
                continue
            for v, w in graph.get(u, []):
                nd = cost + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, hops + 1, v))

        if dist[dst_id] == math.inf:
            return None
        # Reconstruct
        path = [dst_id]
        cur = dst_id
        while prev[cur] is not None:
            cur = prev[cur]
            path.append(cur)
        path.reverse()
        return path

    def route_quality(self, path: list[str]) -> dict:
        """
        Aggregate quality metrics for a multi-hop path.

        Returns dict with end-to-end delay (ms), bottleneck capacity (bps),
        product BER (probability of at least one bit error per hop-chain
        estimate — simplified).
        """
        if not path or len(path) < 2:
            return {"delay_ms": 0.0, "bottleneck_bps": math.inf,
                    "path_ber": 0.0, "hops": 0}
        total_delay = 0.0
        bottleneck = math.inf
        ber_prod = 1.0
        for i in range(len(path) - 1):
            e = self.links.get((path[i], path[i + 1]))
            if e is None or not e.up:
                return {"delay_ms": math.inf, "bottleneck_bps": 0.0,
                        "path_ber": 1.0, "hops": len(path) - 1}
            total_delay += e.delay_ms
            bottleneck = min(bottleneck, e.capacity_bps)
            # Probability a bit survives all hops = product of (1 - ber).
            ber_prod *= (1.0 - min(e.ber, 0.5))
        return {
            "delay_ms": total_delay,
            "bottleneck_bps": bottleneck,
            "path_ber": 1.0 - ber_prod,
            "hops": len(path) - 1,
        }
