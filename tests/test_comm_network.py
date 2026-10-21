"""
Test 8: Communication network simulator and topology.

Verifies dynamic topology construction, link up/down detection, connected
components, multi-hop Dijkstra routing, and the cluster-level ``step`` /
``send`` API of :class:`CommNetworkSimulator`.

Uses free-space channel models with deliberately chosen distances so the
expected connectivity is deterministic.
"""

import sys
import os
import math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from comm import (
    Pose, RadioParameters,
    FreeSpaceChannel, FiberChannel,
    CommNetworkSimulator, Topology,
)


def _pose(x, y=0.0, z=0.0):
    return Pose(position=np.array([float(x), float(y), float(z)]))


# ---------------------------------------------------------------------------
# Topology unit tests
# ---------------------------------------------------------------------------

def test_topology_adjacency():
    """Links above threshold become adjacency True."""
    topo = Topology(["a", "b", "c"], link_up_gain_db=-120.0)
    from comm.channel.base import ChannelState
    states = [
        ChannelState(tx_id="a", rx_id="b", path_gain_db=-80.0),
        ChannelState(tx_id="b", rx_id="a", path_gain_db=-80.0),
        ChannelState(tx_id="a", rx_id="c", path_gain_db=-200.0),  # down
    ]
    topo.update(states)
    assert topo.is_linked("a", "b")
    assert topo.is_linked("b", "a")
    assert not topo.is_linked("a", "c")
    assert topo.adjacency[0, 1] and topo.adjacency[1, 0]
    assert not topo.adjacency[0, 2]
    print(f"  [PASS] topology adjacency: a<->b up, a->c down")


def test_topology_connected_components():
    topo = Topology(["a", "b", "c", "d"], link_up_gain_db=-120.0)
    from comm.channel.base import ChannelState
    states = [
        ChannelState(tx_id="a", rx_id="b", path_gain_db=-80.0),
        ChannelState(tx_id="b", rx_id="a", path_gain_db=-80.0),
        ChannelState(tx_id="c", rx_id="d", path_gain_db=-80.0),
        ChannelState(tx_id="d", rx_id="c", path_gain_db=-80.0),
    ]
    topo.update(states)
    comps = topo.connected_components()
    comp_sets = [frozenset(c) for c in comps]
    assert frozenset({"a", "b"}) in comp_sets
    assert frozenset({"c", "d"}) in comp_sets
    assert len(comps) == 2
    print(f"  [PASS] topology components: {comps}")


def test_topology_routing_dijkstra():
    """a -> b -> c multi-hop when a-c is down."""
    topo = Topology(["a", "b", "c"], link_up_gain_db=-120.0)
    from comm.channel.base import ChannelState
    states = [
        ChannelState(tx_id="a", rx_id="b", path_gain_db=-80.0, delay_ms=1.0),
        ChannelState(tx_id="b", rx_id="a", path_gain_db=-80.0, delay_ms=1.0),
        ChannelState(tx_id="b", rx_id="c", path_gain_db=-80.0, delay_ms=2.0),
        ChannelState(tx_id="c", rx_id="b", path_gain_db=-80.0, delay_ms=2.0),
        # a <-> c is down
        ChannelState(tx_id="a", rx_id="c", path_gain_db=-200.0),
        ChannelState(tx_id="c", rx_id="a", path_gain_db=-200.0),
    ]
    topo.update(states)
    path = topo.route("a", "c")
    assert path == ["a", "b", "c"], f"Expected a->b->c, got {path}"
    q = topo.route_quality(path)
    assert abs(q["delay_ms"] - 3.0) < 1e-6
    assert q["hops"] == 2
    print(f"  [PASS] topology routing: a->b->c, delay={q['delay_ms']:.1f}ms")


def test_topology_route_unreachable():
    topo = Topology(["a", "b"], link_up_gain_db=-120.0)
    from comm.channel.base import ChannelState
    topo.update([ChannelState(tx_id="a", rx_id="b", path_gain_db=-200.0)])
    assert topo.route("a", "b") is None
    print(f"  [PASS] topology routing: unreachable returns None")


# ---------------------------------------------------------------------------
# Network simulator
# ---------------------------------------------------------------------------

def test_simulator_basic_step():
    ch = FreeSpaceChannel(carrier_hz=2.4e9)
    sim = CommNetworkSimulator(ch)
    sim.add_node("a", _pose(0))
    sim.add_node("b", _pose(5))
    snap = sim.step(t=0.0)
    assert snap.n_nodes == 2
    assert len(snap.links) == 2  # both directions
    assert all(lk.up for lk in snap.links)
    assert len(snap.components) == 1
    print(f"  [PASS] simulator step: 2 nodes, 2 links up")


def test_simulator_link_down_when_far():
    """Beyond max_range, the link should be down and a link_down event fired."""
    radio = RadioParameters(max_range_m=20.0)
    ch = FreeSpaceChannel(carrier_hz=2.4e9, radio=radio)
    sim = CommNetworkSimulator(ch)
    sim.add_node("a", _pose(0))
    sim.add_node("b", _pose(5))   # close, up
    snap1 = sim.step(t=0.0)
    assert any(lk.up for lk in snap1.links)
    # Move b far away
    snap2 = sim.step(poses={"b": _pose(100)}, t=1.0)
    assert all(not lk.up for lk in snap2.links)
    # Should have produced a link_down event for a->b and b->a
    down_events = [e for e in snap2.events if e.type == "link_down"]
    assert len(down_events) >= 2, f"Expected >=2 link_down events, got {len(down_events)}"
    print(f"  [PASS] simulator link_down: {len(down_events)} events on move")


def test_simulator_link_up_event_on_first_step():
    ch = FreeSpaceChannel(carrier_hz=2.4e9)
    sim = CommNetworkSimulator(ch)
    sim.add_node("a", _pose(0))
    sim.add_node("b", _pose(5))
    snap = sim.step(t=0.0)
    up_events = [e for e in snap.events if e.type == "link_up"]
    assert len(up_events) >= 2, f"Expected 2 initial link_up, got {len(up_events)}"
    print(f"  [PASS] simulator link_up: {len(up_events)} events on first step")


def test_simulator_multi_hop_send():
    """a can reach c via b when a-c direct is too far."""
    radio = RadioParameters(max_range_m=15.0)
    ch = FreeSpaceChannel(carrier_hz=2.4e9, radio=radio)
    sim = CommNetworkSimulator(ch)
    sim.add_node("a", _pose(0))
    sim.add_node("b", _pose(10))   # within range of a and c
    sim.add_node("c", _pose(20))   # out of range of a directly
    sim.step(t=0.0)
    # a -> c must relay through b
    r = sim.send("a", "c")
    assert r.delivered, f"a->c should be delivered via relay, got {r.drop_reason}"
    assert r.path == ["a", "b", "c"], f"Expected a->b->c, got {r.path}"
    print(f"  [PASS] simulator multi-hop: a->b->c, delay={r.end_to_end_delay_ms:.4f}ms")


def test_simulator_send_no_route():
    radio = RadioParameters(max_range_m=10.0)
    ch = FreeSpaceChannel(carrier_hz=2.4e9, radio=radio)
    sim = CommNetworkSimulator(ch)
    sim.add_node("a", _pose(0))
    sim.add_node("b", _pose(100))  # isolated
    sim.step(t=0.0)
    r = sim.send("a", "b")
    assert not r.delivered
    assert r.drop_reason == "no_route"
    print(f"  [PASS] simulator send: no_route when isolated")


def test_simulator_mixed_channels():
    """Different nodes can use different channel models."""
    sim = CommNetworkSimulator()
    sim.add_node("fiber_a", _pose(0), channel=FiberChannel(capacity_bps=10e9))
    sim.add_node("fiber_b", _pose(2), channel=FiberChannel(capacity_bps=10e9))
    sim.add_node("wifi_c", _pose(3),
                 channel=FreeSpaceChannel(carrier_hz=2.4e9))
    snap = sim.step(t=0.0)
    # fiber_a <-> fiber_b should be up (very low loss)
    fb = [lk for lk in snap.links if lk.tx_id == "fiber_a" and lk.rx_id == "fiber_b"][0]
    assert fb.up and fb.capacity_bps == 10e9
    print(f"  [PASS] simulator mixed channels: fiber link {fb.capacity_bps/1e9:.0f} Gbps")


def test_simulator_snapshot_summary():
    ch = FreeSpaceChannel(carrier_hz=2.4e9)
    sim = CommNetworkSimulator(ch)
    for i, x in enumerate((0, 5, 10)):
        sim.add_node(f"n{i}", _pose(x))
    sim.step(t=0.0)
    s = sim.summary()
    assert s["n_nodes"] == 3
    assert s["n_links"] == 6  # 3 * 2 directions
    assert s["n_links_up"] == 6
    print(f"  [PASS] simulator summary: {s}")


def test_simulator_mobility_changes_topology():
    """A node walking away eventually drops its links — the core scenario
    for mobility-driven communication studies."""
    radio = RadioParameters(max_range_m=30.0)
    ch = FreeSpaceChannel(carrier_hz=2.4e9, radio=radio)
    sim = CommNetworkSimulator(ch)
    sim.add_node("base", _pose(0))
    sim.add_node("rover", _pose(5))
    sim.step(t=0.0)
    assert sim.topology.is_linked("base", "rover")

    # Rover walks away in 5 m increments.
    down_step = None
    for i in range(1, 12):
        x = 5 + i * 5
        sim.step(poses={"rover": _pose(x)}, t=float(i) * 0.1)
        if not sim.topology.is_linked("base", "rover"):
            down_step = i
            break
    assert down_step is not None, "Link should drop as rover moves away"
    print(f"  [PASS] simulator mobility: link dropped at step {down_step} (x={5 + down_step*5}m)")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run():
    print("Test: Communication Network Simulator")
    test_topology_adjacency()
    test_topology_connected_components()
    test_topology_routing_dijkstra()
    test_topology_route_unreachable()
    test_simulator_basic_step()
    test_simulator_link_down_when_far()
    test_simulator_link_up_event_on_first_step()
    test_simulator_multi_hop_send()
    test_simulator_send_no_route()
    test_simulator_mixed_channels()
    test_simulator_snapshot_summary()
    test_simulator_mobility_changes_topology()
    print()


if __name__ == "__main__":
    run()
