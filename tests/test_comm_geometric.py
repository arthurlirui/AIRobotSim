"""
Test 9: Geometric LOS occlusion (Layer 1).

Verifies that :class:`GeometricLOSChannel` correctly detects occlusion by
querying ``GaussianSplatEnv.ray_cast`` and applies the NLOS penalty.

**Critical regression assertion**: the number of down-links with geometric
occlusion must be *strictly greater* than without it when an obstacle is
present — this is the core differentiator of the platform.
"""

import sys
import os
import math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv, GaussianSplat
from comm import (
    Pose, RadioParameters,
    FreeSpaceChannel, MmWaveChannel, NRChannel,
    GeometricLOSChannel, CommNetworkSimulator,
)


def _pose(x, y=0.0, z=0.5):
    return Pose(position=np.array([float(x), float(y), float(z)]))


def _make_env_with_wall() -> GaussianSplatEnv:
    """
    Create a flat-ground env with a tall wall of splats between x=2 and x=3
    that blocks the line of sight at z=0.5.
    """
    env = GaussianSplatEnv.create_flat_ground(size=8.0, resolution=4)
    # Add a wall of splats at x≈2.5, y from -1 to 1, z from 0 to 1.5
    wall_splats = []
    for y in np.linspace(-1.0, 1.0, 8):
        for z in np.linspace(0.3, 1.5, 6):
            wall_splats.append(GaussianSplat(
                position=np.array([2.5, y, z]),
                scale=np.array([0.15, 0.15, 0.15]),
                rotation=np.eye(3),
                opacity=1.0,
            ))
    for s in wall_splats:
        env.add_splat(s)
    return env


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_los_clear_without_obstacle():
    """Without an obstacle, LOS is always True."""
    env = GaussianSplatEnv.create_flat_ground(size=8.0, resolution=4)
    base = FreeSpaceChannel(carrier_hz=2.4e9)
    ch = GeometricLOSChannel(base, nlos_penalty_db=30.0)
    tx, rx = _pose(0), _pose(5)
    st = ch.generate(tx, rx, t=0.0, env=env, tx_id="a", rx_id="b")
    assert st.los is True
    assert st.metadata["occluded"] is False
    print(f"  [PASS] LOS clear without obstacle: pg={st.path_gain_db:.1f} dB")


def test_nlos_with_wall():
    """A wall between tx and rx should flag NLOS and penalise the link."""
    env = _make_env_with_wall()
    base = FreeSpaceChannel(carrier_hz=2.4e9)
    ch = GeometricLOSChannel(base, nlos_penalty_db=30.0)
    tx, rx = _pose(0), _pose(5)  # wall at x=2.5 is between them
    st = ch.generate(tx, rx, t=0.0, env=env, tx_id="a", rx_id="b")
    assert st.los is False, "Expected NLOS due to wall"
    assert st.metadata["occluded"] is True
    # Compare with the un-occluded (no env) path gain
    st_clear = base.generate(tx, rx, tx_id="a", rx_id="b")
    penalty = st_clear.path_gain_db - st.path_gain_db
    assert abs(penalty - 30.0) < 1.0, (
        f"Expected ~30 dB penalty, got {penalty:.1f} dB")
    print(f"  [PASS] NLOS with wall: pg={st.path_gain_db:.1f} dB "
          f"(clear={st_clear.path_gain_db:.1f}, penalty={penalty:.1f} dB)")


def test_hard_block_drops_link():
    """With hard_block=True, NLOS link goes to -inf."""
    env = _make_env_with_wall()
    base = MmWaveChannel(carrier_hz=28e9, shadowing_std=0.0)
    ch = GeometricLOSChannel(base, hard_block=True)
    tx, rx = _pose(0), _pose(5)
    st = ch.generate(tx, rx, t=0.0, env=env, tx_id="a", rx_id="b")
    assert not st.is_up, "mmWave hard-blocked link should be down"
    assert math.isinf(st.path_gain_db)
    print(f"  [PASS] hard_block: mmWave link down when occluded")


def test_mmwave_more_sensitive_than_nr():
    """mmWave (hard_block) drops the link; 5G NR (soft penalty) stays up."""
    env = _make_env_with_wall()
    tx, rx = _pose(0), _pose(5)

    mmwave = GeometricLOSChannel(
        MmWaveChannel(carrier_hz=28e9, shadowing_std=0.0),
        hard_block=True)
    nr = GeometricLOSChannel(
        NRChannel(carrier_hz=3.5e9, shadowing_std=0.0),
        nlos_penalty_db=15.0, hard_block=False)

    st_mm = mmwave.generate(tx, rx, env=env, tx_id="a", rx_id="b")
    st_nr = nr.generate(tx, rx, env=env, tx_id="a", rx_id="b")

    assert not st_mm.is_up, "mmWave should be down (hard block)"
    assert st_nr.is_up, "5G NR should survive with 15 dB NLOS penalty"
    print(f"  [PASS] mmWave blocked, NR survives: mmwave_up={st_mm.is_up}, "
          f"nr_up={st_nr.is_up}, nr_pg={st_nr.path_gain_db:.1f} dB")


def test_no_env_no_occlusion_check():
    """Without env, occlusion is None (not checked) and link is unmodified."""
    base = FreeSpaceChannel(carrier_hz=2.4e9)
    ch = GeometricLOSChannel(base, nlos_penalty_db=30.0)
    tx, rx = _pose(0), _pose(5)
    st = ch.generate(tx, rx, t=0.0, env=None, tx_id="a", rx_id="b")
    assert st.metadata["occluded"] is None
    assert st.los is True  # unchanged from base
    print(f"  [PASS] no env: occlusion not checked, los=True")


def test_control_group_regression():
    """
    **Critical regression**: with an obstacle present, the geometric-LOS
    layer must produce *more* down-links than the pure statistical layer.

    This is the assertion that validates the platform's geometric-coupling
    differentiator: if it fails, the occlusion logic is not working.
    """
    env = _make_env_with_wall()
    radio = RadioParameters(max_range_m=100.0)

    # Three nodes: 0 ---wall--- 1 --- 2
    # Node 0 and 1 are separated by the wall; 1 and 2 are clear.
    nodes = {
        "n0": _pose(0),
        "n1": _pose(5),   # wall between n0 and n1
        "n2": _pose(8),   # clear path to n1
    }

    # --- Control group: statistical only (no geometry) ---
    sim_stat = CommNetworkSimulator(
        FreeSpaceChannel(carrier_hz=2.4e9, radio=radio))
    for nid, p in nodes.items():
        sim_stat.add_node(nid, p)
    snap_stat = sim_stat.step(t=0.0)
    stat_down = sum(1 for lk in snap_stat.links if not lk.up)

    # --- Treatment group: geometric LOS (hard block on occlusion) ---
    geo_ch = GeometricLOSChannel(
        FreeSpaceChannel(carrier_hz=2.4e9, radio=radio),
        hard_block=True)
    sim_geo = CommNetworkSimulator(geo_ch, env=env)
    for nid, p in nodes.items():
        sim_geo.add_node(nid, p)
    snap_geo = sim_geo.step(t=0.0)
    geo_down = sum(1 for lk in snap_geo.links if not lk.up)

    assert geo_down > stat_down, (
        f"Geometric layer must produce more down-links than statistical "
        f"when obstacle present: geo_down={geo_down}, stat_down={stat_down}")
    print(f"  [PASS] control group: stat_down={stat_down}, geo_down={geo_down} "
          f"(geometric occlusion adds {geo_down - stat_down} down-links)")


def test_network_simulator_env_integration():
    """CommNetworkSimulator passes env to GeometricLOSChannel.generate."""
    env = _make_env_with_wall()
    radio = RadioParameters(max_range_m=100.0)
    ch = GeometricLOSChannel(
        FreeSpaceChannel(carrier_hz=2.4e9, radio=radio),
        nlos_penalty_db=50.0, hard_block=False)
    sim = CommNetworkSimulator(ch, env=env)
    sim.add_node("a", _pose(0))
    sim.add_node("b", _pose(5))
    snap = sim.step(t=0.0)
    # a->b and b->a should both be occluded (wall at x=2.5)
    occluded = [lk for lk in snap.links if not lk.los]
    assert len(occluded) >= 2, (
        f"Expected >=2 occluded links, got {len(occluded)}")
    print(f"  [PASS] network env integration: {len(occluded)} links occluded")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run():
    print("Test: Geometric LOS Occlusion (Layer 1)")
    test_los_clear_without_obstacle()
    test_nlos_with_wall()
    test_hard_block_drops_link()
    test_mmwave_more_sensitive_than_nr()
    test_no_env_no_occlusion_check()
    test_control_group_regression()
    test_network_simulator_env_integration()
    print()


if __name__ == "__main__":
    run()
