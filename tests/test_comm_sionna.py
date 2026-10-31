"""
Test 13: Sionna RT channel adapter (Layer 2, optional dependency).

Skipped gracefully when sionna is not installed.  When present:

  - factory registration: ``create_channel("sionna_rt")`` resolves
  - FSPL control group: on flat ground the ray-traced path gain at 100 m
    / 3.5 GHz must match the analytic free-space value within 1 dB
  - range scaling: doubling the distance costs ~6 dB
  - the adapter plugs into CommNetworkSimulator (multi-robot swarm sim)
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from comm.channel import SIONNA_AVAILABLE


def run():
    if not SIONNA_AVAILABLE:
        print("  [SKIP] sionna not installed — SionnaRT tests skipped")
        return

    from comm import Pose, RadioParameters, create_channel, CommNetworkSimulator
    from comm.channel.sionna_rt import SionnaRTChannel

    # ------------------------------------------------------------------
    print("  [1] Factory registration")
    assert "sionna_rt" in create_channel.__globals__["_CHANNEL_REGISTRY"]
    print("      OK — 'sionna_rt' in channel registry")

    # ------------------------------------------------------------------
    print("  [2] FSPL control group (100 m @ 3.5 GHz)")
    ch = SionnaRTChannel(carrier_hz=3.5e9)
    p0 = Pose(position=np.array([0.0, 0.0, 1.0]))
    p1 = Pose(position=np.array([100.0, 0.0, 1.0]))
    s = ch.generate(p0, p1, t=0.0)
    c = 2.99792458e8
    fspl_db = -20 * np.log10(4 * np.pi * 100.0 * 3.5e9 / c)  # path gain (negative)
    assert abs(s.path_gain_db - fspl_db) < 1.0, \
        f"ray-traced {s.path_gain_db:.2f} dB vs FSPL {fspl_db:.2f} dB"
    assert s.los, "flat-ground direct path must be LOS"
    print(f"      OK — ray-traced {s.path_gain_db:.2f} dB ≈ FSPL {fspl_db:.2f} dB")

    # ------------------------------------------------------------------
    print("  [3] Distance scaling (~6 dB per doubling, two-ray allowed)")
    s50 = ch.generate(p0, Pose(position=np.array([50.0, 0.0, 1.0])), t=0.0)
    delta = s50.path_gain_db - s.path_gain_db
    # Ray tracing includes the ground-reflected path; constructive
    # two-ray combining can push the delta beyond the pure-FSPL 6 dB.
    assert 4.0 < delta < 10.0, f"expected ~6 dB (two-ray), got {delta:.2f}"
    print(f"      OK — 50 m vs 100 m: {delta:.2f} dB (two-ray ground reflection)")

    # ------------------------------------------------------------------
    print("  [4] Plugs into CommNetworkSimulator")
    sim = CommNetworkSimulator(ch)
    for i, x in enumerate((0.0, 40.0, 80.0)):
        sim.add_node(f"r{i}", Pose(position=np.array([x, 0.0, 1.0])))
    snap = sim.step(t=0.0)
    up = sum(1 for l in snap.links if l.up)
    assert up >= 2, f"chain topology should have ≥2 live links, got {up}"
    result = sim.send("r0", "r2")
    assert result.delivered, "multi-hop delivery over ray-traced links"
    print(f"      OK — {up} links up, r0→r2 delivered via {result.path}")

    # ------------------------------------------------------------------
    print("  [5] ChannelState metrics populated")
    assert s.snr_db != 0.0 and s.capacity_bps > 0.0
    print(f"      OK — SNR {s.snr_db:.1f} dB, "
          f"capacity {s.capacity_bps / 1e6:.1f} Mbps")

    print("\n  All Sionna RT tests passed.")


if __name__ == "__main__":
    run()
