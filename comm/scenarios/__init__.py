"""
Predefined communication experiment scenarios.

Each scenario is a self-contained function that sets up a swarm, runs the
network simulator, and returns a result summary.  They serve as both
integration tests and usage examples for the communication module.

Available scenarios:
  * :func:`swarm_relay` — N robots in a line; the middle one relays when
    the end-to-end link is too long.
  * :func:`mmwave_occlusion` — mmWave link sensitivity to an obstacle
    between two robots, comparing with 5G sub-6.
  * :func:`thz_short_range` — THz link capacity vs inter-robot spacing.
  * :func:`fso_pointing` — FSO pointing loss vs gait jitter amplitude.
"""

from __future__ import annotations

import numpy as np
from typing import Optional

from comm import (
    Pose, RadioParameters,
    FreeSpaceChannel, CellularChannel, NRChannel, MmWaveChannel,
    GeometricLOSChannel, THzChannel, FSOChannel, FiberChannel,
    CommNetworkSimulator,
)

__all__ = [
    "swarm_relay",
    "mmwave_occlusion",
    "thz_short_range",
    "fso_pointing",
    "run_all_scenarios",
]


# ---------------------------------------------------------------------------
# 1. Swarm relay — line topology with a relay node
# ---------------------------------------------------------------------------

def swarm_relay(n_robots: int = 5,
                spacing_m: float = 15.0,
                max_range_m: float = 20.0) -> dict:
    """
    N robots in a line at fixed spacing.  Only adjacent robots are within
    radio range, so end-to-end delivery requires multi-hop relay.

    Returns a dict with delivery success, path, and hop count.
    """
    radio = RadioParameters(max_range_m=max_range_m, bandwidth_hz=20e6)
    ch = FreeSpaceChannel(carrier_hz=2.4e9, radio=radio)
    sim = CommNetworkSimulator(ch)
    for i in range(n_robots):
        sim.add_node(f"r{i}", Pose(position=np.array([i * spacing_m, 0.0, 0.0])))
    sim.step(t=0.0)

    src, dst = "r0", f"r{n_robots - 1}"
    result = sim.send(src, dst)
    return {
        "scenario": "swarm_relay",
        "n_robots": n_robots,
        "delivered": result.delivered,
        "path": result.path,
        "hops": len(result.path) - 1 if result.path else 0,
        "end_to_end_delay_ms": result.end_to_end_delay_ms,
    }


# ---------------------------------------------------------------------------
# 2. mmWave occlusion — blockage sensitivity vs 5G sub-6
# ---------------------------------------------------------------------------

def mmwave_occlusion(env=None) -> dict:
    """
    Compare a 28 GHz mmWave link and a 3.5 GHz 5G NR link between two
    robots, with and without a geometric occluder (requires a
    ``GaussianSplatEnv`` with an obstacle between the nodes).

    If ``env`` is None a simple flat-ground env is used (no occlusion);
    the caller should supply an env with an obstacle to see the
    difference.
    """
    from core.gaussian_splat_env import GaussianSplatEnv
    if env is None:
        env = GaussianSplatEnv.create_flat_ground(size=12.0, resolution=8)

    tx = Pose(position=np.array([0.0, 0.0, 0.5]))
    rx = Pose(position=np.array([5.0, 0.0, 0.5]))

    # mmWave with geometric occlusion
    mmwave_base = MmWaveChannel(carrier_hz=28e9, shadowing_std=0.0)
    mmwave_geo = GeometricLOSChannel(mmwave_base, nlos_penalty_db=40.0,
                                     hard_block=True)
    # 5G NR with geometric occlusion (penetrates better)
    nr_base = NRChannel(carrier_hz=3.5e9, shadowing_std=0.0)
    nr_geo = GeometricLOSChannel(nr_base, nlos_penalty_db=15.0,
                                 hard_block=False)

    st_mm = mmwave_geo.generate(tx, rx, t=0.0, env=env, tx_id="tx", rx_id="rx")
    st_nr = nr_geo.generate(tx, rx, t=0.0, env=env, tx_id="tx", rx_id="rx")

    return {
        "scenario": "mmwave_occlusion",
        "mmwave_up": st_mm.is_up,
        "mmwave_path_gain_db": st_mm.path_gain_db,
        "mmwave_los": st_mm.los,
        "nr_up": st_nr.is_up,
        "nr_path_gain_db": st_nr.path_gain_db,
        "nr_los": st_nr.los,
    }


# ---------------------------------------------------------------------------
# 3. THz short-range — capacity vs spacing
# ---------------------------------------------------------------------------

def thz_short_range(distances_m: Optional[list[float]] = None) -> dict:
    """
    Measure THz (300 GHz) link capacity at increasing inter-robot spacing.

    Returns a dict mapping distance → capacity (Mbps).
    """
    if distances_m is None:
        distances_m = [1.0, 2.0, 5.0, 10.0, 20.0]
    radio = RadioParameters(bandwidth_hz=10e9, max_range_m=100.0,
                            modulation="16QAM")
    ch = THzChannel(carrier_hz=300e9, radio=radio)
    tx = Pose(position=np.array([0.0, 0.0, 0.5]))
    results = {}
    for d in distances_m:
        rx = Pose(position=np.array([d, 0.0, 0.5]))
        st = ch.generate(tx, rx, tx_id="tx", rx_id="rx")
        results[f"{d}m"] = {
            "capacity_mbps": st.capacity_bps / 1e6,
            "path_gain_db": st.path_gain_db,
            "up": st.is_up,
        }
    return {"scenario": "thz_short_range", "results": results}


# ---------------------------------------------------------------------------
# 4. FSO pointing — loss vs jitter amplitude
# ---------------------------------------------------------------------------

def fso_pointing(jitter_values_m: Optional[list[float]] = None) -> dict:
    """
    Measure FSO link quality at increasing pointing-jitter amplitudes
    (simulating gait-induced vibration).

    Returns a dict mapping jitter → path gain and collection fraction.
    """
    if jitter_values_m is None:
        jitter_values_m = [0.0, 0.001, 0.005, 0.01, 0.02, 0.05]
    tx = Pose(position=np.array([0.0, 0.0, 0.5]))
    rx = Pose(position=np.array([20.0, 0.0, 0.5]))
    results = {}
    for jitter in jitter_values_m:
        ch = FSOChannel(pointing_jitter_m=jitter, cn2=1e-14)
        st = ch.generate(tx, rx, tx_id="tx", rx_id="rx")
        results[f"{jitter*1000:.0f}mm"] = {
            "path_gain_db": st.path_gain_db,
            "pointing_collection": st.metadata.get("pointing_collection", 0.0),
            "up": st.is_up,
        }
    return {"scenario": "fso_pointing", "results": results}


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run_all_scenarios() -> list[dict]:
    """Run all predefined scenarios and return a list of result dicts."""
    results = []
    results.append(swarm_relay())
    results.append(mmwave_occlusion())
    results.append(thz_short_range())
    results.append(fso_pointing())
    return results
