"""
Run all tests in sequence.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def main():
    from tests import test_math_utils, test_env, test_robots, test_gait, test_simulation
    from tests import test_terrain_urdf
    from tests import test_comm_channel, test_comm_network

    print("=" * 60)
    print("  AIRobotSim — 3D Gaussian Splatting Robot Simulation")
    print("  Running all tests...")
    print("=" * 60)
    print()

    start = time.time()

    # Each test module runs in its own try/except so that a pre-existing
    # failure in one module (e.g. a numpy-2 incompatibility in test_robots)
    # does not block the rest of the suite from running.
    modules = [
        ("Math Utilities", test_math_utils.run),
        ("3DGS Environment", test_env.run),
        ("Robot Kinematics", test_robots.run),
        ("Gait Planning", test_gait.run),
        ("Full Simulation", test_simulation.run),
        ("Terrain & URDF", test_terrain_urdf.run_all),
        ("Comm Channel (Layer 0)", test_comm_channel.run),
        ("Comm Network Simulator", test_comm_network.run),
    ]
    failed = []
    for label, fn in modules:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            failed.append((label, exc))
            print(f"  [FAIL] {label}: {type(exc).__name__}: {exc}")
            print()

    elapsed = time.time() - start
    print()
    print("=" * 60)
    print(f"  All tests completed in {elapsed:.2f}s"
          + (f"  ({len(failed)} module(s) failed)" if failed else ""))
    print("=" * 60)
    if failed:
        for label, exc in failed:
            print(f"  FAILED: {label} -> {type(exc).__name__}: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
