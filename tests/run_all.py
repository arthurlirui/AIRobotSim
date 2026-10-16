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

    print("=" * 60)
    print("  AIRobotSim — 3D Gaussian Splatting Robot Simulation")
    print("  Running all tests...")
    print("=" * 60)
    print()

    start = time.time()

    test_math_utils.run()
    test_env.run()
    test_robots.run()
    test_gait.run()
    test_simulation.run()
    test_terrain_urdf.run_all()

    elapsed = time.time() - start
    print()
    print("=" * 60)
    print(f"  All tests completed in {elapsed:.2f}s")
    print("=" * 60)


if __name__ == "__main__":
    main()
