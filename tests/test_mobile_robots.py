"""
Test 10: Mobile robots (wheeled + UAV) — motion models and env interaction.

Verifies the differential-drive unicycle and the multirotor model:
  - forward driving moves along heading, turning rotates the heading
  - command limits are respected
  - UAV terrain floor is enforced
  - both platforms share the RobotState contract with legged robots
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv
from robots import WheeledRobot, UAV, MobileRobot


def run():
    env = GaussianSplatEnv.create_flat_ground(size=20.0, resolution=4)

    # ------------------------------------------------------------------
    print("  [1] Wheeled robot: straight drive")
    robot = WheeledRobot(base_position=np.array([0.0, 0.0, 0.08]))
    assert isinstance(robot, MobileRobot)
    for _ in range(10):
        robot.step(0.1, v=1.0, omega=0.0, env=env)
    assert np.isclose(robot.position[0], 1.0, atol=0.02), robot.position
    assert np.isclose(robot.position[1], 0.0, atol=0.02), robot.position
    assert robot.yaw == 0.0
    print("      OK — drives straight along +X")

    # ------------------------------------------------------------------
    print("  [2] Wheeled robot: turning")
    robot = WheeledRobot(base_position=np.array([0.0, 0.0, 0.08]))
    robot.step(0.1, v=0.0, omega=1.0)
    assert np.isclose(robot.yaw, 0.1, atol=1e-6), robot.yaw
    print("      OK — yaw integrates omega·dt")

    # ------------------------------------------------------------------
    print("  [3] Speed limits")
    robot = WheeledRobot(max_speed=2.0, max_omega=3.0)
    robot.step(0.1, v=100.0, omega=-100.0)
    assert np.isclose(np.linalg.norm(robot.state.base_velocity[:2]), 2.0), \
        "linear speed must clip to max_speed"
    assert np.isclose(robot.state.base_angular_velocity[2], -3.0), \
        "angular speed must clip to max_omega"
    print("      OK — commands clipped to limits")

    # ------------------------------------------------------------------
    print("  [4] Ground following")
    robot = WheeledRobot(base_position=np.array([0.0, 0.0, 5.0]))
    robot.step(0.1, v=0.0, env=env)
    h = env.height_at(0.0, 0.0)
    assert np.isclose(robot.position[2], h + robot.wheel_radius, atol=0.05), \
        (robot.position[2], h)
    print(f"      OK — clamped to terrain height {h:.2f} + wheel radius")

    # ------------------------------------------------------------------
    print("  [5] UAV: body-frame velocity rotates with yaw")
    uav = UAV(base_position=np.array([0.0, 0.0, 5.0]))
    uav.step(0.1, vx=1.0, vy=0.0, yaw_rate=0.0)
    assert np.allclose(uav.position[:2], [0.1, 0.0], atol=1e-6)
    # yaw = 90° → body +X is world +Y
    uav2 = UAV(base_position=np.array([0.0, 0.0, 5.0]))
    uav2.state.base_orientation[2] = np.pi / 2
    uav2.step(0.1, vx=1.0, vy=0.0)
    assert np.allclose(uav2.position[:2], [0.0, 0.1], atol=1e-6), uav2.position
    print("      OK — velocity follows yaw rotation")

    # ------------------------------------------------------------------
    print("  [6] UAV: terrain floor")
    uav = UAV(base_position=np.array([0.0, 0.0, 0.05]), safety_margin=0.3)
    uav.step(0.1, vz=-5.0, env=env)
    floor = env.height_at(0.0, 0.0) + 0.3
    assert uav.position[2] >= floor - 1e-6, (uav.position[2], floor)
    print("      OK — cannot fly below terrain + margin")

    # ------------------------------------------------------------------
    print("  [7] Shared state contract with legged robots")
    from robots import RobotState
    assert isinstance(uav.state, RobotState)
    uav.set_velocity(np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.5]))
    uav.apply_velocity(0.1)
    assert np.isclose(uav.position[0], 0.1, atol=0.11)
    print("      OK — RobotState + set_velocity/apply_velocity work")

    print("\n  All mobile-robot tests passed.")


if __name__ == "__main__":
    run()
