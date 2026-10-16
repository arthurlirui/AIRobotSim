"""
Test 3: Robot kinematics — FK/IK for biped, quadruped, hexapod.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from robots.biped import Biped
from robots.quadruped import Quadruped
from robots.hexapod import Hexapod


def test_biped_creation():
    """Biped should have 2 legs and standing pose."""
    robot = Biped()
    assert robot.n_legs == 2
    assert len(robot.legs) == 2
    assert "leg_L" in robot.legs
    assert "leg_R" in robot.legs
    # Check foot positions exist
    robot.update_foot_positions()
    for name in robot.legs:
        assert name in robot.state.foot_positions
        foot = robot.state.foot_positions[name]
        # Feet should be near the ground
        assert foot[2] < 0.2, f"{name} foot Z={foot[2]}, expected near ground"
    print(f"  [PASS] Biped created with 2 legs, feet near ground")


def test_quadruped_creation():
    """Quadruped should have 4 legs and standing pose."""
    robot = Quadruped()
    assert robot.n_legs == 4
    assert len(robot.legs) == 4
    robot.update_foot_positions()
    for name in robot.legs:
        foot = robot.state.foot_positions[name]
        assert foot[2] < 0.2, f"{name} foot Z={foot[2]}, expected near ground"
    print(f"  [PASS] Quadruped created with 4 legs, feet near ground")


def test_hexapod_creation():
    """Hexapod should have 6 legs and standing pose."""
    robot = Hexapod()
    assert robot.n_legs == 6
    assert len(robot.legs) == 6
    robot.update_foot_positions()
    for name in robot.legs:
        foot = robot.state.foot_positions[name]
        assert foot[2] < 0.2, f"{name} foot Z={foot[2]}, expected near ground"
    # Tripod groups should have 3 legs each
    assert len(robot.tripod_a) == 3
    assert len(robot.tripod_b) == 3
    print(f"  [PASS] Hexapod created with 6 legs, tripod groups correct")


def test_fk_ik_consistency():
    """FK(IK(target)) should produce the same target (within tolerance)."""
    robot = Quadruped()
    robot.update_foot_positions()

    for name, leg in robot.legs.items():
        # Pick a target near the standing foot position
        foot = robot.state.foot_positions[name]
        # Move target slightly
        target = foot + np.array([0.05, 0.02, 0.0])

        angles = robot.leg_inverse_kinematics(name, target)
        assert angles is not None, f"IK failed for {name} target={target}"

        robot.set_joint_angles(name, angles)
        actual = robot.leg_forward_kinematics(name)
        error = np.linalg.norm(actual - target)
        assert error < 0.02, f"{name} FK/IK error={error:.4f}, target={target}, actual={actual}"

    print("  [PASS] FK/IK consistency (error < 20mm) for all legs")


def test_ik_unreachable():
    """IK should return None for unreachable targets."""
    robot = Quadruped()
    # Target very far away
    target = robot.state.base_position + np.array([10, 10, 10])
    angles = robot.leg_inverse_kinematics("leg_FL", target)
    # Should either return None or clip to limits (both acceptable)
    if angles is not None:
        robot.set_joint_angles("leg_FL", angles)
        actual = robot.leg_forward_kinematics("leg_FL")
        # Should not reach the target
        error = np.linalg.norm(actual - target)
        assert error > 0.5, f"IK should not reach unreachable target, error={error}"
    print("  [PASS] IK handles unreachable targets")


def test_support_polygon():
    """Robot standing should have a valid support polygon."""
    robot = Quadruped()
    robot.update_foot_positions()
    for name in robot.legs:
        robot.state.foot_contacts[name] = True
    poly = robot.support_polygon()
    assert poly.shape == (4, 2), f"Support polygon shape: {poly.shape}"
    # COM should be inside support polygon (robot is stable)
    assert robot.is_stable(), "Robot should be stable when standing"
    print("  [PASS] Support polygon and stability check work")


def test_hexapod_stability():
    """Hexapod should be stable with tripod gait (3 feet on ground)."""
    robot = Hexapod()
    robot.update_foot_positions()
    # All 6 feet on ground
    for name in robot.legs:
        robot.state.foot_contacts[name] = True
    assert robot.is_stable(), "Hexapod with all feet down should be stable"
    # Even with tripod (3 feet), should be stable
    for name in robot.tripod_b:
        robot.state.foot_contacts[name] = False
    assert robot.is_stable(), "Hexapod tripod should be stable"
    print("  [PASS] Hexapod stability with tripod gait")


def run():
    print("Test: Robot Kinematics")
    test_biped_creation()
    test_quadruped_creation()
    test_hexapod_creation()
    test_fk_ik_consistency()
    test_ik_unreachable()
    test_support_polygon()
    test_hexapod_stability()
    print()


if __name__ == "__main__":
    run()
