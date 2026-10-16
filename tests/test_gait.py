"""
Test 4: Gait planning — verify gait patterns produce valid foot trajectories.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from control.gait_planner import GaitPlanner, GaitType
from robots.quadruped import Quadruped
from robots.hexapod import Hexapod
from robots.biped import Biped


def test_tripod_gait_contacts():
    """Tripod gait should always have exactly 3 feet on the ground."""
    robot = Hexapod()
    leg_names = list(robot.legs.keys())
    leg_offsets = {name: robot.legs[name].hip_offset for name in leg_names}

    planner = GaitPlanner(GaitType.TRIPOD, stride_length=0.1, step_height=0.05, cycle_time=1.0)

    for phase_t in np.linspace(0, 1.0, 20, endpoint=False):
        planner.t = phase_t * planner.cycle_time
        cmd = planner.get_command(leg_names, leg_offsets)
        contacts = sum(cmd.contacts.values())
        assert contacts == 3, f"Tripod at phase {phase_t:.2f}: {contacts} feet on ground, expected 3"

    print("  [PASS] Tripod gait maintains 3 contact feet at all times")


def test_trot_gait_contacts():
    """Trot gait should have 2 feet on the ground (diagonal pairs)."""
    robot = Quadruped()
    leg_names = list(robot.legs.keys())
    leg_offsets = {name: robot.legs[name].hip_offset for name in leg_names}

    planner = GaitPlanner(GaitType.TROT, stride_length=0.1, step_height=0.05, cycle_time=1.0)

    for phase_t in np.linspace(0, 1.0, 20, endpoint=False):
        planner.t = phase_t * planner.cycle_time
        cmd = planner.get_command(leg_names, leg_offsets)
        contacts = sum(cmd.contacts.values())
        assert contacts == 2, f"Trot at phase {phase_t:.2f}: {contacts} feet on ground, expected 2"

    print("  [PASS] Trot gait maintains 2 contact feet (diagonal pairs)")


def test_walk_gait_contacts():
    """Walk gait should always have 3 feet on the ground."""
    robot = Quadruped()
    leg_names = list(robot.legs.keys())
    leg_offsets = {name: robot.legs[name].hip_offset for name in leg_names}

    planner = GaitPlanner(GaitType.WALK, stride_length=0.1, step_height=0.05, cycle_time=1.0)

    for phase_t in np.linspace(0, 1.0, 20, endpoint=False):
        planner.t = phase_t * planner.cycle_time
        cmd = planner.get_command(leg_names, leg_offsets)
        contacts = sum(cmd.contacts.values())
        assert contacts == 3, f"Walk at phase {phase_t:.2f}: {contacts} feet on ground, expected 3"

    print("  [PASS] Walk gait maintains 3 contact feet")


def test_wave_gait_contacts():
    """Wave gait should always have 5 feet on the ground (hexapod)."""
    robot = Hexapod()
    leg_names = list(robot.legs.keys())
    leg_offsets = {name: robot.legs[name].hip_offset for name in leg_names}

    planner = GaitPlanner(GaitType.WAVE, stride_length=0.1, step_height=0.05, cycle_time=1.0)

    for phase_t in np.linspace(0, 1.0, 20, endpoint=False):
        planner.t = phase_t * planner.cycle_time
        cmd = planner.get_command(leg_names, leg_offsets)
        contacts = sum(cmd.contacts.values())
        assert contacts == 5, f"Wave at phase {phase_t:.2f}: {contacts} feet on ground, expected 5"

    print("  [PASS] Wave gait maintains 5 contact feet")


def test_static_walk_biped():
    """Biped static walk should always have 1 foot on the ground."""
    robot = Biped()
    leg_names = list(robot.legs.keys())
    leg_offsets = {name: robot.legs[name].hip_offset for name in leg_names}

    planner = GaitPlanner(GaitType.STATIC_WALK, stride_length=0.1, step_height=0.05, cycle_time=1.0)

    for phase_t in np.linspace(0, 1.0, 20, endpoint=False):
        planner.t = phase_t * planner.cycle_time
        cmd = planner.get_command(leg_names, leg_offsets)
        contacts = sum(cmd.contacts.values())
        assert contacts == 1, f"Static walk at phase {phase_t:.2f}: {contacts} feet on ground, expected 1"

    print("  [PASS] Biped static walk maintains 1 contact foot")


def test_swing_trajectory_height():
    """Swing feet should lift off the ground (Z increases)."""
    robot = Quadruped()
    leg_names = list(robot.legs.keys())
    leg_offsets = {name: robot.legs[name].hip_offset for name in leg_names}

    planner = GaitPlanner(GaitType.TROT, stride_length=0.1, step_height=0.08, cycle_time=1.0)

    # At mid-swing, the foot should be elevated relative to its stance position
    planner.t = 0.25 * planner.cycle_time  # mid-swing for pair 1
    cmd = planner.get_command(leg_names, leg_offsets)

    # Find the swinging leg and compare its Z to a stance leg's Z
    swing_zs = []
    stance_zs = []
    for name in leg_names:
        z = cmd.foot_targets[name][2]
        if not cmd.contacts[name]:
            swing_zs.append(z)
        else:
            stance_zs.append(z)

    assert swing_zs, "No swinging legs found"
    assert stance_zs, "No stance legs found"
    swing_z = max(swing_zs)
    stance_z = min(stance_zs)
    lift = swing_z - stance_z
    assert lift > 0.01, f"Swing foot not lifted enough: swing_z={swing_z:.4f}, stance_z={stance_z:.4f}, lift={lift:.4f}"
    print("  [PASS] Swing trajectory lifts foot off ground")


def test_gait_advances():
    """GaitPlanner.advance should update the phase correctly."""
    planner = GaitPlanner(GaitType.TROT, cycle_time=1.0)
    assert planner.phase == 0.0
    planner.advance(0.5)
    assert abs(planner.phase - 0.5) < 1e-6
    planner.advance(0.6)  # should wrap around
    assert planner.phase < 0.2  # 0.1 after wrap
    print("  [PASS] Gait phase advances and wraps correctly")


def run():
    print("Test: Gait Planning")
    test_tripod_gait_contacts()
    test_trot_gait_contacts()
    test_walk_gait_contacts()
    test_wave_gait_contacts()
    test_static_walk_biped()
    test_swing_trajectory_height()
    test_gait_advances()
    print()


if __name__ == "__main__":
    run()
