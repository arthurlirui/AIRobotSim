"""
Test 14: Design module — component-level morphology design.

Verifies:
  - component validation rejects broken designs (unknown links, cycles,
    multiple roots, bad actuators)
  - JSON round-trip preserves a design exactly
  - random_design produces valid trees across seeds
  - SimRobot FK is consistent: chain links are one link-length apart
  - Jacobian: J @ dq matches FK displacement numerically
  - IK places a foot at a reachable target within tolerance
  - PD stepping converges to a commanded pose and respects torque limits
  - world interaction: contacts evaluate against 3DGS terrain, ground
    snap lifts the body onto the surface, sensor mounts expose world
    poses usable by sensors/comm
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from design import (
    Link, Joint, Actuator, ContactPoint, SensorMount,
    RobotDesign, random_design, preset_quadruped, preset_hexapod,
    SimRobot, GaitController, gait_groups,
)
from core.gaussian_splat_env import GaussianSplatEnv


def run():
    # ------------------------------------------------------------------
    print("  [1] Design validation rejects broken morphologies")
    # Multiple roots
    d = RobotDesign(name="bad")
    d.links += [Link(name="a", is_root=True), Link(name="b", is_root=True)]
    try:
        d.validate(); assert False, "two roots must fail"
    except ValueError:
        pass
    # Cycle: a↔b
    d = RobotDesign(name="bad")
    d.links += [Link(name="a", is_root=True), Link(name="b")]
    d.joints += [Joint(name="j1", parent="a", child="b"),
                 Joint(name="j2", parent="b", child="a")]
    try:
        d.validate(); assert False, "cycle must fail"
    except ValueError:
        pass
    # Actuator on unknown joint
    d = preset_quadruped()
    d.actuators.append(Actuator(joint="nonexistent"))
    try:
        d.validate(); assert False, "unknown actuator joint must fail"
    except ValueError:
        pass
    print("      OK — multi-root / cycle / bad-actuator rejected")

    # ------------------------------------------------------------------
    print("  [2] JSON round-trip")
    d = preset_quadruped()
    d2 = RobotDesign.from_json(d.to_json())
    assert d2.n_dof == d.n_dof and len(d2.links) == len(d.links)
    assert len(d2.contact_points) == len(d.contact_points)
    assert len(d2.sensor_mounts) == len(d.sensor_mounts)
    assert d2.joint("FL_knee").limits == d.joint("FL_knee").limits
    print(f"      OK — {d.n_dof} DOF, {len(d.links)} links preserved")

    # ------------------------------------------------------------------
    print("  [3] random_design across seeds")
    for seed in range(5):
        rd = random_design(seed=seed)
        assert rd.n_dof >= 4, "2-6 legs × 2-3 joints → ≥4 DOF"
        feet = len(rd.contact_points)
        assert feet == len({c.name for c in rd.contact_points})
    print("      OK — 5 random morphologies valid")

    # ------------------------------------------------------------------
    print("  [4] FK chain consistency")
    d = preset_quadruped()
    robot = SimRobot(d)
    T = robot.forward_kinematics()
    # Hip link sits at the design's hip origin
    assert np.allclose(T["FL_hip_link"].t, [0.18, 0.10, 0.0], atol=1e-9)
    # Knee link coincides with hip link at q=0 (zero-length hip offset)
    assert np.allclose(T["FL_knee_link"].t, T["FL_hip_link"].t, atol=1e-9)
    # Bend the knee → foot link frame rotates; FK stays a valid rotation
    robot.q[:] = 0.0
    robot.q[robot.design.actuated_joints.index(
        robot.design.joint("FL_knee"))] = -0.5
    T = robot.forward_kinematics()
    for name, Tr in T.items():
        assert np.allclose(Tr.R @ Tr.R.T, np.eye(3), atol=1e-9), name
    print("      OK — hip offsets & rotation orthogonality")

    # ------------------------------------------------------------------
    print("  [5] Jacobian matches numeric FK derivative (foot point)")
    robot = SimRobot(preset_quadruped())
    robot.q[:] = np.array([0.1, -0.4, 0.2, -0.5, -0.1, -0.3, 0.3, -0.4])
    cp = next(c for c in robot.design.contact_points if c.name == "foot_FL")
    transforms = robot.forward_kinematics()
    J = robot.link_jacobian(cp.link, transforms, offset=cp.offset)
    eps = 1e-6
    for i in range(robot.n_dof):
        q0 = robot.q.copy()
        robot.q[i] += eps
        Tp = robot.forward_kinematics()[cp.link]
        p_plus = Tp.R @ cp.offset + Tp.t
        robot.q = q0
        robot.q[i] -= eps
        Tm = robot.forward_kinematics()[cp.link]
        p_minus = Tm.R @ cp.offset + Tm.t
        robot.q = q0
        num = (p_plus - p_minus) / (2 * eps)
        assert np.allclose(num, J[:, i], atol=1e-4), \
            f"joint {i}: J={J[:, i]} vs numeric {num}"
    print("      OK — all 8 columns match finite differences")

    # ------------------------------------------------------------------
    print("  [6] IK places foot at reachable target")
    robot = SimRobot(preset_quadruped())
    cp = next(c for c in robot.design.contact_points if c.name == "foot_FL")
    # Build a guaranteed-reachable target: FK at a non-trivial config
    q_star = np.array([0.3, -1.0, -0.2, -0.6, 0.1, -0.4, -0.3, -0.5])
    robot.q = q_star.copy()
    T = robot.forward_kinematics()
    target = T[cp.link].R @ cp.offset + T[cp.link].t
    robot.q = np.zeros(robot.n_dof)          # IK from the neutral pose
    q_start = robot.q.copy()
    sol = robot.solve_ik(cp.link, target, max_iter=200, offset=cp.offset)
    assert sol is not None, "FK-derived target must be reachable"
    # solve_ik must not leave the robot's state modified
    assert np.allclose(robot.q, q_start, atol=1e-12), \
        "solve_ik must restore state"
    robot.q = sol
    T = robot.forward_kinematics()
    foot = T[cp.link].R @ cp.offset + T[cp.link].t
    err = np.linalg.norm(foot - target)
    assert err < 5e-3, f"IK residual {err:.4f} m too large"
    print(f"      OK — IK residual {err * 1000:.2f} mm, state preserved")

    # ------------------------------------------------------------------
    print("  [7] PD stepping converges & respects torque limits")
    robot = SimRobot(preset_quadruped())
    q0 = robot.q.copy()
    q_goal = q0 + np.array([0.2, -0.3, -0.2, 0.3, 0.1, -0.2, -0.1, 0.2])
    for _ in range(300):
        robot.step(0.005, q_goal)
    err = np.abs(robot.q - q_goal)
    assert np.median(err) < 0.05, f"PD must converge, err={err}"
    assert np.all(np.abs(robot.tau) <= robot._tau_lim + 1e-9), \
        "torque must stay within actuator limits"
    print(f"      OK — converged, median |q−q*| = {np.median(err):.4f} rad")

    # ------------------------------------------------------------------
    print("  [8] World interaction: contacts vs 3DGS terrain")
    env = GaussianSplatEnv.create_flat_ground(size=8.0, resolution=4)
    robot = SimRobot(preset_quadruped(),
                     base_position=np.array([0.0, 0.0, 0.0]))
    # Feet dangle below ground at spawn → snap lifts the body
    robot.snap_contacts_to_ground(env)
    active = robot.update_contacts(env)
    assert any(active.values()), "after snap at least one foot must touch"
    for name, world in robot.contact_positions.items():
        ground = env.height_at(world[0], world[1])
        if active[name]:
            assert world[2] - 0.03 <= ground + 1e-6, name
    assert not robot.body_collision(env), "body must not intersect terrain"
    print(f"      OK — {sum(active.values())}/4 feet grounded, no body collision")

    # ------------------------------------------------------------------
    print("  [9] Sensor mounts expose world poses")
    pos, orn = robot.mount_pose("head_cam")
    assert pos.shape == (3,) and orn.shape == (3,)
    # Camera mount is at the body front, pitched forward
    assert pos[0] > robot.base_position[0], "head cam sits at the front"
    ant_pos, _ = robot.mount_pose("antenna")
    assert ant_pos[2] > robot.base_position[2], "antenna sits above body"
    # Plugs into comm Pose
    from comm import Pose
    p = Pose(position=ant_pos)
    assert np.allclose(p.position, ant_pos)
    print("      OK — camera + comm mount poses resolve")

    # ------------------------------------------------------------------
    print("  [10] Gait groups & controller stepping")
    assert gait_groups(["a", "b", "c", "d", "e", "f"]) == [
        ["a", "d"], ["b", "e"], ["c", "f"]]
    assert gait_groups(["a", "b", "c", "d"]) == [["a", "d"], ["b", "c"]]
    env = GaussianSplatEnv.create_flat_ground(size=20.0, resolution=4)
    robot = SimRobot(preset_hexapod(), base_position=np.array([0, 0, 0.30]))
    robot.snap_contacts_to_ground(env)
    gc = GaitController(robot, env, stride=0.05, period=0.6)
    x0 = robot.base_position.copy()
    for _ in range(60):
        gc.step(0.02, heading=0.0, speed=1.0)
    moved = robot.base_position[0] - x0[0]
    assert moved > 0.05, f"robot must walk forward, moved {moved:.3f} m"
    print(f"      OK — hexapod walked {moved:.3f} m in 1.2 s of gait")

    # ------------------------------------------------------------------
    print("  [11] Random morphology also compiles & steps")
    rd = random_design(seed=3)
    rr = SimRobot(rd, base_position=np.array([0, 0, 0.5]))
    rr.snap_contacts_to_ground(env)
    gcr = GaitController(rr, env, stride=0.03, period=0.8)
    for _ in range(30):
        gcr.step(0.02)
    assert np.isfinite(rr.q).all()
    print(f"      OK — random {rd.n_dof}-DOF bot stepped, "
          f"{len(rr.contacts)} feet")

    print("\n  All design-module tests passed.")


if __name__ == "__main__":
    run()
