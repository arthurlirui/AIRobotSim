"""
Compile a RobotDesign into a SimRobot: a generic tree-kinematics robot
that steps with actuator dynamics and interacts with the 3DGS world.

The compiler flattens the design's joint tree into arrays (parent index,
origin transforms, axes) so FK runs vectorized over the whole chain.
Simulation features:

  - ``forward_kinematics()``  — world transforms of every link
  - ``link_jacobian()``       — 3×N geometric Jacobian of any link frame
  - ``solve_ik()``            — damped-least-squares IK to a target pose
  - ``step(dt, q_desired)``   — PD actuator torques → joint states
  - ``update_contacts(env)``  — terrain contact / collision against the
    GaussianSplatEnv (the "interact with the 3D world" half of the goal)
  - ``mount_pose()``          — world pose of any sensor mount (plugs
    directly into ``sensors/`` cameras and ``comm`` Pose)
"""

from __future__ import annotations

import numpy as np
from typing import Optional, TYPE_CHECKING

from core.math_utils import Transform, euler_to_matrix, rot_axis_angle
from design.config import RobotDesign
from design.components import Actuator, ContactPoint, SensorMount

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


# ---------------------------------------------------------------------------
# Compilation
# ---------------------------------------------------------------------------

class SimRobot:
    """A compiled, steppable robot built from a :class:`RobotDesign`."""

    def __init__(self, design: RobotDesign,
                 base_position: Optional[np.ndarray] = None,
                 base_orientation: Optional[np.ndarray] = None):
        self.design = design.validate()
        self.name = design.name

        # --- Base state (floating root) ---
        self.base_position = base_position.copy() if base_position is not None \
            else np.zeros(3)
        self.base_orientation = base_orientation.copy() \
            if base_orientation is not None else np.zeros(3)  # ZYX Euler

        # --- Flatten the joint tree ---
        self.actuated = design.actuated_joints      # ordered revolute joints
        self.n_dof = len(self.actuated)
        self.q = np.zeros(self.n_dof)               # joint angles (rad)
        self.dq = np.zeros(self.n_dof)              # joint velocities
        self.tau = np.zeros(self.n_dof)             # last applied torques

        # All links indexed; joints stored per link parent
        self._links = {l.name: l for l in design.links}
        self._link_index = {l.name: i for i, l in enumerate(design.links)}
        # joint connecting a link to its parent (None for root)
        self._parent_joint = {j.child: j for j in design.joints}

        # Actuator lookup by joint name
        self._actuators: dict[str, Actuator] = \
            {a.joint: a for a in design.actuators}

        # Contact / mounts
        self.contacts: list[ContactPoint] = list(design.contact_points)
        self.contact_positions: dict[str, np.ndarray] = {}
        self.contact_active: dict[str, bool] = {}
        self.sensor_mounts: dict[str, SensorMount] = \
            {m.name: m for m in design.sensor_mounts}

        # Joint impedance defaults (per-actuator overrides via design)
        self._kp = np.array([self._actuators[j.name].kp for j in self.actuated]) \
            if self.n_dof else np.zeros(0)
        self._kv = np.array([self._actuators[j.name].kv for j in self.actuated]) \
            if self.n_dof else np.zeros(0)
        self._tau_lim = np.array([self._actuators[j.name].torque_limit
                                  for j in self.actuated]) \
            if self.n_dof else np.zeros(0)

        self.t = 0.0

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def base_transform(self) -> Transform:
        return Transform(euler_to_matrix(*self.base_orientation),
                         self.base_position)

    def forward_kinematics(self) -> dict[str, Transform]:
        """
        World transform of every link (recursive over the design tree).
        Link frames follow the URDF convention: a child link's frame is
        its parent joint's ``origin`` pose, rotated by the joint angle
        about ``axis`` in the child frame.
        """
        transforms: dict[str, Transform] = {}
        root = next(l.name for l in self.design.links if l.is_root)
        transforms[root] = self.base_transform()

        pending = [root]
        while pending:
            parent = pending.pop()
            for j in self.design.joints:
                if j.parent != parent:
                    continue
                origin = Transform(euler_to_matrix(*j.origin_euler), j.origin)
                if j.type == "revolute" and j.name in \
                        {aj.name for aj in self.actuated}:
                    idx = next(i for i, aj in enumerate(self.actuated)
                               if aj.name == j.name)
                    # URDF revolute semantics: rotate the child frame about
                    # the joint axis (in the child frame) *after* the origin
                    # transform — the whole subtree moves with the rotation.
                    R_joint = Transform(
                        rot_axis_angle(j.axis_vector(), self.q[idx]),
                        np.zeros(3))
                    child_T = transforms[parent] @ origin @ R_joint
                else:
                    child_T = transforms[parent] @ origin
                transforms[j.child] = child_T
                pending.append(j.child)
        return transforms

    def link_jacobian(self, link: str,
                      transforms: Optional[dict] = None,
                      offset: Optional[np.ndarray] = None) -> np.ndarray:
        """
        3×N geometric Jacobian of a point rigidly attached to ``link``
        (its frame origin, or ``offset`` in the link frame — e.g. a foot
        contact point), w.r.t. the actuated joints, in world frame.
        Columns are zero for joints not on the path root → link.
        """
        if transforms is None:
            transforms = self.forward_kinematics()
        target_T = transforms[link]
        if offset is not None:
            point = target_T.R @ np.asarray(offset, dtype=float) + target_T.t
        else:
            point = target_T.t

        # Collect the chain root → link (joint names)
        chain: list[str] = []
        cur = link
        while cur in self._parent_joint:
            j = self._parent_joint[cur]
            chain.append(j.name)
            cur = j.parent
        chain_set = set(chain)

        J = np.zeros((3, self.n_dof))
        for i, aj in enumerate(self.actuated):
            if aj.name not in chain_set:
                continue
            j = next(jj for jj in self.design.joints if jj.name == aj.name)
            parent_T = transforms[j.parent]
            # Joint axis & pivot: parent frame ∘ origin transform (the
            # rotation happens about the axis through the origin point)
            origin_R = euler_to_matrix(*j.origin_euler)
            axis_world = parent_T.R @ origin_R @ j.axis_vector()
            pivot = parent_T.R @ j.origin + parent_T.t
            J[:, i] = np.cross(axis_world, point - pivot)
        return J

    def solve_ik(self, link: str, target_world: np.ndarray,
                 max_iter: int = 60, tol: float = 1e-3,
                 damping: float = 0.05,
                 offset: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Damped-least-squares IK: joint angles placing a point rigidly
        attached to ``link`` (frame origin, or ``offset`` in the link
        frame — e.g. a foot contact point) at ``target_world``.

        Returns the solution as a new array (the robot's state is left
        untouched), or None if not converged within ``max_iter``.
        """
        q_start = self.q.copy()
        for _ in range(max_iter):
            transforms = self.forward_kinematics()
            T = transforms[link]
            if offset is not None:
                point = T.R @ np.asarray(offset, dtype=float) + T.t
            else:
                point = T.t
            err = target_world - point
            if np.linalg.norm(err) < tol:
                solution = self.q.copy()
                self.q = q_start
                return solution
            J = self.link_jacobian(link, transforms, offset=offset)
            # Pseudo-inverse with damping (robust near singularities)
            JT = J.T
            dq_step = JT @ np.linalg.solve(
                J @ JT + damping ** 2 * np.eye(3), err)
            self.q = self.q + dq_step
            # Respect joint limits
            for i, aj in enumerate(self.actuated):
                lo, hi = next(jj.limits for jj in self.design.joints
                              if jj.name == aj.name)
                self.q[i] = np.clip(self.q[i], lo, hi)
        # Final convergence check
        transforms = self.forward_kinematics()
        T = transforms[link]
        point = T.R @ np.asarray(offset, dtype=float) + T.t \
            if offset is not None else T.t
        converged = np.linalg.norm(target_world - point) < 10 * tol
        solution = self.q.copy() if converged else None
        self.q = q_start
        return solution

    # ------------------------------------------------------------------
    # Actuation & stepping
    # ------------------------------------------------------------------

    def step(self, dt: float, q_desired: Optional[np.ndarray] = None):
        """
        Advance one control step: PD torque → joint velocity → angle.
        ``q_desired`` is the commanded position per actuated joint.
        """
        if self.n_dof == 0:
            self.t += dt
            return
        if q_desired is None:
            q_desired = self.q
        q_desired = np.asarray(q_desired, dtype=float)

        self.tau = self._kp * (q_desired - self.q) - self._kv * self.dq
        self.tau = np.clip(self.tau, -self._tau_lim, self._tau_lim)

        # Actuator integration: first-order position tracking.  The
        # commanded torque drives a velocity whose magnitude is capped by
        # the torque/inertia ratio AND the joint velocity limit; the
        # velocity then decays exponentially (viscous damping kv),
        # giving critically-damped convergence without the dt-induced
        # limit cycle a naive explicit integration produced.
        inertia = np.array([
            max(self._links[
                    next(jj.child for jj in self.design.joints
                         if jj.name == aj.name)].mass, 0.1) * 0.01
            for aj in self.actuated]) \
            if self.n_dof else np.zeros(0)
        # Steady-state velocity the torque sustains against kv damping
        v_ss = self.tau / self._kv
        v_cap = np.minimum(self._tau_lim / self._kv,
                           np.array([j.velocity_limit
                                     for j in self.actuated]))
        v_target = np.clip(v_ss, -v_cap, v_cap)
        # First-order velocity relaxation (time constant kv/inertia is
        # fast; use explicit fraction-of-gap update for stability)
        alpha = 1.0 - np.exp(-(self._kv / inertia) * dt)
        self.dq = self.dq + alpha * (v_target - self.dq)
        self.dq = np.clip(self.dq,
                          -np.array([j.velocity_limit for j in self.actuated]),
                          np.array([j.velocity_limit for j in self.actuated]))
        self.q = self.q + self.dq * dt
        # Joint limits (hard stop: zero velocity at the limit)
        for i, aj in enumerate(self.actuated):
            lo, hi = next(jj.limits for jj in self.design.joints
                          if jj.name == aj.name)
            if self.q[i] <= lo or self.q[i] >= hi:
                self.q[i] = np.clip(self.q[i], lo, hi)
                self.dq[i] = 0.0
        self.t += dt

    # ------------------------------------------------------------------
    # World interaction (GaussianSplatEnv)
    # ------------------------------------------------------------------

    def update_contacts(self, env: "GaussianSplatEnv") -> dict[str, bool]:
        """
        Evaluate every contact point against the 3DGS terrain:
        a contact is *active* when the point is at/below the terrain
        height. Also flags body collision. Updates ``contact_positions``
        and ``contact_active`` and returns the active map.
        """
        transforms = self.forward_kinematics()
        for cp in self.contacts:
            T = transforms[cp.link]
            world = T.R @ cp.offset + T.t
            self.contact_positions[cp.name] = world
            ground = env.height_at(world[0], world[1])
            self.contact_active[cp.name] = bool(world[2] - cp.radius <= ground)
        return dict(self.contact_active)

    def body_collision(self, env: "GaussianSplatEnv") -> bool:
        """True if any link's bounding sphere intersects a splat."""
        transforms = self.forward_kinematics()
        for l in self.design.links:
            T = transforms[l.name]
            if len(env.collision(T.t)) > 0:
                return True
        return False

    def snap_contacts_to_ground(self, env: "GaussianSplatEnv"):
        """
        Move the floating base vertically so the lowest contact point
        rests on the terrain (simple ground-attachment for standing).
        """
        self.update_contacts(env)
        if not self.contact_positions:
            return
        lowest = min(p[2] for p in self.contact_positions.values())
        ground = env.height_at(*self.base_position[:2])
        self.base_position[2] += ground - lowest

    # ------------------------------------------------------------------
    # Sensor mounts → platform subsystems
    # ------------------------------------------------------------------

    def mount_pose(self, mount_name: str) -> tuple[np.ndarray, np.ndarray]:
        """
        World (position, orientation-euler) of a sensor mount — feed
        directly to ``sensors`` cameras' ``render(env, pos, orn)`` or to
        ``comm.channel.base.Pose``.
        """
        m = self.sensor_mounts[mount_name]
        transforms = self.forward_kinematics()
        T = transforms[m.link]
        pos = T.R @ m.offset + T.t
        from core.math_utils import matrix_to_euler
        orn = np.array(matrix_to_euler(T.R @ euler_to_matrix(*m.euler)))
        return pos, orn

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    @property
    def mass(self) -> float:
        return sum(l.mass for l in self.design.links)

    def __repr__(self):
        return (f"SimRobot(name={self.name!r}, n_dof={self.n_dof}, "
                f"contacts={len(self.contacts)}, "
                f"mounts={len(self.sensor_mounts)})")
