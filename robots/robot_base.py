"""
Abstract base class for legged robots.

Defines a URDF-like kinematic structure:
  - A base (trunk) with position, orientation, velocity
  - N legs, each with a chain of joints (hip → knee → foot)
  - Each leg has a foot position in world coordinates

The base class provides forward kinematics, inverse kinematics (analytic
for 3-DOF legs), and state management. Concrete robots (biped, quadruped,
hexapod) specify leg configurations.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from typing import Optional

from core.math_utils import Transform, rot_z, euler_to_matrix, normalize


@dataclass
class Joint:
    """A revolute joint."""
    name: str
    axis: np.ndarray          # joint rotation axis in parent frame
    angle: float = 0.0        # current joint angle (rad)
    limits: tuple[float, float] = (-np.pi, np.pi)
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))  # position offset in parent frame


@dataclass
class Leg:
    """A leg: hip → (optional thigh) → knee → foot."""
    name: str
    hip_offset: np.ndarray        # hip position relative to base center
    joints: list[Joint]           # ordered list of joints
    link_lengths: list[float]     # link lengths between joints
    # The last "foot" is at the end of the last link

    @property
    def n_joints(self) -> int:
        return len(self.joints)


@dataclass
class RobotState:
    """Snapshot of robot state."""
    base_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw
    base_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    joint_angles: dict[str, float] = field(default_factory=dict)
    foot_positions: dict[str, np.ndarray] = field(default_factory=dict)
    foot_contacts: dict[str, bool] = field(default_factory=dict)
    t: float = 0.0


class LeggedRobot(ABC):
    """
    Abstract legged robot with forward/inverse kinematics.
    """

    def __init__(self, name: str, mass: float = 10.0,
                 base_position: Optional[np.ndarray] = None):
        self.name = name
        self.mass = mass
        self.state = RobotState(
            base_position=base_position.copy() if base_position is not None else np.zeros(3),
        )
        self.legs: dict[str, Leg] = {}
        self._build_legs()

    # ------------------------------------------------------------------
    # To be implemented by subclasses
    # ------------------------------------------------------------------

    @abstractmethod
    def _build_legs(self):
        """Define legs (hip offsets, joint axes, link lengths). Called in __init__."""
        ...

    @property
    @abstractmethod
    def n_legs(self) -> int:
        ...

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def base_transform(self) -> Transform:
        """World transform of the base."""
        roll, pitch, yaw = self.state.base_orientation
        R = euler_to_matrix(roll, pitch, yaw)
        return Transform(R, self.state.base_position)

    def leg_forward_kinematics(self, leg_name: str) -> np.ndarray:
        """
        Compute the foot position of a leg in world coordinates.
        Uses the current joint angles.
        """
        leg = self.legs[leg_name]
        Tb = self.base_transform()

        # Start at hip position in base frame
        hip_world = Tb.apply(leg.hip_offset.reshape(1, 3))[0]

        # Walk the kinematic chain
        current_pos = hip_world.copy()
        current_R = Tb.R.copy()

        for i, joint in enumerate(leg.joints):
            # Apply joint offset
            offset_world = current_R @ joint.offset
            current_pos = current_pos + offset_world

            # Apply joint rotation (rotating the current frame)
            axis_world = current_R @ joint.axis
            from core.math_utils import rot_axis_angle
            current_R = rot_axis_angle(axis_world, joint.angle) @ current_R

            # Move along link. For a 3-DOF leg (yaw, pitch, knee):
            #   joint 0 (yaw) rotates the hip but the thigh hasn't started yet,
            #   joint 1 (pitch) rotates the thigh → move link_lengths[0],
            #   joint 2 (knee) rotates the shank → move link_lengths[1].
            # So link i-1 is traversed after joint i (for i ≥ 1).
            if i >= 1 and (i - 1) < len(leg.link_lengths):
                link_dir = current_R @ np.array([0, 0, -1])  # links go "down" by convention
                current_pos = current_pos + link_dir * leg.link_lengths[i - 1]

        return current_pos

    def update_foot_positions(self):
        """Recompute all foot positions from FK and store in state."""
        for name in self.legs:
            pos = self.leg_forward_kinematics(name)
            self.state.foot_positions[name] = pos

    # ------------------------------------------------------------------
    # Inverse kinematics (analytic, 3-DOF leg: yaw hip + pitch hip + pitch knee)
    # ------------------------------------------------------------------

    def leg_inverse_kinematics(self, leg_name: str,
                               foot_target_world: np.ndarray) -> Optional[list[float]]:
        """
        Analytic IK for a 3-DOF leg with:
          Joint 0: vertical axis (yaw) — hip rotation
          Joint 1: horizontal axis (pitch) — thigh
          Joint 2: horizontal axis (pitch) — knee

        Returns [j0, j1, j2] or None if unreachable.
        """
        leg = self.legs[leg_name]
        if leg.n_joints != 3:
            return self._numeric_ik(leg_name, foot_target_world)

        Tb = self.base_transform()
        Tb_inv = Tb.inverse()

        # Target in base frame
        target_base = Tb_inv.apply(foot_target_world.reshape(1, 3))[0]
        hip_base = leg.hip_offset

        # Vector from hip to foot in base frame
        d = target_base - hip_base

        L1 = leg.link_lengths[0]  # thigh
        L2 = leg.link_lengths[1]  # shank

        # Hip yaw: angle that aligns the sagittal (XZ) plane with the target.
        # At j0=0 the leg extends in the XZ plane (pitch rotates about Y),
        # so j0 is simply the azimuth of d in the XY plane.
        j0 = np.arctan2(d[1], d[0])

        # After yaw rotation, the leg is in the sagittal plane
        # Rotate d by -j0 about Z to get sagittal-plane coordinates
        Rz = rot_z(-j0)
        d_sag = Rz @ d

        # In the sagittal plane: horizontal distance r and vertical drop h.
        # r is signed (positive = forward in the leg's sagittal X direction)
        # because the FK convention has positive j1 swinging the foot in -X,
        # so the IK uses -r in arctan2 to invert that.
        r = d_sag[0]
        h = -d_sag[2]  # positive when foot is below hip

        # Two-link IK
        dist_sq = r ** 2 + h ** 2
        dist = np.sqrt(dist_sq)

        if dist > (L1 + L2) * 0.99 or dist < abs(L1 - L2) * 1.01:
            return None  # unreachable

        cos_knee = (dist_sq - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
        cos_knee = np.clip(cos_knee, -1, 1)
        j2 = np.arccos(cos_knee)  # knee angle (always positive for this config)

        j1 = np.arctan2(-r, h) - np.arctan2(L2 * np.sin(j2), L1 + L2 * np.cos(j2))

        # Apply joint limits
        angles = [j0, j1, j2]
        for i, (a, joint) in enumerate(zip(angles, leg.joints)):
            lo, hi = joint.limits
            angles[i] = np.clip(a, lo, hi)

        return angles

    def _numeric_ik(self, leg_name: str, foot_target: np.ndarray,
                    max_iter: int = 50, tol: float = 1e-4) -> Optional[list[float]]:
        """Fallback numeric IK using Jacobian transpose."""
        leg = self.legs[leg_name]
        # Save current angles
        saved = [j.angle for j in leg.joints]

        for _ in range(max_iter):
            foot = self.leg_forward_kinematics(leg_name)
            error = foot_target - foot
            if np.linalg.norm(error) < tol:
                return [j.angle for j in leg.joints]

            # Numerical Jacobian
            J = np.zeros((3, leg.n_joints))
            dt = 0.001
            for k in range(leg.n_joints):
                leg.joints[k].angle += dt
                foot_perturbed = self.leg_forward_kinematics(leg_name)
                J[:, k] = (foot_perturbed - foot) / dt
                leg.joints[k].angle -= dt

            # Jacobian transpose step
            step = J.T @ error
            lr = 0.3
            for k in range(leg.n_joints):
                leg.joints[k].angle += lr * step[k]
                lo, hi = leg.joints[k].limits
                leg.joints[k].angle = np.clip(leg.joints[k].angle, lo, hi)

        # Check final error
        foot = self.leg_forward_kinematics(leg_name)
        if np.linalg.norm(foot_target - foot) > 0.01:
            # Restore
            for j, a in zip(leg.joints, saved):
                j.angle = a
            return None
        return [j.angle for j in leg.joints]

    def set_joint_angles(self, leg_name: str, angles: list[float]):
        """Set joint angles for a leg and update foot positions."""
        leg = self.legs[leg_name]
        for j, a in zip(leg.joints, angles):
            j.angle = a
        self.state.joint_angles[leg_name] = angles

    def get_joint_angles(self, leg_name: str) -> list[float]:
        return [j.angle for j in self.legs[leg_name].joints]

    # ------------------------------------------------------------------
    # Dynamics (simplified)
    # ------------------------------------------------------------------

    def apply_velocity(self, dt: float):
        """Integrate base velocity."""
        self.state.base_position += self.state.base_velocity * dt
        self.state.base_orientation += self.state.base_angular_velocity * dt
        self.state.t += dt
        self.update_foot_positions()

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def support_polygon(self) -> np.ndarray:
        """
        Return the convex hull of currently-contacting feet (XY, ordered
        counter-clockwise). Uses Andrew's monotone-chain algorithm.
        """
        contact_pts = []
        for name, contact in self.state.foot_contacts.items():
            if contact and name in self.state.foot_positions:
                contact_pts.append(self.state.foot_positions[name][:2])
        if len(contact_pts) < 3:
            return np.array(contact_pts) if contact_pts else np.zeros((0, 2))

        pts = sorted(contact_pts, key=lambda p: (p[0], p[1]))

        def cross_o(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        # Lower hull
        lower = []
        for p in pts:
            while len(lower) >= 2 and cross_o(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        # Upper hull
        upper = []
        for p in reversed(pts):
            while len(upper) >= 2 and cross_o(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        # Concatenate (omit last point of each half — it's the first of the other)
        hull = lower[:-1] + upper[:-1]
        return np.array(hull)

    def is_stable(self) -> bool:
        """Check if the center of mass projection is inside the support polygon."""
        pts = self.support_polygon()
        if len(pts) < 3:
            return False
        com_xy = self.state.base_position[:2]
        # Simple check: COM inside convex hull (using cross products)
        n = len(pts)
        inside = True
        for i in range(n):
            p1 = pts[i]
            p2 = pts[(i + 1) % n]
            edge = p2 - p1
            to_com = com_xy - p1
            if np.cross(edge, to_com) < 0:
                inside = False
                break
        return inside

    def __repr__(self):
        return (f"{self.__class__.__name__}(name={self.name!r}, "
                f"n_legs={self.n_legs}, pos={self.state.base_position})")
