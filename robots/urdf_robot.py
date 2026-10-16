"""
Robot factory: build LeggedRobot instances from parsed URDF specifications.

Bridges the gap between the abstract ``LeggedRobot`` base class and real
URDF files downloaded from Unitree Robotics.  Given a parsed ``RobotSpec``,
this module constructs a concrete robot with correct joint axes, limits,
link lengths, and hip offsets — no hand-tuned constants required.

Supported robots (by name):
  - "go2"  — Unitree Go2 quadruped (from go2_description.urdf)
  - "go1"  — Unitree Go1 quadruped (from go1.urdf)
  - "a1"   — Unitree A1 quadruped (from a1.urdf)
  - "h1"   — Unitree H1 humanoid (from h1.urdf)
  - "b1"   — Unitree B1 quadruped (from b1.urdf)
  - "laikago" — Unitree Laikago quadruped (from laikago.urdf)

The factory can also build a generic robot from any URDF that follows the
standard legged-robot naming convention.
"""

from __future__ import annotations

import numpy as np
from pathlib import Path
from typing import Optional

from core.urdf_parser import parse_urdf, RobotSpec
from robots.robot_base import LeggedRobot, Leg, Joint


# Default URDF search path (relative to project root)
_URDF_BASE = Path(__file__).resolve().parent.parent / "assets" / "external" / "unitree_ros" / "robots"

# Map short name → (subdir, filename, base_link)
URDF_REGISTRY = {
    "go2":    ("go2_description",  "urdf/go2_description.urdf", "base"),
    "go1":    ("go1_description",  "urdf/go1.urdf",             "base"),
    "a1":     ("a1_description",   "urdf/a1.urdf",              "trunk"),
    "b1":     ("b1_description",   "xacro/b1.urdf",             "trunk"),
    "h1":     ("h1_description",   "urdf/h1.urdf",              "pelvis"),
    "laikago":("laikago_description", "urdf/laikago.urdf",      "trunk"),
}

# Also check legged_gym resources as fallback
_LEGGED_GYM_BASE = Path(__file__).resolve().parent.parent / "assets" / "external" / "legged_gym" / "resources" / "robots"
LEGGED_GYM_REGISTRY = {
    "a1":      ("a1",       "urdf/a1.urdf",       "trunk"),
    "anymal_b":("anymal_b", "urdf/anymal_b.urdf", "base"),
    "anymal_c":("anymal_c", "urdf/anymal_c.urdf", "base"),
    "cassie":  ("cassie",   "urdf/cassie.urdf",   "pelvis"),
}


def _resolve_urdf_path(name: str) -> Optional[Path]:
    """Find the URDF file for a robot by short name."""
    if name in URDF_REGISTRY:
        subdir, filename, _ = URDF_REGISTRY[name]
        p = _URDF_BASE / subdir / filename
        if p.exists():
            return p
    if name in LEGGED_GYM_REGISTRY:
        subdir, filename, _ = LEGGED_GYM_REGISTRY[name]
        p = _LEGGED_GYM_BASE / subdir / filename
        if p.exists():
            return p
    return None


def _normalise_axis(axis: np.ndarray) -> np.ndarray:
    """Ensure axis is a unit vector."""
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0])
    return axis / n


class URDFRobot(LeggedRobot):
    """
    A legged robot constructed entirely from URDF kinematic data.

    Unlike the hand-coded ``Quadruped``, ``Biped``, ``Hexapod`` classes which
    use approximate spec values, this class reads the actual joint axes,
    limits, link lengths, and hip offsets from the parsed URDF.
    """

    def __init__(self, spec: RobotSpec, name: Optional[str] = None,
                 base_position: Optional[np.ndarray] = None):
        self._spec = spec
        # Estimate base height from the longest leg's actual link lengths.
        # We compute reach the same way as _build_legs does.
        max_leg_reach = 0.0
        for prefix in spec.leg_joints:
            joints = spec.leg_joints[prefix]
            rev = [j for j in joints if j.joint_type in ("revolute", "continuous")]
            reach = 0.0
            for i in range(len(rev)):
                if i + 1 < len(rev):
                    vertical = abs(rev[i + 1].origin_xyz[2])
                else:
                    # Estimate foot link length (same logic as _build_legs)
                    foot_len = 0.0
                    child_link = rev[i].child
                    for jname, j in spec.joints.items():
                        if (j.parent == child_link and
                            j.joint_type == "fixed" and
                            abs(j.origin_xyz[2]) > 0.01):
                            foot_len = abs(j.origin_xyz[2])
                            break
                    if foot_len < 0.01 and i > 0:
                        foot_len = abs(rev[i].origin_xyz[2])
                    vertical = foot_len
                if vertical > 0.05:
                    reach += vertical
            max_leg_reach = max(max_leg_reach, reach)

        base_height = max_leg_reach * 0.7  # standing at ~70% of full extension
        if base_position is None:
            base_position = np.array([0.0, 0.0, base_height])

        robot_name = name or spec.name
        super().__init__(robot_name, mass=spec.total_mass,
                         base_position=base_position)

    @property
    def n_legs(self) -> int:
        return len(self._spec.leg_joints)

    def _estimate_foot_length(self, last_joint) -> float:
        """
        Estimate the link length from the last revolute joint to the foot.

        Checks the URDF for fixed joints descending from the last joint's
        child link. If a foot link has a collision geometry with a known
        extent, uses that. Otherwise, falls back to the previous link's
        length (common for symmetric legs like Go2 where thigh = shank).
        """
        # Look through all URDF joints for fixed joints whose parent is
        # the last joint's child link and whose origin has a Z offset
        child_link = last_joint.child
        for jname, j in self._spec.joints.items():
            if (j.parent == child_link and
                j.joint_type == "fixed" and
                abs(j.origin_xyz[2]) > 0.01):
                return abs(j.origin_xyz[2])

        # Check collision geometry of the child link
        if child_link in self._spec.links:
            link = self._spec.links[child_link]
            if link.collision_type == "box" and link.collision_size[2] > 0.01:
                # Box height is a rough proxy for link length
                return link.collision_size[2] * 2  # full height

        # Fallback: use the same length as the previous link
        # (common for symmetric leg designs)
        # Find this joint's index in its leg and get the previous link
        for prefix, joints in self._spec.leg_joints.items():
            rev = [j for j in joints if j.joint_type in ("revolute", "continuous")]
            for i, j in enumerate(rev):
                if j.name == last_joint.name and i > 0:
                    prev_origin = j.origin_xyz  # offset from prev joint
                    prev_len = abs(prev_origin[2])
                    if prev_len > 0.01:
                        return prev_len

        return 0.0  # unknown

    def _build_legs(self):
        """Build legs from the parsed URDF specification.

        Normalises the joint axes so they are compatible with the
        ``LeggedRobot`` FK/IK convention:
          Joint 0: Z-axis (yaw / hip abduction)
          Joint 1: Y-axis (pitch / thigh)
          Joint 2: Y-axis (pitch / knee)

        For URDFs that use X-axis hip joints (e.g. Go2), the axis is
        remapped to Z. For legs with more than 3 DOF (e.g. H1 with 5),
        the extra joints (yaw, roll, ankle) are kept but the pitch/knee
        pair is identified for IK.
        """
        for prefix, joints in self._spec.leg_joints.items():
            # Only take revolute joints (skip fixed)
            rev_joints = [j for j in joints
                          if j.joint_type in ("revolute", "continuous")]
            if not rev_joints:
                continue

            hip_offset = self._spec.get_hip_offset(prefix).copy()
            # Add the lateral offset from the second joint (hip→thigh)
            # to the hip offset, since we zero out joint offsets.
            if len(rev_joints) > 1:
                lateral = rev_joints[1].origin_xyz[1]
                hip_offset[1] += lateral

            # Extract link lengths: for each pair of consecutive joints,
            # keep only the vertical (Z) offset as the link length.
            # For multi-DOF legs (e.g. H1 with 5 joints), the first few
            # joints may have small lateral/forward offsets (hip geometry)
            # that should NOT be counted as link lengths.
            # We produce one link length per joint (for FK compatibility),
            # using 0 for non-link joints.
            all_link_lengths = []
            for i in range(len(rev_joints)):
                if i + 1 < len(rev_joints):
                    next_origin = rev_joints[i + 1].origin_xyz
                    vertical = abs(next_origin[2])
                else:
                    # Last joint: estimate the foot link length.
                    # Try to find a fixed joint (foot) below this joint
                    # in the URDF, or default to the same length as the
                    # previous link.
                    foot_length = self._estimate_foot_length(rev_joints[i])
                    vertical = foot_length

                if vertical > 1e-6:
                    all_link_lengths.append(float(vertical))
                else:
                    all_link_lengths.append(0.0)

            # Build Joint objects, normalising axes to the FK convention
            urdf_joints = []
            for i, uj in enumerate(rev_joints):
                axis = _normalise_axis(uj.axis)
                # Remap X-axis hip joints to Z-axis (yaw) for FK compatibility
                if i == 0 and abs(axis[0]) > 0.9:
                    axis = np.array([0.0, 0.0, 1.0])
                # Remap X-axis roll joints (hip_roll in H1) to Z-axis
                if i == 1 and abs(axis[0]) > 0.9 and len(rev_joints) > 3:
                    axis = np.array([0.0, 0.0, 1.0])

                # Joint offset: zeroed out — hip_offset handles position,
                # link_lengths handle vertical links.
                offset = np.zeros(3)

                # Normalise joint limits: some URDFs (e.g. Go2) use negative
                # ranges for knee joints (-156° to -48°). The FK convention
                # expects positive knee angles, so shift the range.
                lower, upper = uj.lower, uj.upper
                if i >= 2 and lower < 0 and upper < 0:
                    # Knee with negative range → flip to positive
                    lower, upper = -upper, -lower

                urdf_joints.append(Joint(
                    name=uj.name,
                    axis=axis,
                    limits=(lower, upper),
                    offset=offset,
                ))

            # Now assign link_lengths for FK compatibility.
            # The FK in robot_base.py traverses link_lengths[i-1] after
            # joint i (for i >= 1). So the array should be:
            #   [thigh_length, shank_length, 0, ...]
            # all_link_lengths[i] = vertical offset from joint i to joint i+1
            # FK wants: link_lengths[joint_idx - 1] = link traversed after
            # that joint. So link_lengths[0] = all_link_lengths[1], etc.
            link_lengths = [0.0] * len(urdf_joints)
            for i in range(1, len(urdf_joints)):
                if i < len(all_link_lengths):
                    link_lengths[i - 1] = all_link_lengths[i]

            # Ensure we have enough link lengths (one per joint)
            while len(link_lengths) < len(urdf_joints):
                link_lengths.append(0.0)

            leg = Leg(
                name=f"leg_{prefix}",
                hip_offset=hip_offset,
                joints=urdf_joints,
                link_lengths=link_lengths[:len(urdf_joints)],
            )
            self.legs[leg.name] = leg

        self._set_standing_pose()

    def _set_standing_pose(self):
        """Place all feet on the ground beneath their hips.

        For 3-DOF legs with a Z-axis yaw first joint, use the analytic IK.
        For other configurations (e.g. Go2 with X-axis abduction first joint,
        or H1 with 5-DOF legs), use a numeric approach that locks non-pitch
        joints at zero and solves the pitch chain.
        """
        for name, leg in self.legs.items():
            foot_target = (self.state.base_position + leg.hip_offset).copy()
            foot_target[2] = 0.0  # ground level

            if len(leg.joints) == 3 and abs(leg.joints[0].axis[2]) > 0.9:
                # Standard 3-DOF: Z-yaw + Y-pitch + Y-pitch → analytic IK
                angles = self.leg_inverse_kinematics(name, foot_target)
                if angles:
                    self.set_joint_angles(name, angles)
                    continue

            # Numeric / heuristic approach for non-standard legs
            # Strategy: set all non-pitch joints to 0, then solve the
            # pitch-plane (Y-axis joints) as a 2-link IK
            angles = self._heuristic_standing_ik(name, foot_target)
            if angles:
                self.set_joint_angles(name, angles)
            else:
                # Last resort: numeric IK
                angles = self._numeric_ik(name, foot_target)
                if angles:
                    self.set_joint_angles(name, angles)

        self.update_foot_positions()
        for name in self.legs:
            self.state.foot_contacts[name] = True

    def _heuristic_standing_ik(self, leg_name: str,
                               foot_target: np.ndarray) -> Optional[list[float]]:
        """
        Heuristic IK for legs that don't match the standard 3-DOF convention.

        Locks all non-Y-axis joints at zero and solves a 2-link planar IK
        for the Y-axis (pitch) joints (thigh + shank).
        """
        leg = self.legs[leg_name]
        angles = [0.0] * len(leg.joints)

        # Find the Y-axis pitch joints (typically thigh and knee)
        pitch_indices = [i for i, j in enumerate(leg.joints)
                         if abs(j.axis[1]) > 0.9]

        if len(pitch_indices) >= 2:
            # Use the first two pitch joints as thigh and knee
            thigh_idx = pitch_indices[0]
            knee_idx = pitch_indices[1]

            # Estimate link lengths for these two joints
            # Link length for thigh = link_lengths[thigh_idx - 1] if available
            # (since link_lengths[i-1] is traversed after joint i in the FK)
            if thigh_idx - 1 < len(leg.link_lengths) and thigh_idx > 0:
                L1 = leg.link_lengths[thigh_idx - 1]
            elif thigh_idx < len(leg.link_lengths):
                L1 = leg.link_lengths[thigh_idx]
            else:
                L1 = 0.2

            if knee_idx - 1 < len(leg.link_lengths):
                L2 = leg.link_lengths[knee_idx - 1]
            elif knee_idx < len(leg.link_lengths):
                L2 = leg.link_lengths[knee_idx]
            else:
                L2 = 0.2

            # Target in the sagittal plane (relative to hip)
            Tb = self.base_transform()
            Tb_inv = Tb.inverse()
            target_base = Tb_inv.apply(foot_target.reshape(1, 3))[0]
            hip_base = leg.hip_offset
            d = target_base - hip_base

            r = d[0]       # forward distance
            h = -d[2]      # downward distance (positive = below hip)

            dist_sq = r ** 2 + h ** 2
            dist = np.sqrt(dist_sq)

            if dist > (L1 + L2) * 0.99 or dist < abs(L1 - L2) * 1.01:
                return None

            cos_knee = (dist_sq - L1 ** 2 - L2 ** 2) / (2 * L1 * L2)
            cos_knee = np.clip(cos_knee, -1, 1)
            knee_angle = np.arccos(cos_knee)

            thigh_angle = np.arctan2(-r, h) - np.arctan2(
                L2 * np.sin(knee_angle), L1 + L2 * np.cos(knee_angle)
            )

            # Apply joint limits
            lo_t, hi_t = leg.joints[thigh_idx].limits
            lo_k, hi_k = leg.joints[knee_idx].limits
            angles[thigh_idx] = np.clip(thigh_angle, lo_t, hi_t)
            angles[knee_idx] = np.clip(knee_angle, lo_k, hi_k)

            # For 5-DOF legs with an ankle joint, set it to compensate
            # for the knee angle so the foot is flat
            if len(pitch_indices) >= 3:
                ankle_idx = pitch_indices[2]
                lo_a, hi_a = leg.joints[ankle_idx].limits
                # Ankle should approximately cancel the thigh+knee rotation
                # so the foot is flat on the ground
                ankle_angle = -(angles[thigh_idx] + angles[knee_idx])
                angles[ankle_idx] = np.clip(ankle_angle, lo_a, hi_a)

        return angles


def create_robot_from_urdf(name: str,
                           base_position: Optional[np.ndarray] = None,
                           ) -> URDFRobot:
    """
    Create a robot by short name from the URDF registry.

    Args:
        name: short name like "go2", "go1", "a1", "h1", "b1", "laikago".
        base_position: optional initial position [x, y, z].

    Returns:
        A ``URDFRobot`` instance with real kinematic parameters.

    Raises:
        FileNotFoundError: if the URDF file cannot be found.
    """
    urdf_path = _resolve_urdf_path(name)
    if urdf_path is None:
        available = list(URDF_REGISTRY.keys()) + list(LEGGED_GYM_REGISTRY.keys())
        raise FileNotFoundError(
            f"URDF for robot '{name}' not found. Available: {available}"
        )

    # Determine base link
    base_link = "base"
    if name in URDF_REGISTRY:
        base_link = URDF_REGISTRY[name][2]
    elif name in LEGGED_GYM_REGISTRY:
        base_link = LEGGED_GYM_REGISTRY[name][2]

    spec = parse_urdf(urdf_path, base_link=base_link)
    return URDFRobot(spec, name=name.upper(), base_position=base_position)


def create_robot_from_urdf_path(urdf_path: str | Path,
                                base_link: Optional[str] = None,
                                base_position: Optional[np.ndarray] = None,
                                ) -> URDFRobot:
    """
    Create a robot from an explicit URDF file path.

    Args:
        urdf_path: path to the .urdf file.
        base_link: name of the base/trunk link (auto-detected if None).
        base_position: optional initial position.
    """
    spec = parse_urdf(urdf_path, base_link=base_link)
    return URDFRobot(spec, base_position=base_position)


def list_available_robots() -> list[str]:
    """Return all robot names that have downloadable URDF files."""
    available = []
    for name in list(URDF_REGISTRY.keys()) + list(LEGGED_GYM_REGISTRY.keys()):
        if _resolve_urdf_path(name) is not None:
            if name not in available:
                available.append(name)
    return available
