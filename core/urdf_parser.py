"""
URDF parser for extracting robot kinematic structure.

Parses standard URDF (Unified Robot Description Format) files and extracts
the kinematic chain (links, joints, origins, axes, limits) needed to build
``LeggedRobot`` instances with real-world specifications.

The parser is dependency-free (uses only the Python standard library
``xml.etree.ElementTree``) so it works without ROS or any external packages.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path


@dataclass
class URDFJoint:
    """A parsed URDF joint."""
    name: str
    joint_type: str          # 'revolute', 'fixed', 'continuous', etc.
    parent: str
    child: str
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    lower: float = -np.pi
    upper: float = np.pi
    effort: float = 0.0
    velocity: float = 0.0


@dataclass
class URDFLink:
    """A parsed URDF link."""
    name: str
    mass: float = 0.0
    inertial_origin: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Collision geometry (simplified)
    collision_type: str = ""          # 'box', 'cylinder', 'sphere', 'mesh'
    collision_size: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class RobotSpec:
    """
    Parsed robot specification extracted from a URDF file.

    Contains everything needed to build a ``LeggedRobot`` with real
    kinematic parameters: base mass, leg joints with limits, link lengths,
    hip offsets, and per-link masses.
    """
    name: str
    links: dict[str, URDFLink] = field(default_factory=dict)
    joints: dict[str, URDFJoint] = field(default_factory=dict)

    # Convenience views populated by ``analyse()``
    base_link: str = ""
    leg_joints: dict[str, list[URDFJoint]] = field(default_factory=dict)
    total_mass: float = 0.0

    def analyse(self, base_link_name: Optional[str] = None):
        """
        Identify the base link and group joints into legs.

        A "leg" is defined as a chain of revolute joints descending from the
        base (or a fixed-joint child of the base).  Legs are named by the
        prefix of the first joint (e.g. 'FL', 'FR', 'left', 'right').
        """
        # Find base link
        if base_link_name:
            self.base_link = base_link_name
        else:
            # Heuristic: the base is the link that is a parent of the most joints
            child_links = {j.child for j in self.joints.values()}
            parent_links = {j.parent for j in self.joints.values()}
            root_candidates = parent_links - child_links
            # Pick the one with the most children
            best = max(root_candidates, key=lambda l: sum(
                1 for j in self.joints.values() if j.parent == l
            ), default=next(iter(root_candidates), ""))
            self.base_link = best

        # Group revolute joints into legs
        self.leg_joints.clear()
        for jname, joint in self.joints.items():
            if joint.joint_type not in ("revolute", "continuous"):
                continue
            # Exclude non-leg chains (arms, torso, head, etc.)
            if not self._is_leg_joint(jname):
                continue
            # Walk up to find if this joint's chain reaches the base
            chain = self._chain_to_base(joint)
            if chain is None:
                continue
            # Verify the entire chain consists of leg joints
            if not all(self._is_leg_joint(c.name) for c in chain):
                continue
            # The leg prefix is extracted from the first joint in the chain
            first = chain[0].name
            # Extract prefix: common patterns are "FL_", "FR_", "RL_", "RR_",
            # "left_", "right_", "leg_FL_"
            prefix = self._extract_leg_prefix(first)
            if prefix not in self.leg_joints:
                self.leg_joints[prefix] = []
            self.leg_joints[prefix].append(joint)

        # Sort joints within each leg by chain order
        for prefix in self.leg_joints:
            self.leg_joints[prefix].sort(
                key=lambda j: self._depth_from_base(j)
            )

        # Total mass
        self.total_mass = sum(l.mass for l in self.links.values())

    def _chain_to_base(self, joint: URDFJoint) -> Optional[list[URDFJoint]]:
        """Walk from *joint* up to the base, returning the chain or None."""
        chain = [joint]
        current = joint
        visited = {joint.name}
        while current.parent != self.base_link:
            parent_joint = None
            for j in self.joints.values():
                if j.child == current.parent:
                    parent_joint = j
                    break
            if parent_joint is None or parent_joint.name in visited:
                return None
            visited.add(parent_joint.name)
            # Allow fixed joints in the chain (e.g. torso)
            chain.insert(0, parent_joint)
            current = parent_joint
            if len(chain) > 20:
                return None
        return chain

    def _depth_from_base(self, joint: URDFJoint) -> int:
        chain = self._chain_to_base(joint)
        return len(chain) if chain else 0

    @staticmethod
    def _extract_leg_prefix(joint_name: str) -> str:
        """Extract a leg prefix from a joint name."""
        # Try common patterns
        for pat in ["FL", "FR", "RL", "RR", "ML", "MR",
                     "left", "right", "L", "R"]:
            if joint_name.lower().startswith(pat.lower() + "_") or \
               joint_name.lower().startswith(pat.lower()):
                return pat.upper() if len(pat) <= 3 else pat
        # Fallback: first token before underscore
        return joint_name.split("_")[0].upper()

    @staticmethod
    def _is_leg_joint(joint_name: str) -> bool:
        """Heuristic: is this joint part of a leg (not an arm/torso/head)?"""
        name_lower = joint_name.lower()
        # Exclude arms, torso, head, camera, etc.
        excluded = ["shoulder", "elbow", "wrist", "torso", "head",
                    "camera", "neck", "hand", "gripper", "finger"]
        for ex in excluded:
            if ex in name_lower:
                return False
        # Positive match: hip, knee, ankle, thigh, calf, leg, foot
        included = ["hip", "knee", "ankle", "thigh", "calf", "leg", "foot"]
        for inc in included:
            if inc in name_lower:
                return True
        # Also accept the Unitree naming: FL_hip, FR_thigh, etc.
        for pat in ["fl_", "fr_", "rl_", "rr_", "ml_", "mr_",
                     "left_", "right_"]:
            if name_lower.startswith(pat):
                # But still exclude if it's an arm joint with a leg prefix
                for ex in excluded:
                    if ex in name_lower:
                        return False
                return True
        return False

    def get_leg_link_lengths(self, leg_prefix: str) -> list[float]:
        """
        Return link lengths (in metres) for a leg, derived from joint
        origin xyz offsets along the chain.
        """
        joints = self.leg_joints.get(leg_prefix, [])
        lengths = []
        for j in joints:
            # Link length is the magnitude of the origin offset from parent
            length = float(np.linalg.norm(j.origin_xyz))
            if length > 1e-6:
                lengths.append(length)
        return lengths

    def get_hip_offset(self, leg_prefix: str) -> np.ndarray:
        """Return the hip offset (origin of first joint) relative to base."""
        joints = self.leg_joints.get(leg_prefix, [])
        if not joints:
            return np.zeros(3)
        return joints[0].origin_xyz.copy()

    def summary(self) -> str:
        """Return a human-readable summary of the robot spec."""
        lines = [f"Robot: {self.name}  (total mass = {self.total_mass:.3f} kg)"]
        lines.append(f"Base link: {self.base_link}")
        lines.append(f"Legs found: {list(self.leg_joints.keys())}")
        for prefix, joints in self.leg_joints.items():
            lines.append(f"\n  [{prefix}] ({len(joints)} joints)")
            hip_off = self.get_hip_offset(prefix)
            lines.append(f"    hip offset: {hip_off}")
            for j in joints:
                deg_lo = np.degrees(j.lower)
                deg_hi = np.degrees(j.upper)
                lines.append(
                    f"    {j.name:30s}  axis={j.axis}  "
                    f"limits=[{deg_lo:.1f}, {deg_hi:.1f}] deg  "
                    f"effort={j.effort:.1f} N·m"
                )
            lengths = self.get_leg_link_lengths(prefix)
            lines.append(f"    link lengths: {lengths}")
        return "\n".join(lines)


def parse_urdf(urdf_path: str | Path, base_link: Optional[str] = None
               ) -> RobotSpec:
    """
    Parse a URDF file and return a ``RobotSpec``.

    Args:
        urdf_path: path to the .urdf file.
        base_link: optional explicit base link name.
    """
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()

    robot_name = root.attrib.get("name", "unknown")
    spec = RobotSpec(name=robot_name)

    # Parse links
    for link_el in root.findall("link"):
        name = link_el.attrib["name"]
        link = URDFLink(name=name)

        inertial = link_el.find("inertial")
        if inertial is not None:
            mass_el = inertial.find("mass")
            if mass_el is not None:
                link.mass = float(mass_el.attrib.get("value", 0.0))
            origin_el = inertial.find("origin")
            if origin_el is not None:
                xyz = origin_el.attrib.get("xyz", "0 0 0")
                link.inertial_origin = np.array(
                    [float(v) for v in xyz.split()]
                )

        collision = link_el.find("collision")
        if collision is not None:
            origin_el = collision.find("origin")
            geom = collision.find("geometry")
            if geom is not None:
                box = geom.find("box")
                if box is not None:
                    link.collision_type = "box"
                    size = box.attrib.get("size", "0 0 0")
                    link.collision_size = np.array(
                        [float(v) for v in size.split()]
                    )
                cyl = geom.find("cylinder")
                if cyl is not None:
                    link.collision_type = "cylinder"
                    r = float(cyl.attrib.get("radius", 0))
                    h = float(cyl.attrib.get("length", 0))
                    link.collision_size = np.array([r, r, h])
                sphere = geom.find("sphere")
                if sphere is not None:
                    link.collision_type = "sphere"
                    r = float(sphere.attrib.get("radius", 0))
                    link.collision_size = np.array([r, r, r])

        spec.links[name] = link

    # Parse joints
    for joint_el in root.findall("joint"):
        name = joint_el.attrib["name"]
        jtype = joint_el.attrib.get("type", "fixed")
        parent = joint_el.find("parent").attrib["link"]
        child = joint_el.find("child").attrib["link"]

        joint = URDFJoint(
            name=name, joint_type=jtype, parent=parent, child=child
        )

        origin_el = joint_el.find("origin")
        if origin_el is not None:
            xyz = origin_el.attrib.get("xyz", "0 0 0")
            rpy = origin_el.attrib.get("rpy", "0 0 0")
            joint.origin_xyz = np.array([float(v) for v in xyz.split()])
            joint.origin_rpy = np.array([float(v) for v in rpy.split()])

        axis_el = joint_el.find("axis")
        if axis_el is not None:
            xyz = axis_el.attrib.get("xyz", "1 0 0")
            joint.axis = np.array([float(v) for v in xyz.split()])

        limit_el = joint_el.find("limit")
        if limit_el is not None:
            joint.lower = float(limit_el.attrib.get("lower", -np.pi))
            joint.upper = float(limit_el.attrib.get("upper", np.pi))
            joint.effort = float(limit_el.attrib.get("effort", 0.0))
            joint.velocity = float(limit_el.attrib.get("velocity", 0.0))

        spec.joints[name] = joint

    spec.analyse(base_link)
    return spec
