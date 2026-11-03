"""
RobotDesign: a validated, serializable robot morphology.

A design is a flat set of components (links, joints, actuators, contact
points, sensor mounts). Validation enforces the structural rules that the
compiled :class:`~design.builder.SimRobot` relies on:

  - exactly one root link (the floating base)
  - every joint's parent/child exists; every non-root link has exactly
    one parent joint (tree topology, no cycles)
  - actuators reference existing revolute joints
  - contact points / sensor mounts reference existing links

Serialization to/from JSON makes designs portable: save a design, share
it, load it on another machine — or generate one procedurally with
:func:`random_design` (morphology search / curriculum experiments).
"""

from __future__ import annotations

import json
import numpy as np
from dataclasses import dataclass, field, asdict
from typing import Optional

from design.components import (
    Link, Joint, Actuator, ContactPoint, SensorMount,
)


@dataclass
class RobotDesign:
    """A complete robot morphology (component tree, pre-compilation)."""
    name: str = "Robot"
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)
    actuators: list[Actuator] = field(default_factory=list)
    contact_points: list[ContactPoint] = field(default_factory=list)
    sensor_mounts: list[SensorMount] = field(default_factory=list)

    # ------------------------------------------------------------------
    # Access helpers
    # ------------------------------------------------------------------

    def link(self, name: str) -> Link:
        for l in self.links:
            if l.name == name:
                return l
        raise KeyError(f"No link '{name}' in design '{self.name}'")

    def joint(self, name: str) -> Joint:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"No joint '{name}' in design '{self.name}'")

    @property
    def actuated_joints(self) -> list[Joint]:
        """Revolute joints that have an actuator, in design order."""
        actuated = {a.joint for a in self.actuators}
        return [j for j in self.joints
                if j.name in actuated and j.type == "revolute"]

    @property
    def n_dof(self) -> int:
        return len(self.actuated_joints)

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate(self) -> "RobotDesign":
        """Validate structure; raises ValueError on any violation."""
        if not self.links:
            raise ValueError(f"Design '{self.name}': no links")

        names = [l.name for l in self.links]
        if len(names) != len(set(names)):
            raise ValueError(f"Design '{self.name}': duplicate link names")

        for l in self.links:
            l.validate()
        roots = [l for l in self.links if l.is_root]
        if len(roots) != 1:
            raise ValueError(
                f"Design '{self.name}': exactly one root link required, "
                f"found {len(roots)}")

        jnames = [j.name for j in self.joints]
        if len(jnames) != len(set(jnames)):
            raise ValueError(f"Design '{self.name}': duplicate joint names")
        for j in self.joints:
            j.validate()
            if j.parent not in names or j.child not in names:
                raise ValueError(
                    f"Joint '{j.name}': references unknown link "
                    f"({j.parent} → {j.child})")

        # Tree topology: every non-root link has exactly one parent joint
        child_count: dict[str, int] = {}
        for j in self.joints:
            child_count[j.child] = child_count.get(j.child, 0) + 1
        for l in self.links:
            expected = 0 if l.is_root else 1
            got = child_count.get(l.name, 0)
            if got != expected:
                raise ValueError(
                    f"Link '{l.name}': expected {expected} parent joint(s), "
                    f"got {got} — topology must be a tree")

        # No cycles: walking parents from any link must reach the root
        parent_of = {j.child: j.parent for j in self.joints}
        root_name = roots[0].name
        for l in self.links:
            seen, cur = set(), l.name
            while cur != root_name:
                if cur in seen or cur not in parent_of:
                    raise ValueError(
                        f"Design '{self.name}': cycle / broken chain at "
                        f"link '{l.name}'")
                seen.add(cur)
                cur = parent_of[cur]

        # Actuators drive existing revolute joints
        revolute = {j.name for j in self.joints if j.type == "revolute"}
        for a in self.actuators:
            if a.joint not in revolute:
                raise ValueError(
                    f"Actuator drives '{a.joint}' which is not a revolute "
                    f"joint of design '{self.name}'")

        for c in self.contact_points:
            c.validate()
            if c.link not in names:
                raise ValueError(
                    f"ContactPoint '{c.name}': unknown link '{c.link}'")
        for m in self.sensor_mounts:
            m.validate()
            if m.link not in names:
                raise ValueError(
                    f"SensorMount '{m.name}': unknown link '{m.link}'")
        return self

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def to_json(self, indent: int = 2) -> str:
        """Serialize to a JSON string (arrays as lists)."""
        def _np(o):
            if isinstance(o, np.ndarray):
                return o.tolist()
            if isinstance(o, tuple):
                return list(o)
            raise TypeError(str(type(o)))

        data = {
            "name": self.name,
            "links": [asdict(l) for l in self.links],
            "joints": [asdict(j) for j in self.joints],
            "actuators": [asdict(a) for a in self.actuators],
            "contact_points": [asdict(c) for c in self.contact_points],
            "sensor_mounts": [asdict(m) for m in self.sensor_mounts],
        }
        return json.dumps(data, indent=indent, default=_np)

    def save(self, path: str):
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json())

    @classmethod
    def from_json(cls, text: str) -> "RobotDesign":
        data = json.loads(text)
        design = cls(name=data.get("name", "Robot"))
        for l in data.get("links", []):
            l["size"] = tuple(l["size"])
            l["color"] = tuple(l["color"])
            design.links.append(Link(**l))
        for j in data.get("joints", []):
            j["origin"] = np.asarray(j["origin"], dtype=float)
            j["origin_euler"] = np.asarray(j["origin_euler"], dtype=float)
            j["limits"] = tuple(j["limits"])
            design.joints.append(Joint(**j))
        for a in data.get("actuators", []):
            design.actuators.append(Actuator(**a))
        for c in data.get("contact_points", []):
            c["offset"] = np.asarray(c["offset"], dtype=float)
            design.contact_points.append(ContactPoint(**c))
        for m in data.get("sensor_mounts", []):
            m["offset"] = np.asarray(m["offset"], dtype=float)
            m["euler"] = np.asarray(m["euler"], dtype=float)
            design.sensor_mounts.append(SensorMount(**m))
        return design.validate()

    @classmethod
    def load(cls, path: str) -> "RobotDesign":
        with open(path, encoding="utf-8") as fh:
            return cls.from_json(fh.read())


# ---------------------------------------------------------------------------
# Procedural morphology generation
# ---------------------------------------------------------------------------

def random_design(name: str = "RandomBot",
                  n_legs: Optional[int] = None,
                  joints_per_leg: Optional[int] = None,
                  link_length: tuple = (0.10, 0.25),
                  seed: Optional[int] = None) -> RobotDesign:
    """
    Generate a random legged morphology (morphology-search primitive).

    Body is a single root box; N legs (2–6) of 2–3 revolute joints each,
    attached symmetrically along the body's ±X axis. Every actuated joint
    gets an actuator; every foot link gets a contact point.
    """
    rng = np.random.default_rng(seed)
    n_legs = int(n_legs) if n_legs is not None else int(rng.integers(2, 7))
    joints_per_leg = int(joints_per_leg) if joints_per_leg is not None \
        else int(rng.integers(2, 4))

    design = RobotDesign(name=name)
    body_len = float(rng.uniform(0.3, 0.6))
    design.links.append(Link(
        name="body", mass=float(rng.uniform(2.0, 8.0)),
        shape="box", size=(body_len, 0.2, 0.08),
        color=(0.3, 0.5, 0.8), is_root=True))

    half = body_len / 2
    for i in range(n_legs):
        # Legs spread along the body axis; one side +X, one −X alternately
        x = (half * (1 - 2 * (i // 2) / max(n_legs // 2 - 1, 1))
             if n_legs > 2 else (half if i == 0 else -half))
        side = "L" if i % 2 == 0 else "R"
        y = 0.12 if side == "L" else -0.12
        parent = "body"
        pos = np.array([x, y, 0.0])

        # Joint chain: hip (z-axis abduction) → thigh (y) → [shank (y)]
        axes = ["z", "y", "y"][:joints_per_leg]
        for k, ax in enumerate(axes):
            jname = f"leg{i}{side}_j{k}"
            L = float(rng.uniform(*link_length))
            design.joints.append(Joint(
                name=jname, parent=parent, child=f"{jname}_link",
                axis=ax,
                origin=pos.copy(),
                limits=(-0.8, 0.8) if ax == "z" else (-1.5, 1.5)))
            is_foot = (k == len(axes) - 1)
            design.links.append(Link(
                name=f"{jname}_link", mass=float(rng.uniform(0.1, 0.5)),
                shape="cylinder", size=(0.02, L),
                color=(0.8, 0.4, 0.3) if is_foot else (0.5, 0.5, 0.5)))
            design.actuators.append(Actuator(
                joint=jname, torque_limit=float(rng.uniform(10, 40)),
                kp=float(rng.uniform(100, 300)), kv=float(rng.uniform(3, 10))))
            if is_foot:
                design.contact_points.append(ContactPoint(
                    name=f"foot{i}{side}", link=f"{jname}_link",
                    offset=np.array([0.0, 0.0, -L]),
                    type="foot"))
            # Next joint sits at the end of this link (links extend −Z)
            parent = f"{jname}_link"
            pos = np.array([0.0, 0.0, -L])

    return design.validate()


# ---------------------------------------------------------------------------
# Preset designs
# ---------------------------------------------------------------------------

def preset_quadruped() -> RobotDesign:
    """A Go2-class quadruped expressed as a component design."""
    d = RobotDesign(name="PresetQuadruped")
    d.links.append(Link(name="body", mass=12.0, shape="box",
                        size=(0.4, 0.22, 0.08), color=(0.3, 0.5, 0.8),
                        is_root=True))
    d.sensor_mounts.append(SensorMount(
        name="head_cam", link="body", mount_type="camera",
        offset=np.array([0.22, 0.0, 0.05]),
        euler=np.array([0.0, np.pi / 2, 0.0]), camera_type="monocular"))
    d.sensor_mounts.append(SensorMount(
        name="antenna", link="body", mount_type="comm",
        offset=np.array([-0.15, 0.0, 0.08])))

    half_l, half_w = 0.18, 0.10
    for tag, sx, sy in (("FL", +1, +1), ("FR", +1, -1),
                        ("RL", -1, +1), ("RR", -1, -1)):
        hip = f"{tag}_hip"
        d.joints.append(Joint(
            name=hip, parent="body", child=f"{hip}_link", axis="z",
            origin=np.array([sx * half_l, sy * half_w, 0.0]),
            limits=(-0.6, 0.6)))
        d.links.append(Link(name=f"{hip}_link", mass=0.4, shape="box",
                            size=(0.06, 0.06, 0.06)))
        d.actuators.append(Actuator(joint=hip, torque_limit=25.0))

        knee = f"{tag}_knee"
        d.joints.append(Joint(
            name=knee, parent=f"{hip}_link", child=f"{knee}_link", axis="y",
            origin=np.zeros(3), limits=(-2.2, 0.5)))
        d.links.append(Link(name=f"{knee}_link", mass=0.5,
                            shape="cylinder", size=(0.02, 0.30)))
        d.actuators.append(Actuator(joint=knee, torque_limit=35.0))
        d.contact_points.append(ContactPoint(
            name=f"foot_{tag}", link=f"{knee}_link",
            offset=np.array([0.0, 0.0, -0.30])))

    return d.validate()


def preset_hexapod() -> RobotDesign:
    """A six-legged platform: 3 legs per side, 2 joints per leg."""
    d = RobotDesign(name="PresetHexapod")
    d.links.append(Link(name="body", mass=8.0, shape="box",
                        size=(0.5, 0.3, 0.08), color=(0.6, 0.3, 0.6),
                        is_root=True))
    xs = (0.20, 0.0, -0.20)
    for i in range(6):
        side = "L" if i % 2 == 0 else "R"
        x = xs[i // 2]
        y = 0.16 if side == "L" else -0.16
        hip = f"leg{i}{side}_hip"
        d.joints.append(Joint(
            name=hip, parent="body", child=f"{hip}_link", axis="z",
            origin=np.array([x, y, 0.0]), limits=(-0.7, 0.7)))
        d.links.append(Link(name=f"{hip}_link", mass=0.3, shape="box",
                            size=(0.05, 0.05, 0.05)))
        d.actuators.append(Actuator(joint=hip, torque_limit=18.0))

        knee = f"leg{i}{side}_knee"
        d.joints.append(Joint(
            name=knee, parent=f"{hip}_link", child=f"{knee}_link", axis="y",
            origin=np.zeros(3), limits=(-2.0, 0.4)))
        d.links.append(Link(name=f"{knee}_link", mass=0.4,
                            shape="cylinder", size=(0.02, 0.25)))
        d.actuators.append(Actuator(joint=knee, torque_limit=22.0))
        d.contact_points.append(ContactPoint(
            name=f"foot_{i}{side}", link=f"{knee}_link",
            offset=np.array([0.0, 0.0, -0.25])))
    return d.validate()


#: Named preset registry
PRESETS = {
    "quadruped": preset_quadruped,
    "hexapod": preset_hexapod,
}


def create_design(name: str = "quadruped", **kwargs) -> RobotDesign:
    """Build a design by preset name ('quadruped' / 'hexapod')."""
    try:
        return PRESETS[name](**kwargs)
    except KeyError:
        raise KeyError(f"Unknown preset '{name}'. Known: {sorted(PRESETS)}") \
            from None
