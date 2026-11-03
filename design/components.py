"""
Robot components for AI-for-robotics morphology design.

A robot design is a tree of *links* connected by *joints*. Every link can
carry *contact points* (feet / wheels — where the robot touches the 3D
world) and *sensor mounts* (where cameras from ``sensors/`` attach).
Every joint carries an *actuator* with torque limits and PD gains.

This module defines the component dataclasses. Assembly into a
:class:`~design.config.RobotDesign` (validation + serialization) happens
in ``config.py``; the compiled kinematic/dynamic simulation object
(:class:`~design.builder.SimRobot`) lives in ``builder.py``.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Joints
# ---------------------------------------------------------------------------

#: Supported joint axes in the local link frame
JOINT_AXES = {
    "x": np.array([1.0, 0.0, 0.0]),
    "y": np.array([0.0, 1.0, 0.0]),
    "z": np.array([0.0, 0.0, 1.0]),
}

JOINT_TYPES = ("revolute", "fixed")


@dataclass
class Joint:
    """
    A joint connecting a child link to its parent link.

    ``origin`` is the child-link frame expressed in the parent link frame
    (translation + ZYX Euler). ``axis`` is the revolute axis **in the
    child link frame** (URDF convention).
    """
    name: str
    parent: str                        # parent link name
    child: str                         # child link name
    type: str = "revolute"             # "revolute" | "fixed"
    axis: str = "z"                    # "x" | "y" | "z" (revolute only)
    origin: np.ndarray = field(default_factory=lambda: np.zeros(3))
    origin_euler: np.ndarray = field(default_factory=lambda: np.zeros(3))
    limits: tuple = (-np.pi, np.pi)    # (lower, upper) rad
    velocity_limit: float = 10.0       # rad/s

    def axis_vector(self) -> np.ndarray:
        try:
            return JOINT_AXES[self.axis]
        except KeyError:
            raise ValueError(
                f"Joint '{self.name}': axis must be one of {sorted(JOINT_AXES)}, "
                f"got '{self.axis}'") from None

    def validate(self):
        if self.type not in JOINT_TYPES:
            raise ValueError(f"Joint '{self.name}': unknown type '{self.type}'")
        if self.type == "revolute":
            self.axis_vector()  # raises on bad axis
            if self.limits[0] >= self.limits[1]:
                raise ValueError(
                    f"Joint '{self.name}': limits must satisfy lower < upper")
        if not self.parent or not self.child:
            raise ValueError(f"Joint '{self.name}': parent/child must be named")
        if self.parent == self.child:
            raise ValueError(f"Joint '{self.name}': parent == child")


# ---------------------------------------------------------------------------
# Actuators
# ---------------------------------------------------------------------------

@dataclass
class Actuator:
    """
    Joint actuator (motor + gearbox model, PD-controlled).

    ``torque_limit`` caps the commanded torque; ``velocity_limit`` mirrors
    the joint's and is enforced by the torque saturation.
    """
    joint: str                         # joint name it drives
    torque_limit: float = 30.0         # N·m
    kv: float = 6.0                    # velocity damping (N·m·s/rad)
    kp: float = 200.0                  # position gain (N·m/rad)

    def compute_torque(self, q_desired: float, q: float, dq: float,
                       dt: float) -> float:
        """Saturated PD torque toward the desired joint position."""
        tau = self.kp * (q_desired - q) - self.kv * dq
        return float(np.clip(tau, -self.torque_limit, self.torque_limit))


# ---------------------------------------------------------------------------
# Contact points
# ---------------------------------------------------------------------------

CONTACT_TYPES = ("foot", "wheel", "pad")


@dataclass
class ContactPoint:
    """
    A point on a link that can touch the 3D world.

    ``offset`` is in the link's local frame. ``radius`` is used for
    terrain penetration checks.
    """
    name: str
    link: str
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    type: str = "foot"                 # "foot" | "wheel" | "pad"
    radius: float = 0.02               # contact sphere radius (m)
    friction: float = 0.8              # static friction coefficient

    def validate(self):
        if self.type not in CONTACT_TYPES:
            raise ValueError(
                f"ContactPoint '{self.name}': type must be one of "
                f"{CONTACT_TYPES}, got '{self.type}'")
        if self.friction < 0:
            raise ValueError(f"ContactPoint '{self.name}': friction < 0")


# ---------------------------------------------------------------------------
# Sensor mounts
# ---------------------------------------------------------------------------

@dataclass
class SensorMount:
    """
    Mounting point for a camera / IMU / comm antenna on a link.

    ``mount_type`` selects the platform subsystem:
      - ``camera``  → instantiate a class from ``sensors/``
      - ``comm``    → position source for ``comm.channel.base.Pose``
      - ``imu``     → reads the link's world transform
    """
    name: str
    link: str
    mount_type: str = "camera"         # "camera" | "comm" | "imu"
    offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))
    camera_type: str = "monocular"     # sensors/ class name (camera mounts)

    def validate(self):
        if self.mount_type not in ("camera", "comm", "imu"):
            raise ValueError(
                f"SensorMount '{self.name}': mount_type must be "
                f"camera/comm/imu, got '{self.mount_type}'")


# ---------------------------------------------------------------------------
# Links
# ---------------------------------------------------------------------------

LINK_SHAPES = ("box", "cylinder", "sphere")


@dataclass
class Link:
    """
    A rigid body. ``size`` semantics depend on ``shape``:
    box → (length_x, width_y, height_z); cylinder → (radius, length);
    sphere → (radius,).
    """
    name: str
    mass: float = 1.0                  # kg
    shape: str = "box"
    size: tuple = (0.2, 0.1, 0.05)     # meters
    color: tuple = (0.6, 0.6, 0.6)     # RGB [0,1] for rendering
    is_root: bool = False              # the floating base (at most one)

    def validate(self):
        if self.mass <= 0:
            raise ValueError(f"Link '{self.name}': mass must be > 0")
        if self.shape not in LINK_SHAPES:
            raise ValueError(
                f"Link '{self.name}': shape must be one of {LINK_SHAPES}, "
                f"got '{self.shape}'")
        if len(self.size) != (1 if self.shape == "sphere" else
                              2 if self.shape == "cylinder" else 3):
            raise ValueError(f"Link '{self.name}': size arity mismatch "
                             f"for shape '{self.shape}'")

    def bounding_radius(self) -> float:
        """Radius of the smallest sphere containing the link (collision)."""
        if self.shape == "sphere":
            return float(self.size[0])
        if self.shape == "cylinder":
            return float(np.hypot(self.size[0], self.size[1] / 2))
        return float(0.5 * np.linalg.norm(np.asarray(self.size, dtype=float)))
