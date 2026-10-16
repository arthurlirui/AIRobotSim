"""
Math utilities for robot simulation: vectors, rotations, transforms.

All operations use numpy arrays. Conventions:
  - 3D positions are shape (3,) float arrays
  - Rotations are 3x3 rotation matrices (we avoid quaternions in the core
    to keep the dependency surface small; conversions provided where needed)
  - Transforms are 4x4 homogeneous matrices
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------

def rot_x(angle: float) -> np.ndarray:
    """3x3 rotation matrix about the X axis (radians)."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]], dtype=float)


def rot_y(angle: float) -> np.ndarray:
    """3x3 rotation matrix about the Y axis (radians)."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]], dtype=float)


def rot_z(angle: float) -> np.ndarray:
    """3x3 rotation matrix about the Z axis (radians)."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]], dtype=float)


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX intrinsic Euler angles (roll-x, pitch-y, yaw-z) → 3x3 matrix."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_euler(R: np.ndarray) -> tuple[float, float, float]:
    """3x3 matrix → (roll, pitch, yaw) ZYX intrinsic."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if np.isclose(np.cos(pitch), 0.0):
        # Gimbal lock
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Transform (SE3)
# ---------------------------------------------------------------------------

@dataclass
class Transform:
    """Rigid-body transform: rotation R (3x3) + translation t (3,)."""
    R: np.ndarray = None   # 3x3
    t: np.ndarray = None   # 3,

    def __post_init__(self):
        if self.R is None:
            self.R = np.eye(3)
        if self.t is None:
            self.t = np.zeros(3)
        self.R = np.asarray(self.R, dtype=float).reshape(3, 3)
        self.t = np.asarray(self.t, dtype=float).reshape(3)

    @property
    def matrix(self) -> np.ndarray:
        """4x4 homogeneous matrix."""
        M = np.eye(4)
        M[:3, :3] = self.R
        M[:3, 3] = self.t
        return M

    def inverse(self) -> "Transform":
        R_inv = self.R.T
        t_inv = -R_inv @ self.t
        return Transform(R_inv, t_inv)

    def apply(self, points: np.ndarray) -> np.ndarray:
        """Transform a set of points (N, 3) → (N, 3)."""
        points = np.atleast_2d(points)
        return (self.R @ points.T).T + self.t

    def multiply(self, other: "Transform") -> "Transform":
        """Compose: self ∘ other."""
        return Transform(self.R @ other.R, self.R @ other.t + self.t)

    def __matmul__(self, other: "Transform") -> "Transform":
        return self.multiply(other)


# ---------------------------------------------------------------------------
# Interpolation
# ---------------------------------------------------------------------------

def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation."""
    return a + (b - a) * t


def slerp_matrix(R0: np.ndarray, R1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two rotation matrices."""
    R = R0.T @ R1
    # Axis-angle from rotation matrix
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if np.isclose(angle, 0):
        return R0
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]])
    axis_norm = np.linalg.norm(axis)
    if np.isclose(axis_norm, 0):
        return R0
    axis = axis / axis_norm
    partial = rot_axis_angle(axis, angle * t)
    return R0 @ partial


def rot_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


# ---------------------------------------------------------------------------
# Misc
# ---------------------------------------------------------------------------

def normalize(v: np.ndarray) -> np.ndarray:
    """Normalize a vector."""
    n = np.linalg.norm(v)
    return v / (n + 1e-12)


def clamp(v: np.ndarray, lo: float, hi: float) -> np.ndarray:
    """Clamp array element-wise."""
    return np.clip(v, lo, hi)
