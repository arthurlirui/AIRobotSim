"""
Test 1: Math utilities — rotations, transforms, interpolation.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.math_utils import (
    rot_x, rot_y, rot_z, euler_to_matrix, matrix_to_euler,
    Transform, lerp, slerp_matrix, rot_axis_angle, normalize, clamp
)


def test_rotation_matrices_orthogonal():
    """All rotation matrices should be orthogonal (RᵀR = I)."""
    for R in [rot_x(0.5), rot_y(1.2), rot_z(-0.7), euler_to_matrix(0.3, -0.4, 0.8)]:
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-10), f"R not orthogonal: {R}"
    print("  [PASS] Rotation matrices are orthogonal")


def test_euler_round_trip():
    """Euler → matrix → Euler should be identity."""
    for roll, pitch, yaw in [(0.1, 0.2, 0.3), (-0.5, 0.4, -0.6), (0, 0, 1.2)]:
        R = euler_to_matrix(roll, pitch, yaw)
        r2, p2, y2 = matrix_to_euler(R)
        assert np.allclose([roll, pitch, yaw], [r2, p2, y2], atol=1e-8), \
            f"Euler round-trip failed: ({roll},{pitch},{yaw}) → ({r2},{p2},{y2})"
    print("  [PASS] Euler angle round-trip")


def test_transform_apply():
    """Transform.apply should rotate and translate points correctly."""
    T = Transform(R=rot_z(np.pi / 2), t=np.array([1, 0, 0]))
    pts = np.array([[1, 0, 0], [0, 1, 0]])
    result = T.apply(pts)
    # rot_z(90) maps (1,0,0)→(0,1,0), then +t → (1,1,0)
    assert np.allclose(result[0], [1, 1, 0]), f"Unexpected: {result[0]}"
    # (0,1,0) → (-1,0,0), +t → (0,0,0)
    assert np.allclose(result[1], [0, 0, 0]), f"Unexpected: {result[1]}"
    print("  [PASS] Transform.apply works correctly")


def test_transform_inverse():
    """T ∘ T⁻¹ should be identity."""
    T = Transform(R=rot_y(0.5), t=np.array([2, 3, 1]))
    T_inv = T.inverse()
    Ti = T.multiply(T_inv)
    assert np.allclose(Ti.R, np.eye(3), atol=1e-10)
    assert np.allclose(Ti.t, np.zeros(3), atol=1e-10)
    print("  [PASS] Transform inverse works")


def test_slerp():
    """Slerp at t=0 and t=1 should give the endpoints."""
    R0 = rot_x(0)
    R1 = rot_x(np.pi / 2)
    assert np.allclose(slerp_matrix(R0, R1, 0.0), R0, atol=1e-10)
    assert np.allclose(slerp_matrix(R0, R1, 1.0), R1, atol=1e-10)
    # At t=0.5, should be 45 degrees
    R_mid = slerp_matrix(R0, R1, 0.5)
    assert np.allclose(R_mid, rot_x(np.pi / 4), atol=1e-8)
    print("  [PASS] Slerp interpolation works")


def test_normalize_clamp():
    v = np.array([3, 0, 0])
    assert np.allclose(normalize(v), [1, 0, 0])
    assert np.allclose(clamp(np.array([5, -1, 3]), 0, 3), [3, 0, 3])
    print("  [PASS] normalize and clamp work")


def run():
    print("Test: Math Utilities")
    test_rotation_matrices_orthogonal()
    test_euler_round_trip()
    test_transform_apply()
    test_transform_inverse()
    test_slerp()
    test_normalize_clamp()
    print()


if __name__ == "__main__":
    run()
