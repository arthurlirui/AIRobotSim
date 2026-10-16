"""
Bipedal robot — Unitree G1 humanoid (real-spec model).

Based on published Unitree G1 specifications:
  Height: 1.32 m (standing), Weight: 35 kg
  Leg length (thigh+shank): 0.60 m
  Hip width: ~0.24 m (from standing width 450 mm)
  DOF per leg: 6 (hip Y/P/R + knee + ankle P/R) — simplified to 3 (hip Y/P + knee)
  Max speed: 2 m/s
  Joint ranges: hip pitch ±154°, hip roll -30~+170°, hip yaw ±158°, knee 0~165°

Coordinate convention (Z-up):
  X = forward, Y = left, Z = up
"""

from __future__ import annotations

import numpy as np
from .robot_base import LeggedRobot, Leg, Joint


class Biped(LeggedRobot):
    """Unitree G1 humanoid robot (real-spec)."""

    def __init__(self, name: str = "UnitreeG1", mass: float = 35.0,
                 hip_width: float = 0.24, thigh_length: float = 0.30,
                 shank_length: float = 0.30, base_height: float = 0.72,
                 base_position=None):
        """
        Args:
            mass: total robot mass (kg), G1 = 35 kg
            hip_width: distance between left/right hip joints (m)
            thigh_length: thigh link length (m), G1 total leg = 0.60 m
            shank_length: shank link length (m)
            base_height: nominal hip height above ground (m)
                         G1 standing height 1.32 m, hip ~0.72 m
        """
        self.hip_width = hip_width
        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.base_height = base_height
        if base_position is None:
            base_position = np.array([0.0, 0.0, base_height])
        super().__init__(name, mass, base_position)

    @property
    def n_legs(self) -> int:
        return 2

    def _build_legs(self):
        """Build left and right legs with G1 joint ranges."""
        for side, sign in [("L", 1), ("R", -1)]:
            leg = Leg(
                name=f"leg_{side}",
                # Hip offset: hip is slightly below base center, laterally offset
                hip_offset=np.array([0.0, sign * self.hip_width / 2, -0.08]),
                joints=[
                    # Hip yaw (Z axis): ±158° ≈ ±2.76 rad
                    Joint(name=f"{side}_hip_yaw", axis=np.array([0, 0, 1]),
                          limits=(-2.76, 2.76)),
                    # Hip pitch (Y axis): ±154° ≈ ±2.69 rad
                    Joint(name=f"{side}_hip_pitch", axis=np.array([0, 1, 0]),
                          limits=(-2.69, 2.69)),
                    # Knee pitch (Y axis): 0~165° ≈ 0~2.88 rad
                    Joint(name=f"{side}_knee", axis=np.array([0, 1, 0]),
                          limits=(-0.05, 2.88)),
                ],
                link_lengths=[self.thigh_length, self.shank_length],
            )
            self.legs[leg.name] = leg

        self._set_standing_pose()

    def _set_standing_pose(self):
        """Set joint angles so the robot stands on both feet."""
        for name in self.legs:
            # Target foot position: below hip, on ground
            foot_target = np.array([
                self.state.base_position[0],
                self.legs[name].hip_offset[1] + self.state.base_position[1],
                0.0  # ground level
            ])
            angles = self.leg_inverse_kinematics(name, foot_target)
            if angles:
                self.set_joint_angles(name, angles)
        self.update_foot_positions()
        for name in self.legs:
            self.state.foot_contacts[name] = True
