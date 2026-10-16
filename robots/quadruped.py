"""
Quadrupedal robot — Unitree Go2 (real-spec model).

Based on published Unitree Go2 specifications:
  Standing size: 700×310×400 mm (L×W×H)
  Weight: ~15 kg
  12 DOF (3 per leg): hip abduction/adduction + hip flexion/extension + knee
  Max speed: 3.7 m/s (5 m/s rage mode)
  Joint torque: ~45 N·m (knee)
  Joint ranges: body -48~48°, thigh -200~90°, calf -156~-48°
  Max climb: 40° slope, 16 cm step

Coordinate convention (Z-up):
  X = forward, Y = left, Z = up
"""

from __future__ import annotations

import numpy as np
from .robot_base import LeggedRobot, Leg, Joint


class Quadruped(LeggedRobot):
    """Unitree Go2 quadruped robot (real-spec)."""

    def __init__(self, name: str = "UnitreeGo2", mass: float = 15.0,
                 body_length: float = 0.40, body_width: float = 0.22,
                 thigh_length: float = 0.16, shank_length: float = 0.16,
                 base_height: float = 0.28, base_position=None):
        """
        Args:
            mass: total robot mass (kg), Go2 = 15 kg
            body_length: trunk length (m), Go2 standing L=700mm → hip-to-hip ~400mm
            body_width: trunk width (m), Go2 standing W=310mm → hip-to-hip ~220mm
            thigh_length: thigh link length (m)
            shank_length: shank link length (m)
            base_height: nominal standing hip height (m), Go2 H=400mm → hip ~280mm
        """
        self.body_length = body_length
        self.body_width = body_width
        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.base_height = base_height
        if base_position is None:
            base_position = np.array([0.0, 0.0, base_height])
        super().__init__(name, mass, base_position)

    @property
    def n_legs(self) -> int:
        return 4

    def _build_legs(self):
        """
        4 legs with Go2 joint configuration.
        Each leg: hip yaw (Z) + hip pitch (Y) + knee pitch (Y)
        Joint ranges from Go2 specs:
          Body/hip abduction: ±48° ≈ ±0.84 rad
          Thigh/hip pitch: -200°~90° → -3.49~1.57 rad
          Calf/knee: -156°~-48° → -2.72~-0.84 rad (mapped to positive range)
        """
        half_l = self.body_length / 2
        half_w = self.body_width / 2

        leg_configs = [
            ("FL", np.array([ half_l,  half_w, -0.03]), 1),   # front-left
            ("FR", np.array([ half_l, -half_w, -0.03]), -1),  # front-right
            ("RL", np.array([-half_l,  half_w, -0.03]), 1),   # rear-left
            ("RR", np.array([-half_l, -half_w, -0.03]), -1),  # rear-right
        ]

        for tag, hip_offset, side_sign in leg_configs:
            leg = Leg(
                name=f"leg_{tag}",
                hip_offset=hip_offset,
                joints=[
                    # Hip yaw (Z axis): ±48° ≈ ±0.84 rad (abduction)
                    Joint(name=f"{tag}_hip_yaw", axis=np.array([0, 0, 1]),
                          limits=(-0.84, 0.84)),
                    # Hip pitch (Y axis): -200°~90°
                    Joint(name=f"{tag}_hip_pitch", axis=np.array([0, 1, 0]),
                          limits=(-1.57, 3.49)),
                    # Knee pitch (Y axis): mapped to 0~2.72 rad
                    Joint(name=f"{tag}_knee", axis=np.array([0, 1, 0]),
                          limits=(-0.05, 2.72)),
                ],
                link_lengths=[self.thigh_length, self.shank_length],
            )
            self.legs[leg.name] = leg

        self._set_standing_pose()

    def _set_standing_pose(self):
        """Standing pose: all four feet on ground."""
        for name, leg in self.legs.items():
            foot_target = (self.state.base_position + leg.hip_offset).copy()
            foot_target[2] = 0.0
            angles = self.leg_inverse_kinematics(name, foot_target)
            if angles:
                self.set_joint_angles(name, angles)
        self.update_foot_positions()
        for name in self.legs:
            self.state.foot_contacts[name] = True
