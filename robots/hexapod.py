"""
Hexapod robot — DOBOT Hexplorer (real-spec model).

Based on published DOBOT Hexplorer specifications:
  Standing size: 863×470×480 mm (L×W×H)
  Weight: ~20 kg
  18 DOF (3 per leg): hip yaw + hip pitch + knee pitch
  Max speed: 1.8 m/s
  Max climb: 40° slope, 20 cm step
  Payload: 10 kg rated, 15 kg max
  Joint range: front/mid hip -60°~180°, rear hip different

Coordinate convention (Z-up):
  X = forward, Y = left, Z = up
"""

from __future__ import annotations

import numpy as np
from .robot_base import LeggedRobot, Leg, Joint


class Hexapod(LeggedRobot):
    """DOBOT Hexplorer hexapod robot (real-spec)."""

    def __init__(self, name: str = "Hexplorer", mass: float = 20.0,
                 body_length: float = 0.55, body_width: float = 0.38,
                 thigh_length: float = 0.16, shank_length: float = 0.16,
                 base_height: float = 0.30, base_position=None):
        """
        Args:
            mass: total robot mass (kg), Hexplorer = 20 kg
            body_length: body length (m), standing L=863mm → hip span ~550mm
            body_width: body width (m), standing W=470mm → hip span ~380mm
            thigh_length: thigh/coxa link length (m)
            shank_length: shank/femur link length (m)
            base_height: nominal standing height (m), H=480mm → body ~300mm
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
        return 6

    def _build_legs(self):
        """
        6 legs with Hexplorer joint configuration.
        Each leg: hip yaw (Z) + hip pitch (Y) + knee pitch (Y), 3 DOF per leg.
        Joint ranges from Hexplorer specs:
          Front/mid hip: -60°~180° → -1.05~3.14 rad
          Rear hip: different range (approx -30°~180°)
        Simplified to uniform ranges for all legs.
        """
        half_l = self.body_length / 2
        half_w = self.body_width / 2

        leg_configs = [
            ("FL", np.array([ half_l,        half_w, -0.02]),  1),
            ("ML", np.array([ 0.0,           half_w, -0.02]),  1),
            ("RL", np.array([-half_l,        half_w, -0.02]),  1),
            ("FR", np.array([ half_l,       -half_w, -0.02]), -1),
            ("MR", np.array([ 0.0,          -half_w, -0.02]), -1),
            ("RR", np.array([-half_l,       -half_w, -0.02]), -1),
        ]

        for tag, hip_offset, side_sign in leg_configs:
            leg = Leg(
                name=f"leg_{tag}",
                hip_offset=hip_offset,
                joints=[
                    # Hip yaw (Z axis): -60°~180° → -1.05~3.14 rad
                    Joint(name=f"{tag}_hip_yaw", axis=np.array([0, 0, 1]),
                          limits=(-1.05, 3.14)),
                    # Hip pitch (Y axis)
                    Joint(name=f"{tag}_hip_pitch", axis=np.array([0, 1, 0]),
                          limits=(-1.57, 1.57)),
                    # Knee pitch (Y axis)
                    Joint(name=f"{tag}_knee", axis=np.array([0, 1, 0]),
                          limits=(-0.05, 2.50)),
                ],
                link_lengths=[self.thigh_length, self.shank_length],
            )
            self.legs[leg.name] = leg

        self._set_standing_pose()

    def _set_standing_pose(self):
        """Standing pose: all six feet on ground, splayed outward."""
        for name, leg in self.legs.items():
            foot_target = self.state.base_position + leg.hip_offset
            # Push foot outward (in Y) for stability
            side = 1.0 if "L" in name else -1.0
            foot_target[1] += side * 0.04
            foot_target[2] = 0.0
            angles = self.leg_inverse_kinematics(name, foot_target)
            if angles:
                self.set_joint_angles(name, angles)
        self.update_foot_positions()
        for name in self.legs:
            self.state.foot_contacts[name] = True

    @property
    def tripod_a(self) -> list[str]:
        """Legs in tripod group A (FL, MR, RL)."""
        return ["leg_FL", "leg_MR", "leg_RL"]

    @property
    def tripod_b(self) -> list[str]:
        """Legs in tripod group B (FR, ML, RR)."""
        return ["leg_FR", "leg_ML", "leg_RR"]
