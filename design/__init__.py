"""
Design package: component-level robot morphology design for AI for robotics.

Pipeline:
    components (Link / Joint / Actuator / ContactPoint / SensorMount)
      → RobotDesign (validated + JSON-serializable morphology)  [config]
      → SimRobot (compiled tree kinematics + PD actuator stepping)  [builder]
      → GaitController (walk the design inside the 3DGS world)  [locomotion]
"""

from design.components import Link, Joint, Actuator, ContactPoint, SensorMount
from design.config import (
    RobotDesign, random_design, preset_quadruped, preset_hexapod,
    create_design, PRESETS,
)
from design.builder import SimRobot
from design.locomotion import GaitController, gait_groups

__all__ = [
    "Link", "Joint", "Actuator", "ContactPoint", "SensorMount",
    "RobotDesign", "random_design", "preset_quadruped", "preset_hexapod",
    "create_design", "PRESETS",
    "SimRobot", "GaitController", "gait_groups",
]
