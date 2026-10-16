"""Robots package init."""
from .robot_base import LeggedRobot, Leg, Joint, RobotState
from .biped import Biped
from .quadruped import Quadruped
from .hexapod import Hexapod

# URDF-based robot factory (optional — requires downloaded URDFs)
try:
    from .urdf_robot import (
        create_robot_from_urdf,
        create_robot_from_urdf_path,
        list_available_robots,
        URDFRobot,
    )
except ImportError:
    pass

__all__ = [
    "LeggedRobot", "Leg", "Joint", "RobotState",
    "Biped", "Quadruped", "Hexapod",
]
