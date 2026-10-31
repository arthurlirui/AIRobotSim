"""
Mobile (non-legged) robot base: wheeled robots and UAVs.

While :class:`~robots.robot_base.LeggedRobot` models a URDF-style leg chain,
wheeled robots and drones are driven by simpler motion models — a
differential-drive unicycle (two driven wheels) or a multirotor point-mass
with attitude. Both share the same *world interaction* contract as legged
robots: they own a :class:`RobotState`, integrate velocity, and query the
:class:`~core.gaussian_splat_env.GaussianSplatEnv` for ground height and
collision, so the interaction / comm layers can treat every platform
uniformly.
"""

from __future__ import annotations

import numpy as np
from abc import ABC, abstractmethod
from typing import Optional

from robots.robot_base import RobotState
from core.math_utils import euler_to_matrix


class MobileRobot(ABC):
    """
    Abstract mobile robot (wheeled / aerial) with a simple motion model.

    Subclasses implement :meth:`step` — advance the motion model by one
    control step — and may use :meth:`set_velocity` for direct velocity
    commands (the same interface the legged robots use).
    """

    def __init__(self, name: str, mass: float = 1.0,
                 base_position: Optional[np.ndarray] = None):
        self.name = name
        self.mass = mass
        self.state = RobotState(
            base_position=base_position.copy() if base_position is not None else np.zeros(3),
        )

    # ------------------------------------------------------------------
    # Common state helpers (shared with LeggedRobot for uniform handling)
    # ------------------------------------------------------------------

    @property
    def position(self) -> np.ndarray:
        return self.state.base_position

    @property
    def yaw(self) -> float:
        return self.state.base_orientation[2]

    def base_transform(self) -> np.ndarray:
        """World rotation matrix of the body frame."""
        roll, pitch, yaw = self.state.base_orientation
        return euler_to_matrix(roll, pitch, yaw)

    def set_velocity(self, velocity: np.ndarray,
                     angular_velocity: Optional[np.ndarray] = None):
        """Direct velocity command (world frame, m/s and rad/s)."""
        self.state.base_velocity = np.asarray(velocity, dtype=float).copy()
        if angular_velocity is not None:
            self.state.base_angular_velocity = np.asarray(angular_velocity, dtype=float).copy()

    def apply_velocity(self, dt: float):
        """Integrate base velocity (shared integration contract)."""
        self.state.base_position += self.state.base_velocity * dt
        self.state.base_orientation += self.state.base_angular_velocity * dt
        self.state.t += dt

    @abstractmethod
    def step(self, dt: float, **controls):
        """Advance the platform's motion model by ``dt`` seconds."""
        ...

    # ------------------------------------------------------------------
    # Environment interaction (mirrors the legged robots' contract)
    # ------------------------------------------------------------------

    def clamp_to_ground(self, env, offset: float = 0.0):
        """Pin the robot to the terrain surface (ground-following robots)."""
        h = env.height_at(self.state.base_position[0], self.state.base_position[1])
        self.state.base_position[2] = h + offset

    def check_collision(self, env) -> bool:
        """True if the body center collides with any splat in the env."""
        return len(env.collision(self.state.base_position)) > 0

    def __repr__(self):
        return (f"{self.__class__.__name__}(name={self.name!r}, "
                f"pos={self.state.base_position})")


class WheeledRobot(MobileRobot):
    """
    Differential-drive wheeled robot (e.g. TurtleBot-style, ROS-2-wheeled base).

    Motion model (unicycle):

        x'  = v · cos(yaw)
        y'  = v · sin(yaw)
        yaw'= ω

    Control inputs: linear velocity ``v`` (m/s, forward) and angular
    velocity ``omega`` (rad/s). Optionally terrain-adaptive: the body
    height / pitch follows the ground underneath the wheel axles.
    """

    def __init__(self, name: str = "WheeledBot", mass: float = 10.0,
                 wheel_radius: float = 0.08, wheel_base: float = 0.30,
                 max_speed: float = 2.0, max_omega: float = 3.0,
                 base_position=None):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base          # lateral distance between wheels
        self.max_speed = max_speed
        self.max_omega = max_omega
        super().__init__(name, mass, base_position)

    def step(self, dt: float, v: float = 0.0, omega: float = 0.0, env=None):
        """
        Advance the unicycle model.

        Args:
            v: forward linear velocity command (m/s)
            omega: yaw angular velocity command (rad/s)
            env: optional environment — when given, the robot follows the
                terrain height under its center.
        """
        v = float(np.clip(v, -self.max_speed, self.max_speed))
        omega = float(np.clip(omega, -self.max_omega, self.max_omega))

        yaw = self.yaw
        self.state.base_velocity = np.array([v * np.cos(yaw), v * np.sin(yaw), 0.0])
        self.state.base_angular_velocity = np.array([0.0, 0.0, omega])

        self.state.base_position[:2] += self.state.base_velocity[:2] * dt
        self.state.base_orientation[2] += omega * dt
        self.state.t += dt

        if env is not None:
            self.clamp_to_ground(env, offset=self.wheel_radius)

    def wheel_speeds(self) -> tuple[float, float]:
        """Per-wheel speeds (m/s) for the current (v, ω) command."""
        v = float(np.hypot(*self.state.base_velocity[:2])) * (
            1.0 if self.state.base_velocity[0] >= 0 else -1.0) \
            if np.linalg.norm(self.state.base_velocity) > 1e-9 else 0.0
        omega = self.state.base_angular_velocity[2]
        return (v - omega * self.wheel_base / 2,
                v + omega * self.wheel_base / 2)


class UAV(MobileRobot):
    """
    Quadrotor UAV (point-mass with yaw attitude).

    Motion model: velocity commands in the **body frame** (forward /
    lateral / vertical) are rotated by the yaw into world frame. Roll /
    pitch are user-settable attitude angles (visual / sensor modeling),
    not integrated dynamics — sufficient for sensor placement, comm
    topology studies, and swarm experiments at this fidelity level.

    Altitude is clamped to a floor (terrain height + safety margin) when
    an environment is provided, so UAVs cannot fly through the ground.
    """

    def __init__(self, name: str = "UAV", mass: float = 1.5,
                 max_speed: float = 10.0, max_climb_rate: float = 3.0,
                 safety_margin: float = 0.3,
                 base_position=None):
        self.max_speed = max_speed
        self.max_climb_rate = max_climb_rate
        self.safety_margin = safety_margin
        super().__init__(name, mass, base_position)

    def step(self, dt: float, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
             yaw_rate: float = 0.0, env=None):
        """
        Advance the multirotor model.

        Args:
            vx, vy: body-frame horizontal velocity commands (m/s)
            vz: vertical velocity command (m/s, world frame)
            yaw_rate: yaw angular velocity command (rad/s)
            env: optional environment — enforces a terrain floor
        """
        v_body = np.array([vx, vy])
        speed = float(np.linalg.norm(v_body))
        if speed > self.max_speed:
            v_body = v_body / speed * self.max_speed
        vz = float(np.clip(vz, -self.max_climb_rate, self.max_climb_rate))

        yaw = self.yaw
        c, s = np.cos(yaw), np.sin(yaw)
        v_world = np.array([c * v_body[0] - s * v_body[1],
                            s * v_body[0] + c * v_body[1],
                            vz])
        self.state.base_velocity = v_world
        self.state.base_angular_velocity = np.array([0.0, 0.0, yaw_rate])

        self.state.base_position += v_world * dt
        self.state.base_orientation[2] += yaw_rate * dt
        self.state.t += dt

        if env is not None:
            floor = env.height_at(self.state.base_position[0],
                                  self.state.base_position[1]) \
                    + self.safety_margin
            if self.state.base_position[2] < floor:
                self.state.base_position[2] = floor
                self.state.base_velocity[2] = max(0.0, self.state.base_velocity[2])

    def set_attitude(self, roll: float = 0.0, pitch: float = 0.0):
        """Set roll/pitch for sensor / camera modeling (rad)."""
        self.state.base_orientation[0] = roll
        self.state.base_orientation[1] = pitch
