"""
Gym-style navigation environments over the simulation core.

``NavigationEnv`` places a mobile robot (wheeled or UAV) in a
:class:`~core.gaussian_splat_env.GaussianSplatEnv` and tasks it with
reaching a goal position.  Observation: relative goal vector (rotated
into the body frame), current velocity, and a short forward ray-scan
(the same ray queries the sensor / comm layers use).  Reward: progress
toward the goal + reach bonus − time penalty − collision penalty.

An optional **communication reward** couples the task with the comm
module: each step computes the path gain from the robot to a fixed base
station through a ``ChannelModel`` (any Layer 0/1/2 model) and adds a
shaped bonus for keeping the link alive — enabling research on
communication-constrained navigation.
"""

from __future__ import annotations

import numpy as np
from typing import Optional

from core.gaussian_splat_env import GaussianSplatEnv
from robots.mobile_base import MobileRobot, WheeledRobot, UAV


class NavigationEnv:
    """
    Point-goal navigation for mobile robots in a 3DGS environment.

    Follows the Gymnasium step contract:
        reset() -> obs
        step(action) -> (obs, reward, terminated, truncated, info)
    """

    metadata = {"name": "NavigationEnv-v0"}

    def __init__(self, robot: MobileRobot, env: GaussianSplatEnv,
                 goal: np.ndarray,
                 max_steps: int = 200,
                 reach_radius: float = 0.5,
                 n_rays: int = 8,
                 ray_range: float = 5.0,
                 channel=None,
                 channel_reward_weight: float = 0.0,
                 base_station_position: Optional[np.ndarray] = None,
                 seed: int = 0):
        self.robot = robot
        self.env = env
        self.goal = np.asarray(goal, dtype=float)
        self.max_steps = max_steps
        self.reach_radius = reach_radius
        self.n_rays = n_rays
        self.ray_range = ray_range
        self.channel = channel
        self.channel_reward_weight = channel_reward_weight
        self.base_station_position = base_station_position
        self.rng = np.random.default_rng(seed)
        self._steps = 0
        self._start: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Spaces (informal — arrays, not gym.Space objects)
    # ------------------------------------------------------------------

    @property
    def observation_dim(self) -> int:
        # goal (3) + velocity (3) + rays (n_rays)
        return 3 + 3 + self.n_rays

    @property
    def action_dim(self) -> int:
        return 2 if isinstance(self.robot, WheeledRobot) else 3

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    def _observation(self) -> np.ndarray:
        # Relative goal in the body frame (yaw-rotated)
        yaw = self.robot.yaw
        c, s = np.cos(yaw), np.sin(yaw)
        rel = self.goal - self.robot.position
        goal_body = np.array([c * rel[0] + s * rel[1],
                              -s * rel[0] + c * rel[1],
                              rel[2]])
        goal_body[0] /= 20.0
        goal_body[1] /= 20.0
        goal_body[2] /= 5.0

        vel = self.robot.state.base_velocity / 5.0

        # Forward ray scan in the horizontal plane
        rays = np.ones(self.n_rays)
        for i in range(self.n_rays):
            ang = -np.pi / 2 + np.pi * i / max(self.n_rays - 1, 1)
            d = np.array([c * np.cos(ang), s * np.cos(ang), np.sin(ang)])
            hit = self.env.ray_cast(self.robot.position + np.array([0, 0, 0.1]),
                                    d, max_dist=self.ray_range)
            rays[i] = (hit if hit is not None else self.ray_range) / self.ray_range

        return np.concatenate([goal_body, vel, rays])

    def reset(self, *, start: Optional[np.ndarray] = None) -> np.ndarray:
        if start is not None:
            self.robot.state.base_position = np.asarray(start, dtype=float).copy()
        self.robot.state.base_velocity[:] = 0
        self.robot.state.base_angular_velocity[:] = 0
        self._start = self.robot.position.copy()
        self._steps = 0
        return self._observation()

    def step(self, action: np.ndarray):
        action = np.clip(np.asarray(action, dtype=float), -1, 1)
        prev_dist = float(np.linalg.norm(self.goal - self.robot.position))

        if isinstance(self.robot, WheeledRobot):
            self.robot.step(self.dt, v=action[0] * self.robot.max_speed,
                            omega=action[1] * self.robot.max_omega, env=self.env)
        else:  # UAV
            self.robot.step(self.dt, vx=action[0] * 2.0,
                            vy=action[1] * 2.0, vz=action[2] * 1.0,
                            env=self.env)

        new_dist = float(np.linalg.norm(self.goal - self.robot.position))
        self._steps += 1

        # Reward: progress + reach bonus − time − collision
        reward = (prev_dist - new_dist) * 2.0
        terminated = new_dist < self.reach_radius
        if terminated:
            reward += 20.0
        reward -= 0.01  # time penalty

        collided = self.robot.check_collision(self.env)
        if collided:
            reward -= 0.5

        # Optional communication-coupled reward
        if self.channel is not None and self.channel_reward_weight > 0:
            from comm.channel.base import Pose
            tx = Pose(position=self.robot.position.copy())
            bs = self.base_station_position if self.base_station_position \
                is not None else np.zeros(3)
            rx = Pose(position=np.asarray(bs, dtype=float))
            state = self.channel.generate(tx, rx, t=self.robot.state.t)
            if state.is_up:
                # Normalize: −80 dB … 0 dB → 0 … 1
                quality = np.clip((state.path_gain_db + 80.0) / 80.0, 0.0, 1.0)
            else:
                quality = 0.0
            reward += self.channel_reward_weight * quality

        truncated = (not terminated) and (self._steps >= self.max_steps)
        info = {"distance": new_dist, "collided": collided, "steps": self._steps}
        return self._observation(), float(reward), terminated, truncated, info

    @property
    def dt(self) -> float:
        return 0.1
