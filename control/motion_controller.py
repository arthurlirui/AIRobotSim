"""
Motion controller: orchestrates gait planning, IK, and base movement.

The controller:
  1. Gets a gait command (foot targets in base frame) from the gait planner
  2. Converts foot targets to world frame
  3. Adapts foot targets to terrain (queries 3DGS env / height-field)
  4. Solves IK for each leg
  5. Updates robot state (base velocity → position integration)
  6. Checks for collisions and stability
"""

from __future__ import annotations

import numpy as np
from typing import Optional

from core.math_utils import Transform
from core.gaussian_splat_env import GaussianSplatEnv
from core.terrain import TerrainHeightField
from robots.robot_base import LeggedRobot
from control.gait_planner import GaitPlanner, GaitType, GaitCommand


class MotionController:
    """
    Whole-body motion controller for a legged robot in a 3DGS environment.
    """

    def __init__(self, robot: LeggedRobot, env: GaussianSplatEnv,
                 gait_type: GaitType = GaitType.TRIPOD,
                 stride_length: float = 0.12, step_height: float = 0.06,
                 cycle_time: float = 1.0,
                 terrain_resolution: float = 0.1):
        self.robot = robot
        self.env = env
        self.gait_planner = GaitPlanner(gait_type, stride_length, step_height, cycle_time)

        # Pre-compute terrain height-field for fast queries
        stats = env.stats()
        bmin = stats["bounds_min"]
        bmax = stats["bounds_max"]
        bounds = (bmin[0] - 1, bmax[0] + 1, bmin[1] - 1, bmax[1] + 1)
        self.terrain = TerrainHeightField(env, bounds, terrain_resolution)

        # Base velocity command (set by user)
        self.target_velocity: np.ndarray = np.zeros(3)    # linear (m/s)
        self.target_yaw_rate: float = 0.0                 # angular (rad/s)

        # Logging
        self.history: list[dict] = []

    # ------------------------------------------------------------------
    # Step the controller forward by dt
    # ------------------------------------------------------------------

    def step(self, dt: float):
        """Advance the simulation by one time step."""
        # 1. Advance gait phase
        self.gait_planner.advance(dt)

        # 2. Get gait command (foot targets in base frame)
        leg_names = list(self.robot.legs.keys())
        leg_offsets = {name: self.robot.legs[name].hip_offset for name in leg_names}
        cmd = self.gait_planner.get_command(leg_names, leg_offsets)

        # 3. Convert to world frame and adapt to terrain
        Tb = self.robot.base_transform()
        world_targets: dict[str, np.ndarray] = {}
        for name in leg_names:
            target_base = cmd.foot_targets[name]
            target_world = Tb.apply(target_base.reshape(1, 3))[0]

            # Terrain adaptation: project foot onto ground for swing legs
            if not cmd.contacts[name]:
                # Swing leg: keep the X/Y from gait, but snap Z to terrain
                # when the foot is near the ground (end of swing)
                ground_h = self.terrain.height_at(target_world[0], target_world[1])
                # Only adapt the final part of the swing
                if target_world[2] < ground_h + self.gait_planner.step_height * 0.5:
                    target_world[2] = max(target_world[2], ground_h)
            else:
                # Stance leg: place foot on terrain
                ground_h = self.terrain.height_at(target_world[0], target_world[1])
                target_world[2] = ground_h

            world_targets[name] = target_world

        # 4. Solve IK for each leg
        for name in leg_names:
            angles = self.robot.leg_inverse_kinematics(name, world_targets[name])
            if angles is not None:
                self.robot.set_joint_angles(name, angles)
            self.robot.state.foot_contacts[name] = cmd.contacts[name]

        # 5. Update base position from velocity command
        self.robot.state.base_velocity = self.target_velocity
        self.robot.state.base_angular_velocity = np.array([0, 0, self.target_yaw_rate])
        self.robot.apply_velocity(dt)

        # 6. Adjust base height to terrain (follow ground)
        ground_h = self.terrain.height_at(
            self.robot.state.base_position[0],
            self.robot.state.base_position[1])
        # Smoothly follow terrain
        target_z = ground_h + self.robot.base_height
        self.robot.state.base_position[2] += (target_z - self.robot.state.base_position[2]) * 0.1

        # 7. Update foot positions
        self.robot.update_foot_positions()

        # 8. Check stability and collisions
        stable = self.robot.is_stable()
        collisions = self._check_body_collision()

        # 9. Log
        self.history.append({
            "t": self.robot.state.t,
            "base_pos": self.robot.state.base_position.copy(),
            "stable": stable,
            "collisions": collisions,
            "contacts": dict(self.robot.state.foot_contacts),
        })

    def _check_body_collision(self) -> list[int]:
        """Check if the robot body collides with any 3DGS splat."""
        return self.env.collision(self.robot.state.base_position, radius=0.1)

    # ------------------------------------------------------------------
    # Command interface
    # ------------------------------------------------------------------

    def walk_forward(self, speed: float = 0.2):
        """Command the robot to walk forward at the given speed (m/s)."""
        self.target_velocity = np.array([speed, 0, 0])
        self.target_yaw_rate = 0.0

    def turn(self, yaw_rate: float = 0.3):
        """Command the robot to turn."""
        self.target_velocity = np.zeros(3)
        self.target_yaw_rate = yaw_rate

    def stop(self):
        """Stop the robot."""
        self.target_velocity = np.zeros(3)
        self.target_yaw_rate = 0.0

    # ------------------------------------------------------------------
    # Run a full simulation
    # ------------------------------------------------------------------

    def simulate(self, duration: float, dt: float = 0.01,
                 speed: float = 0.2) -> dict:
        """
        Run a simulation for `duration` seconds.
        Returns summary statistics.
        """
        self.walk_forward(speed)
        n_steps = int(duration / dt)

        for i in range(n_steps):
            self.step(dt)

        return self._summary()

    def _summary(self) -> dict:
        if not self.history:
            return {"steps": 0}
        positions = np.array([h["base_pos"] for h in self.history])
        stabilities = [h["stable"] for h in self.history]
        return {
            "steps": len(self.history),
            "distance": np.linalg.norm(positions[-1] - positions[0]),
            "final_pos": positions[-1],
            "stability_ratio": sum(stabilities) / len(stabilities),
            "avg_height": positions[:, 2].mean(),
            "height_std": positions[:, 2].std(),
            "t": self.robot.state.t,
        }
