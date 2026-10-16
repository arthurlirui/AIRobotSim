"""
Robot-environment interaction in 3DGS scenes.

This module handles:
  - Terrain adaptation: adjusting foot placements to follow 3DGS terrain
  - Collision response: detecting and reacting to body/foot collisions
  - Sensing: ray-cast "lidar" into the 3DGS environment for perception
  - Interaction events: stepping on objects, pushing splats, etc.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from core.gaussian_splat_env import GaussianSplatEnv
from core.terrain import TerrainHeightField
from robots.robot_base import LeggedRobot


@dataclass
class InteractionEvent:
    """An interaction event between robot and environment."""
    type: str            # "foot_contact", "body_collision", "step_on", "push"
    leg_name: Optional[str] = None
    splat_index: Optional[int] = None
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    timestamp: float = 0.0
    details: dict = field(default_factory=dict)


class EnvironmentInteraction:
    """
    Manages all interactions between a robot and a 3DGS environment.
    """

    def __init__(self, robot: LeggedRobot, env: GaussianSplatEnv,
                 terrain: Optional[TerrainHeightField] = None):
        self.robot = robot
        self.env = env
        if terrain is None:
            stats = env.stats()
            bmin, bmax = stats["bounds_min"], stats["bounds_max"]
            terrain = TerrainHeightField(env,
                (bmin[0] - 1, bmax[0] + 1, bmin[1] - 1, bmax[1] + 1))
        self.terrain = terrain
        self.events: list[InteractionEvent] = []
        self.perturbed_splats: set[int] = set()  # splats that have been interacted with

    # ------------------------------------------------------------------
    # Terrain adaptation
    # ------------------------------------------------------------------

    def adapt_foot_to_terrain(self, foot_pos: np.ndarray,
                              is_swing: bool,
                              step_height: float = 0.06) -> np.ndarray:
        """
        Adjust a foot position to match the terrain height.
        For swing feet, only adjust near the end of the swing (descent phase).
        For stance feet, snap to ground.
        """
        ground_h = self.terrain.height_at(foot_pos[0], foot_pos[1])
        if is_swing:
            # If foot is descending (close to ground), snap to terrain
            if foot_pos[2] < ground_h + step_height * 0.3:
                return np.array([foot_pos[0], foot_pos[1], ground_h])
            return foot_pos
        else:
            return np.array([foot_pos[0], foot_pos[1], ground_h])

    def get_terrain_slope(self, x: float, y: float) -> float:
        """Get the slope angle at a position."""
        return self.terrain.slope_at(x, y)

    # ------------------------------------------------------------------
    # Collision detection and response
    # ------------------------------------------------------------------

    def check_foot_collision(self, leg_name: str) -> list[int]:
        """Check if a foot is inside any 3DGS splat."""
        if leg_name not in self.robot.state.foot_positions:
            return []
        foot = self.robot.state.foot_positions[leg_name]
        return self.env.collision(foot, radius=0.02)

    def check_body_collision(self) -> list[int]:
        """Check if the robot body collides with any splat."""
        return self.env.collision(self.robot.state.base_position, radius=0.15)

    def collision_response(self) -> dict[str, list[int]]:
        """
        Check all collisions and return a per-leg + body report.
        Also generates interaction events.
        """
        report = {"body": [], "feet": {}}
        t = self.robot.state.t

        # Body collision
        body_hits = self.check_body_collision()
        if body_hits:
            report["body"] = body_hits
            for idx in body_hits:
                self.events.append(InteractionEvent(
                    type="body_collision",
                    splat_index=idx,
                    position=self.robot.state.base_position.copy(),
                    timestamp=t,
                ))

        # Foot collisions
        for name in self.robot.legs:
            foot_hits = self.check_foot_collision(name)
            report["feet"][name] = foot_hits
            if foot_hits and self.robot.state.foot_contacts.get(name, False):
                for idx in foot_hits:
                    self.perturbed_splats.add(idx)
                    self.events.append(InteractionEvent(
                        type="foot_contact",
                        leg_name=name,
                        splat_index=idx,
                        position=self.robot.state.foot_positions[name].copy(),
                        timestamp=t,
                    ))

        return report

    # ------------------------------------------------------------------
    # Sensing: ray-cast lidar
    # ------------------------------------------------------------------

    def sense_lidar(self, n_rays: int = 36, max_range: float = 5.0,
                    height_offset: float = 0.2) -> np.ndarray:
        """
        Simulate a 360° lidar scan from the robot's base position.
        Returns an (n_rays, 2) array of (angle, distance).
        """
        origin = self.robot.state.base_position + np.array([0, 0, height_offset])
        results = np.zeros((n_rays, 2))
        for i in range(n_rays):
            angle = 2 * np.pi * i / n_rays
            direction = np.array([np.cos(angle), np.sin(angle), 0.0])
            dist = self.env.ray_cast(origin, direction, max_range)
            results[i] = [angle, dist if dist is not None else max_range]
        return results

    def sense_forward_obstacle(self, max_range: float = 3.0) -> Optional[float]:
        """Check for obstacles directly ahead. Returns distance or None."""
        origin = self.robot.state.base_position + np.array([0, 0, 0.1])
        direction = np.array([1.0, 0, 0])
        yaw = self.robot.state.base_orientation[2]
        from core.math_utils import rot_z
        direction = rot_z(yaw) @ direction
        return self.env.ray_cast(origin, direction, max_range)

    # ------------------------------------------------------------------
    # Interaction: pushing/deforming splats
    # ------------------------------------------------------------------

    def push_splat(self, splat_idx: int, force_dir: np.ndarray,
                   force_magnitude: float = 0.01):
        """
        Apply a push to a 3DGS splat (simulating the robot interacting with
        soft/deformable environment objects).
        """
        if splat_idx < 0 or splat_idx >= len(self.env.splats):
            return
        splat = self.env.splats[splat_idx]
        splat.position += force_dir * force_magnitude
        self.perturbed_splats.add(splat_idx)
        self.events.append(InteractionEvent(
            type="push",
            splat_index=splat_idx,
            position=splat.position.copy(),
            timestamp=self.robot.state.t,
            details={"force": force_magnitude, "dir": force_dir.tolist()},
        ))

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    def interaction_summary(self) -> dict:
        """Summarize all interactions that occurred."""
        type_counts: dict[str, int] = {}
        for e in self.events:
            type_counts[e.type] = type_counts.get(e.type, 0) + 1
        return {
            "total_events": len(self.events),
            "event_types": type_counts,
            "perturbed_splats": len(self.perturbed_splats),
            "body_collisions": type_counts.get("body_collision", 0),
            "foot_contacts": type_counts.get("foot_contact", 0),
        }
