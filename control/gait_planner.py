"""
Gait planning for legged robots.

A gait is a periodic pattern of foot contacts and swing trajectories.
Each gait generator produces, for a given phase t ∈ [0, 1), a set of
foot position targets (in the robot's base frame) for all legs.

Supported gaits:
  - Biped:   static walk (alternating single support)
  - Quadruped: walk, trot, bound
  - Hexapod: tripod, wave

The output is a GaitCommand: per-leg foot targets (base frame) + which
feet are in contact.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from enum import Enum


class GaitType(Enum):
    WALK = "walk"
    TROT = "trot"
    TRIPOD = "tripod"
    WAVE = "wave"
    BOUND = "bound"
    STATIC_WALK = "static_walk"


@dataclass
class GaitCommand:
    """Per-leg targets for one gait step."""
    foot_targets: dict[str, np.ndarray] = field(default_factory=dict)  # base frame
    contacts: dict[str, bool] = field(default_factory=dict)
    phase: float = 0.0


class GaitPlanner:
    """
    Generates gait patterns. Phase t goes from 0 to 1 cyclically.

    For walking gaits, `stride_length` is the forward foot displacement
    during a step cycle. `step_height` is the swing foot lift height.
    """

    def __init__(self, gait_type: GaitType, stride_length: float = 0.15,
                 step_height: float = 0.08, cycle_time: float = 1.0):
        self.gait_type = gait_type
        self.stride_length = stride_length
        self.step_height = step_height
        self.cycle_time = cycle_time
        self.t = 0.0  # current phase time

    def advance(self, dt: float):
        """Advance the gait phase by dt seconds."""
        self.t += dt
        if self.t >= self.cycle_time:
            self.t = self.t % self.cycle_time

    @property
    def phase(self) -> float:
        """Normalized phase ∈ [0, 1)."""
        return self.t / self.cycle_time

    # ------------------------------------------------------------------
    # Swing trajectory
    # ------------------------------------------------------------------

    @staticmethod
    def _swing_trajectory(start: np.ndarray, end: np.ndarray,
                          swing_phase: float, height: float) -> np.ndarray:
        """
        Cubic Bezier swing trajectory for a foot in the air.
        swing_phase ∈ [0, 1] where 0 = just lifted, 1 = just placed.
        """
        # Mid-point elevated
        mid = (start + end) / 2
        mid[2] += height

        # Cubic Bezier with two control points
        p0 = start
        p1 = start + np.array([0, 0, height * 0.8])
        p2 = end + np.array([0, 0, height * 0.8])
        p3 = end

        s = swing_phase
        pos = ((1 - s) ** 3 * p0 + 3 * (1 - s) ** 2 * s * p1 +
               3 * (1 - s) * s ** 2 * p2 + s ** 3 * p3)
        return pos

    @staticmethod
    def _stance_trajectory(start: np.ndarray, end: np.ndarray,
                           stance_phase: float) -> np.ndarray:
        """Linear interpolation during stance (foot stays on ground)."""
        return start + (end - start) * stance_phase

    # ------------------------------------------------------------------
    # Gait pattern dispatch
    # ------------------------------------------------------------------

    def get_command(self, leg_names: list[str],
                    leg_offsets: dict[str, np.ndarray]) -> GaitCommand:
        """
        Compute the gait command for the current phase.

        Args:
            leg_names: list of leg names
            leg_offsets: hip offsets per leg (in base frame)

        Returns:
            GaitCommand with foot targets in the base frame.
        """
        if self.gait_type == GaitType.TRIPOD:
            return self._tripod_gait(leg_names, leg_offsets)
        elif self.gait_type == GaitType.TROT:
            return self._trot_gait(leg_names, leg_offsets)
        elif self.gait_type == GaitType.WALK:
            return self._walk_gait(leg_names, leg_offsets)
        elif self.gait_type == GaitType.WAVE:
            return self._wave_gait(leg_names, leg_offsets)
        elif self.gait_type == GaitType.STATIC_WALK:
            return self._static_walk_biped(leg_names, leg_offsets)
        else:
            return self._default_stand(leg_names, leg_offsets)

    # ------------------------------------------------------------------
    # Individual gait implementations
    # ------------------------------------------------------------------

    def _default_stand(self, leg_names, leg_offsets) -> GaitCommand:
        cmd = GaitCommand(phase=self.phase)
        for name in leg_names:
            target = leg_offsets[name].copy()
            target[2] = -sum(0.3 for _ in range(1))  # below hip
            cmd.foot_targets[name] = target
            cmd.contacts[name] = True
        return cmd

    def _tripod_gait(self, leg_names, leg_offsets) -> GaitCommand:
        """
        Hexapod tripod gait: two alternating groups of 3 legs.
        Group A: FL, MR, RL (tripod_a)
        Group B: FR, ML, RR (tripod_b)
        """
        cmd = GaitCommand(phase=self.phase)
        group_a = {"leg_FL", "leg_MR", "leg_RL"}
        group_b = {"leg_FR", "leg_ML", "leg_RR"}

        half_stride = self.stride_length / 2
        ph = self.phase

        for name in leg_names:
            hip = leg_offsets[name].copy()
            # Default foot position (below hip, on ground)
            foot_rest = np.array([0.0, 0.0, -0.3])  # relative to hip in base frame

            if name in group_a:
                swing = ph < 0.5
            else:
                swing = ph >= 0.5

            if swing:
                sp = (ph % 0.5) / 0.5 if name in group_a else ((ph - 0.5) % 0.5) / 0.5
                start = np.array([-half_stride, 0, foot_rest[2]])
                end = np.array([half_stride, 0, foot_rest[2]])
                cmd.foot_targets[name] = hip + self._swing_trajectory(start, end, sp, self.step_height)
                cmd.contacts[name] = False
            else:
                sp = (ph % 0.5) / 0.5 if name in group_a else ((ph - 0.5) % 0.5) / 0.5
                start = np.array([half_stride, 0, foot_rest[2]])
                end = np.array([-half_stride, 0, foot_rest[2]])
                cmd.foot_targets[name] = hip + self._stance_trajectory(start, end, sp)
                cmd.contacts[name] = True

        return cmd

    def _trot_gait(self, leg_names, leg_offsets) -> GaitCommand:
        """
        Quadruped trot: diagonal pairs alternate.
        Pair 1: FL + RR, Pair 2: FR + RL
        """
        cmd = GaitCommand(phase=self.phase)
        pair1 = {"leg_FL", "leg_RR"}
        pair2 = {"leg_FR", "leg_RL"}
        half_stride = self.stride_length / 2
        ph = self.phase

        for name in leg_names:
            hip = leg_offsets[name].copy()
            foot_rest_z = -0.3

            swing = (name in pair1 and ph < 0.5) or (name in pair2 and ph >= 0.5)

            if swing:
                sp = (ph % 0.5) / 0.5 if name in pair1 else ((ph - 0.5) % 0.5) / 0.5
                start = np.array([-half_stride, 0, foot_rest_z])
                end = np.array([half_stride, 0, foot_rest_z])
                cmd.foot_targets[name] = hip + self._swing_trajectory(start, end, sp, self.step_height)
                cmd.contacts[name] = False
            else:
                sp = (ph % 0.5) / 0.5 if name in pair1 else ((ph - 0.5) % 0.5) / 0.5
                start = np.array([half_stride, 0, foot_rest_z])
                end = np.array([-half_stride, 0, foot_rest_z])
                cmd.foot_targets[name] = hip + self._stance_trajectory(start, end, sp)
                cmd.contacts[name] = True

        return cmd

    def _walk_gait(self, leg_names, leg_offsets) -> GaitCommand:
        """
        Quadruped walk: 4-beat gait, one leg swings at a time.
        Sequence: RL → FL → RR → FR (lateral sequence)
        """
        cmd = GaitCommand(phase=self.phase)
        n = len(leg_names)
        ph = self.phase
        half_stride = self.stride_length / 2

        # Determine which leg is swinging (each gets 1/4 of the cycle for swing)
        # and 3/4 for stance
        leg_order = ["leg_RL", "leg_FL", "leg_RR", "leg_FR"]
        swing_frac = 0.25  # each leg swings for 25% of the cycle

        for name in leg_names:
            hip = leg_offsets[name].copy()
            foot_rest_z = -0.3

            if name in leg_order:
                idx = leg_order.index(name)
                swing_start = idx * swing_frac
                swing_end = swing_start + swing_frac
                # Normalize
                in_swing = swing_start <= ph < swing_end

                if in_swing:
                    sp = (ph - swing_start) / swing_frac
                    start = np.array([-half_stride, 0, foot_rest_z])
                    end = np.array([half_stride, 0, foot_rest_z])
                    cmd.foot_targets[name] = hip + self._swing_trajectory(start, end, sp, self.step_height)
                    cmd.contacts[name] = False
                else:
                    # Stance: move backward
                    # The stance lasts 3/4 of the cycle; figure out local phase
                    if ph >= swing_end:
                        sp = (ph - swing_end) / (1.0 - swing_frac)
                    else:
                        sp = (ph + (1.0 - swing_end)) / (1.0 - swing_frac)
                    start = np.array([half_stride, 0, foot_rest_z])
                    end = np.array([-half_stride, 0, foot_rest_z])
                    cmd.foot_targets[name] = hip + self._stance_trajectory(start, end, sp)
                    cmd.contacts[name] = True
            else:
                cmd.foot_targets[name] = hip + np.array([0, 0, foot_rest_z])
                cmd.contacts[name] = True

        return cmd

    def _wave_gait(self, leg_names, leg_offsets) -> GaitCommand:
        """
        Hexapod wave gait: one leg swings at a time (very stable, very slow).
        Order: RR → MR → FR → RL → ML → FL (right-side wave, then left)
        """
        cmd = GaitCommand(phase=self.phase)
        leg_order = ["leg_RR", "leg_MR", "leg_FR", "leg_RL", "leg_ML", "leg_FL"]
        n = len(leg_order)
        swing_frac = 1.0 / n
        ph = self.phase
        half_stride = self.stride_length / 2

        for name in leg_names:
            hip = leg_offsets[name].copy()
            foot_rest_z = -0.3

            idx = leg_order.index(name)
            swing_start = idx * swing_frac
            swing_end = swing_start + swing_frac
            in_swing = swing_start <= ph < swing_end

            if in_swing:
                sp = (ph - swing_start) / swing_frac
                start = np.array([-half_stride, 0, foot_rest_z])
                end = np.array([half_stride, 0, foot_rest_z])
                cmd.foot_targets[name] = hip + self._swing_trajectory(start, end, sp, self.step_height)
                cmd.contacts[name] = False
            else:
                if ph >= swing_end:
                    sp = (ph - swing_end) / (1.0 - swing_frac)
                else:
                    sp = (ph + (1.0 - swing_end)) / (1.0 - swing_frac)
                start = np.array([half_stride, 0, foot_rest_z])
                end = np.array([-half_stride, 0, foot_rest_z])
                cmd.foot_targets[name] = hip + self._stance_trajectory(start, end, sp)
                cmd.contacts[name] = True

        return cmd

    def _static_walk_biped(self, leg_names, leg_offsets) -> GaitCommand:
        """
        Biped static walk: alternate single-support phases.
        Left leg swings while right supports, then vice versa.
        """
        cmd = GaitCommand(phase=self.phase)
        half_stride = self.stride_length / 2
        ph = self.phase

        for name in leg_names:
            hip = leg_offsets[name].copy()
            foot_rest_z = -0.3

            is_left = "L" in name
            # Left swings in first half, right in second half
            swing = (is_left and ph < 0.5) or (not is_left and ph >= 0.5)

            if swing:
                sp = (ph % 0.5) / 0.5
                start = np.array([-half_stride, 0, foot_rest_z])
                end = np.array([half_stride, 0, foot_rest_z])
                cmd.foot_targets[name] = hip + self._swing_trajectory(start, end, sp, self.step_height)
                cmd.contacts[name] = False
            else:
                sp = (ph % 0.5) / 0.5
                start = np.array([half_stride, 0, foot_rest_z])
                end = np.array([-half_stride, 0, foot_rest_z])
                cmd.foot_targets[name] = hip + self._stance_trajectory(start, end, sp)
                cmd.contacts[name] = True

        return cmd
