"""
Gait controller: make a compiled SimRobot walk inside the 3DGS world.

The controller groups the design's contact points into alternating gait
groups (tripod for 6 feet, trot for 4, alternating for 2) and executes
the classic swing/stance cycle per group:

  - **swing** feet follow a sine-arc trajectory from the previous
    foothold to the next one (``stride`` along the commanded heading),
    landing on the *actual* terrain height of the reconstruction
  - **stance** feet stay pinned to their landing position (world-frame
    IK target) while the body advances over them

Each control step computes per-foot world targets, solves per-leg IK at
the foot contact point, merges the per-chain joint solutions, and
PD-steps the robot. Foot landings use ``GaussianSplatEnv.height_at`` —
the designed morphology interacts with the reconstructed terrain.
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from design.builder import SimRobot

if TYPE_CHECKING:
    from core.gaussian_splat_env import GaussianSplatEnv


def gait_groups(foot_names: list[str], pattern: str = "auto") -> list[list[str]]:
    """
    Split feet into alternating groups.

    pattern="auto": tripod (3 groups of 2) for 6 feet; trot (diagonal
    pairs) for 4; alternating for 2. Any other count → round-robin pairs.
    """
    n = len(foot_names)
    if n == 0:
        return []
    if pattern == "auto":
        if n == 6:
            return [foot_names[0::3], foot_names[1::3], foot_names[2::3]]
        if n == 4:
            return [[foot_names[0], foot_names[3]],
                    [foot_names[1], foot_names[2]]]
        if n == 2:
            return [[foot_names[0]], [foot_names[1]]]
    if n % 2 == 0:
        return [foot_names[0::2], foot_names[1::2]]
    return [[f] for f in foot_names]


class GaitController:
    """
    Open-loop gait executor over a SimRobot's contact points.

    Args:
        robot: compiled SimRobot with contact points
        env: the GaussianSplatEnv the robot lives in
        stride: forward distance per swing (m)
        lift: swing apex height (m)
        period: full gait cycle duration (s)
    """

    def __init__(self, robot: SimRobot, env: "GaussianSplatEnv",
                 stride: float = 0.08, lift: float = 0.06,
                 period: float = 1.0):
        self.robot = robot
        self.env = env
        self.stride = stride
        self.lift = lift
        self.period = period

        self.feet = [c.name for c in robot.contacts]
        if not self.feet:
            raise ValueError("GaitController needs a robot with contact points")
        self.groups = gait_groups(self.feet)
        self.phase = 0.0                     # 0..1 cycle phase

        # Per-foot info: contact point, leg link, joint chain indices
        self._foot_cp = {c.name: c for c in robot.contacts}
        self._foot_link = {c.name: c.link for c in robot.contacts}
        self._chain_idx: dict[str, list[int]] = {
            f: self._leg_chain(f) for f in self.feet}

        # Current + previous foothold (world) per foot
        self._landed: dict[str, np.ndarray] = {}
        self._prev: dict[str, np.ndarray] = {}
        self._init_footholds()

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _leg_chain(self, foot: str) -> list[int]:
        """Indices (into actuated joints) of the root→foot joint chain."""
        link = self._foot_link[foot]
        chain, cur = [], link
        while cur in self.robot._parent_joint:
            j = self.robot._parent_joint[cur]
            if j.type == "revolute":
                for i, aj in enumerate(self.robot.actuated):
                    if aj.name == j.name:
                        chain.append(i)
                        break
            cur = j.parent
        return chain

    def _foot_world(self, transforms, foot: str) -> np.ndarray:
        cp = self._foot_cp[foot]
        T = transforms[cp.link]
        return T.R @ cp.offset + T.t

    def _init_footholds(self):
        transforms = self.robot.forward_kinematics()
        for f in self.feet:
            w = self._foot_world(transforms, f)
            self._landed[f] = w.copy()
            self._prev[f] = w.copy()

    # ------------------------------------------------------------------
    # Per-step control
    # ------------------------------------------------------------------

    def _group_of(self, foot: str) -> int:
        for gi, grp in enumerate(self.groups):
            if foot in grp:
                return gi
        return 0

    def step(self, dt: float, heading: float = 0.0, speed: float = 1.0):
        """
        Advance the gait by ``dt``. ``heading`` is the desired travel
        direction (world-frame yaw, rad); ``speed`` scales the stride.
        """
        n_groups = len(self.groups)
        swing_frac = 1.0 / n_groups      # each group swings 1/n of cycle
        prev_phase = self.phase
        self.phase = (self.phase + dt / self.period) % 1.0

        heading_vec = np.array([np.cos(heading), np.sin(heading), 0.0])

        # Start-of-swing detection per group: phase crossed g_start
        def _crossed(g_start: float) -> bool:
            if self.phase < prev_phase:   # wrap: treat as full crossing
                return True
            return prev_phase < g_start <= self.phase

        # Assemble the joint command by merging per-leg IK solutions
        q_desired = self.robot.q.copy()
        transforms = self.robot.forward_kinematics()

        for foot in self.feet:
            gi = self._group_of(foot)
            g_start = gi * swing_frac
            in_swing = g_start <= self.phase < g_start + swing_frac

            if _crossed(g_start) or (self.phase < prev_phase and gi == 0):
                # A new swing begins: previous target becomes the source
                self._prev[foot] = self._landed[foot].copy()
                # Next foothold: stride ahead of the landing point
                nxt = self._landed[foot] + heading_vec * self.stride * speed
                nxt[2] = self.env.height_at(nxt[0], nxt[1])
                self._landed[foot] = nxt

            if in_swing:
                swing_t = (self.phase - g_start) / swing_frac
                # Sine arc from prev foothold to the new one
                s = 0.5 * (1 - np.cos(np.pi * swing_t))   # smooth 0→1
                goal = self._prev[foot] + \
                    (self._landed[foot] - self._prev[foot]) * s
                goal[2] += self.lift * np.sin(np.pi * swing_t)
            else:
                # Stance: pinned to the landing point
                goal = self._landed[foot].copy()

            # IK at the foot contact point; merge only this leg's chain
            sol = self.robot.solve_ik(self._foot_link[foot], goal,
                                      max_iter=40,
                                      offset=self._foot_cp[foot].offset)
            if sol is not None:
                for i in self._chain_idx[foot]:
                    q_desired[i] = sol[i]

        # PD-step toward the merged joint targets
        self.robot.step(dt, q_desired)

        # Body propulsion: advance the base over the stance feet
        self.robot.base_position += heading_vec * (self.stride * speed) \
            * dt / self.period * n_groups

        # Keep grounded: lowest active contact pins the body height
        self.robot.update_contacts(self.env)
        active_z = [self.robot.contact_positions[f][2]
                    for f, a in self.robot.contact_active.items() if a]
        if active_z:
            target_z = float(np.mean(active_z))
            # Body hover height from the design's neutral stance
            self.robot.base_position[2] += 0.5 * (target_z - target_z)

        return q_desired
