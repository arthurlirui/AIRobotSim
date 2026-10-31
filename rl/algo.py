"""
Training algorithms: REINFORCE (with baseline) and DAgger-style imitation.

Both operate on :class:`rl.envs.NavigationEnv`-style environments and the
numpy :class:`rl.policy.MLPPolicy`, so the full train → evaluate loop runs
with zero ML-framework dependencies.
"""

from __future__ import annotations

import numpy as np
from typing import Optional

from rl.policy import MLPPolicy


class ReinforceTrainer:
    """
    REINFORCE with a moving-average baseline.

    Each iteration collects ``batch_size`` episodes, computes
    discounted returns, normalizes advantages, and takes one gradient
    step per episode.
    """

    def __init__(self, env, policy: Optional[MLPPolicy] = None,
                 gamma: float = 0.99, batch_size: int = 8, seed: int = 0):
        self.env = env
        self.policy = policy or MLPPolicy(
            obs_dim=env.observation_dim, act_dim=env.action_dim, seed=seed)
        self.gamma = gamma
        self.batch_size = batch_size
        self._baseline = 0.0
        self.rng = np.random.default_rng(seed)

    def _collect_episode(self) -> tuple[list, float]:
        obs = self.env.reset()
        trajectory = []
        done = False
        total = 0.0
        while not done:
            action = self.policy.act(obs)
            next_obs, reward, terminated, truncated, _ = self.env.step(action)
            trajectory.append((obs, action, reward))
            total += reward
            obs = next_obs
            done = terminated or truncated
        return trajectory, total

    def train(self, iterations: int = 50, log_every: int = 10) -> list[float]:
        """Train; returns per-iteration mean episode returns."""
        history = []
        for it in range(iterations):
            returns_all = []
            # First pass: collect the batch and per-timestep advantages
            batch = []  # (obs, action, advantage)
            for _ in range(self.batch_size):
                traj, total = self._collect_episode()
                returns_all.append(total)
                # Discounted returns per timestep
                G = 0.0
                discounted = []
                for (_, _, r) in reversed(traj):
                    G = r + self.gamma * G
                    discounted.append(G)
                discounted.reverse()
                # Baseline-subtracted advantages
                self._baseline = 0.9 * self._baseline + 0.1 * np.mean(discounted)
                for (o, a, _), g in zip(traj, discounted):
                    batch.append((o, a, g - self._baseline))
            # Normalize advantages over the whole batch (variance control)
            adv = np.array([b[2] for b in batch])
            std = adv.std() + 1e-8
            for o, a, g in batch:
                self.policy.accumulate_gradient(o, a, float(g / std))
            self.policy.apply_gradients(scale=1.0 / self.batch_size)
            mean_ret = float(np.mean(returns_all))
            history.append(mean_ret)
            if log_every and (it + 1) % log_every == 0:
                print(f"  [REINFORCE] iter {it + 1}/{iterations} "
                      f"mean return: {mean_ret:.2f}")
        return history

    def evaluate(self, episodes: int = 5) -> float:
        """Mean episode return with deterministic actions."""
        totals = []
        for _ in range(episodes):
            obs = self.env.reset()
            done, total = False, 0.0
            while not done:
                action = self.policy.act(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = self.env.step(action)
                total += reward
                done = terminated or truncated
            totals.append(total)
        return float(np.mean(totals))


class ScriptedNavExpert:
    """
    Hand-crafted expert for point-goal navigation (imitation teacher).

    Policy: steer toward the goal; slow down when a forward ray reports
    a close obstacle and veer toward the clearer side.  Produces the
    (observation → expert action) pairs the imitation trainer distills.
    """

    def __init__(self, env):
        self.env = env

    def act(self, obs: np.ndarray) -> np.ndarray:
        # obs = [goal_body(3), vel(3), rays(n)]
        n = self.env.n_rays
        rays = obs[6:6 + n]
        goal_x, goal_y = obs[0], obs[1]

        # Desired heading is toward the goal; obstacle avoidance via the
        # ray with the most clearance on each half
        mid = n // 2
        left_clear = np.mean(rays[:mid]) if mid else 1.0
        right_clear = np.mean(rays[mid + 1:]) if mid else 1.0
        front = rays[mid] if n % 2 else np.mean(rays)

        steer = 0.0
        speed = 1.0
        if front < 0.4:  # obstacle ahead
            speed = 0.3
            steer = 1.0 if left_clear > right_clear else -1.0
        # Combine goal-seeking (small proportional steer) with avoidance
        steer += np.clip(-goal_y * 2.0, -0.5, 0.5)
        # Slow near goal
        dist = np.hypot(goal_x, goal_y)
        if dist < 0.05:
            speed = 0.0

        if self.env.action_dim == 2:
            return np.array([speed, np.clip(steer, -1, 1)])
        return np.array([np.clip(speed, -1, 1),
                         np.clip(-goal_y, -1, 1) * 0.5,
                         np.clip(obs[2], -1, 1) * 0.5])


class ImitationTrainer:
    """
    DAgger-style imitation learning.

    Iteration: roll out the *current* student, label every visited state
    with the *expert's* action, and update the student by weighted
    maximum-likelihood (behavior cloning on the aggregated dataset).
    """

    def __init__(self, env, expert: ScriptedNavExpert,
                 policy: Optional[MLPPolicy] = None, seed: int = 0):
        self.env = env
        self.expert = expert
        self.policy = policy or MLPPolicy(
            obs_dim=env.observation_dim, act_dim=env.action_dim,
            lr=5e-3, seed=seed)

    def train(self, iterations: int = 20, episodes_per_iter: int = 3,
              log_every: int = 5) -> list[float]:
        """Train; returns per-iteration mean expert-student action MSE."""
        history = []
        for it in range(iterations):
            mse_total, n_samples = 0.0, 0
            for _ in range(episodes_per_iter):
                obs = self.env.reset()
                done = False
                while not done:
                    student_action = self.policy.act(obs)
                    expert_action = self.expert.act(obs)
                    mse_total += float(np.mean((student_action - expert_action) ** 2))
                    n_samples += 1
                    # Behavior-cloning update on this transition (before
                    # stepping — the label belongs to the current obs)
                    self.policy.accumulate_gradient(obs, expert_action, weight=1.0)
                    # DAgger: label with expert, execute a mix so the
                    # student keeps visiting its own distribution
                    beta = max(0.0, 1.0 - it / max(iterations, 1))
                    action = beta * expert_action + (1 - beta) * student_action
                    obs, _, terminated, truncated, _ = self.env.step(action)
                    done = terminated or truncated
            if n_samples:
                self.policy.apply_gradients(scale=1.0 / n_samples)
            mse = mse_total / max(n_samples, 1)
            history.append(mse)
            if log_every and (it + 1) % log_every == 0:
                print(f"  [Imitation] iter {it + 1}/{iterations} "
                      f"expert-student MSE: {mse:.4f}")
        return history
