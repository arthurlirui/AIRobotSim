"""
Trainable policy: a small MLP with a Gaussian action head.

Numpy-only forward / backward so the training loop runs without torch.
The network is a 2-hidden-layer MLP mapping observations (flatten) to
action-dim mean; log-std is a learned per-dimension parameter.  Actions
are squashed to the env's action range by clipping.
"""

from __future__ import annotations

import numpy as np


class MLPPolicy:
    """Gaussian MLP policy: obs → action ~ N(tanh(f(obs))·range, σ²)."""

    def __init__(self, obs_dim: int, act_dim: int,
                 hidden: int = 64, lr: float = 3e-3,
                 action_limit: float = 1.0, seed: int = 0):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        self.hidden = hidden
        self.lr = lr
        self.action_limit = action_limit
        self.rng = np.random.default_rng(seed)

        def init(*shape, scale=None):
            s = scale if scale is not None else np.sqrt(2.0 / shape[0])
            return self.rng.normal(0, s, shape)

        self.W1 = init(obs_dim, hidden)
        self.b1 = np.zeros(hidden)
        self.W2 = init(hidden, hidden)
        self.b2 = np.zeros(hidden)
        self.W3 = init(hidden, act_dim)
        self.b3 = np.zeros(act_dim)
        self.log_std = np.zeros(act_dim)

        # Gradients accumulated over a rollout
        self._grads: dict[str, np.ndarray] = {}

    # ------------------------------------------------------------------
    # Forward
    # ------------------------------------------------------------------

    def forward(self, obs: np.ndarray) -> dict:
        """Forward pass; caches intermediates for backprop."""
        h1 = np.tanh(obs @ self.W1 + self.b1)
        h2 = np.tanh(h1 @ self.W2 + self.b2)
        mu = h2 @ self.W3 + self.b3
        return {"h1": h1, "h2": h2, "mu": mu}

    def act(self, obs: np.ndarray, deterministic: bool = False) -> np.ndarray:
        """Sample an action (clipped to ±action_limit)."""
        f = self.forward(np.asarray(obs, dtype=float).ravel())
        if deterministic:
            a = np.tanh(f["mu"])
        else:
            std = np.exp(self.log_std)
            noise = self.rng.normal(0, 1, self.act_dim)
            a = np.tanh(f["mu"] + std * noise)
        return np.clip(a, -self.action_limit, self.action_limit)

    # ------------------------------------------------------------------
    # Policy-gradient updates (REINFORCE with a baseline)
    # ------------------------------------------------------------------

    def log_prob(self, obs: np.ndarray, action: np.ndarray) -> float:
        f = self.forward(np.asarray(obs, dtype=float).ravel())
        mu = np.tanh(f["mu"])
        std = np.exp(self.log_std)
        # tanh-squashed Gaussian: approximate log-prob ignoring the
        # tanh Jacobian (fine for the small-scale trainer here)
        lp = -0.5 * np.sum(((action - mu) / std) ** 2) \
             - np.sum(self.log_std) \
             - 0.5 * self.act_dim * np.log(2 * np.pi)
        return float(lp)

    def accumulate_gradient(self, obs: np.ndarray, action: np.ndarray,
                            weight: float):
        """
        Backprop of d(weight · log π(a|s)) / dθ, accumulated in-place.

        ``weight`` is the (advantage- or imitation-)weighted scalar.
        """
        obs = np.asarray(obs, dtype=float).ravel()
        f = self.forward(obs)
        mu = np.tanh(f["mu"])
        std = np.exp(self.log_std)
        # d(log-prob)/dmu through the tanh squash
        d_mu = weight * (action - mu) / (std ** 2) * (1 - mu ** 2)

        # Output layer
        g = {}
        g["W3"] = np.outer(f["h2"], d_mu)
        g["b3"] = d_mu.copy()
        d_h2 = d_mu @ self.W3.T * (1 - f["h2"] ** 2)

        # Hidden layer 2
        g["W2"] = np.outer(f["h1"], d_h2)
        g["b2"] = d_h2.copy()
        d_h1 = d_h2 @ self.W2.T * (1 - f["h1"] ** 2)

        # Hidden layer 1
        g["W1"] = np.outer(obs, d_h1)
        g["b1"] = d_h1.copy()

        # log_std gradient
        d_std = weight * (((action - mu) / std) ** 2 - 1.0)
        g["log_std"] = d_std

        for k, v in g.items():
            self._grads[k] = self._grads.get(k, 0) + v

    def apply_gradients(self, scale: float = 1.0):
        """
        Gradient-ascent step on the accumulated (log-prob) gradients,
        then clear them.  Ascent, not descent: the accumulated gradient
        is d(log π) / dθ, and both REINFORCE (weighted by advantage) and
        behavior cloning (weighted by +1) maximize log π.
        """
        for k in ("W1", "b1", "W2", "b2", "W3", "b3"):
            if k in self._grads:
                setattr(self, k, getattr(self, k) + self.lr * scale * self._grads[k])
        if "log_std" in self._grads:
            self.log_std = np.clip(self.log_std + self.lr * scale
                                   * self._grads["log_std"], -2.0, 0.5)
        self._grads.clear()
