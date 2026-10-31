"""
Reinforcement learning and imitation learning for the simulation platform.

The package provides three layers:

1. **Gym-style environments** (``rl/envs``) wrapping the simulation core —
   point-goal navigation for wheeled robots and UAVs inside the 3DGS
   environment, optionally with a communication-reward term so swarm /
   comm-constrained policies can be trained.
2. **Training algorithms** (``rl/algo``): a dependency-free REINFORCE /
   PPO-style policy-gradient trainer that runs on numpy alone, plus
   optional DAgger-style imitation learning from a scripted expert.
3. **Policies** (``rl/policy``): a small MLP with Gaussian actions that
   serves as the trainable policy for both paradigms.

Heavy frameworks (Stable-Baselines3, Gymnasium, IsaacLab) are deliberately
**not** required: the environments expose the canonical
``reset() / step(action) -> (obs, reward, terminated, truncated, info)``
API so they can be dropped into any standard trainer, while the built-in
trainer proves the loop end-to-end with zero extra dependencies.
"""

from rl.policy import MLPPolicy
from rl.envs import NavigationEnv
from rl.algo import ReinforceTrainer, ImitationTrainer, ScriptedNavExpert

__all__ = [
    "MLPPolicy", "NavigationEnv",
    "ReinforceTrainer", "ImitationTrainer", "ScriptedNavExpert",
]
