"""
Test 12: RL / imitation learning module.

Verifies:
  - NavigationEnv obeys the Gym step contract and observation/action dims
  - episode termination on goal reach, truncation at max_steps
  - REINFORCE improves navigation returns (control group: late-phase
    mean return must exceed early-phase mean — the platform's
    "mandatory control group" philosophy applied to learning)
  - imitation learning reduces expert-student MSE
  - communication-coupled reward changes the reward signal
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv
from robots import WheeledRobot, UAV
from comm import FreeSpaceChannel
from rl import NavigationEnv, ReinforceTrainer, ImitationTrainer, \
    ScriptedNavExpert, MLPPolicy


def run():
    env3d = GaussianSplatEnv.create_flat_ground(size=30.0, resolution=4)

    # ------------------------------------------------------------------
    print("  [1] Gym-style step contract")
    robot = WheeledRobot(base_position=np.array([-8.0, 0.0, 0.08]))
    nav = NavigationEnv(robot, env3d, goal=np.array([8.0, 0.0, 0.0]),
                        max_steps=50, n_rays=8, seed=0)
    obs = nav.reset()
    assert obs.shape == (nav.observation_dim,)
    assert nav.observation_dim == 3 + 3 + 8
    assert nav.action_dim == 2
    out = nav.step(np.array([1.0, 0.0]))
    assert len(out) == 5, "must return (obs, reward, terminated, truncated, info)"
    obs, reward, terminated, truncated, info = out
    assert obs.shape == (nav.observation_dim,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool) and isinstance(truncated, bool)
    assert "distance" in info
    print("      OK — reset/step contract + info dict")

    # ------------------------------------------------------------------
    print("  [2] Termination on goal reach")
    robot = WheeledRobot(base_position=np.array([7.5, 0.0, 0.08]))
    nav2 = NavigationEnv(robot, env3d, goal=np.array([8.0, 0.0, 0.0]),
                         reach_radius=0.5, max_steps=100)
    nav2.reset()
    _, _, terminated, _, info = nav2.step(np.array([1.0, 0.0]))
    assert terminated and info["distance"] < 0.5
    print("      OK — terminates when within reach radius")

    # ------------------------------------------------------------------
    print("  [3] Truncation at max_steps")
    robot = WheeledRobot(base_position=np.array([-8.0, 0.0, 0.08]))
    nav3 = NavigationEnv(robot, env3d, goal=np.array([8.0, 0.0, 0.0]),
                         max_steps=5)
    nav3.reset()
    done = False
    steps = 0
    while not done:
        _, _, term, trunc, _ = nav3.step(np.array([0.0, 0.0]))
        done = term or trunc
        steps += 1
    assert steps == 5 and not term
    print("      OK — truncates exactly at max_steps")

    # ------------------------------------------------------------------
    print("  [4] REINFORCE improves returns (control group)")
    robot = WheeledRobot(base_position=np.array([-8.0, 0.0, 0.08]))
    nav4 = NavigationEnv(robot, env3d, goal=np.array([8.0, 0.0, 0.0]),
                         max_steps=100, n_rays=8, seed=0)
    trainer = ReinforceTrainer(nav4, batch_size=16, seed=0)
    history = trainer.train(iterations=25, log_every=0)
    early = float(np.mean(history[:5]))
    late = float(np.mean(history[-5:]))
    assert late > early, \
        f"REINFORCE must improve: early={early:.2f} late={late:.2f}"
    assert late > 0.0, f"late-phase return should be positive: {late:.2f}"
    print(f"      OK — mean return early={early:.2f} → late={late:.2f}")

    # ------------------------------------------------------------------
    print("  [5] Imitation learning reduces expert-student MSE")
    uav = UAV(base_position=np.array([-8.0, 0.0, 3.0]))
    nav5 = NavigationEnv(uav, env3d, goal=np.array([8.0, 0.0, 3.0]),
                         max_steps=60, n_rays=8, seed=0)
    imit = ImitationTrainer(nav5, ScriptedNavExpert(nav5), seed=0)
    mse_hist = imit.train(iterations=10, log_every=0)
    assert mse_hist[-1] < mse_hist[0], \
        f"imitation MSE must decrease: {mse_hist[0]:.4f} → {mse_hist[-1]:.4f}"
    print(f"      OK — MSE {mse_hist[0]:.4f} → {mse_hist[-1]:.4f}")

    # ------------------------------------------------------------------
    print("  [6] Communication-coupled reward")
    robot = WheeledRobot(base_position=np.array([0.0, 0.0, 0.08]))
    nav_plain = NavigationEnv(robot, env3d, goal=np.array([8.0, 0.0, 0.0]),
                              max_steps=1, seed=0)
    robot2 = WheeledRobot(base_position=np.array([0.0, 0.0, 0.08]))
    nav_comm = NavigationEnv(robot2, env3d, goal=np.array([8.0, 0.0, 0.0]),
                             max_steps=1, seed=0,
                             channel=FreeSpaceChannel(carrier_hz=2.4e9),
                             channel_reward_weight=0.5,
                             base_station_position=np.array([0.0, 0.0, 1.0]))
    nav_plain.reset(); nav_comm.reset()
    _, r_plain, _, _, _ = nav_plain.step(np.array([0.0, 0.0]))
    _, r_comm, _, _, _ = nav_comm.step(np.array([0.0, 0.0]))
    assert r_comm > r_plain, \
        "comm reward must add a positive bonus when the link is up"
    print(f"      OK — plain={r_plain:.3f} < with-comm={r_comm:.3f}")

    # ------------------------------------------------------------------
    print("  [7] Policy reproducibility")
    p1 = MLPPolicy(obs_dim=14, act_dim=2, seed=7)
    p2 = MLPPolicy(obs_dim=14, act_dim=2, seed=7)
    obs = np.random.default_rng(1).normal(0, 1, 14)
    assert np.allclose(p1.act(obs, deterministic=True),
                       p2.act(obs, deterministic=True))
    print("      OK — same seed → same policy")

    print("\n  All RL / imitation tests passed.")


if __name__ == "__main__":
    run()
