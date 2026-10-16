"""
Generate animation data: run simulations for biped, quadruped, and hexapod
in a 3DGS terrain environment, recording per-frame state for visualization.
Outputs a JSON file with all frame data.
"""

import sys
import os
import json
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.gaussian_splat_env import GaussianSplatEnv
from robots.biped import Biped
from robots.quadruped import Quadruped
from robots.hexapod import Hexapod
from control.gait_planner import GaitType
from control.motion_controller import MotionController


def collect_splat_data(env, max_splats=400):
    """Collect 3DGS splat positions/scales for rendering."""
    splats = []
    indices = np.linspace(0, len(env.splats) - 1, min(max_splats, len(env.splats)), dtype=int)
    for i in indices:
        s = env.splats[i]
        splats.append({
            "pos": [round(float(s.position[0]), 3),
                    round(float(s.position[1]), 3),
                    round(float(s.position[2]), 3)],
            "scale": [round(float(s.scale[0]), 3),
                      round(float(s.scale[1]), 3),
                      round(float(s.scale[2]), 3)],
            "color": [round(float(c), 3) for c in s.color],
        })
    return splats


def simulate_robot(robot_type, env, duration, dt, speed):
    """Run simulation and collect per-frame data."""
    if robot_type == "biped":
        robot = Biped(base_position=np.array([0.0, 0.0, 0.9]))
        gait = GaitType.STATIC_WALK
        cycle = 2.0
        stride = 0.08
        step_h = 0.06
    elif robot_type == "quadruped":
        robot = Quadruped(base_position=np.array([0.0, 0.0, 0.35]))
        gait = GaitType.TROT
        cycle = 0.8
        stride = 0.1
        step_h = 0.05
    elif robot_type == "hexapod":
        robot = Hexapod(base_position=np.array([0.0, 0.0, 0.2]))
        gait = GaitType.TRIPOD
        cycle = 1.0
        stride = 0.1
        step_h = 0.05
    else:
        raise ValueError(f"Unknown robot type: {robot_type}")

    controller = MotionController(robot, env, gait,
                                  stride_length=stride, step_height=step_h,
                                  cycle_time=cycle)
    controller.walk_forward(speed)

    n_steps = int(duration / dt)
    frames = []
    for i in range(n_steps):
        controller.step(dt)
        frame = {
            "t": round(robot.state.t, 4),
            "base_pos": [round(float(v), 4) for v in robot.state.base_position],
            "base_orient": [round(float(v), 4) for v in robot.state.base_orientation],
            "feet": {},
            "contacts": {},
        }
        for name in robot.legs:
            if name in robot.state.foot_positions:
                fp = robot.state.foot_positions[name]
                frame["feet"][name] = [round(float(v), 4) for v in fp]
                frame["contacts"][name] = bool(robot.state.foot_contacts.get(name, False))
        frames.append(frame)

    # Determine leg connection info (hip offsets for drawing body)
    leg_info = {}
    for name, leg in robot.legs.items():
        leg_info[name] = {
            "hip_offset": [round(float(v), 4) for v in leg.hip_offset],
            "link_lengths": leg.link_lengths,
        }

    return {
        "robot_type": robot_type,
        "n_legs": robot.n_legs,
        "leg_info": leg_info,
        "frames": frames,
    }


def main():
    print("Generating animation data...")

    # Use terrain environment
    env = GaussianSplatEnv.create_terrain(size=8.0, resolution=14,
                                          roughness=0.12, seed=42)
    print(f"  Environment: {env.stats()['num_splats']} splats")

    splat_data = collect_splat_data(env, max_splats=300)

    duration = 6.0
    dt = 0.05  # 20 fps for animation

    results = {}
    for rtype, speed in [("hexapod", 0.08), ("quadruped", 0.08), ("biped", 0.04)]:
        print(f"  Simulating {rtype}...")
        # Each robot gets its own copy of the env (recreate to avoid state contamination)
        env_copy = GaussianSplatEnv.create_terrain(size=8.0, resolution=14,
                                                    roughness=0.12, seed=42)
        data = simulate_robot(rtype, env_copy, duration, dt, speed)
        results[rtype] = data
        print(f"    {len(data['frames'])} frames collected")

    output = {
        "duration": duration,
        "dt": dt,
        "fps": int(1.0 / dt),
        "splats": splat_data,
        "robots": results,
    }

    output_path = os.path.join(os.path.dirname(__file__), "..", "assets", "animation_data.json")
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(output, f)

    file_size = os.path.getsize(output_path) / 1024
    print(f"\nAnimation data saved to: {output_path}")
    print(f"File size: {file_size:.1f} KB")


if __name__ == "__main__":
    main()
