# AIRobotSim — 3D Gaussian Splatting Robot Simulation

A modular simulation framework for bipedal, quadrupedal, and hexapodal robots
navigating and interacting in 3D Gaussian Splatting (3DGS) environments.

## Structure

```
core/               Math utilities, 3DGS environment, collision queries
  math_utils.py     Vector math, rotations, transforms
  gaussian_splat_env.py  3DGS environment representation with spatial queries
  terrain.py        Terrain height-field extracted from 3DGS
  terrain_generator.py   Advanced procedural terrain generator (9 terrain types)
  urdf_parser.py    URDF parser for real robot kinematic data
robots/             Robot base class + biped / quadruped / hexapod + URDF factory
  robot_base.py     Abstract legged robot with URDF-like kinematic chain
  biped.py          2-legged humanoid robot (Unitree G1)
  quadruped.py      4-legged robot (Unitree Go2)
  hexapod.py        6-legged robot (DOBOT Hexplorer)
  urdf_robot.py     Factory for building robots from real URDF files
control/            Gait planning, motion controllers
interaction/        Robot-environment interaction (terrain adaptation, collision response)
tests/              Unit tests and simulation scenarios
assets/             Sample 3DGS data, configs
  external/         Downloaded robot URDF models & reference code
    unitree_ros/    Unitree robot URDFs (Go2, Go1, A1, B1, H1, Laikago)
    legged_gym/     Legged Gym terrain generation reference (ETH Zurich)
```

## Quick Start

```bash
pip install numpy scipy
python -m tests.run_all
```

## Advanced Terrain Generator

The `core/terrain_generator.py` module provides 9 procedurally generated
terrain types for testing legged robot locomotion, inspired by the
[legged_gym](https://github.com/leggedrobotics/legged_gym) framework and the
[GenTe](https://github.com/HaronW/GenTe) benchmark:

| Terrain Type | Description |
|---|---|
| `flat` | Perfectly flat ground |
| `rough` | Random uniform rough terrain |
| `slope` | Pyramid-shaped slopes with central platform |
| `stairs` | Pyramid staircase (ascending from all sides) |
| `obstacles` | Random rectangular discrete obstacles |
| `stepping_stones` | Grid of raised stones with gaps |
| `gap` | Crevasse/gap running across terrain |
| `pit` | Pit in the centre |
| `wave` | Rolling sinusoidal hills |

Each terrain type supports difficulty scaling (0.0–1.0) and can be combined
into a curriculum grid for progressive training.

```python
from core.terrain_generator import TerrainGenerator

gen = TerrainGenerator(horizontal_scale=0.1, terrain_length=8.0, terrain_width=8.0)
env = gen.generate_single("stairs", difficulty=0.5)  # single terrain
env = gen.generate_curriculum(num_rows=5, num_cols=8)  # curriculum grid
```

## URDF Robot Models

The `core/urdf_parser.py` and `robots/urdf_robot.py` modules parse real
URDF files and build robots with accurate kinematic parameters. Downloaded
URDFs from [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros)
are included in `assets/external/`:

| Robot | Type | DOF | Source |
|---|---|---|---|
| Go2 | Quadruped | 12 | `go2_description/urdf/go2_description.urdf` |
| Go1 | Quadruped | 12 | `go1_description/urdf/go1.urdf` |
| A1 | Quadruped | 12 | `a1_description/urdf/a1.urdf` |
| B1 | Quadruped | 12 | `b1_description/xacro/b1.urdf` |
| H1 | Humanoid | 20 | `h1_description/urdf/h1.urdf` |
| Laikago | Quadruped | 12 | `laikago_description/urdf/laikago.urdf` |
| ANYmal B | Quadruped | 12 | legged_gym resources |
| ANYmal C | Quadruped | 12 | legged_gym resources |

```python
from robots.urdf_robot import create_robot_from_urdf, list_available_robots

print(list_available_robots())  # ['go2', 'go1', 'a1', 'b1', 'h1', 'laikago', ...]
robot = create_robot_from_urdf("go2")  # real Go2 with 15 kg, 0.213m links, 12 DOF
```

## Modules

| Module | Description |
|--------|-------------|
| `core.math_utils` | Vector math, rotations, transforms |
| `core.gaussian_splat_env` | 3DGS environment representation with spatial queries |
| `core.terrain` | Terrain height-field extracted from 3DGS |
| `core.terrain_generator` | Advanced procedural terrain generator (9 types + curriculum) |
| `core.urdf_parser` | URDF parser for extracting robot kinematic structure |
| `robots.robot_base` | Abstract legged robot with URDF-like kinematic chain |
| `robots.biped` | 2-legged humanoid robot |
| `robots.quadruped` | 4-legged robot (e.g., dog-like) |
| `robots.hexapod` | 6-legged robot (e.g., spider-like) |
| `robots.urdf_robot` | Factory for building robots from real URDF files |
| `control.gait_planner` | Gait pattern generation (walk, trot, wave, etc.) |
| `control.motion_controller` | Whole-body motion control |
| `interaction.env_interaction` | Terrain adaptation, collision response |
