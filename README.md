# AIRobotSim — 基于三维重建的机器人集群交互实验平台

[English](#english) | 简体中文

> 一个以**高保真三维重建**为场景、以 **Three.js** 为渲染与交互层、以真实机器人模型为主体的机器人集群 / 通信 / 交互**实验平台**。

AIRobotSim 面向的核心问题是：当多台足式机器人被放进一个由真实世界三维重建（3D Gaussian Splatting, 3DGS / NeRF 类高保真重建）生成的虚拟场景中时，**集群协同、机间通信、个体-环境交互**这些在物理世界中昂贵且难以复现的问题，能否在一个可重复、可观测、可改造的数字场景中被研究、验证和迭代。

平台目前以 Python 仿真内核 + Three.js 前端的形式组织：Python 侧负责机器人运动学 / 动力学、步态规划、地形适应与碰撞；Three.js 侧（`assets/preview.html`）负责 3DGS 场景渲染、机器人可视化和实时交互。两者通过 JSON 动画数据（`assets/animation_data.json`）解耦，方便独立迭代。

---

## 设计目标

AIRobotSim 不是一个通用物理引擎，而是一个**面向研究的实验平台**，目标集中在三件事：

1. **高保真场景**——以 3DGS 等高保真三维重建结果作为机器人所处的虚拟环境，保留真实世界的几何与外观复杂度，而非使用人造简化地形。
2. **真实机器人模型**——直接解析厂商 URDF（Unitree Go2 / Go1 / A1 / B1 / H1 / Laikago、ANYmal B/C 等），使用真实的运动学参数；同时接入 `external/jumper` 这类 22-DoF 多足机器人 AI 工具链作为扩展形态。
3. **集群 / 通信 / 交互问题**——在上述场景与模型之上，研究多机器人集群行为、机间通信约束对协同的影响，以及单机与复杂重建地形之间的交互（足端落点、碰撞响应、感知）。

这三点共同构成了平台区别于传统仿真器的定位：**用真实重建的场景 + 真实机器人模型，去研究物理世界难以低成本复现的集群与交互问题**。

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                        Three.js 渲染 / 交互层                      │
│  assets/preview.html — 3DGS 场景渲染、机器人可视化、实时相机与控件  │
└───────────────────────────────┬──────────────────────────────────┘
                                │  JSON 动画数据 (assets/animation_data.json)
┌───────────────────────────────┴──────────────────────────────────┐
│                          Python 仿真内核                          │
│                                                                   │
│  core/        数学工具 · 3DGS 环境 · 地形 · URDF 解析              │
│  robots/      足式机器人基类 + 双足 / 四足 / 六足 + URDF 工厂       │
│  control/     步态规划 · 整体运动控制                              │
│  interaction/ 机器人-环境交互（地形适应 · 碰撞响应 · 射线感知）      │
│                                                                   │
│  external/jumper/  22-DoF crab 机器人 AI 工具链（git submodule）   │
└───────────────────────────────────────────────────────────────────┘
```

渲染层与仿真内核完全解耦：内核产出离散时间步的机器人状态序列，序列化成 JSON 后由 Three.js 侧回放。这意味着可以在不改前端的情况下替换仿真后端，也可以在不跑仿真的情况下用前端预录数据做交互实验。

---

## 目录结构

```
core/               数学工具 · 3DGS 环境 · 碰撞查询
  math_utils.py             向量、旋转、变换
  gaussian_splat_env.py     3DGS 环境表示，提供空间查询（高度 / 碰撞 / 法向）
  terrain.py                从 3DGS 提取的地形高度场
  terrain_generator.py      高级程序化地形生成器（9 种地形类型）
  urdf_parser.py            URDF 解析，提取真实机器人运动学
robots/             机器人基类 + 双足 / 四足 / 六足 + URDF 工厂
  robot_base.py             抽象足式机器人，URDF 式运动链
  biped.py                  双足人形（Unitree G1）
  quadruped.py              四足（Unitree Go2）
  hexapod.py                六足（DOBOT Hexplorer）
  urdf_robot.py             从真实 URDF 构建机器人的工厂
control/            步态规划 · 运动控制
  gait_planner.py           步态模式生成（walk / trot / wave / tripod …）
  motion_controller.py      整体运动控制
interaction/        机器人-环境交互
  env_interaction.py        地形适应 · 碰撞响应 · 射线"激光雷达"感知 · 交互事件
tests/              单元测试与仿真场景
assets/             3DGS 样例数据 · 配置 · Three.js 前端
  preview.html              Three.js 渲染与交互前端
  animation_data.json       仿真内核 → 前端的动画数据
  external/                 下载的机器人 URDF 与参考代码
    unitree_ros/            Unitree URDF（Go2 / Go1 / A1 / B1 / H1 / Laikago）
    legged_gym/             Legged Gym 地形生成参考（ETH Zurich）
external/jumper/    22-DoF crab 机器人 AI 工具链（git submodule）
```

---

## 环境与 Quick Start

整个实验平台使用 conda 环境 **`rtg132`** 统一维护，所有依赖在该环境内安装与升级，避免污染 base 环境。

```bash
# 1. 创建（或确认已存在）rtg132 环境
conda env create -f environment.yml          # 首次安装
# 或已存在则更新
conda env update -f environment.yml --prune

# 2. 激活
conda activate rtg132

# 3. 运行全部测试
python -m tests.run_all
```

> 平台依赖的 base conda 是 Lucid Vision Labs 自带环境（Python 3.7），**不要**在其上直接安装；请始终使用 `rtg132`。仓库根目录的 `environment.yml` 记录了 `rtg132` 的完整依赖规格。

要在浏览器中查看三维场景与机器人回放，直接打开 `assets/preview.html`，或用 `tests/generate_animation_data.py` 重新生成动画数据。

---

## 高保真三维重建场景

`core/gaussian_splat_env.py` 把一组 3D Gaussians 作为环境的**体积占据模型**——不是为了渲染（渲染交给 Three.js 侧的光栅化器），而是利用高斯椭球的几何特性做：

- **地形高度查询**——把椭球投影到地面平面得到可通行高度
- **碰撞检测**——机器人足端 / 本体与椭球体的相交判断
- **表面法向估计**——用于足端姿态适应

没有真实 `.ply` 重建文件时，模块会生成一套合成程序化地形场景，使整条管线可以端到端跑通。接入真实 3DGS 重建只需提供符合 `GaussianSplat` 接口的点云数据即可。

---

## 高级地形生成器

`core/terrain_generator.py` 提供 9 种程序化地形，用于足式机器人运动测试，灵感来自 [legged_gym](https://github.com/leggedrobotics/legged_gym) 与 [GenTe](https://github.com/HaronW/GenTe)：

| 地形 | 说明 |
|---|---|
| `flat` | 平地 |
| `rough` | 随机粗糙地形 |
| `slope` | 金字塔形坡道，中央带平台 |
| `stairs` | 金字塔楼梯（四面上行） |
| `obstacles` | 随机矩形离散障碍 |
| `stepping_stones` | 带间隙的凸起石块网格 |
| `gap` | 横贯地形的裂缝 |
| `pit` | 中央坑洞 |
| `wave` | 起伏正弦地形 |

每种地形支持 0.0–1.0 难度缩放，并可组合成课程网格用于渐进式训练。

```python
from core.terrain_generator import TerrainGenerator

gen = TerrainGenerator(horizontal_scale=0.1, terrain_length=8.0, terrain_width=8.0)
env = gen.generate_single("stairs", difficulty=0.5)        # 单一地形
env = gen.generate_curriculum(num_rows=5, num_cols=8)      # 课程网格
```

---

## 真实机器人 URDF 模型

`core/urdf_parser.py` 与 `robots/urdf_robot.py` 解析真实 URDF 并按准确的运动学参数构建机器人。下载自 [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros) 的 URDF 位于 `assets/external/`：

| 机器人 | 类型 | DOF | 来源 |
|---|---|---|---|
| Go2 | 四足 | 12 | `go2_description/urdf/go2_description.urdf` |
| Go1 | 四足 | 12 | `go1_description/urdf/go1.urdf` |
| A1 | 四足 | 12 | `a1_description/urdf/a1.urdf` |
| B1 | 四足 | 12 | `b1_description/xacro/b1.urdf` |
| H1 | 人形 | 20 | `h1_description/urdf/h1.urdf` |
| Laikago | 四足 | 12 | `laikago_description/urdf/laikago.urdf` |
| ANYmal B | 四足 | 12 | legged_gym 资源 |
| ANYmal C | 四足 | 12 | legged_gym 资源 |

```python
from robots.urdf_robot import create_robot_from_urdf, list_available_robots

print(list_available_robots())          # ['go2', 'go1', 'a1', 'b1', 'h1', 'laikago', ...]
robot = create_robot_from_urdf("go2")   # 真实 Go2：15 kg，0.213m 连杆，12 DOF
```

### Jumper 子模块

`external/jumper` 是一个 22-DoF crab 机器人的 AI 工具链（外观设计、动作训练、场景创建），以 git submodule 形式引入，作为平台对更多足态形态的扩展入口。

```bash
git submodule update --init --recursive
```

---

## 集群 / 通信 / 交互

平台的研究重心在于把多台上述真实机器人放进同一个高保真重建场景后，所涌现出的协同与交互问题。当前内核已经提供的单机能力（步态、地形适应、碰撞响应、射线感知）是研究这些问题的基础设施；在此之上，典型的实验方向包括：

- **集群行为**——多机器人在共享重建场景中的编队、避让、协同通过复杂地形
- **通信约束**——机间通信的可达性、延迟、丢包对集群协同策略的影响
- **个体-环境交互**——足端在重建地形上的落点选择、碰撞响应、基于射线感知的环境探索
- **场景可复现性**——同一套重建场景在不同算法 / 不同集群规模下的横向对比

`interaction/env_interaction.py` 已经抽象出 `InteractionEvent`（足端接触、本体碰撞、踩踏、推挤）作为交互的统一事件模型，便于后续在集群层面做事件聚合与策略评估。

---

## 模块一览

| 模块 | 说明 |
|--------|-------------|
| `core.math_utils` | 向量、旋转、变换 |
| `core.gaussian_splat_env` | 3DGS 环境表示与空间查询 |
| `core.terrain` | 从 3DGS 提取的地形高度场 |
| `core.terrain_generator` | 高级程序化地形生成器（9 种 + 课程） |
| `core.urdf_parser` | URDF 解析，提取机器人运动学结构 |
| `robots.robot_base` | 抽象足式机器人，URDF 式运动链 |
| `robots.biped` | 双足人形机器人 |
| `robots.quadruped` | 四足机器人 |
| `robots.hexapod` | 六足机器人 |
| `robots.urdf_robot` | 从真实 URDF 构建机器人的工厂 |
| `control.gait_planner` | 步态模式生成（walk / trot / wave / tripod …） |
| `control.motion_controller` | 整体运动控制 |
| `interaction.env_interaction` | 地形适应、碰撞响应、射线感知、交互事件 |

---

## 测试

`tests/` 采用"静默失败回归 + 强制对照组"的测试哲学，`python -m tests.run_all` 顺序运行：

- `test_math_utils` — 数学工具
- `test_env` — 3DGS 环境与空间查询
- `test_robots` — 机器人运动学
- `test_gait` — 步态规划
- `test_simulation` — 3DGS 场景下的整机仿真集成
- `test_terrain_urdf` — 地形生成器与 URDF 工厂

---

## 致谢

- [legged_gym](https://github.com/leggedrobotics/legged_gym) — ETH Zurich 足式机器人地形生成参考
- [GenTe](https://github.com/HaronW/GenTe) — 地形 benchmark
- [unitree_ros](https://github.com/unitreerobotics/unitree_ros) — Unitree 机器人 URDF
- [Three.js](https://threejs.org/) — 浏览器端三维渲染
- [Jumper](https://github.com/KingKongRobotics/jumper-design) — 22-DoF crab 机器人 AI 工具链

---

## English

# AIRobotSim — Reconstruction-Based Robot Swarm Interaction Platform

An **experimental platform** that studies swarm coordination, inter-robot communication, and robot-environment interaction by placing real robot models inside **high-fidelity 3D reconstructions** (3D Gaussian Splatting) rendered on a **Three.js** front end.

The platform is organized as a Python simulation kernel (kinematics, gait planning, terrain adaptation, collision) plus a Three.js rendering / interaction layer (`assets/preview.html`), decoupled via JSON animation data. The reconstruction serves as the robot's environment, the URDF models keep the robots physically credible, and on top of that the platform targets the problems that are expensive to reproduce in the physical world: **swarm behavior, communication constraints, and interaction with complex reconstructed terrain**.

See the Chinese section above for the full module reference, quick start, and design rationale.
