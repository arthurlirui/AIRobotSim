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

┌──────────────────────────────────────────────────────────────────┐
│              浏览器端交互式虚拟实验室（前端子项目）                │
│  chemlab/       3D 虚拟化学实验室（Three.js + Vite，开发中）      │
│  physics-lab/   3D 虚拟物理实验室（Three.js + cannon-es，开发中） │
└──────────────────────────────────────────────────────────────────┘
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
comm/               通信模拟模块（多制式信道 · 几何遮挡 · 射线追踪 · 集群网络仿真）
  channel/                  信道模型（适配器模式，统一接口）
    base.py                 ChannelState · ChannelModel 基类 · Pose · RadioParameters
    statistical.py          Layer 0：自由空间 / 对数距离 / 3G·4G / 5G NR / mmWave
    geometric_los.py        Layer 1：复用 GaussianSplatEnv.ray_cast 的几何遮挡
    sionna_rt.py            Layer 2：Sionna RT 射线追踪（可选依赖，反射/绕射/CIR）
    thz.py                  THz 分子吸收信道（100 GHz – 1 THz）
    fso.py                  FSO 自由空间光通信（Gamma-Gamma 湍流 + 指向误差）
    fiber.py                光纤近似无损管道
  phy/                      调制库 + 链路预算（SNR / Shannon 容量 / BER）
  network/                  集群网络仿真（节点 · 动态拓扑 · Dijkstra 中继路由 · 事件）
  scenarios/                预置实验场景（集群中继 · mmWave 遮挡 · THz 短距 · FSO 指向）
sensors/            相机光学模型（共用针孔几何，按类型差异化的测量与噪声模型）
  base.py                   CameraIntrinsics · CameraExtrinsics · CameraModel 基类
  monocular.py              可见光 RGB（shot + read 噪声）
  stereo.py                 双目深度（视差 → 深度，σ ∝ Z²，最远测距限制）
  itof.py                   iToF 深度（相位卷绕 · 多径 · confidence）
  thermal.py                长波红外热成像（NETD + 固定模式噪声）
  nir.py                    近红外主动照明（1/Z² 光强衰减 + 红边增强）
rl/                 强化学习 / 模仿学习
  envs.py                   Gym 风格导航环境（可选通信耦合奖励）
  policy.py                 高斯 MLP 策略（numpy 实现，含反传）
  algo.py                   REINFORCE（带基线）+ DAgger 风格模仿学习
design/             元器件级机器人构型设计（AI for robotics）
  components.py             元器件库：连杆 / 关节 / 执行器 / 接触点 / 传感器挂载
  config.py                 RobotDesign 校验 + JSON 序列化 + 随机构型生成器 + 预设
  builder.py                SimRobot：树状 FK / 雅可比 / IK / 执行器步进 / 环境交互
  locomotion.py             步态控制器：足端落点取 3DGS 地形高度
tests/              单元测试与仿真场景
assets/             3DGS 样例数据 · 配置 · Three.js 前端
  preview.html              Three.js 渲染与交互前端
  animation_data.json       仿真内核 → 前端的动画数据
  external/                 下载的机器人 URDF 与参考代码
    unitree_ros/            Unitree URDF（Go2 / Go1 / A1 / B1 / H1 / Laikago）
    legged_gym/             Legged Gym 地形生成参考（ETH Zurich）
external/jumper/    22-DoF crab 机器人 AI 工具链（git submodule）
external/splatshop/ 3DGS 场景编辑器（git submodule，m-schuetz/Splatshop）

# —— 统一仿真控制台（前后端 Web 应用）——
server/             仿真控制台后端（FastAPI：REST + WebSocket）
  app.py                    /api/catalog · /api/scenarios/run · /api/comm/* · /api/design/* · /ws/sim
  sim_session.py            WebSocket 会话：场景配置 → 可逐步步进的 World
  payloads.py               仿真对象 → JSON 帧负载（机器人位姿 / 激光雷达 / 通信链路 / 指标）
app/                仿真控制台前端（Vue 3 + Pinia + Three.js + Vite）
  src/views/                实时仿真（WebSocket 实时流 + 3D 视口）· 通信实验 · 构型设计
  src/components/           SimViewport：Three.js 场景渲染（地形 / 机器人 / 激光 / 通信链路）
  src/stores/sim.js         WebSocket 连接与实时状态管理
  src/api/                  REST 客户端

# —— 浏览器端交互式虚拟实验室（前端子项目，独立工程化）——
chemlab/            3D 虚拟化学实验室（Three.js + Vite Web 应用，开发中）
  package.json             子项目依赖与脚本（vite dev / build / preview）
  src/                     应用源码（脚手架，待填充）
  public/                  静态资源（脚手架，待填充）
physics-lab/        3D 虚拟物理实验室（Three.js + cannon-es 物理引擎，开发中）
  vendor/                  三方库本地副本（three.module.js r160、cannon-es.js）
  js/                      应用源码目录（core / equipment / experiments / scene / ui / utils，脚手架）
  css/  docs/  assets/     样式、文档、资源（脚手架，待填充）
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

### 启动统一仿真控制台（前后端）

一条命令即可启动带 Web 前端的仿真控制台（后端自动托管 `app/dist` 构建产物）：

```bash
# 1. 构建前端（仅需一次，或前端改动后重新构建）
cd app && npm install && npm run build && cd ..

# 2. 启动后端（rtg132 环境）
python -m uvicorn server.app:app --port 8000

# 3. 打开 http://127.0.0.1:8000
```

前端开发模式（热更新，API/WS 自动代理到后端）：

```bash
cd app && npm run dev        # http://localhost:5173
```

控制台包含三个标签页：

| 标签页 | 通道 | 能力 |
|---|---|---|
| 实时仿真 | WebSocket `/ws/sim` | 场景配置（地形/障碍/算法/通信）→ Three.js 实时渲染机器人、激光雷达扫描、通信链路；播放/暂停/单步/重置/倍速；算法指标实时曲线 |
| 通信实验 | REST `/api/comm/*` | 运行 `comm/scenarios` 预置实验（集群中继 / mmWave 遮挡 / THz 短距 / FSO 指向），查看投递路径与时延 |
| 构型设计 | REST `/api/design/*` | 运行 `design/` 预设构型的步态 FK 预览，逐帧回放连杆运动 |

后端 API 自测：`python -m tests.test_server_api`（REST 端点 + WebSocket 帧流冒烟测试）。

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

## 通信模拟模块（`comm/`）

通信模块是平台研究"通信约束"的核心基础设施。它通过**适配器模式**将传统 3G/4G/5G 与先进 mmWave/THz/FSO/光纤统一到同一套 `ChannelModel.generate(tx_pose, rx_pose, t) → ChannelState` 接口下，仿真内核只面对统一接口，底层物理模型差异由各制式适配器吸收。

### 分层架构

| 层 | 类 | 依赖 | 能力 |
|---|---|---|---|
| Layer 0 统计基线 | `FreeSpaceChannel` `LogDistanceChannel` `CellularChannel` `NRChannel` `MmWaveChannel` | numpy | 距离驱动路径损耗 + 衰落，覆盖 80% 集群实验 |
| Layer 1 几何遮挡 | `GeometricLOSChannel` | `GaussianSplatEnv.ray_cast` | 复用 3DGS 场景几何判定 LOS/NLOS，遮挡即降级或断链 |
| Layer 0+ THz | `THzChannel` | numpy | 分子吸收 (Beer-Lambert) + FSPL，100 GHz – 1 THz |
| Layer 0+ FSO | `FSOChannel` | numpy | Gamma-Gamma 湍流 (Rytov) + Farid-Uysal 指向误差 |
| 光纤 | `FiberChannel` | numpy | 近似无损固定容量管道 |

### 支持的通信制式

| 制式 | 类 | 关键参数 | 底层模型 |
|---|---|---|---|
| 3G/4G | `CellularChannel` | 900/1900/700/1800/2600 MHz | 对数距离 + 阴影衰落 |
| 5G NR | `NRChannel` | sub-6 GHz | 3GPP TR 38.901 CI 模型 |
| mmWave | `MmWaveChannel` | 28/60/73 GHz | CI 模型 + 氧气吸收 |
| THz | `THzChannel` | 100 GHz – 1 THz | 分子吸收 (ITU-R P.676) + FSPL |
| FSO | `FSOChannel` | 850/1550 nm | Rytov 湍流 + 指向误差 |
| 光纤 | `FiberChannel` | 任意容量 | 0.2 dB/km 近似无损 |

### 快速使用

```python
from comm import (
    create_channel, CommNetworkSimulator, Pose, RadioParameters,
    GeometricLOSChannel, MmWaveChannel, NRChannel,
)

# 1. 创建信道模型（工厂或直接构造）
ch = create_channel("mmwave", carrier_hz=28e9)

# 2. 创建集群网络仿真器
sim = CommNetworkSimulator(ch)
sim.add_node("r0", Pose(position=np.array([0, 0, 0.5])))
sim.add_node("r1", Pose(position=np.array([10, 0, 0.5])))
sim.add_node("r2", Pose(position=np.array([20, 0, 0.5])))

# 3. 每步更新位姿 → 重算所有链路 → 更新拓扑
snap = sim.step(t=0.0)
print(f"links up: {sum(1 for l in snap.links if l.up)}")

# 4. 多跳投递
result = sim.send("r0", "r2")
print(f"delivered={result.delivered}, path={result.path}, hops={len(result.path)-1}")
```

### 几何遮挡（Layer 1）

平台的差异化能力在于：链路质量由 3DGS 重建场景的**实际几何**驱动，而非纯统计模型。`GeometricLOSChannel` 包装任意 Layer-0 信道模型，对每对收发节点发射射线调用 `GaussianSplatEnv.ray_cast` 判定 LOS/NLOS：

```python
from core.gaussian_splat_env import GaussianSplatEnv

env = GaussianSplatEnv.create_flat_ground(size=8.0, resolution=4)
# ... 向 env 添加障碍物 splats ...

# mmWave 对遮挡极敏感 → hard_block=True 直接断链
mmwave = GeometricLOSChannel(
    MmWaveChannel(carrier_hz=28e9), hard_block=True)
# 5G NR 穿透性更好 → 软惩罚 15 dB
nr = GeometricLOSChannel(
    NRChannel(carrier_hz=3.5e9), nlos_penalty_db=15.0)

sim = CommNetworkSimulator(mmwave, env=env)  # env 传递给几何遮挡判定
```

### 预置实验场景

`comm/scenarios/` 提供四个开箱即用的实验，同时作为集成测试与使用示例：

| 场景 | 函数 | 研究问题 |
|---|---|---|
| 集群中继 | `swarm_relay()` | N 机器人线形排列，端到端投递需多跳中继 |
| mmWave 遮挡 | `mmwave_occlusion(env)` | mmWave vs 5G NR 对遮挡的敏感性对比 |
| THz 短距 | `thz_short_range()` | THz 容量随间距衰减，确定可用距离窗口 |
| FSO 指向 | `fso_pointing()` | 步态抖动幅度 vs FSO 指向损耗与 BER |

---

## 传感器光学模型（`sensors/`）

感知研究的基础设施：五类相机共用同一套针孔几何（`CameraIntrinsics` / `CameraExtrinsics`），但各自实现**差异化测量物理**——对同一 3DGS 场景、同一位姿渲染时，各传感器的输出必须可区分：

| 相机 | 输出 | 关键物理 |
|---|---|---|
| `MonocularCamera` | RGB + 深度 | 散粒噪声 + 读出噪声，颜色随距离衰减 |
| `StereoCamera` | RGB + 深度 | 视差模型 σ_Z ∝ Z²/(f·B)，超出测距上限返回无效 |
| `ITofCamera` | 深度 + confidence | 相位卷绕（Z mod c/2f_mod）、多径偏差、回波强度置信度 |
| `ThermalCamera` | 表观温度 | NETD 时域噪声 + 逐像素固定模式噪声 |
| `NearInfraredCamera` | NIR 强度 | 主动照明 1/Z² 衰减，植被红边反射增强 |

所有模型基于 `GaussianSplatEnv.ray_cast` 逐像素投射，渲染用 `stride` 参数子采样以控制开销；`seed()` 保证实验可复现。

```python
from sensors import StereoCamera, ITofCamera
from sensors.base import CameraExtrinsics
import numpy as np

# 相机朝 +X 安装（光轴 +Z → 世界 +X）
ext = CameraExtrinsics(translation=np.array([0.1, 0, 0.1]),
                       euler=np.array([0, np.pi/2, 0]))
cam = StereoCamera(extrinsics=ext); cam.seed(0)
frame = cam.render(env, robot.position, robot.state.base_orientation, stride=4)
```

---

## 强化学习 / 模仿学习（`rl/`）

`NavigationEnv` 提供 Gym 风格接口（`reset / step → (obs, reward, terminated, truncated, info)`），观测为机体坐标系目标向量 + 速度 + 前向射线扫描；奖励 = 前进进度 + 到达奖励 − 时间/碰撞惩罚。可选**通信耦合奖励**：每步通过任意 `ChannelModel`（Layer 0/1/2 均可）计算与基站的链路质量并注入奖励，用于研究通信约束下的导航策略。

内置训练算法均为 numpy 实现、零额外依赖：`ReinforceTrainer`（带滑动基线 + 批内优势归一化）与 `ImitationTrainer`（DAgger 风格，`ScriptedNavExpert` 提供专家标签）。环境接口与 Gymnasium 兼容，可直接替换为 Stable-Baselines3 等框架训练。

```python
from rl import NavigationEnv, ReinforceTrainer

nav = NavigationEnv(wheeled_robot, env, goal=np.array([8., 0., 0.]),
                    channel=create_channel("mmwave"),      # 可选：通信耦合
                    channel_reward_weight=0.3)
trainer = ReinforceTrainer(nav, batch_size=16)
returns = trainer.train(iterations=50)                       # 回收曲线
```

---

## Sionna RT 射线追踪（Layer 2）

安装 sionna（`pip install sionna`，需要 Python ≥ 3.10）后，`SionnaRTChannel` 将 3DGS 场景中的 splat 转换为 Mitsuba 网格场景，用 Sionna RT 计算精确的多径信道（镜面反射、漫散射、边缘绕射、完整 CIR）：

```python
from comm import create_channel

ch = create_channel("sionna_rt", env=env, carrier_hz=28e9, max_depth=3)
state = ch.generate(tx_pose, rx_pose)
print(state.path_gain_db, state.los, state.cir)   # 含功率时延谱
```

与 Layer 0/1 同一接口，可直接插入 `CommNetworkSimulator` 做多机器人射线追踪级网络仿真。未安装 sionna 时工厂自动隐藏该制式，其余层级不受影响。

### Splatshop 子模块

`external/splatshop`（[m-schuetz/Splatshop](https://github.com/m-schuetz/Splatshop)）是基于 Web 的 3DGS 场景编辑器，用于查看、裁剪和编辑 `.ply` 高斯泼溅重建结果——平台用它准备投喂给 `GaussianSplatEnv` 的场景资产。

```bash
git submodule update --init --recursive
# 本地起一个静态服务器打开编辑器
python -m http.server -d external/splatshop 8000
```

---

## 元器件级构型设计（`design/`）

`robots/` 提供固定构型的真实机器人；`design/` 解决相反的问题——**构型本身是研究对象**。从元器件（连杆 / 关节 / 执行器 / 接触点 / 传感器挂载）出发搭建任意树状构型，经结构校验（单根 / 树状 / 无环）后编译成 `SimRobot`，与 3DGS 世界做接触、碰撞与行走交互；构型可 JSON 序列化，也可由 `random_design` 程序化生成（形态搜索原语）。

```python
from design import Link, Joint, Actuator, ContactPoint, SensorMount
from design import RobotDesign, SimRobot, GaitController, random_design

# 1) 组装元器件 → 构型
d = RobotDesign(name="MyBot")
d.links.append(Link(name="body", mass=2.0, size=(0.3, 0.15, 0.06), is_root=True))
d.joints.append(Joint(name="hip", parent="body", child="thigh",
                      axis="y", origin=np.array([0.15, 0.08, 0]),
                      limits=(-1.5, 1.5)))
d.links.append(Link(name="thigh", mass=0.3, shape="cylinder", size=(0.02, 0.2)))
d.actuators.append(Actuator(joint="hip", torque_limit=20.0))
d.contact_points.append(ContactPoint(name="foot", link="thigh",
                                     offset=np.array([0, 0, -0.2])))
d.validate()                      # 结构校验
d.save("my_bot.json")             # 序列化分享

# 2) 编译 → 与 3DGS 世界交互
robot = SimRobot(d, base_position=np.array([0, 0, 0.3]))
robot.snap_contacts_to_ground(env)             # 贴地
robot.update_contacts(env)                     # 接触判定
pos, orn = robot.mount_pose(...)               # 传感器挂载 → sensors/comm

# 3) 步态行走（足端落点取重建地形高度）
gait = GaitController(robot, env, stride=0.05)
for _ in range(200):
    gait.step(0.02, heading=0.0)

# 4) 随机构型（形态搜索）
d_rand = random_design(seed=0)   # 2–6 腿 × 2–3 关节
```

设计细节见 [docs/design-module-design.md](docs/design-module-design.md)。

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
| `robots.mobile_base` | 轮式（差速驱动）与无人机（四旋翼）移动平台 |
| `control.gait_planner` | 步态模式生成（walk / trot / wave / tripod …） |
| `control.motion_controller` | 整体运动控制 |
| `interaction.env_interaction` | 地形适应、碰撞响应、射线感知、交互事件 |
| `comm.channel.base` | 信道模型基类、ChannelState、Pose、RadioParameters |
| `comm.channel.statistical` | Layer 0 统计信道：自由空间 / 对数距离 / 3G·4G / 5G NR / mmWave |
| `comm.channel.geometric_los` | Layer 1 几何遮挡，复用 GaussianSplatEnv.ray_cast |
| `comm.channel.sionna_rt` | Layer 2 Sionna RT 射线追踪（反射/绕射/信道冲激响应，可选依赖） |
| `comm.channel.thz` | THz 分子吸收信道（100 GHz – 1 THz） |
| `comm.channel.fso` | FSO 自由空间光通信（湍流 + 指向误差） |
| `comm.channel.fiber` | 光纤近似无损管道 |
| `comm.phy.modulations` | 调制库（BPSK / QPSK / M-QAM / OOK）→ BER |
| `comm.phy.link_budget` | 链路预算：SNR / Shannon 容量 / BER |
| `comm.network.node` | 通信节点（绑定机器人位姿与信道模型） |
| `comm.network.topology` | 动态拓扑：邻接矩阵 · 连通分量 · Dijkstra 中继路由 |
| `comm.network.network_simulator` | 集群级仿真：step / send / 事件 / 快照 |
| `comm.scenarios` | 预置实验场景（中继 · 遮挡 · THz · FSO） |
| `sensors.base` | 相机基类：针孔内参 / 外参 / 统一渲染接口 |
| `sensors.monocular` | 可见光单目相机（shot + read 噪声） |
| `sensors.stereo` | 双目深度相机（视差模型，σ∝Z²，测距上限） |
| `sensors.itof` | iToF 深度相机（相位卷绕 + 多径 + confidence） |
| `sensors.thermal` | 长波红外热像仪（NETD + 固定模式噪声） |
| `sensors.nir` | 近红外主动照明相机（1/Z² 衰减 + 红边增强） |
| `rl.envs` | Gym 风格导航环境（支持通信耦合奖励） |
| `rl.policy` | 高斯 MLP 策略（numpy 实现） |
| `rl.algo` | REINFORCE（带基线）与 DAgger 风格模仿学习 |
| `design.components` | 元器件库：Link / Joint / Actuator / ContactPoint / SensorMount |
| `design.config` | RobotDesign 构型校验 + JSON + random_design + 预设构型 |
| `design.builder` | SimRobot：FK / 雅可比 / 阻尼 IK / PD 执行器 / 3DGS 接触 |
| `design.locomotion` | GaitController：通用步态（tripod/trot），足端落在重建地形上 |
| `chemlab` | 3D 虚拟化学实验室前端子项目（Three.js + Vite，脚手架） |
| `physics-lab` | 3D 虚拟物理实验室前端子项目（Three.js + cannon-es，脚手架） |

---

## 测试

`tests/` 采用"静默失败回归 + 强制对照组"的测试哲学，`python -m tests.run_all` 顺序运行：

- `test_math_utils` — 数学工具
- `test_env` — 3DGS 环境与空间查询
- `test_robots` — 机器人运动学
- `test_gait` — 步态规划
- `test_simulation` — 3DGS 场景下的整机仿真集成
- `test_terrain_urdf` — 地形生成器与 URDF 工厂
- `test_comm_channel` — 通信信道模型（Layer 0+：3G/4G/5G/mmWave/THz/FSO/光纤）
- `test_comm_network` — 集群网络仿真器（拓扑 / 中继 / 移动性）
- `test_comm_geometric` — 几何遮挡（Layer 1，含对照组回归断言）
- `test_mobile_robots` — 轮式 / 无人机运动模型与环境交互
- `test_sensors` — 相机光学模型（含 iToF 相位卷绕 vs 单目真值的对照组）
- `test_rl` — 强化学习 / 模仿学习（含"训练必须改进"的对照组断言）
- `test_comm_sionna` — Sionna RT 射线追踪适配器（未安装 sionna 时自动跳过；含 FSPL 对照）
- `test_design` — 元器件构型设计（雅可比/IK 数值对照 + 3DGS 接触/行走交互）

---

## 致谢

- [legged_gym](https://github.com/leggedrobotics/legged_gym) — ETH Zurich 足式机器人地形生成参考
- [GenTe](https://github.com/HaronW/GenTe) — 地形 benchmark
- [unitree_ros](https://github.com/unitreerobotics/unitree_ros) — Unitree 机器人 URDF
- [Three.js](https://threejs.org/) — 浏览器端三维渲染
- [Jumper](https://github.com/KingKongRobotics/jumper-design) — 22-DoF crab 机器人 AI 工具链
- [Splatshop](https://github.com/m-schuetz/Splatshop) — 3DGS 场景编辑器
- [Sionna](https://nvlabs.github.io/sionna/) — NVIDIA 可微分射线追踪无线仿真
- [cannon-es](https://github.com/pmndrs/cannon-es) — `physics-lab/` 使用的轻量物理引擎

---

## 浏览器端交互式虚拟实验室（前端子项目）

除了机器人仿真内核，平台还在浏览器端探索**交互式虚拟实验室**方向：用 Three.js 把"在真实重建场景里做实验"的体验做成可直接操作的 Web 应用。两个子项目独立于 Python 内核，各自维护 `package.json` / vendor 依赖，定位与 `assets/preview.html` 同源（都是 Three.js 前端），但走 Vite 工程化、按学科独立组织。

| 子项目 | 技术栈 | 定位 | 状态 |
|---|---|---|---|
| `chemlab/` | Three.js 0.180 + Vite 7 | 3D 虚拟化学实验室，网页交互式实验 | 脚手架（`src/` `public/` 待填充） |
| `physics-lab/` | Three.js r160 + cannon-es | 3D 虚拟物理实验室，含物理引擎交互 | 脚手架（`js/` 目录结构已规划，源码待填充） |

两者目前均处于脚手架阶段：依赖与目录结构已就位，业务源码尚未写入。`physics-lab/` 的 `vendor/` 目录内附了 three.js (r160) 与 cannon-es 物理引擎的本地副本，并按 `core / equipment / experiments / scene / ui / utils` 规划了源码分层——这一分层与机器人仿真内核（core / robots / control / interaction）的职责划分思路一致：核心引擎、设备/对象、实验编排、场景、交互 UI、工具各成一层。

```bash
# 在子项目目录下用 Vite 启动开发服务器（需 Node.js）
cd chemlab && npm install && npm run dev      # http://localhost:5173
cd physics-lab                                # 纯静态 vendor，可直接用任意静态服务器预览
```

> 这两个子项目与 Python 仿真内核**解耦**：它们不依赖 `rtg132` 环境，运行在浏览器/Node.js 侧，未来可通过 JSON 数据或 HTTP 接口与内核联动，也可独立作为教学/演示工具使用。

---

## English

# AIRobotSim — Reconstruction-Based Robot Swarm Interaction Platform

An **experimental platform** that studies swarm coordination, inter-robot communication, and robot-environment interaction by placing real robot models inside **high-fidelity 3D reconstructions** (3D Gaussian Splatting) rendered on a **Three.js** front end.

The platform is organized as a Python simulation kernel (kinematics, gait planning, terrain adaptation, collision) plus a Three.js rendering / interaction layer (`assets/preview.html`), decoupled via JSON animation data. The reconstruction serves as the robot's environment, the URDF models keep the robots physically credible, and on top of that the platform targets the problems that are expensive to reproduce in the physical world: **swarm behavior, communication constraints, and interaction with complex reconstructed terrain**.

Two browser-side **interactive virtual-lab front ends** (`chemlab/` — 3D chemistry lab, `physics-lab/` — 3D physics lab with cannon-es) are scaffolded as separate Vite/Three.js sub-projects, decoupled from the Python kernel.

See the Chinese section above for the full module reference, quick start, and design rationale.
