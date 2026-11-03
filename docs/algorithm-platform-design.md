# 算法设计平台设计 (Algorithm Design Platform)

> 在三维高斯泼溅（3D Gaussian Splatting, 3DGS）场景中，统一测试 **SLAM、导航、集群控制、检测** 四类机器人算法的可伸缩、可扩展、高性能模拟平台。
>
> 本模块在现有 `core / robots / control / interaction / comm / sensors / rl / design` 框架之上，新增两个顶层包：
> - **`algorithms/`** —— 四类算法的统一抽象与若干参考实现；
> - **`platform/`** —— 把“场景 + 机器人 + 通信 + 传感器 + 算法”编排成可复现实验的模拟内核（world / scenario / benchmark / metrics / registry）。

---

## 1. 设计目标

| 目标 | 含义 |
| --- | --- |
| **可伸缩 (Scalable)** | 单机单进程内可从 1 台机器人扩到数百台；通信与感知采用批处理 + 空间哈希，复杂度近线性于节点数。`platform/world.py` 提供 `VectorizedSwarm` 以向量化步进。 |
| **可扩展 (Extensible)** | 每类算法都通过抽象基类 + 注册表（`registry`）接入。新算法只需 `@register("slam", "my_slam")` 装饰一个子类即可被 scenario / benchmark 引用，无需改动平台代码。 |
| **高性能 (High-performance)** | 仿真内核零外部重依赖（纯 numpy）；热点（ray_cast、信道 pairwise）支持批处理；通信/感知可选 Sionna RT 等高保真后端但不强依赖。 |
| **可复现 (Reproducible)** | `Scenario` 全量配置化（JSON 可序列化），`Benchmark` 固定随机种子并产出 `BenchmarkReport`，含逐 episode 指标与聚合统计。 |

## 2. 在现有框架中的位置

```
┌──────────────────────────────────────────────────────────────────┐
│                     platform/  (本设计新增)                       │
│  world.py        World —— 单一仿真世界，持有 env/robots/comm/sensors│
│  scenario.py     Scenario / Episode —— 配置化实验编排             │
│  benchmark.py    Benchmark —— 多 episode 批跑 + 报告              │
│  metrics.py      Metrics —— 各类算法的统一指标采集与聚合          │
│  registry.py     AlgorithmRegistry —— 算法注册与发现              │
│  orchestrator.py Orchestrator —— 分布式/并行编排（进程级 fan-out） │
└───────────────────────┬──────────────────────────────────────────┘
                        │ 调用
┌───────────────────────┴──────────────────────────────────────────┐
│                     algorithms/  (本设计新增)                      │
│  base.py         AlgorithmBase / SLAMBase / NavigationBase /      │
│                  SwarmControlBase / DetectionBase                 │
│  slam.py         占栅格 SLAM、位姿图 SLAM 参考实现                 │
│  navigation.py   势场导航、A* 全局 + DWA 局部参考实现              │
│  swarm.py        Boids 集群、领航-跟随、一致性共识参考实现         │
│  detection.py    欧氏聚类检测、射线包络检测参考实现               │
└───────────────────────┬──────────────────────────────────────────┘
                        │ 复用
┌───────────────────────┴──────────────────────────────────────────┐
│  core/  robots/  control/  interaction/  comm/  sensors/  rl/      │
│  design/  external/jumper/                                        │
└──────────────────────────────────────────────────────────────────┘
```

`algorithms/` 只依赖现有包的公开 API（`GaussianSplatEnv.ray_cast`、`MobileRobot.step`、`CommNetworkSimulator.step`、`CameraModel.capture` 等），不修改它们。`platform/` 依赖 `algorithms/` 与现有包，做编排与度量。

## 3. 算法统一抽象 (`algorithms/base.py`)

所有算法实现一个共同的生命周期，使其能在同一个 `World.step` 循环里被无差别调度：

```python
class AlgorithmBase(ABC):
    name: str
    kind: str                       # "slam" | "navigation" | "swarm" | "detection"

    def setup(self, world: "World", cfg: dict) -> None: ...
    def step(self, world: "World", observations: dict) -> dict: ...
    def teardown(self, world: "World") -> dict: ...
    def metrics(self) -> dict: ...
```

- `setup` —— 在 episode 开始时注入世界句柄与配置（如机器人 id 列表、目标点）。
- `step` —— 每个仿真步接收该步的观测（lidar / 相机 / 通信快照 / 位姿真值，按算法需要），返回动作或内部状态变更，并产出本步的算法输出（如 SLAM 的地图、检测的目标列表）。
- `teardown` —— episode 结束时返回总结性数据。
- `metrics` —— 返回当前累计指标字典，供 `platform/metrics.py` 聚合。

四个子基类 (`SLAMBase`, `NavigationBase`, `SwarmControlBase`, `DetectionBase`) 仅收紧 `kind` 与 `step` 的输入输出语义，不增加新的强制方法，保持接入成本最低。

## 4. 参考实现

每个算法类别提供一个**可运行、可被 benchmark 复现**的参考实现，用于验证平台管线并作为对照基线：

| 类别 | 文件 | 参考实现 | 关键思路 |
| --- | --- | --- | --- |
| SLAM | `slam.py` | `OccupancyGridSLAM` | lidar 射线击中/未击中更新占据栅格；`PoseGraphSLAM` 在其上加位姿节点 + 简单回环检测 |
| 导航 | `navigation.py` | `PotentialFieldNav`, `AStarDwaNav` | 势场：引力到目标 + 斥力来自 ray 障碍；A* 在栅格上规划全局路径，DWA 在局部窗口内选最优速度 |
| 集群控制 | `swarm.py` | `BoidsSwarm`, `LeaderFollowerSwarm`, `ConsensusSwarm` | Boids 三规则；领航-跟随编队偏移；一致性共识对齐速度/位置 |
| 检测 | `detection.py` | `EuclideanClusterDetector`, `RayEnvelopeDetector` | 对相机/lidar 点云做欧氏聚类得到目标框；射线包络用多射线首击点拟合凸包 |

参考实现刻意保持轻量（纯 numpy），目的是证明接口可用、指标可算，而非追求 SOTA。

## 5. 模拟内核 (`platform/`)

### 5.1 `World`
单一仿真世界，聚合：
- `GaussianSplatEnv`（场景）
- `dict[str, MobileRobot | LeggedRobot]`（机器人）
- `CommNetworkSimulator`（可选通信）
- `dict[str, CameraModel]`（可选传感器，按 robot_id 索引）
- 注册的算法实例集合
- 时间 `t`、步长 `dt`、随机数生成器

`World.step()` 的单步顺序（固定，确保算法间观测一致）：
1. 采集观测：lidar / 相机 / 通信快照 / 位姿真值（按各算法声明的需求）。
2. 依次调用每个算法的 `step`，把动作写回机器人（导航/集群控制）或更新内部状态（SLAM/检测）。
3. 机器人积分（`robot.step(dt, ...)`），地形/碰撞由 `interaction` 处理。
4. 通信网络 `step(poses)` 更新拓扑。
5. `t += dt`，记录 metrics。

`VectorizedSwarm` 是 `World` 的一个混入/子类，把同构机器人的位姿堆叠成 `(N,3)` 数组统一积分，通信 pairwise 也向量化，用于大规模集群场景。

### 5.2 `Scenario`
全量配置化的单个实验：
```
Scenario = {
  "env": {...},            # 3DGS 场景生成参数
  "robots": [{...}],       # 机器人列表（类型、初始位姿）
  "comm": {...} | null,    # 信道与网络配置
  "sensors": [{...}],      # 传感器配置
  "algorithms": [{         # 本 scenario 要跑的算法
     "kind": "slam", "name": "occupancy_grid", "config": {...}, "robot_ids": ["r0"]
  }],
  "steps": 200, "dt": 0.1, "seed": 0,
  "metrics": ["slam.map_iou", "nav.reach_rate", ...]
}
```
`Scenario.from_dict` 构造、`Scenario.to_dict` 序列化、`Scenario.run()` 跑完返回 `EpisodeResult`。

### 5.3 `Benchmark`
跑一组 `Scenario`（可共享基配置 + 参数扫描），聚合指标为 `BenchmarkReport`：
- 每个 episode 的逐指标值；
- 跨 episode 的均值/标准差/分位数；
- 对照组对比表（如 `occupancy_grid` vs `pose_graph`）。

### 5.4 `metrics.py`
统一指标计算，按算法类别分组：
- SLAM：地图 IoU、位姿 ATE/RPE、回环数。
- 导航：成功率、路径长度、用时、碰撞次数。
- 集群控制：编队保持率、连通率、收敛步数、能量。
- 检测：precision/recall/F1、IoU、平均检测时延。
`Metrics` 在 `World.step` 中按算法实例收集，`Benchmark` 末尾聚合。

### 5.5 `registry.py`
进程内单例 `AlgorithmRegistry`：
```python
@register("slam", "occupancy_grid")
class OccupancyGridSLAM(SLAMBase): ...
registry.create("slam", "occupancy_grid")  # → 实例
registry.list("slam")
```

### 5.6 `orchestrator.py`
可选的进程级并行编排器，用于跨 scenario / 参数扫描的 fan-out。基于 `concurrent.futures.ProcessPoolExecutor`，每个 worker 跑一个 `Scenario.run()`，主进程汇总 `BenchmarkReport`。大规模扫描时使用；小规模单进程即可。

## 6. 可伸缩性 / 可扩展性策略

- **批处理热点**：`World._pairwise_comm`、`VectorizedSwarm.step` 用 numpy 广播一次算完全部 pairwise 距离 / 路径增益；`GaussianSplatEnv` 已有空间哈希，新增批量 `ray_cast_batch`。
- **算法解耦**：算法只通过 `World` 句柄 + `observations` 字典交互，互不直接依赖，可任意组合（如 SLAM + 导航 + 集群同时跑）。
- **后端可选**：通信信道可在 Layer 0/1/2 间切换；感知可在针孔相机 / Sionna RT 间切换；算法不感知后端差异。
- **并行编排**：`Orchestrator` 让参数扫描天然并行，单机多核可线性加速。

## 7. 验收

- `tests/test_algorithm_platform.py` 覆盖：四类算法各跑通一个参考实现、`Scenario.run` 产出 `EpisodeResult`、`Benchmark` 聚合报告、注册表发现、向量化集群 vs 串行结果一致。
- 接入 `tests/run_all.py`，全量测试绿。
