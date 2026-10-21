# AIRobotSim 通信模拟模块设计方案

> 状态：设计草案（2026-10-21）
> 范围：在 AIRobotSim 实验平台中新增 `comm/` 模块，支持传统 3G/4G/5G 与先进 mmWave/THz/FSO/光纤制式，在高保真三维重建场景中模拟无线电波传播与移动机器人集群通信。

---

## 1. 设计目标

通信模块要解决的核心问题：**当多台足式机器人在 3DGS 高保真重建场景中移动时，它们之间的通信链路如何随几何遮挡、距离、制式、移动性而变化，并反过来影响集群协同行为**。

具体目标：

1. **多制式统一抽象**——3G/4G/5G/mmWave/THz/FSO/光纤共用同一套链路接口，底层模型可插拔。
2. **几何耦合**——链路质量由 3DGS 场景几何驱动（遮挡、反射面），而非纯统计模型。这是平台相对纯统计仿真的差异化能力。
3. **移动时变**——机器人位姿每步更新，信道随之演化（多普勒、遮挡切换、拓扑变化）。
4. **集群网络级**——支持 N 个节点、动态拓扑、中继、广播/单播，输出可供集群控制实验使用的链路状态序列。
5. **分层保真**——从"快速统计基线"到"物理光线追踪"分四档，按实验需求与算力选择，默认档零额外依赖。

非目标：不实现完整协议栈（MAC 调度、HARQ、RRC）；协议级保真通过可选外挂 ns-3/CORE 实现，不在核心模块内。

---

## 2. 调研结论（2024-2026 开源生态）

### 2.1 射线追踪 / 无线信道工具

| 工具 | 能力 | 与本平台契合度 |
|---|---|---|
| **Sionna / Sionna RT** (NVlabs, Apache-2.0 风格许可证) | 可微链路级+系统级，GPU 光线追踪（基于 Mitsuba 3），覆盖 sub-6 到 THz，ISAC 通感一体化 | **高保真首选**。Python 原生，可 import；3DGS 场景需转网格+材料 |
| **DeepMIMO v4** (ASU, Apache-2.0) | 场景化信道数据集生成，与 Sionna 互操作 | 数据驱动补充，离线生成 trace |
| **ns-3 + 5G-LENA** (GPL-2.0) | 协议级仿真，MAC/RLC/RRC 真实行为 | 远期可选，通过 ZMQ 桥接 |
| **QuaDRiGa** (Fraunhofer) | 3GPP 标准化统计信道，MATLAB | 轻量基线对照，非 Python |
| **CORE** (coreemu, Python) | 网络仿真器，支持移动节点无线 | 协议级可选后端 |

### 2.2 机器人集群 + 通信联合仿真

| 工作 | 贡献 | 借鉴点 |
|---|---|---|
| **ns3-cosim** (NIST) | ns-3 作为独立服务 + ZMQ/protobuf 消息总线解耦 | 外挂网络仿真器的架构范式 |
| **gazebo-sionna-pipeline** (MIT) | 仿真器场景几何 → Sionna RT 场景的完整转换管线 | 3DGS → Sionna 场景转换范本 |
| **ns3sionna** (arXiv 2412.20524) | Sionna 光线追踪接入 ns-3 替代统计衰落 | 几何信道 → 网络仿真可行 |
| **DT-RaDaR** (arXiv 2411.12284) | 差分光线追踪，移动时增量更新 RF 地图 | 移动场景效率优化 |
| **ARGoS RAB** | range-and-bearing 距离+角度门限通信模型 | 轻量级集群通信抽象模板 |
| **BotNet** (arXiv 2108.13606) | 随节点数/负载变化的 RF mesh PDR 模型 | 统计层保真度基准 |
| Isaac Sim 无原生无线模块 | 业界一致采用"物理引擎 + 外挂 Sionna" | 验证本平台外挂式架构合理性 |

### 2.3 先进制式建模成熟度

| 制式 | Python 生态 | 核心模型 | 集成难度 |
|---|---|---|---|
| 3G/4G | 高（CommPy/Sionna） | 成熟统计模型 | 低 |
| 5G NR sub-6 | 中（Sionna/DeepMIMO） | 3GPP TR 38.901 簇生 | 中 |
| mmWave (28-73 GHz) | 中（Sionna） | 38.901 + 氧气吸收 + 阻挡 | 中 |
| THz (100 GHz-1 THz) | 低 | 分子吸收 (HITRAN/hapi) + LoS 为主 | 高（需自实现） |
| FSO | 低 | Gamma-Gamma 湍流 + 指向误差 | 中（scipy.stats 可用） |
| 光纤 | 高 | 近似无损管道 | 极低 |

### 2.4 关键趋势

2024-2026 的明确趋势是 **几何光线追踪替代统计信道模型**——Sionna-Isaac Sim 闭环、Habitat-Sionna RT benchmark 等工作都把精确 3D 几何作为信道保真度的基础。本平台已有 3DGS 场景与 `GaussianSplatEnv.ray_cast`，天然契合这一趋势。

---

## 3. 分层架构

模块按保真度分四层，每层是上一层的增强，默认使用第 0 层（零额外依赖）：

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 3  协议级 (可选)    ns-3 / CORE via ZMQ 桥接          │
├─────────────────────────────────────────────────────────────┤
│  Layer 2  物理信道 (可选)  Sionna RT 光线追踪 (反射/衍射)    │
├─────────────────────────────────────────────────────────────┤
│  Layer 1  几何遮挡 (首推)  复用 GaussianSplatEnv.ray_cast    │
├─────────────────────────────────────────────────────────────┤
│  Layer 0  统计基线 (默认)  距离驱动路径损耗 + 衰落           │
└─────────────────────────────────────────────────────────────┘
```

各层职责：

- **Layer 0 统计基线**：自由空间路径损耗 + 可选对数距离 / Nakagami 衰落，距离驱动邻接（拓扑变化）。覆盖 80% 集群控制实验，纯 numpy，无新依赖。
- **Layer 1 几何遮挡**：对每对收发节点发射射线，与 `GaussianSplatEnv` 求交判定 LOS/NLOS。NLOS 时按穿透材质衰减或直接断链。直接体现"3D 场景耦合"，成本远低于完整光线追踪。
- **Layer 2 物理信道**：import Sionna RT，将 3DGS 场景转换为 Mitsuba 场景（参考 gazebo-sionna-pipeline），每帧对机器人位姿做光线追踪，输出路径增益 / 信道冲激响应 / 多普勒。支持差分更新（参考 DT-RaDaR）。
- **Layer 3 协议级**：通过 ZMQ/protobuf 桥接 ns-3 或 CORE，获得 MAC 竞争、排队、路由的真实涌现行为。ns3sionna 提供现成的"Sionna 信道 → ns-3"集成路径。

---

## 4. 模块结构

```
comm/                          通信模拟模块
  __init__.py
  channel/
    base.py                    ChannelModel 抽象基类 + ChannelState 数据类
    statistical.py             Layer 0：自由空间 / 对数距离 / 38.901 统计
    geometric_los.py           Layer 1：复用 GaussianSplatEnv.ray_cast 的 LOS 遮挡
    sionna_rt.py               Layer 2：Sionna RT 光线追踪（可选依赖）
    thz.py                     THz 分子吸收信道（hapi 可选）
    fso.py                     FSO Gamma-Gamma 湍流 + 指向误差
    fiber.py                   光纤近似无损管道
  phy/
    modulations.py             调制方式参数库（BPSK/QPSK/QAM…）→ BER/SNR 曲线
    link_budget.py             链路预算：从 ChannelState 推导 SNR/容量/BER
  network/
    node.py                    CommNode：机器人绑定的通信节点
    topology.py                动态拓扑：邻接矩阵 + 中继路由
    network_simulator.py       集群级仿真：每步更新所有链路 + 事件队列
    protocol_bridge.py         Layer 3：ns-3/CORE ZMQ 桥接（可选）
  scenarios/                   预置通信实验场景
    swarm_relay.py             集群中继拓扑实验
    mmwave_occlusion.py        mmWave 遮挡敏感性实验
    thz_short_range.py         THz 短距回传实验
    fso_pointing.py            FSO 指向误差实验
tests/
  test_comm_channel.py         各制式信道模型单元测试
  test_comm_network.py         集群拓扑/中继测试
  test_comm_simulation.py      通信+运动联合仿真集成测试
```

### 4.1 与现有模块的接口

通信模块通过两个薄接口与平台其余部分交互：

- **场景几何**：复用 `core.gaussian_splat_env.GaussianSplatEnv` 的 `ray_cast(origin, direction)` 与 `collision(point)`，不复制几何数据。
- **机器人位姿**：从 `robots.robot_base.RobotState` 读取 `base_position` / `orientation`，通信节点挂在机器人上，每仿真步由 `MotionController` 更新位姿后调用 `network_simulator.step()`。

```
MotionController.simulate()
  └─ 每步: 更新所有 robot 位姿
        └─ comm.network_simulator.step(poses, t)
              └─ 对每对节点: channel_model.generate(tx_pose, rx_pose, t) → ChannelState
              └─ 更新拓扑 / 触发链路事件
              └─ 返回 CommSnapshot（供控制层 / 前端使用）
```

---

## 5. 核心接口设计

### 5.1 统一信道接口（适配器模式）

所有制式对外暴露同一接口，底层模型差异由适配器吸收：

```python
# comm/channel/base.py
from dataclasses import dataclass
import numpy as np

@dataclass
class ChannelState:
    """一对收发节点在时刻 t 的链路状态。"""
    tx_id: str
    rx_id: str
    timestamp: float
    path_gain_db: float          # 路径增益 (dB)，-inf 表示断链
    los: bool                    # 是否视距无遮挡
    delay_ms: float              # 传播 + 处理延迟
    doppler_hz: float            # 多普勒频移
    bandwidth_hz: float          # 可用带宽
    snr_db: float                # 信噪比
    capacity_bps: float          # Shannon 容量估计
    ber: float                   # 误码率（按当前调制）
    cir: np.ndarray | None       # 信道冲激响应（高保真档才有）
    metadata: dict               # 制式特定信息（材质穿透、吸收谱…）

class ChannelModel:
    """所有制式信道模型的抽象基类。"""
    def generate(self, tx_pose: Pose, rx_pose: Pose, t: float,
                 env: GaussianSplatEnv | None = None) -> ChannelState:
        raise NotImplementedError

    @property
    def fidelity_level(self) -> int:
        """0=统计, 1=几何遮挡, 2=光线追踪, 3=协议级"""
        raise NotImplementedError
```

### 5.2 制式适配器

每个制式实现一个 `ChannelModel` 子类，参数化频率、带宽、发射功率、天线增益、调制方式：

| 制式 | 类 | 核心参数 | 底层模型 |
|---|---|---|---|
| 3G/4G | `CellularChannel` | 0.9/1.8/2.6 GHz, 5-20 MHz | 自由空间 + 对数距离衰落 |
| 5G NR | `NRChannel` | sub-6 / mmWave, 38.901 场景 | 3GPP TR 38.901 簇生（Sionna 或自实现） |
| mmWave | `MmWaveChannel` | 28/60/73 GHz, BF 增益 | 38.901 + 氧气吸收 + 几何阻挡 |
| THz | `THzChannel` | 100-300 GHz, 短距 LoS | 分子吸收 PL(f,d) + LoS 为主 |
| FSO | `FSOChannel` | 1550 nm, 视距 | Gamma-Gamma + 指向误差（耦合位姿抖动） |
| 光纤 | `FiberChannel` | 任意容量 | 近似无损固定管道 |

### 5.3 集群网络仿真器

```python
# comm/network/network_simulator.py
class CommNetworkSimulator:
    def __init__(self, env: GaussianSplatEnv, channel_model: ChannelModel):
        self.env = env
        self.channel = channel_model
        self.nodes: list[CommNode] = []

    def add_node(self, robot_id: str, pose: Pose, radio_params: dict): ...

    def step(self, poses: dict[str, Pose], t: float) -> CommSnapshot:
        """每仿真步调用：更新所有节点位姿，重算所有链路，更新拓扑。"""
        # 1. 对每对节点算 channel.generate()
        # 2. 按路径增益阈值更新邻接矩阵
        # 3. 计算中继路由（多跳）
        # 4. 入队链路事件（断链/建链/质量降级）
        # 5. 返回快照供控制层与前端使用

    def send(self, src_id: str, dst_id: str, payload: bytes) -> DeliveryResult:
        """按当前拓扑与链路质量投递消息，返回延迟/是否到达/丢包原因。"""
```

### 5.4 双分辨率

为平衡精度与规模：

- **高分辨率**：`ChannelState.cir` 非空，含完整信道冲激响应，用于关键链路分析、波束预测、信道估计 ML 训练。
- **低分辨率**：只用 `path_gain_db` / `capacity_bps` / `ber`，用于百级节点集群网络级仿真，每步仅算标量链路质量。

由 `ChannelModel` 的 `fidelity_level` 与 `network_simulator` 的规模配置共同决定。

---

## 6. 与 3DGS 场景的耦合

这是模块的核心差异化能力。具体耦合方式按层递进：

**Layer 1（几何遮挡，首推默认增强）**：
- 对每对节点，从 `tx_pose` 到 `rx_pose` 发射一条射线，调用 `GaussianSplatEnv.ray_cast(origin, direction)`。
- 有交点且交点距离 < 收发距离 → NLOS，按穿透高斯椭球的累积衰减估算穿透损耗，或直接断链（mmWave/THz/FSO 对遮挡极敏感）。
- 平台已有 `ray_cast` 实现，此层零新几何代码，仅复用。

**Layer 2（光线追踪，高保真可选）**：
- 将 3DGS 场景离线转换为 Sionna/Mitsuba 网格场景（参考 gazebo-sionna-pipeline 的转换流程）。转换器放 `comm/channel/scene_converter.py`，处理高斯椭球 → 网格 + 材质属性赋值。
- 转换是一次性预处理；运行时每帧用机器人位姿触发 Sionna RT，输出含反射/衍射/散射的完整信道。
- 移动场景采用差分更新（参考 DT-RaDaR）：只重算位姿变化超阈值的链路。

**材质属性**：3DGS 的 `color` / `scale` 可启发式映射到无线电材质参数（介电常数、反射系数），放 `comm/channel/material_db.py`，可手动校正。

---

## 7. 移动时变建模

机器人移动导致信道时变，模块按以下方式建模：

| 现象 | 建模方式 | 所需层 |
|---|---|---|
| 距离变化 → 路径损耗变化 | 每步重算 `generate()` | 0 |
| 遮挡变化（走进/走出遮挡） | 每步 ray_cast 重判 LOS | 1 |
| 多普勒频移 | `f_d = v·cos(θ)·f_c / c`，v 由位姿差分得 | 0+ |
| 拓扑变化 | 邻接矩阵按路径增益阈值更新 | 0+ |
| 中继切换 | 路由表随拓扑重算 | 0+ |
| 多径时变 | 完整 CIR 每帧更新 | 2 |

多普勒用解析公式低成本实现（速度 × 载频 / 光速），无需光线追踪。切换对足式集群（通常 Mesh 无基站）主要是中继路径重选，由 `topology.py` 的最短路径重算处理。

---

## 8. 实验场景预置

`comm/scenarios/` 预置典型实验，每个是一段可独立运行的脚本，同时作为集成测试：

1. **swarm_relay** — N 台四足在复杂地形中保持连通，当中继节点被遮挡时拓扑自愈，测量端到端投递率与延迟。
2. **mmwave_occlusion** — mmWave 链路对行人/障碍物遮挡的敏感性，对比 5G sub-6 的鲁棒性差异。
3. **thz_short_range** — THz 短距高速回传，机器人编队间距 vs 可用容量的关系。
4. **fso_pointing** — FSO 链路在机器人步态抖动下的指向误差与 BER，耦合 `RobotState` 位姿噪声。
5. **heterogeneous_swarm** — 混合制式集群：部分节点 5G、部分 mmWave、部分 FSO 回传，研究异构网络下的协同。

每个场景输出 `CommSnapshot` 序列，可序列化为 JSON 供 Three.js 前端可视化链路（射线颜色按质量渐变、断链用虚线）。

---

## 9. 前端可视化（Three.js）

`assets/preview.html` 已用 Three.js 渲染机器人，通信模块扩展可视化：

- **链路射线**：每对活跃链路画一条线，颜色按 `path_gain_db` 从绿（强）到红（弱），断链不画或画虚线。
- **遮挡高亮**：NLOS 链路用不同样式，射线与 3DGS 场景交点高亮。
- **拓扑面板**：侧栏显示邻接矩阵 / 中继路径 / 链路质量时间序列。
- **事件提示**：建链/断链/质量降级事件实时弹出。

数据通过 `CommSnapshot` 序列化进 `animation_data.json`，与现有动画数据通道并列。

---

## 10. 依赖与集成策略

### 10.1 默认零额外依赖

Layer 0 + Layer 1 只用 numpy + 现有 `GaussianSplatEnv`，在 `rtg132` 环境内开箱即用。

### 10.2 可选依赖（按需启用）

| 层 | 依赖 | 安装 |
|---|---|---|
| Layer 2 | `sionna`, `mitsuba3` | `pip install sionna` (GPU 推荐) |
| THz 吸收谱 | `hapi` (HITRAN) | `pip install hapi` |
| Layer 3 | `ns-3` 或 `coreemu` | 独立进程，ZMQ 桥接 |

`comm/__init__.py` 用 try/except 延迟导入可选依赖，缺失时降级到低层并打印提示，不阻断平台启动。

### 10.3 环境管理

按平台约定，所有依赖进 `rtg132` conda 环境。新增的可选依赖写入 `environment.yml` 的注释段，标注"通信模块 Layer 2/3 可选"。默认安装不含 Sionna，避免给非通信实验增加负担。

### 10.4 许可证合规

- Sionna：NVIDIA 源代码许可证，**商用需法务审查**，学术研究可用。
- ns-3 / CORE：GPL-2.0，作为独立进程桥接不传染本平台。
- hapi：HITRAN 数据 License，学术免费。

---

## 11. 测试策略

遵循平台"静默失败回归 + 强制对照组"哲学（pytest-only）：

| 测试 | 内容 |
|---|---|
| `test_comm_channel.py` | 各制式 `generate()` 输出 `ChannelState` 字段完整性；自由空间 PL 公式数值校验；LOS/NLOS 判定对照（同一场景有/无遮挡两组） |
| `test_comm_network.py` | 拓扑更新正确性；中继路由最短路径；节点移出范围断链 |
| `test_comm_simulation.py` | 通信+运动联合：机器人走过遮挡物，链路质量时间序列应呈现"连通→断→连通"变化；强制对照组（统计层 vs 几何遮挡层）结果差异符合预期 |
| `test_comm_scenarios.py` | 各预置场景可独立运行并产出有效 `CommSnapshot` |

关键回归断言：**几何遮挡层在存在遮挡物时的断链次数必须严格大于统计层**（否则几何耦合失效）。

---

## 12. 落地路线

| 阶段 | 内容 | 依赖 |
|---|---|---|
| **P0** | `comm/channel/base.py` 接口 + `statistical.py` (Layer 0) + `fiber.py` + `network_simulator.py` 基础拓扑 | 无新依赖 |
| **P1** | `geometric_los.py` (Layer 1，复用 `ray_cast`) + `network_simulator` 遮挡感知 + 前端链路可视化 | 无新依赖 |
| **P2** | `NRChannel` / `MmWaveChannel` (38.901 统计) + `thz.py` + `fso.py` + 预置场景 1-4 | hapi 可选 |
| **P3** | `sionna_rt.py` (Layer 2) + 3DGS→Sionna 场景转换器 + 差分更新 | sionna, mitsuba3 |
| **P4** | `protocol_bridge.py` (Layer 3, ns-3/CORE ZMQ) + 预置场景 5 | ns-3 或 coreemu |

P0-P1 是核心，零额外依赖，直接在 `rtg132` 内可开发测试，建议作为首个 PR 的范围。P2 扩展制式覆盖。P3-P4 是高保真/协议级增强，按研究需求推进。

---

## 13. 与相关工作的定位差异

| 维度 | 本平台 | Sionna 独立用 | ns-3 独立用 | gazebo-sionna |
|---|---|---|---|---|
| 场景来源 | 3DGS 高保真重建 | Mitsuba 网格 | 抽象 | Gazebo 网格 |
| 机器人模型 | 真实 URDF 足式 | 无 | 无 | URDF |
| 集群规模 | 中（十-百级） | 链路级 | 大 | 小 |
| 协议栈 | 可选外挂 | PHY 为主 | 完整 | 无 |
| 默认保真 | 统计+几何 | 光线追踪 | 协议级 | 光线追踪 |

差异化在于：**3DGS 重建场景 + 真实足式机器人 + 分层可插拔保真**，三者组合在现有开源工作中无直接对标。

---

## 参考资料

- Sionna / Sionna RT: https://github.com/NVlabs/sionna-rt
- DeepMIMO: https://www.deepmimo.net/
- ns3sionna: arXiv 2412.20524, https://github.com/tkn-tub/ns3sionna
- DT-RaDaR (差分光线追踪): arXiv 2411.12284
- gazebo-sionna-pipeline: https://github.com/Teleinfrastructure-Research-Lab/gazebo-sionna-pipeline
- ns3-cosim: https://github.com/usnistgov/ns3-cosim
- CORE: https://github.com/coreemu/core
- ARGoS: https://github.com/ilpincy/argos3
- BotNet: arXiv 2108.13606
- Sionna-Isaac Sim 闭环: arXiv 2610.08618
- Habitat-Sionna RT benchmark: arXiv 2610.08105
- 3GPP TR 38.901 (5G NR 信道模型)
- ITU-R P.676 / HITRAN (THz 分子吸收)
