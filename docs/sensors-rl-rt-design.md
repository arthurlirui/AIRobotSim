# 传感器、学习与射线追踪扩展设计（sensors / rl / Sionna RT）

> 本文档记录平台在"通信 + 3DGS 场景"核心之上新增的三块能力的设计方案：
> 相机光学模型（`sensors/`）、强化学习 / 模仿学习（`rl/`）、以及 Sionna RT
> 射线追踪信道（Layer 2）。前置阅读：`docs/env-robot-module-design.md` 与
> `docs/comm-module-design.md`。

## 1. 设计原则

三块扩展遵循同一套平台级约定：

1. **统一抽象，差异吸收在适配器内**——正如通信模块把 3G/5G/THz 统一到
   `ChannelModel.generate()`，传感器把五类相机统一到
   `CameraModel.render()`，学习模块把轮式/无人机统一到
   `MobileRobot.step()`；仿真内核只面对统一接口。
2. **几何保真、外观近似**——所有传感器渲染基于
   `GaussianSplatEnv.ray_cast` 逐像素投射，几何（深度/遮挡）是精确的，
   外观（颜色/辐射）用 splat 属性近似；真实感渲染仍由 Three.js /
   Splatshop 前端负责。
3. **对照组断言**——延续 jumper 的测试哲学：iToF 必须与单目在相位卷绕
   场景下可区分（`test_sensors` 对照组）；REINFORCE 后期回报必须优于
   前期（`test_rl` 对照组）；Sionna RT 在平地必须复现 FSPL 理论值
   （`test_comm_sionna` 对照组，实测 -83.30 vs 理论 -83.33 dB）。
4. **重依赖可选化**——sionna（含 Mitsuba/Drjit，约 1 GB）导入失败时
   工厂自动降级，核心功能零依赖 numpy 之外的重型库。

## 2. 移动机器人（robots/mobile_base.py）

`MobileRobot` 抽象基类与 `LeggedRobot` 共享 `RobotState` / 速度积分 /
环境交互契约，使 interaction 与 comm 层可以统一对待所有平台：

- **`WheeledRobot`**：差速驱动 unicyle 模型（v, ω → x', y', yaw'），
  命令限幅、地形跟随（`clamp_to_ground`）、轮速分解。
- **`UAV`**：四旋翼点质量模型，机体坐标速度指令经 yaw 旋转到世界系，
  地形地板约束（安全余量），roll/pitch 作为可设定姿态（传感器建模用）。

刻意不做完整动力学：平台的研究对象是集群/通信/感知层面的问题，
运动学级保真足够，且与足式机器人的现有简化动力学同级。

## 3. 相机光学模型（sensors/）

### 3.1 共享几何

`CameraIntrinsics`（针孔 + FOV 构造器 + 逐像素光线生成）与
`CameraExtrinsics`（机体 → 相机安装位姿，光学约定 X 右 / Y 下 / Z 前）
为所有相机共用；`CameraFrame` 是统一帧容器，各传感器按类型填充通道
（rgb / depth / confidence / thermal / nir）。

### 3.2 差异化测量物理（核心价值）

| 传感器 | 测量方程 | 噪声模型 |
|---|---|---|
| 单目 | 颜色 × 距离衰减 | 散粒 √signal + 高斯读出 |
| 双目 | Z = f·B/d | σ_Z = Z²/(f·B)·σ_d，d < d_min 时无效 |
| iToF | Z = (φ/2π)·c/2f_mod（模 Z_max 卷绕） | σ ∝ Z²，多径正偏差，confidence ∝ 1/Z² |
| 热红外 | splat 表观温度（或环境温度） | NETD 高斯 + 冻结的固定模式噪声 |
| 近红外 | P·ρ·veg_boost/(1+Z²) | 散粒 + 读出 |

iToF 的**相位卷绕**是最有研究价值的失真：9 m 目标在 20 MHz 调制下
报为 1.5 m——这正是实机 iToF 在长走廊里的经典失效模式，测试用单目
真值作对照组验证其可区分性。

### 3.3 渲染开销

逐像素 ray_cast 是 O(H·W·steps)，因此 `stride` 参数子采样是默认
使用方式（测试与 RL 观测都用 stride 4–16）；空间哈希网格让每次
ray_cast 只查询邻近 splat。

## 4. 强化学习 / 模仿学习（rl/）

### 4.1 NavigationEnv

Gymnasium 兼容契约 `reset/step → (obs, reward, terminated, truncated, info)`；
观测 = 机体系目标向量 + 归一化速度 + 前向射线扫描（复用 env 的
ray_cast，与传感器同源）。奖励 = 2×进度 + 20×到达 − 时间 − 碰撞。

**通信耦合奖励**是平台的特色设计：`channel_reward_weight > 0` 时，
每步用任意 `ChannelModel` 计算机器人 ↔ 基站的链路质量
（path_gain 归一化到 [0,1]）注入奖励，使"保持通信可达"成为可学习的
任务约束——这把 comm 模块与学习模块在环境层直接耦合起来。

### 4.2 训练算法

- `MLPPolicy`：2 隐层 tanh MLP + tanh 压缩高斯动作头，手写反传
  （accumulate_gradient / apply_gradients），梯度累积批间执行。
- `ReinforceTrainer`：滑动基线 + **批内优势归一化**（方差控制，
  修复前回报震荡）；实测 25 轮内平均回报 −1.9 → +4.2。
- `ImitationTrainer`：DAgger——学生滚动访问自身分布，专家标注，
  β 随迭代衰减；实测 MSE 0.78 → 0.52。

刻意不用 torch/GPU：平台的学习研究可平滑升级到 Stable-Baselines3
（环境契约已兼容），内置训练器保证"零依赖跑通全流程"。

## 5. Sionna RT 信道（comm/channel/sionna_rt.py）

### 5.1 适配器设计

`SionnaRTChannel(FIDELITY_LEVEL=2)` 实现与 Layer 0/1 完全相同的
`generate(tx, rx, t) → ChannelState`：

- **场景构建**：3DGS splat → 轴对齐盒网格（二进制 PLY，1σ 半宽）+
  砖墙 ITU 材质 + 地面；场景缓存复用，仅移动收发位置。
- **求解**：`PathSolver`（LOS + 镜面反射，可选散射/绕射），CIR
  聚合为 1 ns 分辨率功率时延谱填入 `ChannelState.cir`。
- **验证**：平地 100 m @ 3.5 GHz 得 −83.30 dB vs FSPL 理论 −83.33 dB；
  50 m 处地面反射双径叠加使距离差达 8 dB（物理正确，非 bug）。

### 5.2 接入点

- 工厂：`create_channel("sionna_rt", env=..., carrier_hz=...)`
  （别名 `ray_tracing` / `rt`）；sionna 缺失时注册表自动隐藏。
- 集群：直接传给 `CommNetworkSimulator`，每条链路独立射线追踪，
  多跳路由在射线追踪级链路质量上计算。

## 6. Splatshop 子模块

`external/splatshop` 是 3DGS 场景编辑器（Web 前端，git submodule）。
在平台工作流中的定位：**真实重建 → Splatshop 编辑/裁剪 → `.ply`
导入 `GaussianSplatEnv` → 机器人/传感器/通信仿真**。它不参与 Python
运行时，纯作为场景资产的前置处理工具。

## 7. 测试覆盖

| 模块 | 测试 | 对照组断言 |
|---|---|---|
| 轮式/无人机 | `test_mobile_robots` | 速度限幅 / 地形地板 / 状态契约 |
| 传感器 | `test_sensors` | iToF 卷绕 vs 单目真值；σ 随距离单调 |
| RL/IL | `test_rl` | 训练后期回报 > 前期；MSE 下降 |
| Sionna RT | `test_comm_sionna` | 射线追踪 ≈ FSPL 理论值（<1 dB） |
