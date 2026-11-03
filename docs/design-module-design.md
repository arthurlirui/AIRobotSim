# 元器件级机器人构型设计模块（design/）

> 本文档记录 AI for robotics 元器件模拟模块的设计方案：在平台上用
> 元器件组合出机器人构型（morphology），并让构型与 3DGS 三维世界
> 交互。前置阅读：`docs/env-robot-module-design.md`。

## 1. 定位与目标

`robots/` 包提供**固定构型**的真实机器人（Go2、G1、Hexplorer…）；
`design/` 包解决相反的问题——**构型本身是研究对象**：

- 用户（或搜索算法）从元器件出发搭建任意树状构型
- 构型经过校验后编译成可仿真对象，与 3DGS 环境做接触/碰撞/行走交互
- 构型可序列化（JSON）分享、复现，或由 `random_design` 程序化生成
  （形态搜索 / 课程学习实验的原语）

## 2. 三层流水线

```
元器件（components.py）
  Link / Joint / Actuator / ContactPoint / SensorMount
        │  组装
        ▼
构型描述（config.py）
  RobotDesign：结构校验 + JSON 序列化 + random_design + 预设
        │  编译
        ▼
仿真对象（builder.py）
  SimRobot：树状 FK / 雅可比 / 阻尼最小二乘 IK / 执行器步进 / 环境交互
        │  驱动
        ▼
步态控制（locomotion.py）
  GaitController：摆动/支撑循环，足端落在 3DGS 重建地形上
```

## 3. 元器件（components.py）

| 元器件 | 职责 | 关键参数 |
|---|---|---|
| `Link` | 刚体（box/cylinder/sphere） | 质量、外形尺寸、包围球半径（碰撞） |
| `Joint` | 父子连杆连接（revolute/fixed） | origin 位姿、轴（URDF 约定：子连杆系）、限位、限速 |
| `Actuator` | 关节执行器 | 力矩上限、PD 增益（kp/kv） |
| `ContactPoint` | 连杆上的触地点（foot/wheel/pad） | 局部偏移、接触半径、摩擦系数 |
| `SensorMount` | 传感器挂载（camera/comm/imu） | 挂载位姿、相机类型（对接 `sensors/`） |

每个元器件自带 `validate()`，错误信息精确到元器件与字段。

## 4. 构型描述（config.py）

`RobotDesign.validate()` 强制五类结构不变量，编译器与步态控制器
都依赖它们：

1. 恰好一个根连杆（浮动基座）
2. 每个非根连杆恰好一个父关节（树，无重父）
3. 沿父链回溯必达根（无环/无断链）
4. 执行器只驱动存在的 revolute 关节
5. 接触点 / 传感器挂载引用存在的连杆

序列化用 dataclass → JSON，数组降级为 list；`from_json` 复原后重新
校验。`random_design(seed)` 生成 2–6 腿 × 2–3 关节的随机构型，腿沿
机体 ±X 对称分布——形态搜索的输入分布。预置 `preset_quadruped()`
（8 DOF）与 `preset_hexapod()`（12 DOF）作为参考构型与测试基准。

## 5. 仿真对象（builder.py）

### 5.1 运动学

- **FK**：按树递推 `T_child = T_parent @ origin @ R_joint`（URDF
  revolute 语义：绕子系轴的旋转作用于整个子树）
- **雅可比**：任意连杆系内点（`offset`，如足端接触点）对全部驱动
  关节的 3×N 几何雅可比；轴取 `T_parent.R @ origin_R @ axis`，
  支点取关节 origin 的世界位置。测试 [5] 用中心差分数值导数逐列
  对照（8 列全部吻合）
- **IK**：阻尼最小二乘（DLS），支持 offset；失败/成功均恢复原状态
  （调用方决定是否采用解）。2-DOF 腿链的可达集是二维曲面——IK
  对不可达目标正确返回 None（测试 [6] 用 FK 生成保证可达的目标，
  残差 0.39 mm）

### 5.2 执行器步进

显式二阶积分（tau/inertia → ddq → dq → q）在惯量 ~0.005 kg·m²、
kv ≈ 6、dt = 5 ms 的参数组合下产生极限环震荡（dq 每步冲到 ±10
限速后过冲反向）。改用**一阶速度跟踪**：

```
v_ss = tau / kv                       # 力矩对粘性阻尼的稳态速度
v_target = clip(v_ss, ±v_cap)         # 力矩/限速双上限
dq += (1 − e^(−kv/inertia·dt))·(v_target − dq)
q += dq·dt
```

PD 收敛（300 步后中位误差 0.024 rad），力矩始终在执行器限内
（测试 [7]）。关节限位为硬停（速度归零）。

### 5.3 与三维世界交互

- `update_contacts(env)`：每个接触点对 `GaussianSplatEnv.height_at`
  判定接地（点 − 半径 ≤ 地形高度）
- `body_collision(env)`：连杆包围球 vs splat 碰撞
- `snap_contacts_to_ground(env)`：垂直调整基座使最低接触点贴地
- `mount_pose(name)`：传感器挂载的世界位姿，直接对接
  `sensors/` 相机 `render(env, pos, orn)` 与 `comm.Pose`

## 6. 步态控制（locomotion.py）

`gait_groups` 按足数自动分组（6 足 tripod / 4 足 trot / 2 足交替）。
`GaitController.step(dt, heading, speed)` 每步：

1. 相位推进；跨越组起始相位时记录前落点、生成新落点
   （`stride` 沿航向 + **3DGS 地形高度**）
2. 摆动足沿正弦弧线（`lift` 抬高）从旧落点插值到新落点；
   支撑足钉在落点
3. 逐足在接触点处解 IK（带 offset），**按腿链合并**到关节指令向量
   （互不覆盖）
4. `robot.step(dt, q_desired)` PD 步进；机体沿航向推进

六足预置构型 1.2 s 步态前进 0.300 m（测试 [10]）；`random_design`
生成的 12-DOF 随机构型同样可编译、可步进（测试 [11]）。

## 7. 与平台其他模块的对接

| 对接点 | 接口 |
|---|---|
| `sensors/` 相机 | `mount_pose("head_cam")` → `render(env, pos, orn)` |
| `comm/` 通信 | `mount_pose("antenna")` → `Pose(position=...)`，整机接入 `CommNetworkSimulator` |
| `rl/` 学习 | `SimRobot` 状态可作观测；`GaitController` 参数（stride/lift/period）可作动作空间 |
| `core/` 3DGS 环境 | `update_contacts` / `body_collision` / `height_at` 落点 |
| URDF 生态 | `Joint.origin/axis` 遵循 URDF 语义，设计可双向转换（待实现 URDF 导出） |

## 8. 测试覆盖（tests/test_design.py，11 项）

| # | 内容 | 对照性质 |
|---|---|---|
| 1 | 破损构型被拒（多根/环/坏执行器） | 校验器负样本 |
| 2 | JSON 往返无损 | 序列化 |
| 3 | 5 个随机种子构型全部有效 | 生成器 |
| 4 | FK 髋偏置与旋转正交性 | 运动学 |
| 5 | 雅可比 vs 中心差分（8 列） | **数值对照** |
| 6 | IK 残差 0.39 mm + 状态恢复 | **可达性对照** |
| 7 | PD 收敛 0.024 rad + 力矩限内 | 动力学 |
| 8 | 4/4 足端接地 + 无机体碰撞 | **3DGS 交互** |
| 9 | 相机/天线挂载位姿 | 平台对接 |
| 10 | 六足 1.2 s 行走 0.300 m | **3DGS 行走** |
| 11 | 随机 12-DOF 构型可步进 | **泛化** |
