# 三方向扩展计划：机器人 + RL 训练 + 无线电磁仿真

## 一、探索结论

### 机器人资产
- Isaac Lab 内置 25+ 种机器人 `ArticulationCfg`（Cartpole/Franka/ANYmal/Unitree G1-H1-Go2/Spot 等）
- 添加机器人到现有 3DGS 场景：在 `InteractiveSceneCfg` 子类上加一个 `robot = SOME_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")` 字段
- 机器人 USD 资产在 AWS S3（`omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`），Isaac Sim 启动时自动下载
- 运动控制通过 `scene.articulations["robot"]` 获取句柄，调用 `set_joint_position_target()` / `write_data_to_sim()`

### RL 训练
- `rsl-rl-lib==5.0.1` **未安装**，需安装（不会降级 torch，纯 Python 包）
- Cartpole 任务已注册为 `Isaac-Cartpole-v0`，可直接训练
- 训练入口：`python scripts/reinforcement_learning/train.py --rl_library rsl_rl --task Isaac-Cartpole-v0 --headless`
- RL 适配器 `RslRlVecEnvWrapper` 已在 `isaaclab_rl/rsl_rl/` 中

### 无线电磁仿真
- `sionna-rt` 可安全安装在 isaacsim 环境（基于 Mitsuba 3 + Dr.Jit，**不依赖 torch**，不会降级 torch）
- Sionna RT 读 Mitsuba XML 场景（mesh + RF 材料），**不读 USD**
- 现有 `collision.py` 已有 Poisson 重建生成 trimesh，可直接扩展导出 OBJ/PLY
- WRF-GS 有开源代码（github.com/wenchaozheng/WRF-GS），但需单独环境

## 二、实施计划（三阶段）

### 阶段 1：机器人资产加载与运动控制

**目标目录结构：**
```
F:\Code\AIRobotSim\src\
├── gsplat/                      # (已有) 3DGS 集成
├── robots/                      # 新增：机器人集成模块
│   ├── __init__.py
│   ├── config.py                # 机器人配置数据类（支持 YAML）
│   ├── controller.py            # 运动控制（关节位置/速度/力矩控制）
│   └── loader.py                # 机器人资产加载（封装 ArticulationCfg）
├── examples/
│   ├── load_robot_scene.py      # 新增：3DGS场景 + 机器人 + 运动控制示例
│   └── ...
├── assets/
│   └── robots/                  # 本地机器人 USD（如需离线使用）
└── configs/
    └── robots/
        └── example_robot.yaml   # 机器人场景配置
```

**模块设计：**

1. **`robots/config.py`** — 机器人配置数据类
   - `RobotConfig`: 机器人类型（cartpole/franka/anymal_go2/unitree_g1 等）、初始位置/朝向、关节初始值
   - `RobotSceneConfig`: 3DGS 场景 + 机器人 + 相机，支持 YAML 加载
   - 内置机器人类型→ArticulationCfg 的映射表

2. **`robots/loader.py`** — 机器人资产加载
   - `get_robot_cfg(robot_type: str) -> ArticulationCfg`：从内置映射表获取
   - `add_robot_to_scene(scene_cfg, robot_config)`：在 InteractiveSceneCfg 上添加 robot 字段
   - 支持自定义 USD 路径（覆盖 Nucleus URL）

3. **`robots/controller.py`** — 运动控制
   - `RobotController` 类：封装 `scene.articulations["robot"]` 的操作
   - `set_joint_targets(positions)` / `apply_efforts(efforts)` / `get_observations()` → 关节位置/速度
   - `move_to_position(target_xy)`：简单的基座移动控制（适合轮式/四足）
   - `reset_to_initial_state()`

4. **`examples/load_robot_scene.py`** — 完整示例
   - 加载 3DGS 场景 + Cartpole 机器人
   - 运行几步物理仿真，打印关节状态
   - 渲染一帧包含机器人的场景
   - 然后切换为 Franka 机械臂，演示关节位置控制

5. **`configs/robots/example_robot.yaml`** — 配置文件
   ```yaml
   scene:
     name: "robot_demo"
     gsplat: { ... }  # 复用 3DGS 配置
     collision: { ... }
     ground: { enabled: true }
     robot:
       type: "cartpole"        # cartpole / franka / anymal_go2 / unitree_g1
       position: [0.0, 0.0, 2.0]
       joint_pos: { slider_to_cart: 0.0, cart_to_pole: 0.0 }
     cameras:
       - name: "robot_view"
         position: [5.0, 5.0, 3.0]
         target: [0.0, 0.0, 1.0]
   ```

**验证步骤：**
- 运行 `load_robot_scene.py --headless`：加载 Cartpole 在 3DGS 场景中，运行物理仿真 10 步，渲染包含机器人的场景帧
- 切换为 Franka，演示关节位置控制

### 阶段 2：RL 训练环境封装

**目标目录结构：**
```
F:\Code\AIRobotSim\src\
├── rl/                          # 新增：RL 训练环境封装
│   ├── __init__.py
│   ├── train.py                 # 训练入口脚本（封装 isaaclab train 命令）
│   ├── play.py                  # 评估/推理脚本
│   └── configs/
│       └── cartpole_rsl_rl.yaml # RL 训练参数配置
├── examples/
│   ├── train_cartpole.py        # 新增：Cartpole 训练一键示例
│   └── ...
└── scripts/
    ├── train_rl.bat             # 新增：RL 训练启动脚本
    └── play_rl.bat              # 新增：RL 评估启动脚本
```

**步骤：**

1. **安装 rsl-rl-lib==5.0.1**
   - `pip install rsl-rl-lib==5.0.1`（纯 Python，不降级 torch）
   - 验证：`python -c "import rsl_rl; print(rsl_rl.__version__)"`

2. **`rl/train.py`** — 训练入口封装
   - 封装 `isaaclab.bat train` 的调用，预设常用参数
   - 支持 `--task`、`--num_envs`、`--max_iterations`、`--headless`
   - 自动设置环境变量（NO_PROXY、OMNI_KIT_ACCEPT_EULA、ENABLE_CAMERAS）

3. **`rl/play.py`** — 评估/推理封装
   - 加载训练好的 checkpoint
   - 运行推理，导出 policy.pt / policy.onnx

4. **`examples/train_cartpole.py`** — Cartpole 训练一键示例
   - 调用 `rl/train.py` 训练 Cartpole（150 iterations，4096 envs 或更少如 64）
   - 训练完成后自动运行评估
   - 导出训练好的策略

5. **`scripts/train_rl.bat` / `play_rl.bat`** — 启动脚本

**验证步骤：**
- 运行 `train_cartpole.py --headless --num_envs 64 --max_iterations 10`：跑通训练流程（少量迭代验证管线）
- 运行评估：加载 checkpoint，运行推理

### 阶段 3：无线电磁仿真集成（Sionna RT）

**目标目录结构：**
```
F:\Code\AIRobotSim\src\
├── rf/                          # 新增：无线电磁仿真模块
│   ├── __init__.py
│   ├── sionna_channel.py        # Sionna RT 信道仿真封装
│   ├── mesh_exporter.py         # 3DGS → mesh (OBJ/PLY) 导出
│   ├── mitsuba_scene.py         # Mitsuba XML 场景生成
│   └── materials.py             # ITU RF 材料库
├── examples/
│   ├── rf_channel_sim.py        # 新增：RF 信道仿真示例
│   └── ...
├── assets/
│   └── rf/
│       └── scene_rf.xml         # Mitsuba 场景文件（生成）
└── configs/
    └── rf/
        └── example_rf.yaml      # RF 仿真配置
```

**步骤：**

1. **安装 sionna-rt**
   - `pip install sionna-rt`（基于 Mitsuba 3 + Dr.Jit，不依赖 torch，安全）
   - 验证：`python -c "import sionna.rt, mitsuba; print('ok')"`

2. **`rf/mesh_exporter.py`** — 从 3DGS 导出 mesh
   - 复用 `gsplat/collision.py` 的 `_read_gaussian_centers` 和 Poisson 重建
   - 额外导出为 `.obj` / `.ply` 文件（trimesh `mesh.export()`）
   - `export_mesh_for_rf(ply_path, output_mesh_path, method="poisson")` → OBJ/PLY

3. **`rf/materials.py`** — ITU RF 材料库
   - 预定义 ITU-R P.2040 材料（concrete、glass、wood、metal、plasterboard 等）
   - 每种材料含介电常数、电导率参数
   - `get_material(name) -> dict`

4. **`rf/mitsuba_scene.py`** — Mitsuba XML 场景生成
   - `build_mitsuba_scene(mesh_path, materials, output_xml_path)` → 生成 XML
   - XML 模板：`<scene>` + `<shape type="ply">` + `<radio_material>` 定义
   - 支持多材质分配（按区域/随机/统一）

5. **`rf/sionna_channel.py`** — Sionna RT 信道仿真封装
   - `compute_cir(scene_xml, tx_pos, rx_pos, frequency=3.5e9)` → `(a, tau, path_loss_db)`
   - `compute_coverage_map(scene_xml, tx_pos, resolution=[256,256])` → RSS 热力图
   - `compute_path_loss(tx_pos, rx_pos, frequency)` → 标量路径损耗
   - 支持多 Tx/Rx 批量计算

6. **`examples/rf_channel_sim.py`** — 完整示例
   - 从已有 3DGS PLY 导出 mesh → 生成 Mitsuba 场景
   - 在场景中放置 Tx（模拟基站）和 Rx（模拟机器人/UE）
   - 计算信道冲激响应（CIR）和路径损耗
   - 生成覆盖热力图并保存为 PNG

7. **`configs/rf/example_rf.yaml`** — RF 仿真配置
   ```yaml
   rf:
     mesh_source: "..."  # PLY 路径（复用 3DGS）
     mesh_method: "poisson"
     scene_xml: "assets/rf/scene_rf.xml"
     frequency: 3.5e9     # 3.5 GHz (5G n78)
     materials:
       default: "concrete"
     transmitters:
       - name: "gNB"
         position: [10.0, 0.0, 5.0]
         orientation: [0, 0, 0]
     receivers:
       - name: "UE_1"
         position: [0.0, 5.0, 1.5]
     coverage_map:
       resolution: [256, 256]
       max_depth: 5
   ```

**验证步骤：**
- 运行 `rf_channel_sim.py --headless`：从 3DGS PLY 导出 mesh → Sionna RT 计算信道 → 输出 CIR + 路径损耗 + 覆盖热力图

## 三、实施顺序与依赖

```
阶段 1: 机器人集成 (不依赖 RL/RF)
  └── 安装 rsl-rl-lib (为阶段 2 准备)
阶段 2: RL 训练 (依赖阶段 1 的机器人加载)
  └── 安装 sionna-rt (为阶段 3 准备)
阶段 3: 无线仿真 (依赖 3DGS mesh 导出，独立于 RL)
```

## 四、不包含在本次

- WRF-GS 训练（需单独环境，后续扩展）
- AODT 集成（需许可证）
- 自定义 RL 任务（3DGS 场景中的导航任务，后续扩展）
- 多机器人协调控制
- 实时 3DGS 重建管道
