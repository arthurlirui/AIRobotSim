# AIRobotSim 环境模块与机器人类型模块扩展设计

> 状态：设计方案（2026-10-24）
> 范围：扩展平台的环境模块以支持更多三维空间环境类型；扩展机器人模块以支持多种常见机器人类型（足式/轮式/飞行/机械臂/混合形态）和机器人集群。

---

## 1. 现状与问题

### 1.1 环境模块现状

当前环境能力集中在 `core/gaussian_splat_env.py` 的 `GaussianSplatEnv` 类：

- 唯一的环境表示方式：高斯椭球集合（3DGS）
- 四种程序化场景生成：`create_terrain` / `create_flat_ground` / `create_stairs` / `TerrainGenerator`（9 种地形类型）
- 空间查询：`height_at` / `normal_at` / `collision` / `ray_cast`

**不足**：
1. **无环境抽象基类**——`GaussianSplatEnv` 既是环境表示也是唯一实现，无法接入非 3DGS 环境（如网格、高度场、点云、OSM 城市模型）。
2. **无环境物理属性**——不支持可调重力、流体（水下）、风等场景物理参数。
3. **无障碍物/实体管理**——障碍物以 splat 形式混入地形，无法区分静态地形与动态障碍物。
4. **无语义标注**——不支持区域类型、可通行性、任务锚点等语义信息。
5. **无多环境批量**——不支持并行多环境（RL 批量训练需要）。

### 1.2 机器人模块现状

当前机器人能力集中在 `robots/robot_base.py` 的 `LeggedRobot` 抽象基类：

- 三种足式机器人：`Biped`（G1）、`Quadruped`（Go2）、`Hexapod`（Hexplorer）
- URDF 工厂：`urdf_robot.py` 可从真实 URDF 构建（Go2/Go1/A1/B1/H1/Laikago/ANYmal B/C）
- 运动学：FK / 解析 IK（3-DoF 腿） / 数值 IK 后备
- 步态：`control/gait_planner.py` 支持 walk/trot/wave/tripod/static_walk

**不足**：
1. **基类绑死足式**——`LeggedRobot` 的 `legs` / `foot_positions` / `support_polygon` / `is_stable` 假设所有机器人都有腿和足端接触，无法表示轮式、飞行、机械臂。
2. **无轮式/飞行/机械臂支持**——平台定位是"机器人集群通信交互实验"，但只支持足式，限制了实验场景的多样性。
3. **无集群管理**——单机器人实例无 robot_id/命名空间概念，无多机器人编排层，`comm/` 模块的 `CommNetworkSimulator` 虽支持多节点但需手动逐个 `add_node`。
4. **无混合形态**——不支持轮足结合（如 Unitree B2-W）等混合形态机器人。

---

## 2. 设计目标

1. **环境模块**：引入统一 `Environment` 抽象基类，`GaussianSplatEnv` 成为其子类；新增网格环境、高度场环境、OSM 城市环境等；支持环境物理属性（重力/流体/风）、障碍物管理、语义标注、多环境批量。
2. **机器人模块**：向上抽象出形态无关的 `Robot` 基类，`LeggedRobot` 成为其子类（零迁移现有足式代码）；新增 `WheeledRobot` / `AerialRobot` / `ManipulatorRobot` / `HybridRobot` 平级子类；支持异构机器人集群管理。
3. **零破坏**：现有足式机器人代码、`comm/` 模块、测试套件不受影响。
4. **零额外依赖**：核心抽象层纯 numpy，与现有 `rtg132` 环境兼容。

---

## 3. 环境模块设计

### 3.1 分层架构

```
Environment（统一抽象基类）
├── 地形层（Terrain）          高度场 / 网格 / 3DGS / 平面
├── 障碍物层（Obstacles）       静态障碍物 + 动态障碍物
├── 物理属性层（PhysicsWorld）  重力 / 流体 / 风 / 材质
├── 语义层（Semantics）         区域类型 / 可通行性 / 任务锚点
└── 批量层（EnvBatch）          多环境克隆与并行

GaussianSplatEnv（现有）→ Environment 子类（3DGS 后端）
MeshEnv（新增）              → 从 OBJ/STL/GLB 网格加载
HeightFieldEnv（新增）       → 从高度矩阵/PNG 加载
OSMCityEnv（新增）           → 从 OpenStreetMap 生成城市
IndoorEnv（新增）            → 室内房间/走廊/仓库（程序化或 HSSD）
PointCloudEnv（新增）        → 从 LiDAR 点云加载
```

### 3.2 统一环境接口

```python
# core/environment.py (新增)

class Environment(ABC):
    """所有三维空间环境的统一抽象基类。"""

    # --- 场景几何查询（所有子类必须实现）---
    @abstractmethod
    def height_at(self, x: float, y: float) -> float: ...
    @abstractmethod
    def normal_at(self, x: float, y: float) -> np.ndarray: ...
    @abstractmethod
    def collision(self, point: np.ndarray, radius: float = 0.1) -> list: ...
    @abstractmethod
    def ray_cast(self, origin: np.ndarray, direction: np.ndarray,
                 max_dist: float = 10.0) -> Optional[float]: ...

    # --- 边界 ---
    @property
    @abstractmethod
    def bounds(self) -> tuple[np.ndarray, np.ndarray]: ...  # (min, max)

    # --- 物理属性（有默认实现，子类可覆盖）---
    @property
    def physics(self) -> "PhysicsWorld": ...   # 重力/流体/风
    @property
    def gravity(self) -> np.ndarray: ...        # 默认 [0,0,-9.81]

    # --- 障碍物管理 ---
    def add_obstacle(self, obstacle: "Obstacle") -> None: ...
    def remove_obstacle(self, obstacle_id: str) -> None: ...
    @property
    def obstacles(self) -> list["Obstacle"]: ...

    # --- 语义标注 ---
    def add_semantic_region(self, region: "SemanticRegion") -> None: ...
    def is_traversable(self, x: float, y: float) -> bool: ...
    @property
    def task_anchors(self) -> dict: ...   # 导航目标/抓取点/巡检路径

    # --- 统计与序列化 ---
    @abstractmethod
    def stats(self) -> dict: ...
    def to_dict(self) -> dict: ...        # 供前端可视化序列化
```

### 3.3 物理世界属性

```python
# core/environment.py

@dataclass
class PhysicsWorld:
    """环境的全局物理参数。"""
    gravity: np.ndarray = field(default_factory=lambda: np.array([0, 0, -9.81]))
    # 流体（水下/空中）
    fluid_density: float = 0.0        # kg/m³，0 = 无流体
    fluid_viscosity: float = 0.0
    # 风/扰动
    wind: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # 默认接触材质
    default_friction: float = 0.8
    default_restitution: float = 0.0
```

预设物理世界：

| 预设 | 重力 | 流体密度 | 场景 |
|---|---|---|---|
| `EARTH` | -9.81 m/s² | 0 | 默认地面 |
| `MOON` | -1.62 m/s² | 0 | 月面/低重力 |
| `ZERO_G` | 0 | 0 | 太空 |
| `UNDERWATER` | -9.81 | 1000 (水) | 水下 |
| `MARS` | -3.71 m/s² | 0.02 (稀薄CO₂) | 火星 |

### 3.4 障碍物与语义

```python
@dataclass
class Obstacle:
    """环境中的障碍物（静态或动态）。"""
    obstacle_id: str
    position: np.ndarray
    shape: str                 # "box" / "sphere" / "cylinder" / "mesh"
    dimensions: np.ndarray     # 形状参数
    is_dynamic: bool = False   # 动态障碍物可移动
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    material: dict = field(default_factory=dict)  # 摩擦/恢复系数

@dataclass
class SemanticRegion:
    """语义区域标注。"""
    name: str                  # "room_A" / "corridor" / "danger_zone"
    region_type: str           # "room" / "corridor" / "warehouse_aisle" / "outdoor"
    polygon: np.ndarray        # (N,2) XY 多边形边界
    traversable: bool = True
    properties: dict = field(default_factory=dict)
```

### 3.5 新增环境类型

#### 3.5.1 MeshEnv（网格环境）

从 OBJ/STL/GLB 三角网格加载环境。用于 CAD/BIM 模型、真实重建网格化结果、Habitat 场景。

```python
class MeshEnv(Environment):
    """从三角网格加载的环境。"""
    def __init__(self, mesh_path: str, physics: PhysicsWorld = None): ...
    # height_at: 对网格做 XY 投影求最高交点
    # collision: 点是否在网格碰撞体内（BVH 加速）
    # ray_cast: 射线与三角网格求交（trimesh.ray.intersects_first）
```

依赖：`trimesh`（纯 Python，可 pip install），提供 BVH 加速的射线投射与碰撞检测。

#### 3.5.2 HeightFieldEnv（高度场环境）

从高度矩阵（numpy 数组或 PNG）加载。这是 MuJoCo/legged_gym 的标准地形表示。

```python
class HeightFieldEnv(Environment):
    """高度场环境。"""
    def __init__(self, heightfield: np.ndarray,    # (rows, cols) 高度矩阵
                 horizontal_scale: float = 0.1,    # 米/格
                 vertical_scale: float = 1.0,
                 physics: PhysicsWorld = None): ...
    # height_at: 双线性插值
    # normal_at: 有限差分梯度
    # ray_cast: 沿射线步进采样高度场
```

可与现有 `TerrainGenerator` 无缝衔接——`TerrainGenerator` 产出的高度场可直接构造 `HeightFieldEnv`。

#### 3.5.3 OSMCityEnv（城市环境）

从 OpenStreetMap 生成城市街道环境，用于自动驾驶/室外导航实验。

```python
class OSMCityEnv(Environment):
    """从 OpenStreetMap 数据生成的城市环境。"""
    def __init__(self, osm_data,     # OSM XML 或经纬度范围
                 building_height: float = 10.0,
                 road_width: float = 8.0,
                 physics: PhysicsWorld = None): ...
    # 建筑 → 障碍物（Box）
    # 道路 → 可通行语义区域
    # 地面 → 平面 + 道路材质
```

依赖：`osmnx`（可选，用于下载 OSM 数据）；或直接解析 OSM XML。

#### 3.5.4 IndoorEnv（室内环境）

程序化生成室内场景（房间、走廊、仓库），用于导航/搜救/物流实验。

```python
class IndoorEnv(Environment):
    """程序化室内环境。"""
    @classmethod
    def create_rooms(cls, n_rooms: int = 5, corridor_width: float = 1.5,
                     room_size: tuple = (4.0, 4.0), seed: int = 42): ...
    @classmethod
    def create_warehouse(cls, n_aisles: int = 4, aisle_width: float = 2.0,
                         shelf_height: float = 2.0): ...
    @classmethod
    def create_corridor(cls, length: float = 20.0, width: float = 2.0,
                        n_turns: int = 2): ...
    # 墙体 → 障碍物 + 语义区域
    # 房间 → SemanticRegion(traversable=True)
    # 墙 → SemanticRegion(traversable=False)
```

#### 3.5.5 GaussianSplatEnv（现有，改为 Environment 子类）

```python
class GaussianSplatEnv(Environment):  # 新增继承
    """3DGS 环境（现有实现，新增 Environment 接口实现）。"""
    # 现有 height_at/normal_at/collision/ray_cast 不变
    # 新增 bounds 属性（从 stats() 推导）
    # 新增 physics 属性（默认 EARTH）
    # 障碍物/语义方法继承 Environment 默认实现
```

### 3.6 环境工厂与注册

```python
# core/environment.py

_ENV_REGISTRY: dict[str, type[Environment]] = {}

def register_env(name: str, env_class: type[Environment]): ...
def create_env(name: str, **kwargs) -> Environment: ...
def list_envs() -> list[str]: ...

# 预注册
register_env("3dgs", GaussianSplatEnv)
register_env("mesh", MeshEnv)
register_env("heightfield", HeightFieldEnv)
register_env("osm_city", OSMCityEnv)
register_env("indoor", IndoorEnv)
register_env("pointcloud", PointCloudEnv)
```

### 3.7 多环境批量

```python
# core/env_batch.py (新增)

class EnvBatch:
    """管理多个并行环境（RL 批量训练场景）。"""
    def __init__(self, env_configs: list[dict]): ...
    def __getitem__(self, i: int) -> Environment: ...
    def __len__(self) -> int: ...
    def reset(self, env_ids: list[int] = None) -> None: ...
    def clone(self, n: int, spacing: float = 10.0) -> "EnvBatch": ...
    # 同构克隆：同一配置复制 n 份，env_origins 网格排列
```

---

## 4. 机器人类型模块设计

### 4.1 抽象层次

```
Robot（统一基类：形态无关）
├── LeggedRobot（足式：浮动基 + 接触 + 步态）  ← 现有，改为 Robot 子类
│   ├── Biped / Quadruped / Hexapod          ← 现有
│   └── URDFRobot                             ← 现有
├── WheeledRobot（轮式：非完整约束 + 驱动轮）   ← 新增
│   ├── DifferentialRobot（差速：TurtleBot）
│   ├── OmniRobot（全向：Mecanum）
│   └── AckermannRobot（阿克曼：汽车类）
├── AerialRobot（飞行：6DoF 刚体 + 推力）       ← 新增
│   └── Quadrotor（四旋翼：Crazyflie/PX4 Iris）
├── ManipulatorRobot（机械臂：固定基 + IK）     ← 新增
│   └── ArmRobot（Franka/UR5/KUKA）
├── SnakeRobot（蛇形：超冗余串联）              ← 新增（远期）
└── HybridRobot（混合：多模式切换）             ← 新增
    └── WheeledLeggedRobot（轮足：B2-W）
```

### 4.2 统一 Robot 基类

```python
# robots/robot.py (新增)

class BaseState(Enum):
    FLOATING = "floating"   # 足式/飞行
    FIXED = "fixed"         # 机械臂
    PLANAR = "planar"       # 轮式（地面平面）

class ActionType(Enum):
    JOINT_TORQUE = "joint_torque"      # 足式/机械臂力控
    JOINT_POSITION = "joint_position"  # 足式/机械臂 PD
    BASE_TWIST = "base_twist"          # 轮式 (v,ω) / 飞行速度指令
    MOTOR_MIXER = "motor_mixer"        # 飞行推力/力矩原始

@dataclass
class RobotState:
    """所有机器人形态共有的状态。"""
    base_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))  # RPY
    base_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    base_angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    joint_angles: dict[str, float] = field(default_factory=dict)
    joint_velocities: dict[str, float] = field(default_factory=dict)
    contacts: dict[str, bool] = field(default_factory=dict)  # 接触点接触状态
    t: float = 0.0

class Robot(ABC):
    """所有机器人形态的统一抽象基类。"""

    def __init__(self, name: str, mass: float = 10.0,
                 base_position: np.ndarray = None,
                 robot_id: str = None):
        self.name = name
        self.mass = mass
        self.robot_id = robot_id or name
        self.state = RobotState(
            base_position=base_position.copy() if base_position is not None else np.zeros(3),
        )

    # --- 形态标识（子类声明）---
    @property
    @abstractmethod
    def base_type(self) -> BaseState: ...
    @property
    @abstractmethod
    def action_type(self) -> ActionType: ...
    @property
    @abstractmethod
    def num_dof(self) -> int: ...
    @property
    def num_actuated_dof(self) -> int:
        """可控自由度（浮动基 6DoF 不可全控）。"""
        return self.num_dof

    # --- 位姿（所有形态统一）---
    def get_base_pose(self) -> tuple[np.ndarray, np.ndarray]: ...  # (pos, rpy)
    def set_base_pose(self, position, orientation=None): ...
    def get_base_velocity(self) -> tuple[np.ndarray, np.ndarray]: ...

    # --- 关节空间（所有形态统一接口）---
    def get_joint_positions(self) -> dict[str, float]: ...
    def set_joint_positions(self, angles: dict[str, float]): ...
    def get_joint_velocities(self) -> dict[str, float]: ...

    # --- 动作（统一入口，子类实现语义翻译）---
    def step_action(self, action) -> None:
        """应用动作。语义由 action_type 决定。"""
        self._apply_action(action)
    @abstractmethod
    def _apply_action(self, action) -> None: ...

    # --- 观测（统一 dict 输出）---
    def get_observation(self) -> dict:
        return {
            "base_pose": (*self.get_base_pose()),
            "base_vel": (*self.get_base_velocity()),
            "joint_state": self.get_joint_positions(),
            "contacts": dict(self.state.contacts),
            "sensors": self._read_sensors(),
        }
    def _read_sensors(self) -> dict:
        return {}  # 子类按需挂载传感器

    # --- 接触（形态相关）---
    def get_contacts(self) -> dict[str, bool]:
        return dict(self.state.contacts)

    # --- 生命周期 ---
    def reset(self, base_position=None, joint_angles=None): ...
    def step(self, dt: float):
        """推进一个仿真步（积分运动学/动力学）。"""
        self.state.t += dt

    # --- 传感器组件（组合模式）---
    def add_sensor(self, name: str, sensor: "Sensor"): ...
    def get_sensor(self, name: str) -> "Sensor": ...
```

### 4.3 LeggedRobot 改造（零破坏）

```python
# robots/robot_base.py（改造）

class LeggedRobot(Robot):  # 改为继承 Robot
    """足式机器人（浮动基 + 腿 + 接触 + 步态）。"""

    @property
    def base_type(self) -> BaseState:
        return BaseState.FLOATING

    @property
    def action_type(self) -> ActionType:
        return ActionType.JOINT_POSITION  # 默认 PD 控

    @property
    def num_dof(self) -> int:
        return 6 + sum(leg.n_joints for leg in self.legs.values())  # 浮动基 + 关节

    # 现有 legs / foot_positions / FK / IK / support_polygon / is_stable 全部保留
    # _apply_action: 将关节位置指令写入 joint_angles
    # get_contacts: 返回足端接触状态
```

**关键保证**：`LeggedRobot` 的所有现有公开 API（`legs`、`foot_positions`、`leg_forward_kinematics`、`leg_inverse_kinematics`、`support_polygon`、`is_stable`、`update_foot_positions`）签名不变。现有 `Biped`/`Quadruped`/`Hexapod`/`URDFRobot` 零改动——它们继承 `LeggedRobot`，`LeggedRobot` 继承 `Robot`，新增的抽象方法由 `LeggedRobot` 统一实现。

### 4.4 新增机器人类型

#### 4.4.1 WheeledRobot（轮式）

```python
# robots/wheeled.py (新增)

class WheeledRobot(Robot):
    """轮式机器人基类（非完整约束）。"""
    @property
    def base_type(self) -> BaseState:
        return BaseState.PLANAR
    @property
    def action_type(self) -> ActionType:
        return ActionType.BASE_TWIST  # (v, ω)

    def __init__(self, name, mass, wheel_radius, wheelbase, ...):
        super().__init__(name, mass, ...)
        self.wheel_radius = wheel_radius
        self.wheelbase = wheelbase
        self.wheels: dict[str, Wheel] = {}  # 驱动轮/转向轮

    def _apply_action(self, action):
        """action = (linear_vel, angular_vel) → 轮速逆解 → 积分位姿。"""
        v, omega = action
        # 子类实现 (v,ω) → 各轮速度的逆解
        wheel_speeds = self._inverse_kinematics(v, omega)
        # 非完整约束运动学积分
        self._integrate_kinematics(v, omega, dt)

class DifferentialRobot(WheeledRobot):
    """差速驱动（TurtleBot3 式）。"""
    # 左右轮速度差转向

class OmniRobot(WheeledRobot):
    """全向轮（Mecanum）——可任意方向平移。"""

class AckermannRobot(WheeledRobot):
    """阿克曼转向——前轮转向约束，转弯半径受限。"""
```

#### 4.4.2 AerialRobot（飞行）

```python
# robots/aerial.py (新增)

class AerialRobot(Robot):
    """飞行机器人基类（6-DoF 刚体 + 推力）。"""
    @property
    def base_type(self) -> BaseState:
        return BaseState.FLOATING
    @property
    def action_type(self) -> ActionType:
        return ActionType.MOTOR_MIXER  # 各电机推力

    def __init__(self, name, mass, motors: list["Motor"], ...):
        super().__init__(name, mass, ...)
        self.motors = motors  # 旋翼位置 + 推力方向

    def _apply_action(self, action):
        """action = 各电机推力 → 力/力矩 → 6DoF 刚体动力学积分。"""
        forces, torques = self._mixer(action)
        self._integrate_rigid_body(forces, torques, dt)

class Quadrotor(AerialRobot):
    """四旋翼（Crazyflie / PX4 Iris 式）。"""
    # 4 个旋翼 X 形布置，混控矩阵
```

#### 4.4.3 ManipulatorRobot（机械臂）

```python
# robots/manipulator.py (新增)

class ManipulatorRobot(Robot):
    """机械臂（固定基 + 串联关节链）。"""
    @property
    def base_type(self) -> BaseState:
        return BaseState.FIXED
    @property
    def action_type(self) -> ActionType:
        return ActionType.JOINT_POSITION  # 或 JOINT_TORQUE

    def __init__(self, name, mass, joints: list["ArmJoint"], ...):
        super().__init__(name, mass, ...)
        self.joints = joints  # 串联关节链

    # FK / IK / Jacobian
    def forward_kinematics(self, joint_angles) -> np.ndarray: ...  # 末端位姿
    def inverse_kinematics(self, target_pose) -> list[float]: ...
    def jacobian(self, joint_angles) -> np.ndarray: ...

class ArmRobot(ManipulatorRobot):
    """通用机械臂（从 URDF 加载，如 Franka 7-DoF / UR5 6-DoF）。"""
```

#### 4.4.4 HybridRobot（混合形态）

```python
# robots/hybrid.py (新增)

class OperationMode(Enum):
    WHEELED = "wheeled"
    LEGGED = "legged"

class HybridRobot(Robot):
    """混合形态机器人（如轮足结合 B2-W）。"""
    def __init__(self, name, wheeled: WheeledRobot, legged: LeggedRobot, ...):
        super().__init__(name, ...)
        self.wheeled_component = wheeled
        self.legged_component = legged
        self.mode = OperationMode.WHEELED

    def switch_mode(self, mode: OperationMode): ...
    def _apply_action(self, action):
        if self.mode == OperationMode.WHEELED:
            self.wheeled_component._apply_action(action)
        else:
            self.legged_component._apply_action(action)
```

### 4.5 机器人工厂与注册

```python
# robots/__init__.py（扩展）

_ROBOT_REGISTRY: dict[str, type[Robot]] = {}

def register_robot(name: str, robot_class: type[Robot]): ...
def create_robot(name: str, **kwargs) -> Robot: ...
def list_robots() -> list[str]: ...

# 预注册
register_robot("biped", Biped)
register_robot("quadruped", Quadruped)
register_robot("hexapod", Hexapod)
register_robot("differential", DifferentialRobot)
register_robot("omni", OmniRobot)
register_robot("ackermann", AckermannRobot)
register_robot("quadrotor", Quadrotor)
register_robot("arm", ArmRobot)
# URDF 机器人动态注册
for name in list_available_robots():  # go2, a1, h1, ...
    register_robot(name, URDFRobot)
```

---

## 5. 机器人集群设计

### 5.1 SwarmManager（集群管理器）

```python
# robots/swarm.py (新增)

@dataclass
class SwarmConfig:
    """集群配置。"""
    robot_configs: list[dict]    # 每个机器人的 {type, name, pose, params}
    env: Environment = None      # 共享环境
    collision_filter: str = "all"  # "all" / "none" / "same_type" / "cross_type"

class SwarmManager:
    """管理多个异构机器人的集群。"""

    def __init__(self, config: SwarmConfig):
        self.robots: dict[str, Robot] = {}
        self.env = config.env
        self._build_swarm(config)

    def _build_swarm(self, config):
        for rc in config.robot_configs:
            robot = create_robot(rc["type"], name=rc["name"],
                                 base_position=rc.get("pose"), **rc.get("params", {}))
            self.add_robot(robot)

    def add_robot(self, robot: Robot) -> None: ...
    def remove_robot(self, robot_id: str) -> None: ...
    def get_robot(self, robot_id: str) -> Robot: ...
    def get_robots_by_type(self, robot_type: str) -> list[Robot]: ...

    # --- 集群步进 ---
    def step(self, dt: float, actions: dict[str, any] = None):
        """推进所有机器人一个仿真步。"""
        for robot in self.robots.values():
            if actions and robot.robot_id in actions:
                robot.step_action(actions[robot.robot_id])
            robot.step(dt)
        self._update_contacts()

    def _update_contacts(self):
        """更新所有机器人的接触状态（查询环境 + 机器人间碰撞）。"""
        for robot in self.robots.values():
            # 查询环境碰撞
            # 查询机器人间碰撞（按 collision_filter）
            ...

    # --- 集群观测 ---
    def get_observation(self) -> dict[str, dict]:
        return {rid: r.get_observation() for rid, r in self.robots.items()}

    def get_poses(self) -> dict[str, tuple]:
        """所有机器人当前位姿（供 comm/ 模块使用）。"""
        return {rid: r.get_base_pose() for rid, r in self.robots.items()}

    # --- 与 comm 模块集成 ---
    def to_comm_network(self, channel_model) -> "CommNetworkSimulator":
        """从集群构建通信网络仿真器。"""
        from comm import CommNetworkSimulator, Pose
        sim = CommNetworkSimulator(channel_model, env=self.env)
        for rid, robot in self.robots.items():
            pos, orient = robot.get_base_pose()
            sim.add_node(rid, Pose(position=pos, orientation=orient),
                         label=robot.name)
        return sim

    # --- 集群统计 ---
    def summary(self) -> dict:
        type_counts = {}
        for r in self.robots.values():
            t = r.__class__.__name__
            type_counts[t] = type_counts.get(t, 0) + 1
        return {"n_robots": len(self.robots), "type_counts": type_counts}
```

### 5.2 异构集群示例

```python
from robots import create_robot, SwarmManager, SwarmConfig
from core.environment import create_env

env = create_env("indoor", variant="warehouse", n_aisles=4)
config = SwarmConfig(
    robot_configs=[
        {"type": "quadruped", "name": "scout_0", "pose": np.array([0, 0, 0.35])},
        {"type": "quadruped", "name": "scout_1", "pose": np.array([2, 0, 0.35])},
        {"type": "differential", "name": "carrier_0", "pose": np.array([0, 3, 0.1])},
        {"type": "quadrotor", "name": "drone_0", "pose": np.array([5, 5, 3.0])},
    ],
    env=env,
)
swarm = SwarmManager(config)
print(swarm.summary())  # {'n_robots': 4, 'type_counts': {'Quadruped': 2, 'DifferentialRobot': 1, 'Quadrotor': 1}}
```

### 5.3 与 comm 模块的集成

`SwarmManager.to_comm_network()` 自动从集群构建 `CommNetworkSimulator`，将每个机器人的位姿映射为通信节点。每仿真步：

```python
swarm.step(dt, actions)
comm_sim = swarm.to_comm_network(channel)  # 或复用已有实例
snap = comm_sim.step(poses=swarm.get_poses(), t=swarm.robots[...].state.t)
```

---

## 6. 传感器组件设计

```python
# robots/sensors.py (新增)

class Sensor(ABC):
    """可插拔传感器组件。"""
    @property
    @abstractmethod
    def sensor_type(self) -> str: ...
    @abstractmethod
    def read(self, robot: Robot, env: Environment) -> dict: ...

class IMU(Sensor):
    """惯性测量单元——线加速度 + 角速度 + 姿态。"""
    def read(self, robot, env):
        return {"accel": robot.state.base_velocity / dt,
                "gyro": robot.state.base_angular_velocity,
                "orientation": robot.state.base_orientation}

class Lidar(Sensor):
    """激光雷达——多射线距离测量。"""
    def __init__(self, n_rays=360, max_range=10.0, fov_deg=360): ...
    def read(self, robot, env):
        # 从机器人位姿发射射线，调用 env.ray_cast
        ...

class GPSSensor(Sensor):
    """GPS——全局位置（飞行器/室外）。"""
    def read(self, robot, env):
        return {"position": robot.state.base_position.copy()}

class Camera(Sensor):
    """相机——预留接口，渲染由 Three.js 前端处理。"""
    def read(self, robot, env):
        return {"pose": robot.get_base_pose()}  # 前端按位姿渲染
```

传感器通过组合模式挂载到任意 `Robot`：

```python
robot = create_robot("quadrotor", name="drone_0")
robot.add_sensor("imu", IMU())
robot.add_sensor("lidar", Lidar(n_rays=16))
robot.add_sensor("gps", GPSSensor())
obs = robot.get_observation()
# obs["sensors"] = {"imu": {...}, "lidar": {...}, "gps": {...}}
```

---

## 7. 模块结构

```
core/
  environment.py           Environment 基类 + PhysicsWorld + Obstacle + SemanticRegion
  env_batch.py             EnvBatch 多环境批量
  gaussian_splat_env.py    GaussianSplatEnv（改为 Environment 子类）
  mesh_env.py              MeshEnv（新增，trimesh 后端）
  heightfield_env.py       HeightFieldEnv（新增）
  osm_env.py               OSMCityEnv（新增）
  indoor_env.py            IndoorEnv（新增）
  pointcloud_env.py        PointCloudEnv（新增）
  terrain_generator.py     现有，不变
  terrain.py               现有，不变
  math_utils.py            现有，不变
  urdf_parser.py           现有，不变

robots/
  robot.py                 Robot 统一基类 + BaseState + ActionType + RobotState（新）
  robot_base.py            LeggedRobot（改为 Robot 子类，现有 API 不变）
  biped.py                 现有，不变
  quadruped.py             现有，不变
  hexapod.py               现有，不变
  urdf_robot.py            现有，不变
  wheeled.py               WheeledRobot + Differential/Omni/Ackermann（新增）
  aerial.py                AerialRobot + Quadrotor（新增）
  manipulator.py           ManipulatorRobot + ArmRobot（新增）
  hybrid.py                HybridRobot + WheeledLeggedRobot（新增）
  sensors.py               Sensor + IMU/Lidar/GPS/Camera（新增）
  swarm.py                 SwarmManager + SwarmConfig（新增）

comm/                      现有，不变（SwarmManager.to_comm_network 提供集成）
```

---

## 8. 与现有模块的兼容性

| 现有模块 | 影响 | 兼容措施 |
|---|---|---|
| `LeggedRobot` / `Biped` / `Quadruped` / `Hexapod` | 改为 `Robot` 子类 | 公开 API 不变；新增抽象方法由 `LeggedRobot` 统一实现 |
| `URDFRobot` | 无改动 | 继承 `LeggedRobot` 不变 |
| `control/gait_planner` / `motion_controller` | 无改动 | 操作 `LeggedRobot` 接口不变 |
| `interaction/env_interaction` | 无改动 | 操作 `GaussianSplatEnv` 接口不变；`GaussianSplatEnv` 新增 `Environment` 继承但保留现有方法 |
| `comm/` | 无改动 | `SwarmManager.to_comm_network()` 提供便捷集成，但 `CommNetworkSimulator` 可独立使用 |
| `tests/` | 现有测试不变 | 新增环境/机器人测试文件 |

---

## 9. 落地路线

| 阶段 | 内容 | 依赖 | 优先级 |
|---|---|---|---|
| **E0** | `Environment` 基类 + `PhysicsWorld` + `GaussianSplatEnv` 改为子类 | 无 | 高 |
| **E1** | `HeightFieldEnv` + `IndoorEnv`（程序化，纯 numpy） | 无 | 高 |
| **R0** | `Robot` 基类 + `LeggedRobot` 改为子类（零破坏） | 无 | 高 |
| **R1** | `WheeledRobot` + `DifferentialRobot` | 无 | 高 |
| **S0** | `SwarmManager` + `to_comm_network` 集成 | E0, R0 | 高 |
| **E2** | `MeshEnv`（trimesh 可选依赖） | trimesh | 中 |
| **R2** | `AerialRobot` + `Quadrotor` | 无 | 中 |
| **R3** | `ManipulatorRobot` + `ArmRobot`（从 URDF 加载） | 无 | 中 |
| **SE** | `Sensor` 组件 + IMU/Lidar/GPS | R0 | 中 |
| **E3** | `OSMCityEnv`（osmnx 可选依赖） | osmnx | 低 |
| **R4** | `HybridRobot` + `WheeledLeggedRobot` | R0, R1 | 低 |
| **R5** | `SnakeRobot` | R0 | 低（远期） |
| **EB** | `EnvBatch` 多环境批量 | E0 | 中 |

E0 + R0 + S0 是核心，零额外依赖，直接在 `rtg132` 内可开发测试。E1/R1 扩展环境与机器人覆盖。E2/R2/R3/SE 按研究需求推进。

---

## 10. 与相关工作的定位

| 维度 | 本平台 | Isaac Lab | Gazebo | legged_gym | ARGoS |
|---|---|---|---|---|---|
| 环境来源 | 3DGS + 程序化 + 网格 + OSM | USD + 程序化 | SDF + DEM | 程序化高度场 | 程序化 |
| 机器人形态 | 足+轮+飞+臂+混合 | 全形态 | 全形态 | 仅足式 | 简单动力学 |
| 集群 | 异构 SwarmManager | GPU 向量化 | ROS2 fleet | 批量同构 | 大规模异构 |
| 通信 | 内置 comm 模块 | 无 | ROS2 DDS | 无 | RAB 简化 |
| 物理后端 | 轻量运动学 | PhysX | ODE/Bullet | PhysX | 自研 |
| 定位 | 通信+交互实验平台 | RL 训练 | 部署验证 | 足式 RL | swarm 研究 |

差异化：**3DGS 重建环境 + 异构机器人集群 + 内置通信仿真**，三者组合使本平台聚焦于"高保真场景中异构集群的通信与交互研究"，与训练导向（Isaac/legged_gym）和部署导向（Gazebo）的平台互补。

---

## 参考资料

- Isaac Lab `InteractiveScene` + `TerrainImporter`: https://isaac-sim.github.io/IsaacLab/
- legged_gym `Terrain` 类: https://github.com/leggedrobotics/legged_gym
- MuJoCo MJCF `<option>` / `<hfield>`: https://mujoco.readthedocs.io/
- Gazebo SDF `<world>`: https://gazebosim.org/api/sdf/
- ARGoS 多引擎集群: https://github.com/ilpincy/argos3
- Habitat-Sim 场景: https://aihabitat.org/
- Unitree URDF 全系列: https://github.com/unitreerobotics/unitree_ros
- Crazyflie 仿真: https://github.com/bitcraze/crazyflie-simulation
- Crazyswarm2 ROS2 集群: https://github.com/IMRCLab/crazyswarm2
- trimesh（网格环境后端）: https://github.com/mikedh/trimesh
- osmnx（OSM 城市环境）: https://github.com/gboeing/osmnx
