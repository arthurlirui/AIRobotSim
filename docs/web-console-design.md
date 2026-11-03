# 统一仿真控制台设计（server/ + app/）

> 2026-11 新增。目标：用一个完善的前后端框架把 AIRobotSim 的多个模拟系统
> （simlab 实时仿真、comm 通信实验、design 构型设计）统一到一个 Web 控制台中，
> 提供搭建、运行与可视化。

## 1. 架构

```
┌────────────────────────────────────────────────────┐
│  浏览器  app/（Vue 3 + Pinia + Three.js + Vite）    │
│  ├─ 实时仿真   SimView   ←WS→  /ws/sim             │
│  ├─ 通信实验   CommView  ←REST→ /api/comm/*        │
│  └─ 构型设计   DesignView←REST→ /api/design/*      │
└───────────────────────┬────────────────────────────┘
                        │ JSON
┌───────────────────────┴────────────────────────────┐
│  后端  server/（FastAPI，uvicorn 单进程）            │
│  ├─ app.py          REST 端点 + WebSocket 端点      │
│  ├─ sim_session.py  场景配置 → 常驻 World 会话      │
│  └─ payloads.py     numpy/仿真对象 → JSON 帧负载    │
└───────────────────────┬────────────────────────────┘
                        │ 直接 import
┌───────────────────────┴────────────────────────────┐
│  Python 仿真内核（既有模块，零改动）                  │
│  simlab/ algorithms/ comm/ design/ core/ robots/   │
└────────────────────────────────────────────────────┘
```

设计原则：

- **内核零侵入**：`server/` 只 import 既有模块，不修改任何仿真代码。
- **两条通道**：无状态的批量实验走 REST（请求→跑完→返回结果）；
  有状态的实时仿真走 WebSocket（create → 多次 step → 每步一帧）。
- **静态托管**：`app/dist` 构建产物由 FastAPI 挂载在 `/`，一条
  `uvicorn` 命令启动整套控制台；开发时 Vite dev server 把 `/api`、`/ws`
  代理到后端。

## 2. WebSocket 协议（/ws/sim）

客户端 → 服务端：

| type | 字段 | 说明 |
|---|---|---|
| `create` | `config`（场景配置 dict） | 构建 World，返回 `created` |
| `step` | `n`（步数，默认 1） | 步进 n 次，每步返回一个 `frame` |
| `reset` | — | 用原配置重建 World |
| `pause` | — | 客户端停止发 `step` 即暂停；服务端回 `paused` 确认 |

服务端 → 客户端：

| type | 负载 |
|---|---|
| `created` | robots（id/kind/初始位姿）、algorithms、dt、has_comm、terrain（splats 静态几何，一次下发） |
| `frame` | t、step、robots（位姿）、lidar（每机器人扫描）、comm_links（链路 SNR/时延/LOS）、algorithms（指标）、actions |
| `paused` / `error` | 状态确认 / 错误消息 |

场景 `config` 与 `simlab.scenario.Scenario` 的输入格式一致（env / robots /
algorithms / comm / world / steps），实时模式下 `steps` 无意义，由客户端
逐步驱动。

## 3. REST 端点

| 端点 | 方法 | 说明 |
|---|---|---|
| `/api/catalog` | GET | 机器人（mobile + URDF）、9 种地形、四类算法注册表、通信制式 |
| `/api/scenarios/run` | POST | 同步运行一个 Scenario，返回 final_metrics + 逐步 history |
| `/api/comm/scenarios` | GET | 通信实验列表（swarm_relay / mmwave_occlusion / thz_short_range / fso_pointing） |
| `/api/comm/simulate` | POST | 运行指定实验，返回投递结果 |
| `/api/design/presets` | GET | 构型预设（quadruped / hexapod） |
| `/api/design/simulate` | POST | 运行步态 FK 预览，返回逐帧连杆变换 |

## 4. 前端结构

```
app/
  src/
    App.vue                 顶栏 + 三个标签页切换
    views/SimView.vue       场景配置面板 + 播放控制 + 实时指标
    views/CommView.vue      实验选择 + 结果 KV 网格
    views/DesignView.vue    预设选择 + FK 帧回放
    components/SimViewport.vue  Three.js 视口：地形 splats、机器人、
                                激光射线、通信链路、轨道相机
    stores/sim.js           Pinia store：WS 连接 / 帧状态
    api/index.js            REST fetch 封装
```

坐标约定：仿真内核的世界系为 X-Y 平面 + Z 高度；Three.js 的 Y 是高度轴，
渲染时做 (x, y, z) → (x, z, y) 的映射。

## 5. 已知限制 / 后续方向

- 单 WebSocket 会话单 World：刷新页面即重建；多场景并行需要会话管理器。
- 实时流速率由客户端 `step` 请求节拍决定（默认 10 Hz，可调 1–60），
  未做服务端自动推送循环。
- `terrain` splats 全量下发，大场景需要改为增量 / 高度场纹理。
- design 预览目前渲染连杆质点；可升级为按 `design.components` 的
  连杆几何渲染。
- 前端点击在部分自动化环境（无头/内嵌浏览器）下 Playwright locator
  点击可能超时，DOM click 正常——人工使用不受影响。

## 6. 测试

- `python -m tests.test_server_api`：REST 5 项 + WebSocket 帧流冒烟测试。
- 全量回归：`python -m tests.run_all`。
