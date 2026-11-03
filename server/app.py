"""
FastAPI service layer: REST + WebSocket endpoints over the simulation
packages (:mod:`simlab`, :mod:`algorithms`, :mod:`comm`, :mod:`design`,
:mod:`robots`, :mod:`core`).

REST (stateless, request/response):

* ``GET  /api/catalog``          — robots, terrains, algorithms, comm models
* ``POST /api/scenarios/run``    — run one scenario synchronously, get result
* ``GET  /api/robots/urdf``      — available URDF robots
* ``GET  /api/design/presets``   — robot design presets
* ``POST /api/design/simulate``  — FK/gait preview for a robot design
* ``GET  /api/comm/scenarios``   — preset comm experiment scenarios
* ``POST /api/comm/simulate``    — run a comm experiment, get link records

WebSocket (stateful, live stepping):

* ``/ws/sim`` — create a :class:`~simlab.scenario.Scenario` world, stream
  per-tick snapshots (robot poses, lidar, comm links, algorithm metrics).

Designed to be served together with the built SPA frontend (``app/``) so
that one ``uvicorn server.app:app`` command brings up the whole console.
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import Any, Optional

import numpy as np

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from core.terrain_generator import TerrainGenerator, TERRAIN_FUNCTIONS
from robots.mobile_base import WheeledRobot, UAV
from comm.channel.statistical import FreeSpaceChannel
from comm.network.network_simulator import CommNetworkSimulator
import comm.scenarios as comm_scenarios
from simlab.scenario import Scenario
from simlab.world import World, WorldConfig
from simlab.registry import registry
from simlab.metrics import collect_episode_metrics
import algorithms  # noqa: F401  — registers all built-in algorithms

from server.sim_session import SimSession
from server.payloads import (
    snapshot_payload, lidar_payload, comm_links_payload,
    algorithm_metrics_payload, terrain_payload, _jsonable,
)

app = FastAPI(
    title="AIRobotSim Simulation Console",
    version="0.1.0",
    description="REST + WebSocket API over the AIRobotSim simulation packages",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------------------------------------------------------------------------
# Catalog endpoint — everything the frontend needs to build forms/menus.
# ---------------------------------------------------------------------------

ALGORITHM_KINDS = ["slam", "navigation", "detection", "swarm"]


@app.get("/api/catalog")
def get_catalog() -> dict:
    """One-stop catalog: robots, terrains, algorithms, comm channel models."""
    from robots.urdf_robot import list_available_robots

    algos: dict[str, list[dict]] = {}
    for kind in ALGORITHM_KINDS:
        algos[kind] = [
            {"kind": k, "name": n} for (k, n) in registry.list(kind)
        ]
    return {
        "robots": {
            "mobile": ["wheeled", "uav"],
            "urdf": sorted(list_available_robots()),
        },
        "terrains": list(TERRAIN_FUNCTIONS),
        "algorithms": algos,
        "comm_channels": [
            "free_space", "log_distance", "three_g", "four_g",
            "five_g_nr", "mmwave", "geometric_los", "fiber",
        ],
    }


# ---------------------------------------------------------------------------
# Scenario run (synchronous, stateless)
# ---------------------------------------------------------------------------

class ScenarioRunRequest(BaseModel):
    config: dict = Field(default_factory=dict)
    name: str = "scenario"


@app.post("/api/scenarios/run")
def run_scenario(req: ScenarioRunRequest) -> dict:
    t0 = time.perf_counter()
    sc = Scenario.from_dict({"name": req.name, **req.config})
    result = sc.run()
    wall = time.perf_counter() - t0
    return {
        "scenario": result.scenario_name,
        "steps": result.steps,
        "final_metrics": _jsonable(result.final_metrics),
        "history": _jsonable(result.history),
        "config": _jsonable(result.config),
        "wall_time_sec": round(wall, 3),
    }


# ---------------------------------------------------------------------------
# Comm experiments
# ---------------------------------------------------------------------------

COMM_EXPERIMENTS = {
    "swarm_relay": comm_scenarios.swarm_relay,
    "mmwave_occlusion": comm_scenarios.mmwave_occlusion,
    "thz_short_range": comm_scenarios.thz_short_range,
    "fso_pointing": comm_scenarios.fso_pointing,
}


@app.get("/api/comm/scenarios")
def list_comm_scenarios() -> dict:
    return {"scenarios": [
        {
            "name": name,
            "description": (fn.__doc__ or "").strip().splitlines()[0]
            if fn.__doc__ else "",
        }
        for name, fn in COMM_EXPERIMENTS.items()
    ]}


@app.post("/api/comm/simulate")
def run_comm_experiment(body: dict) -> dict:
    name = body.get("experiment", "swarm_relay")
    if name not in COMM_EXPERIMENTS:
        return {"error": f"unknown experiment {name!r}; "
                         f"available: {list(COMM_EXPERIMENTS)}"}
    res = COMM_EXPERIMENTS[name]()
    return _jsonable(res)


# ---------------------------------------------------------------------------
# Design module
# ---------------------------------------------------------------------------

@app.get("/api/design/presets")
def list_design_presets() -> dict:
    from design.config import PRESETS

    return {"presets": [
        {"name": name, "description": (fn.__doc__ or "").strip()
         if fn.__doc__ else ""}
        for name, fn in PRESETS.items()
    ]}


class DesignSimulateRequest(BaseModel):
    preset: str
    stride: float = 0.08
    lift: float = 0.06
    period: float = 1.0
    speed: float = 1.0
    steps: int = 100
    dt: float = 0.05


@app.post("/api/design/simulate")
def simulate_design(req: DesignSimulateRequest) -> dict:
    from core.gaussian_splat_env import GaussianSplatEnv
    from design.config import create_design
    from design.builder import SimRobot
    from design.locomotion import GaitController

    design = create_design(req.preset)
    robot = SimRobot(design)
    env = GaussianSplatEnv()          # flat ground for the preview
    ctrl = GaitController(robot, env, stride=req.stride,
                          lift=req.lift, period=req.period)
    frames = []
    for _ in range(req.steps):
        ctrl.step(req.dt, speed=req.speed)
        transforms = robot.forward_kinematics()
        frames.append({
            "base": _vec(getattr(robot, "base_position",
                                 getattr(robot, "position",
                                         np.zeros(3)))),
            "links": {
                name: {"position": _vec(T.t),
                       "rotation": T.R.tolist()}
                for name, T in transforms.items()
            },
        })
    return {
        "preset": req.preset,
        "frames": frames,
    }


def _vec(v) -> list[float]:
    return [float(x) for x in np.asarray(v).ravel()[:3]]


# ---------------------------------------------------------------------------
# Live simulation over WebSocket
# ---------------------------------------------------------------------------

@app.websocket("/ws/sim")
async def ws_sim(ws: WebSocket) -> None:
    await ws.accept()
    session: Optional[SimSession] = None
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send_json({"type": "error",
                                    "message": "invalid JSON"})
                continue
            mtype = msg.get("type")

            if mtype == "create":
                cfg = msg.get("config", {})
                session = SimSession(cfg)
                info = session.describe()
                await ws.send_json({"type": "created", **info,
                                    "terrain": terrain_payload(session.env)})

            elif mtype == "step":
                if session is None:
                    await ws.send_json({"type": "error",
                                        "message": "no active simulation"})
                    continue
                n = int(msg.get("n", 1))
                for _ in range(n):
                    outputs = session.world.step()
                    await ws.send_json({
                        "type": "frame",
                        "t": session.world.t,
                        "step": session.step_index,
                        "robots": snapshot_payload(session.world),
                        "lidar": {rid: lidar_payload(rid, session.world)
                                  for rid in session.world.robots},
                        "comm_links": comm_links_payload(session.world),
                        "algorithms": algorithm_metrics_payload(session.world),
                        "actions": {rid: _jsonable([float(x) for x in out.actions.get(rid, [])])
                                    for out in outputs for rid in out.actions},
                    })
                    session.step_index += 1

            elif mtype == "pause":
                # The client simply stops sending "step"; nothing to do
                # server-side, but acknowledge for UI state.
                await ws.send_json({"type": "paused"})

            elif mtype == "reset":
                if session is not None:
                    cfg = session.config
                    session = SimSession(cfg)
                    await ws.send_json({
                        "type": "created",
                        **session.describe(),
                        "terrain": terrain_payload(session.env),
                    })

            else:
                await ws.send_json({"type": "error",
                                    "message": f"unknown type {mtype!r}"})
    except WebSocketDisconnect:
        return


# ---------------------------------------------------------------------------
# Serve the built SPA (app/dist) at "/" when it exists.
# ---------------------------------------------------------------------------

from pathlib import Path

_DIST = Path(__file__).resolve().parent.parent / "app" / "dist"
if _DIST.is_dir():
    app.mount("/", StaticFiles(directory=str(_DIST), html=True), name="spa")
