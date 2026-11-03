"""
Helpers that turn simulation objects into JSON-safe payloads for the WebSocket
stream. Kept separate from ``app.py`` so the wire format can evolve without
touching endpoint logic.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def _jsonable(obj: Any) -> Any:
    """Recursively convert numpy types / dataclasses into JSON-safe values."""
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _jsonable(obj.tolist())
    if isinstance(obj, (np.floating, np.integer)):
        return _jsonable(obj.item())
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if hasattr(obj, "__dataclass_fields__"):
        return _jsonable(
            {f: getattr(obj, f) for f in obj.__dataclass_fields__})
    return obj


def snapshot_payload(world) -> dict:
    """Per-robot pose snapshot for one frame."""
    out = {}
    for rid, robot in world.robots.items():
        out[rid] = {
            "position": [float(x) for x in robot.position],
            "yaw": float(robot.yaw),
            "kind": type(robot).__name__,
        }
    return out


def lidar_payload(robot_id: str, world) -> dict:
    """Lidar scan of one robot (list of [angle, dist] pairs)."""
    robot = world.robots.get(robot_id)
    if robot is None:
        return {"rays": []}
    scan = world._lidar(robot)
    return {
        "rays": [[float(a), float(d)] for a, d in scan],
        "max_range": float(world.cfg.lidar_range),
    }


def comm_links_payload(world) -> list[dict]:
    """Active comm links from the latest snapshot."""
    snap = world.last_snapshot
    if snap is None:
        return []
    links = []
    for link in getattr(snap, "links", []):
        links.append({
            "tx": link.tx_id,
            "rx": link.rx_id,
            "up": bool(link.up),
            "snr_db": _finite(link.snr_db),
            "path_gain_db": _finite(link.path_gain_db),
            "delay_ms": _finite(link.delay_ms),
            "capacity_bps": _finite(link.capacity_bps),
            "distance_m": _finite(link.distance_m),
            "los": bool(link.los),
        })
    return links


def _finite(x):
    if x is None:
        return None
    x = float(x)
    return x if math.isfinite(x) else None


def algorithm_metrics_payload(world) -> dict:
    """Metrics of every registered algorithm, keyed by repr."""
    out = {}
    for alg in world.algorithms:
        try:
            out[repr(alg)] = _jsonable(alg.metrics())
        except Exception:
            continue
    return out


def terrain_payload(env) -> dict:
    """Static terrain geometry (splats) sent once on world creation."""
    splats = []
    for s in getattr(env, "splats", []):
        splats.append({
            "position": [float(x) for x in s.position],
            "scale": [float(x) for x in s.scale],
            "opacity": float(s.opacity),
        })
    return {"splats": splats,
            "ground_level": float(getattr(env, "ground_level", 0.0))}
