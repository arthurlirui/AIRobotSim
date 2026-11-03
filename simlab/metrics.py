"""
Unified metric aggregation across algorithm families.

Each algorithm reports its own metrics via ``AlgorithmBase.metrics()`` (e.g.
``slam.ate_mean``, ``nav.reach_rate``, ``swarm.connectivity_rate``,
``detection.f1``). This module provides helpers to:

* collect a flat metric dict from a world after an episode,
* aggregate a list of episode metric dicts into mean / std / min / max,
* build a comparison table across named algorithm variants (for benchmarks).
"""

from __future__ import annotations

import numpy as np
from typing import Iterable


def collect_episode_metrics(world) -> dict[str, float]:
    """Flatten ``world.algorithm_metrics()`` into ``{metric: value}``."""
    out: dict[str, float] = {}
    for _alg_repr, m in world.algorithm_metrics().items():
        for k, v in m.items():
            # keep the last occurrence; metric keys are globally unique by
            # family prefix so collisions shouldn't happen in practice.
            out[k] = float(v) if isinstance(v, (int, float, np.floating)) else 0.0
    return out


def aggregate(episodes: Iterable[dict[str, float]]) -> dict[str, dict[str, float]]:
    """
    Aggregate a sequence of per-episode metric dicts.

    Returns ``{metric: {mean, std, min, max, n}}``.
    """
    eps = list(episodes)
    if not eps:
        return {}
    keys = set()
    for e in eps:
        keys.update(e.keys())
    summary: dict[str, dict[str, float]] = {}
    for k in sorted(keys):
        vals = np.array([float(e.get(k, 0.0)) for e in eps], dtype=float)
        summary[k] = {
            "mean": float(vals.mean()),
            "std": float(vals.std()),
            "min": float(vals.min()),
            "max": float(vals.max()),
            "n": int(len(vals)),
        }
    return summary


def compare(variants: dict[str, list[dict[str, float]]],
            metric_keys: Iterable[str]) -> dict[str, dict[str, float]]:
    """
    Build a comparison table across algorithm variants.

    Args:
        variants: ``{variant_name: [episode_metrics, ...]}``.
        metric_keys: metric names to compare.

    Returns ``{metric: {variant: mean}}``.
    """
    out: dict[str, dict[str, float]] = {}
    for k in metric_keys:
        out[k] = {}
        for vname, eps in variants.items():
            vals = [float(e.get(k, 0.0)) for e in eps]
            out[k][vname] = float(np.mean(vals)) if vals else 0.0
    return out
