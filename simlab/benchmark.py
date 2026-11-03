"""
Run many scenarios (parameter sweeps / algorithm comparisons) and aggregate.

A :class:`Benchmark` takes a list of :class:`~simlab.scenario.Scenario`
(or a base config + a sweep spec) and runs them all, then produces a
:class:`BenchmarkReport` with per-episode metrics, aggregated statistics and
a cross-variant comparison table.
"""

from __future__ import annotations

import copy
import numpy as np
from dataclasses import dataclass, field
from typing import Optional

from simlab.scenario import Scenario, EpisodeResult
from simlab.metrics import aggregate, compare


@dataclass
class BenchmarkReport:
    name: str
    results: list[EpisodeResult] = field(default_factory=list)

    @property
    def n_episodes(self) -> int:
        return len(self.results)

    def episode_metrics(self) -> list[dict[str, float]]:
        return [r.final_metrics for r in self.results]

    def summary(self) -> dict[str, dict[str, float]]:
        return aggregate(self.episode_metrics())

    def comparison(self, metric_keys: list[str],
                   variant_key: str = "variant") -> dict[str, dict[str, float]]:
        """Compare metrics across variants grouped by ``config[variant_key]``."""
        variants: dict[str, list[dict[str, float]]] = {}
        for r in self.results:
            v = r.config.get(variant_key, r.scenario_name)
            variants.setdefault(v, []).append(r.final_metrics)
        return compare(variants, metric_keys)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "n_episodes": self.n_episodes,
            "summary": self.summary(),
            "episodes": [{"scenario": r.scenario_name,
                          "metrics": r.final_metrics} for r in self.results],
        }


class Benchmark:
    """Run a set of scenarios and aggregate the results."""

    def __init__(self, name: str = "benchmark"):
        self.name = name
        self.scenarios: list[Scenario] = []

    def add(self, scenario: Scenario) -> "Benchmark":
        self.scenarios.append(scenario)
        return self

    def add_sweep(self, base: dict, param: str,
                  values: list, variant_key: str = "variant") -> "Benchmark":
        """Add one scenario per value, copying ``base`` and setting ``param``."""
        for v in values:
            cfg = copy.deepcopy(base)
            cfg[param] = v
            cfg[variant_key] = f"{param}={v}"
            cfg.setdefault("name", f"{param}_{v}")
            self.add(Scenario.from_dict(cfg))
        return self

    def run(self) -> BenchmarkReport:
        report = BenchmarkReport(name=self.name)
        for sc in self.scenarios:
            report.results.append(sc.run())
        return report
