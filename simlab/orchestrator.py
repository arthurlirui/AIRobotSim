"""
Optional process-level parallel orchestrator for parameter sweeps.

Each worker runs one :class:`~simlab.scenario.Scenario` end-to-end in a
child process and returns its :class:`~simlab.scenario.EpisodeResult`;
the parent aggregates them into a :class:`~simlab.benchmark.BenchmarkReport`.

Use this for large sweeps where individual scenarios are CPU-bound and
independent. For a handful of scenarios the overhead of spawning processes
exceeds the benefit — just run the benchmark in-process.
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from typing import Iterable

from simlab.scenario import Scenario, EpisodeResult
from simlab.benchmark import Benchmark, BenchmarkReport


def _run_scenario(scenario_dict: dict) -> dict:
    """Worker entrypoint — runs one scenario and returns a serializable dict."""
    sc = Scenario.from_dict(scenario_dict)
    res = sc.run()
    return {
        "scenario_name": res.scenario_name,
        "steps": res.steps,
        "final_metrics": res.final_metrics,
        "config": res.config,
    }


class Orchestrator:
    """Fan a benchmark's scenarios across a process pool."""

    def __init__(self, max_workers: int | None = None):
        self.max_workers = max_workers

    def run(self, benchmark: Benchmark) -> BenchmarkReport:
        scenario_dicts = [sc.to_dict() for sc in benchmark.scenarios]
        report = BenchmarkReport(name=benchmark.name)
        if not scenario_dicts:
            return report
        # small batches: run in-process to avoid spawn overhead
        if len(scenario_dicts) <= 2 or self.max_workers == 1:
            for sc in benchmark.scenarios:
                report.results.append(sc.run())
            return report
        with ProcessPoolExecutor(max_workers=self.max_workers) as pool:
            for d in pool.map(_run_scenario, scenario_dicts):
                report.results.append(EpisodeResult(
                    scenario_name=d["scenario_name"],
                    steps=d["steps"],
                    final_metrics=d["final_metrics"],
                    history=[],
                    config=d["config"],
                ))
        return report
