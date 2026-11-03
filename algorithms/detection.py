"""
Detection reference implementations.

Both consume a lidar scan (the platform's common perception observation) and
emit a list of detections:

    {"center": (2,), "bbox": [xmin, ymin, xmax, ymax], "score": float,
     "label": str}

* :class:`EuclideanClusterDetector` — clusters lidar hit points in the
  horizontal plane with a simple flood-fill over a 2-D grid; each cluster
  becomes one detection.
* :class:`RayEnvelopeDetector` — fits a convex-hull-free envelope (axis-aligned
  bounding box over contiguous hit rays) — cheaper, lower fidelity.
"""

from __future__ import annotations

import numpy as np
from typing import TYPE_CHECKING

from algorithms.base import DetectionBase, AlgorithmOutput
from simlab.registry import register

if TYPE_CHECKING:
    from simlab.world import World


def _hits_from_lidar(pos: np.ndarray, lidar: np.ndarray,
                     max_range: float) -> np.ndarray:
    """Return (k,2) world-frame hit points that are actual hits (rng<max)."""
    if lidar is None or len(lidar) == 0:
        return np.zeros((0, 2))
    yaw = pos[2] if pos.size == 3 else 0.0
    mask = lidar[:, 1] < max_range
    if not mask.any():
        return np.zeros((0, 2))
    ang = lidar[mask, 0] + yaw
    rng = lidar[mask, 1]
    return np.stack([pos[0] + rng * np.cos(ang),
                     pos[1] + rng * np.sin(ang)], axis=1)


class EuclideanClusterDetector(DetectionBase):
    """Grid-based Euclidean clustering of lidar hits."""

    name = "euclidean_cluster"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.robot_id: str = self.cfg.get("robot_id", "r0")
        self.cell = float(self.cfg.get("cell", 0.3))
        self.min_pts = int(self.cfg.get("min_pts", 3))
        self.cluster_eps = float(self.cfg.get("cluster_eps", 0.6))
        self._tp = 0
        self._fp = 0
        self._fn = 0
        self._steps = 0

    def _cluster(self, pts: np.ndarray) -> list[np.ndarray]:
        # quantize to grid, flood-fill connected components
        if len(pts) == 0:
            return []
        q = np.floor(pts / self.cell).astype(int)
        seen: dict[tuple[int, int], int] = {}
        labels = np.full(len(pts), -1, dtype=int)
        cur = 0
        for i in range(len(pts)):
            if labels[i] != -1:
                continue
            stack = [i]
            labels[i] = cur
            members = [i]
            while stack:
                j = stack.pop()
                dq = np.abs(q[j] - q)
                nbr = np.where((dq <= 1).all(axis=1) & (labels == -1))[0]
                for k in nbr:
                    labels[k] = cur
                    stack.append(k)
                    members.append(k)
            cur += 1
        clusters = []
        for c in range(cur):
            m = np.where(labels == c)[0]
            if len(m) >= self.min_pts:
                clusters.append(pts[m])
        return clusters

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        self._steps += 1
        obs = observations.get(self.robot_id, {})
        pos = np.asarray(obs.get("pose_truth", [0, 0, 0]), float)
        lidar = obs.get("lidar")
        max_range = float(obs.get("lidar_max_range", 5.0))
        hits = _hits_from_lidar(pos, lidar, max_range)
        clusters = self._cluster(hits)

        dets = []
        for cl in clusters:
            c = cl.mean(axis=0)
            dets.append({
                "center": c.copy(),
                "bbox": [float(cl[:, 0].min()), float(cl[:, 1].min()),
                         float(cl[:, 0].max()), float(cl[:, 1].max())],
                "score": float(len(cl) / 20.0),
                "label": "obstacle",
            })

        # evaluate against ground-truth splat positions if provided
        gt = obs.get("obstacle_centers")
        if gt is not None and len(gt) > 0:
            gt = np.asarray(gt, float)
            matched = set()
            for d in dets:
                dd = np.linalg.norm(gt - d["center"], axis=1)
                j = int(np.argmin(dd)) if len(dd) else -1
                if j >= 0 and dd[j] < 1.0 and j not in matched:
                    matched.add(j)
                    self._tp += 1
                else:
                    self._fp += 1
            self._fn += len(gt) - len(matched)

        return AlgorithmOutput(data={"detections": dets})

    def metrics(self) -> dict:
        tp, fp, fn = self._tp, self._fp, self._fn
        prec = tp / max(tp + fp, 1)
        rec = tp / max(tp + fn, 1)
        f1 = 2 * prec * rec / max(prec + rec, 1e-9)
        return {
            "detection.precision": prec,
            "detection.recall": rec,
            "detection.f1": f1,
            "detection.steps": self._steps,
        }


@register("detection", "euclidean_cluster")
class _RegisteredEuclidean(EuclideanClusterDetector):
    pass


class RayEnvelopeDetector(DetectionBase):
    """
    Cheap envelope detector: group contiguous hit rays and emit one AABB per
    contiguous run. Lower fidelity than clustering but O(n_rays).
    """

    name = "ray_envelope"

    def setup(self, world: "World", cfg: dict | None = None) -> None:
        super().setup(world, cfg)
        self.robot_id: str = self.cfg.get("robot_id", "r0")
        self.min_run = int(self.cfg.get("min_run", 3))
        self._tp = 0
        self._fp = 0
        self._fn = 0
        self._steps = 0

    def step(self, world: "World", observations: dict) -> AlgorithmOutput:
        self._steps += 1
        obs = observations.get(self.robot_id, {})
        pos = np.asarray(obs.get("pose_truth", [0, 0, 0]), float)
        lidar = obs.get("lidar")
        max_range = float(obs.get("lidar_max_range", 5.0))
        hits = _hits_from_lidar(pos, lidar, max_range)
        dets = []
        if len(hits) > 0:
            # contiguous runs by hit index (hits are ordered by ray angle)
            run = [0]
            for i in range(1, len(hits)):
                if np.linalg.norm(hits[i] - hits[i - 1]) < 0.5:
                    run.append(i)
                else:
                    if len(run) >= self.min_run:
                        dets.append(self._box(hits[run]))
                    run = [i]
            if len(run) >= self.min_run:
                dets.append(self._box(hits[run]))

        gt = obs.get("obstacle_centers")
        if gt is not None and len(gt) > 0:
            gt = np.asarray(gt, float)
            matched = set()
            for d in dets:
                dd = np.linalg.norm(gt - d["center"], axis=1)
                j = int(np.argmin(dd)) if len(dd) else -1
                if j >= 0 and dd[j] < 1.5 and j not in matched:
                    matched.add(j)
                    self._tp += 1
                else:
                    self._fp += 1
            self._fn += len(gt) - len(matched)

        return AlgorithmOutput(data={"detections": dets})

    @staticmethod
    def _box(pts: np.ndarray) -> dict:
        return {
            "center": pts.mean(axis=0),
            "bbox": [float(pts[:, 0].min()), float(pts[:, 1].min()),
                     float(pts[:, 0].max()), float(pts[:, 1].max())],
            "score": float(len(pts) / 20.0),
            "label": "obstacle",
        }

    def metrics(self) -> dict:
        tp, fp, fn = self._tp, self._fp, self._fn
        prec = tp / max(tp + fp, 1)
        rec = tp / max(tp + fn, 1)
        f1 = 2 * prec * rec / max(prec + rec, 1e-9)
        return {
            "detection.precision": prec,
            "detection.recall": rec,
            "detection.f1": f1,
            "detection.steps": self._steps,
        }


@register("detection", "ray_envelope")
class _RegisteredRayEnvelope(RayEnvelopeDetector):
    pass
