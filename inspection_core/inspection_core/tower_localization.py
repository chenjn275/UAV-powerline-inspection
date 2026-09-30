"""Dependency-light tower localization from a lidar point-cloud sample.

This is deliberately conservative: it estimates a tower footprint from a
finite point set, rejects stale/degenerate observations, and returns an
uncertainty radius that the planner can add to its safety margin. It does not
claim semantic recognition of arbitrary towers; the input must already be
cropped to the tower class or region of interest.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class TowerObservation:
    center_x: float
    center_y: float
    base_z: float
    top_z: float
    point_count: int
    uncertainty_m: float

    @property
    def height_m(self) -> float:
        return self.top_z - self.base_z


def _percentile(values: Sequence[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("point cloud is empty")
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def estimate_tower_observation(
    points: Iterable[Sequence[float]],
    *,
    min_points: int = 30,
    min_height_m: float = 2.0,
    trim_fraction: float = 0.05,
) -> TowerObservation:
    """Estimate tower center and vertical extent from ``(x, y, z)`` points.

    The XY center uses a trimmed mean to reduce influence from conductors and
    isolated returns. Z bounds use robust percentiles. ``uncertainty_m`` is a
    conservative horizontal spread estimate and should be added to the
    vehicle/line clearance margin before selecting a radius.
    """
    rows = []
    for point in points:
        if len(point) < 3:
            raise ValueError("each point must contain x, y and z")
        xyz = tuple(float(value) for value in point[:3])
        if not all(math.isfinite(value) for value in xyz):
            continue
        rows.append(xyz)
    if len(rows) < min_points:
        raise ValueError(f"insufficient tower points: {len(rows)} < {min_points}")
    if not 0.0 <= trim_fraction < 0.5:
        raise ValueError("trim_fraction must be in [0, 0.5)")
    xs = [row[0] for row in rows]
    ys = [row[1] for row in rows]
    zs = [row[2] for row in rows]
    lo = trim_fraction
    hi = 1.0 - trim_fraction
    x_lo, x_hi = _percentile(xs, lo), _percentile(xs, hi)
    y_lo, y_hi = _percentile(ys, lo), _percentile(ys, hi)
    inlier = [row for row in rows if x_lo <= row[0] <= x_hi and y_lo <= row[1] <= y_hi]
    if len(inlier) < min_points:
        raise ValueError("tower ROI became too small after robust trimming")
    center_x = sum(row[0] for row in inlier) / len(inlier)
    center_y = sum(row[1] for row in inlier) / len(inlier)
    base_z = _percentile(zs, 0.02)
    top_z = _percentile(zs, 0.98)
    if top_z - base_z < min_height_m:
        raise ValueError("tower observation has insufficient vertical extent")
    radial_errors = [math.hypot(row[0] - center_x, row[1] - center_y) for row in inlier]
    uncertainty = max(0.05, _percentile(radial_errors, 0.90))
    return TowerObservation(center_x, center_y, base_z, top_z, len(inlier), uncertainty)
