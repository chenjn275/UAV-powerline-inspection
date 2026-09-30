"""Observation-driven coverage bookkeeping for interrupted inspections."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CoverageObservation:
    s: float
    theta: float
    distance: float
    visible: bool
    clear: bool
    sharpness: float


class CoverageGrid:
    """Discretize the task as arc-length x observation-angle cells.

    A cell is completed only when the observation satisfies all configured
    quality gates.  This keeps arrival at a waypoint separate from valid image
    coverage, as required by the task baseline.
    """

    def __init__(
        self,
        *,
        s_min: float,
        s_max: float,
        s_bins: int,
        theta_min: float,
        theta_max: float,
        theta_bins: int,
        min_distance: float,
        max_distance: float,
        min_sharpness: float = 0.0,
    ) -> None:
        if not (s_max > s_min and theta_max > theta_min):
            raise ValueError("coverage bounds must be increasing")
        if s_bins < 1 or theta_bins < 1:
            raise ValueError("coverage grid dimensions must be positive")
        if min_distance < 0.0 or max_distance < min_distance:
            raise ValueError("invalid observation distance bounds")
        self.s_min = float(s_min)
        self.s_max = float(s_max)
        self.s_bins = int(s_bins)
        self.theta_min = float(theta_min)
        self.theta_max = float(theta_max)
        self.theta_bins = int(theta_bins)
        self.min_distance = float(min_distance)
        self.max_distance = float(max_distance)
        self.min_sharpness = float(min_sharpness)
        self._completed = [[False for _ in range(self.theta_bins)] for _ in range(self.s_bins)]

    def _index(self, s: float, theta: float) -> tuple[int, int] | None:
        if not (self.s_min <= s <= self.s_max and self.theta_min <= theta <= self.theta_max):
            return None
        s_fraction = min(1.0, max(0.0, (s - self.s_min) / (self.s_max - self.s_min)))
        theta_fraction = min(1.0, max(0.0, (theta - self.theta_min) / (self.theta_max - self.theta_min)))
        return (
            min(self.s_bins - 1, int(s_fraction * self.s_bins)),
            min(self.theta_bins - 1, int(theta_fraction * self.theta_bins)),
        )

    def mark(self, observation: CoverageObservation) -> bool:
        """Record an observation and return whether it completed a cell."""

        index = self._index(observation.s, observation.theta)
        quality_ok = (
            self.min_distance <= observation.distance <= self.max_distance
            and observation.visible
            and observation.clear
            and observation.sharpness >= self.min_sharpness
        )
        if index is None or not quality_ok:
            return False
        s_index, theta_index = index
        changed = not self._completed[s_index][theta_index]
        self._completed[s_index][theta_index] = True
        return changed

    @property
    def total_cells(self) -> int:
        return self.s_bins * self.theta_bins

    @property
    def completed_cells(self) -> int:
        return sum(sum(row) for row in self._completed)

    @property
    def coverage_ratio(self) -> float:
        return self.completed_cells / self.total_cells

    def uncovered_cells(self) -> tuple[tuple[float, float], ...]:
        cells = []
        for s_index in range(self.s_bins):
            for theta_index in range(self.theta_bins):
                if self._completed[s_index][theta_index]:
                    continue
                s = self.s_min + (s_index + 0.5) * (self.s_max - self.s_min) / self.s_bins
                theta = self.theta_min + (theta_index + 0.5) * (self.theta_max - self.theta_min) / self.theta_bins
                cells.append((s, theta))
        return tuple(cells)

    def choose_reconnect(
        self,
        *,
        current_s: float,
        current_theta: float,
        reachable: callable | None = None,
    ) -> tuple[float, float] | None:
        """Choose the nearest safe uncovered cell, or ``None`` if no cell fits."""

        candidates = self.uncovered_cells()
        if reachable is not None:
            candidates = tuple(cell for cell in candidates if reachable(*cell))
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda cell: math.hypot(cell[0] - current_s, cell[1] - current_theta),
        )
