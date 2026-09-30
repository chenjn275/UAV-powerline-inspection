"""Deterministic dynamic-obstacle update and safe-candidate replanning."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .candidate_search import CandidateEvaluation, evaluate_candidates, select_best_candidate
from .line_model import PolylineLineModel, SphereObstacle


@dataclass(frozen=True)
class ReplanResult:
    sequence: int
    obstacle_count: int
    selected: CandidateEvaluation | None
    reason: str


class DynamicReplanner:
    def __init__(self, line: PolylineLineModel, *, radii: Iterable[float], line_exclusion_radius: float,
                 obstacle_margin: float = 0.5, start_s: float = 0.0, end_s: float | None = None):
        self.line = line
        self.radii = tuple(float(r) for r in radii)
        self.line_exclusion_radius = float(line_exclusion_radius)
        self.obstacle_margin = float(obstacle_margin)
        self.start_s = float(start_s)
        self.end_s = line.length if end_s is None else float(end_s)
        self.sequence = 0

    def update(self, obstacles: Iterable[SphereObstacle]) -> ReplanResult:
        current = tuple(obstacles)
        evaluations = evaluate_candidates(
            self.line, start_s=self.start_s, end_s=self.end_s, radii=self.radii,
            line_exclusion_radius=self.line_exclusion_radius, obstacles=current,
            obstacle_margin=self.obstacle_margin, sample_dt=0.1, max_step=0.1,
            reference_radius=min(self.radii),
        )
        self.sequence += 1
        selected = select_best_candidate(evaluations)
        return ReplanResult(
            sequence=self.sequence, obstacle_count=len(current), selected=selected,
            reason="SAFE_CANDIDATE" if selected is not None else "NO_SAFE_CANDIDATE",
        )
