"""Candidate radius/angle selection and conservative swept checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable, Sequence

from .line_model import ArcSample, ArcScanSpec, ClearanceReport, PolylineLineModel, SphereObstacle, generate_arc_scan, validate_clearance


@dataclass(frozen=True)
class CandidateWeights:
    coverage: float = 1.0
    radius_deviation: float = 1.0
    angle_deviation: float = 0.2
    motion_cost: float = 0.01


@dataclass(frozen=True)
class CandidateEvaluation:
    radius: float
    theta_center: float
    theta_amplitude: float
    trajectory: tuple[ArcSample, ...]
    clearance: ClearanceReport
    coverage_gain: float
    motion_cost: float
    score: float

    @property
    def accepted(self) -> bool:
        return self.clearance.accepted


def _path_length(points: Sequence[ArcSample]) -> float:
    return sum(math.dist(a.position, b.position) for a, b in zip(points, points[1:]))


def _densify(points: Sequence[ArcSample], max_step: float) -> tuple[tuple[float, float, float], ...]:
    if max_step <= 0.0:
        raise ValueError("max_step must be positive")
    dense = []
    for index, point in enumerate(points):
        if index == 0:
            dense.append(point.position)
            continue
        previous = points[index - 1].position
        distance = math.dist(previous, point.position)
        subdivisions = max(1, int(math.ceil(distance / max_step)))
        for subdivision in range(1, subdivisions + 1):
            ratio = subdivision / subdivisions
            dense.append(tuple(previous[axis] + (point.position[axis] - previous[axis]) * ratio for axis in range(3)))
    return tuple(dense)


def validate_swept_clearance(
    line: PolylineLineModel,
    trajectory: Sequence[ArcSample],
    *,
    line_exclusion_radius: float,
    obstacles: Iterable[SphereObstacle] = (),
    obstacle_margin: float = 0.0,
    max_step: float = 0.25,
) -> ClearanceReport:
    """Densify trajectory segments before checking the full geometry."""
    return validate_clearance(
        line,
        _densify(trajectory, max_step),
        line_exclusion_radius=line_exclusion_radius,
        obstacles=obstacles,
        obstacle_margin=obstacle_margin,
    )


def evaluate_candidates(
    line: PolylineLineModel,
    *,
    start_s: float,
    end_s: float,
    radii: Iterable[float],
    theta_centers: Iterable[float] = (0.0,),
    theta_amplitudes: Iterable[float] = (math.radians(10.0),),
    period: float = 16.0,
    along_line_speed: float = 0.5,
    sample_dt: float = 0.05,
    line_exclusion_radius: float,
    obstacles: Iterable[SphereObstacle] = (),
    obstacle_margin: float = 0.0,
    max_step: float = 0.25,
    reference_radius: float | None = None,
    reference_theta_center: float = 0.0,
    reference_theta_amplitude: float = math.radians(10.0),
    weights: CandidateWeights = CandidateWeights(),
    coverage_gain: Callable[[tuple[ArcSample, ...]], float] | None = None,
) -> tuple[CandidateEvaluation, ...]:
    """Evaluate an explicit small candidate set and rank safe candidates."""
    radii = tuple(float(radius) for radius in radii)
    if not radii:
        return ()
    reference_radius = float(reference_radius if reference_radius is not None else min(radii))
    obstacle_list = tuple(obstacles)
    evaluations = []
    for radius in radii:
        for theta_center in theta_centers:
            for theta_amplitude in theta_amplitudes:
                scan = generate_arc_scan(line, ArcScanSpec(
                    start_s=start_s, end_s=end_s, radius=radius,
                    theta_center=float(theta_center), theta_amplitude=float(theta_amplitude),
                    period=period, along_line_speed=along_line_speed, sample_dt=sample_dt,
                ))
                clearance = validate_swept_clearance(
                    line, scan, line_exclusion_radius=line_exclusion_radius,
                    obstacles=obstacle_list, obstacle_margin=obstacle_margin, max_step=max_step,
                )
                gain = float(coverage_gain(scan) if coverage_gain is not None else 0.0)
                motion_cost = _path_length(scan)
                score = (weights.coverage * gain
                         - weights.radius_deviation * abs(radius - reference_radius)
                         - weights.angle_deviation * abs(float(theta_center) - reference_theta_center)
                         - weights.angle_deviation * abs(float(theta_amplitude) - reference_theta_amplitude)
                         - weights.motion_cost * motion_cost)
                if not clearance.accepted:
                    score = -math.inf
                evaluations.append(CandidateEvaluation(
                    radius=radius, theta_center=float(theta_center), theta_amplitude=float(theta_amplitude),
                    trajectory=scan, clearance=clearance, coverage_gain=gain,
                    motion_cost=motion_cost, score=score,
                ))
    return tuple(sorted(evaluations, key=lambda evaluation: evaluation.score, reverse=True))


def select_best_candidate(evaluations: Iterable[CandidateEvaluation]) -> CandidateEvaluation | None:
    return next((evaluation for evaluation in evaluations if evaluation.accepted), None)
