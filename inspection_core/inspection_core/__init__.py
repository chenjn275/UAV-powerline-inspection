"""Reusable, dependency-light inspection geometry primitives."""

from .coverage import CoverageGrid, CoverageObservation
from .candidate_search import CandidateEvaluation, CandidateWeights, evaluate_candidates, select_best_candidate, validate_swept_clearance
from .health import HealthSnapshot, HealthState
from .state_machine import InspectionState, InspectionTask
from .coordinate import LocalAlignment
from .line_model import (
    ArcScanSpec,
    ArcSample,
    ClearanceReport,
    PolylineLineModel,
    SphereObstacle,
    generate_arc_scan,
    validate_clearance,
)
from .tower_localization import TowerObservation, estimate_tower_observation
from .dynamic_replanner import DynamicReplanner, ReplanResult

__all__ = [
    "ArcSample",
    "ArcScanSpec",
    "CandidateEvaluation",
    "CandidateWeights",
    "ClearanceReport",
    "CoverageGrid",
    "CoverageObservation",
    "HealthSnapshot",
    "HealthState",
    "InspectionState",
    "InspectionTask",
    "LocalAlignment",
    "PolylineLineModel",
    "SphereObstacle",
    "generate_arc_scan",
    "evaluate_candidates",
    "select_best_candidate",
    "validate_swept_clearance",
    "validate_clearance",
    "TowerObservation",
    "estimate_tower_observation",
    "DynamicReplanner",
    "ReplanResult",
]
