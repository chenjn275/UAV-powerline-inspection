#!/usr/bin/env python3
"""Run a small deterministic geometry safety scenario suite."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from inspection_core import PolylineLineModel, SphereObstacle, evaluate_candidates, select_best_candidate


def run_suite(output: Path) -> dict:
    scenarios = (
        ("straight_clear", [(0, 0, 10), (50, 0, 10)], ()),
        ("bent_line", [(0, 0, 10), (25, 0, 10), (50, 10, 10)], ()),
        ("no_safe_candidate", [(0, 0, 10), (50, 0, 10)],
         (SphereObstacle((25, 0, -5), 20.0, "blocking_obstacle"),)),
    )
    results = []
    for name, points, obstacles in scenarios:
        line = PolylineLineModel(points, model_version=name)
        evaluations = evaluate_candidates(
            line, start_s=0.0, end_s=line.length, radii=(4.0, 5.0, 6.0),
            line_exclusion_radius=2.0, obstacles=obstacles, obstacle_margin=1.0,
            sample_dt=0.1, max_step=0.25, reference_radius=5.0,
        )
        selected = select_best_candidate(evaluations)
        results.append({
            "scenario": name,
            "line_length_m": line.length,
            "candidate_count": len(evaluations),
            "accepted_count": sum(evaluation.accepted for evaluation in evaluations),
            "selected_radius_m": selected.radius if selected else None,
            "selected_min_line_distance_m": selected.clearance.min_line_distance if selected else None,
            "failure_reasons": sorted({
                evaluation.clearance.violation_reason
                for evaluation in evaluations
                if not evaluation.accepted and evaluation.clearance.violation_reason
            }),
        })
    report = {"suite": "inspection-geometry-baseline-v1", "scenarios": results}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/scenario_suite.json"))
    args = parser.parse_args()
    print(json.dumps(run_suite(args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
