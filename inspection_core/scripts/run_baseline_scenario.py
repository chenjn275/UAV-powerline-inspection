#!/usr/bin/env python3
"""Run the deterministic single-span candidate and recovery baseline."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from inspection_core import (
    CoverageGrid,
    CoverageObservation,
    PolylineLineModel,
    SphereObstacle,
    evaluate_candidates,
    select_best_candidate,
)


def run(output: Path) -> dict:
    line = PolylineLineModel([(0, 0, 10), (50, 0, 10)], model_version="straight-single-span-v1")
    obstacle = SphereObstacle(center=(25, -4, 5), radius=1.0, name="tower-side")
    evaluations = evaluate_candidates(
        line,
        start_s=0.0,
        end_s=50.0,
        radii=(4.0, 5.0, 6.0),
        line_exclusion_radius=2.0,
        obstacles=(obstacle,),
        obstacle_margin=1.0,
        sample_dt=0.1,
        max_step=0.25,
        reference_radius=5.0,
    )
    selected = select_best_candidate(evaluations)
    if selected is None:
        raise RuntimeError("no safe candidate in baseline scenario")

    grid = CoverageGrid(
        s_min=0.0, s_max=50.0, s_bins=25,
        theta_min=-math.radians(10), theta_max=math.radians(10), theta_bins=8,
        min_distance=4.0, max_distance=6.0, min_sharpness=0.8,
    )
    # Simulate a valid first half and a blocked second half.  This is a
    # bookkeeping test, not a claim about camera performance.
    for sample in selected.trajectory:
        if sample.s > 25.0:
            continue
        grid.mark(CoverageObservation(
            s=sample.s, theta=sample.theta, distance=line.nearest_distance(sample.position),
            visible=True, clear=True, sharpness=0.95,
        ))
    reconnect = grid.choose_reconnect(
        current_s=25.0,
        current_theta=0.0,
        reachable=lambda s, theta: line.nearest_distance((s, 5.0, 10.0)) >= 2.0,
    )
    report = {
        "scenario": "straight-single-span-with-interruption",
        "model_version": line.model_version,
        "selected": {
            "radius_m": selected.radius,
            "theta_amplitude_deg": math.degrees(selected.theta_amplitude),
            "min_line_distance_m": selected.clearance.min_line_distance,
            "score": selected.score,
        },
        "candidate_evaluations": [
            {
                "radius_m": evaluation.radius,
                "accepted": evaluation.accepted,
                "min_line_distance_m": evaluation.clearance.min_line_distance,
                "min_obstacle_clearance_m": evaluation.clearance.min_obstacle_clearance,
                "violation_reason": evaluation.clearance.violation_reason,
            }
            for evaluation in evaluations
        ],
        "coverage": {
            "completed_cells": grid.completed_cells,
            "total_cells": grid.total_cells,
            "ratio": grid.coverage_ratio,
            "uncovered_cells": len(grid.uncovered_cells()),
            "reconnect_cell": reconnect,
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/baseline_scenario.json"))
    args = parser.parse_args()
    report = run(args.output)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
