#!/usr/bin/env python3
"""Run deterministic A/B/C inspection-planning ablation metrics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from inspection_core import CoverageGrid, CoverageObservation, PolylineLineModel, SphereObstacle, evaluate_candidates, select_best_candidate


def run(output: Path) -> dict:
    line = PolylineLineModel([(0, 0, 10), (50, 0, 10)], model_version="ablation-v1")
    obstacle = SphereObstacle((25, 0.6155, 5.0380), 0.2, "central_block")
    common = dict(start_s=0.0, end_s=50.0, line_exclusion_radius=2.0, obstacles=(obstacle,), obstacle_margin=0.5, sample_dt=0.1, max_step=0.25)
    fixed = evaluate_candidates(line, radii=(5.0,), reference_radius=5.0, **common)
    candidates = evaluate_candidates(line, radii=(4.0, 5.0, 6.0), reference_radius=5.0, **common)
    selected = select_best_candidate(candidates)
    grid = CoverageGrid(s_min=0, s_max=50, s_bins=25, theta_min=-0.2, theta_max=0.2, theta_bins=8, min_distance=4, max_distance=6, min_sharpness=0.8)
    if selected:
        for sample in selected.trajectory:
            if sample.s <= 25:
                grid.mark(CoverageObservation(sample.s, sample.theta, line.nearest_distance(sample.position), True, True, 0.95))
    reconnect = grid.choose_reconnect(current_s=25, current_theta=0)
    report = {"suite": "ablation-v1", "scenarios": [
        {"variant": "A_fixed_radius", "accepted": bool(select_best_candidate(fixed)), "accepted_count": sum(e.accepted for e in fixed)},
        {"variant": "B_candidate_search", "accepted": bool(selected), "accepted_count": sum(e.accepted for e in candidates)},
        {"variant": "C_candidate_plus_coverage_reconnect", "accepted": bool(selected), "accepted_count": sum(e.accepted for e in candidates), "coverage_ratio": grid.coverage_ratio, "reconnect_cell": reconnect},
    ]}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/ablation.json"))
    args = parser.parse_args()
    print(json.dumps(run(args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
