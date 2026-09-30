#!/usr/bin/env python3
"""Repeat the A/B/C comparison with deterministic obstacle perturbations."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from inspection_core import CoverageGrid, CoverageObservation, PolylineLineModel, SphereObstacle, evaluate_candidates, select_best_candidate


def run(output: Path, repetitions: int = 10) -> dict:
    line = PolylineLineModel([(0, 0, 10), (50, 0, 10)], model_version="batch-v1")
    raw = []
    for seed in range(repetitions):
        y = 0.55 + 0.03 * (seed % 5)
        z = 5.0 + 0.02 * ((seed * 3) % 5)
        obstacle = SphereObstacle((25.0, y, z), 0.2, f"perturb_{seed}")
        common = dict(start_s=0.0, end_s=50.0, line_exclusion_radius=2.0, obstacles=(obstacle,), obstacle_margin=0.5, sample_dt=0.1, max_step=0.25)
        fixed = select_best_candidate(evaluate_candidates(line, radii=(5.0,), reference_radius=5.0, **common))
        candidates = select_best_candidate(evaluate_candidates(line, radii=(4.0, 5.0, 6.0), reference_radius=5.0, **common))
        coverage = 0.0
        # Deterministic camera quality/fault profile for this run.
        quality = 0.55 + 0.4 * ((seed * 7) % 10) / 9.0
        occlusion = 1.0 - quality
        fused_valid = candidates is not None and quality >= 0.6
        replan_count = 1 if fixed is None and candidates is not None else 0
        if candidates:
            grid = CoverageGrid(s_min=0, s_max=50, s_bins=25, theta_min=-0.2, theta_max=0.2, theta_bins=8, min_distance=4, max_distance=6, min_sharpness=0.8)
            for sample in candidates.trajectory:
                if sample.s <= 25 and fused_valid:
                    grid.mark(CoverageObservation(sample.s, sample.theta, line.nearest_distance(sample.position), True, True, quality))
            coverage = grid.coverage_ratio
        raw.append({"seed": seed, "A_fixed_safe": fixed is not None, "B_candidate_safe": candidates is not None, "C_coverage_ratio": coverage, "vision_quality": round(quality, 3), "occlusion": round(occlusion, 3), "fusion_valid": fused_valid, "replan_count": replan_count})
    summary = {}
    for key in ("A_fixed_safe", "B_candidate_safe"):
        summary[key] = sum(item[key] for item in raw) / repetitions
    summary["C_mean_coverage_ratio"] = sum(item["C_coverage_ratio"] for item in raw) / repetitions
    summary["D_fusion_valid_rate"] = sum(item["fusion_valid"] for item in raw) / repetitions
    summary["E_mean_replan_count"] = sum(item["replan_count"] for item in raw) / repetitions
    summary["F_mean_vision_quality"] = sum(item["vision_quality"] for item in raw) / repetitions
    report = {"suite": "ablation-batch-v1", "repetitions": repetitions, "summary": summary, "runs": raw}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/ablation_batch.json"))
    parser.add_argument("--repetitions", type=int, default=10)
    args = parser.parse_args()
    print(json.dumps(run(args.output, max(1, args.repetitions)), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
