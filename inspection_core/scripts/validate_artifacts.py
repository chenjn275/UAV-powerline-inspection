#!/usr/bin/env python3
"""Validate archived JSON verification artifacts without trusting their prose."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def validate(path: Path) -> list[str]:
    errors = []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [f"invalid JSON: {exc}"]
    suite = data.get("suite")
    if suite is None and data.get("scenario") == "straight-single-span-with-interruption":
        if not data.get("selected") or not data.get("coverage"):
            errors.append("baseline report lacks selected or coverage evidence")
        return errors
    if suite == "ablation-batch-v1":
        runs = data.get("runs")
        if not isinstance(runs, list) or len(runs) != data.get("repetitions"):
            errors.append("batch runs do not match repetitions")
        summary = data.get("summary", {})
        for key in ("A_fixed_safe", "B_candidate_safe", "C_mean_coverage_ratio"):
            if not isinstance(summary.get(key), (int, float)) or not 0.0 <= summary[key] <= 1.0:
                errors.append(f"invalid batch summary: {key}")
    elif suite == "ablation-v1":
        variants = [item.get("variant") for item in data.get("scenarios", [])]
        if variants != ["A_fixed_radius", "B_candidate_search", "C_candidate_plus_coverage_reconnect"]:
            errors.append("ablation variants are incomplete or out of order")
    elif suite == "inspection-geometry-baseline-v1":
        if not data.get("scenarios"):
            errors.append("scenario suite is empty")
    else:
        errors.append(f"unsupported suite: {suite}")
    return errors


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path)
    args = parser.parse_args()
    failures = 0
    for path in args.paths:
        errors = validate(path)
        if errors:
            failures += 1
            print(f"FAIL {path}: " + "; ".join(errors))
        else:
            print(f"OK   {path}")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
