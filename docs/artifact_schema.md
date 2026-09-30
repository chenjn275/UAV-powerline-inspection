# Verification artifact schema

Every reproducible scenario stores one JSON report under `artifacts/` with:

- `scenario`, `model_version`, and the configuration/seed used;
- every candidate's radius, acceptance, minimum line/obstacle clearance and rejection reason;
- the selected candidate and its score;
- coverage cell counts, ratio, uncovered-cell count and reconnect target;
- links or filenames for raw bags/logs when a live ROS or PX4 run is added.

The baseline generator is:

```bash
cd /home/venom/Documents/ChatGPT/巡检
PYTHONPATH=SUPER/inspection_core \
  python3 SUPER/inspection_core/scripts/run_baseline_scenario.py \
  --output artifacts/baseline_scenario.json
```

The report is evidence for geometry and bookkeeping behavior only. It does not
prove camera visibility, LIO accuracy, PX4 tracking, or electrical clearance.

## Current archived artifacts

- `artifacts/baseline_scenario.json`: interruption and reconnect baseline;
- `artifacts/scenario_suite.json`: straight, bent-line and no-safe-candidate cases;
- `artifacts/ablation.json`: one A/B/C comparison;
- `artifacts/ablation_batch.json`: ten deterministic perturbation runs with raw rows and summary.

The batch summary currently records fixed-radius safe rate `0.0`, candidate-search safe rate `1.0`, and mean bookkeeping coverage `0.285`. These values describe this synthetic geometry suite only.
