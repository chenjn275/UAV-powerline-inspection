import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from run_ablation_batch import run


class AblationBatchTests(unittest.TestCase):
    def test_batch_keeps_raw_runs_and_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            report = run(Path(directory) / "batch.json", repetitions=3)
        self.assertEqual(len(report["runs"]), 3)
        self.assertIn("A_fixed_safe", report["summary"])
        self.assertIn("C_mean_coverage_ratio", report["summary"])
