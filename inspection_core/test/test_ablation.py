import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from run_ablation import run


class AblationTests(unittest.TestCase):
    def test_ablation_reports_three_variants(self):
        with tempfile.TemporaryDirectory() as directory:
            report = run(Path(directory) / "ablation.json")
        self.assertEqual([item["variant"] for item in report["scenarios"]], ["A_fixed_radius", "B_candidate_search", "C_candidate_plus_coverage_reconnect"])
        self.assertIsNotNone(report["scenarios"][2]["reconnect_cell"])
