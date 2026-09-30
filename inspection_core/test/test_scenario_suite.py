import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from run_scenario_suite import run_suite


class ScenarioSuiteTests(unittest.TestCase):
    def test_suite_contains_safe_bent_and_rejected_cases(self):
        with tempfile.TemporaryDirectory() as directory:
            report = run_suite(Path(directory) / "suite.json")
        results = {item["scenario"]: item for item in report["scenarios"]}
        self.assertGreater(results["straight_clear"]["accepted_count"], 0)
        self.assertGreater(results["bent_line"]["accepted_count"], 0)
        self.assertEqual(results["no_safe_candidate"]["accepted_count"], 0)
