import json
import tempfile
import unittest
from pathlib import Path

from inspection_core.baseline_scenario import run


class BaselineScenarioTests(unittest.TestCase):
    def test_report_contains_candidate_and_reconnect_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            report = run(Path(directory) / "baseline.json")
            self.assertEqual(report["scenario"], "straight-single-span-with-interruption")
            self.assertGreaterEqual(len(report["candidate_evaluations"]), 3)
            self.assertGreater(report["coverage"]["uncovered_cells"], 0)
            self.assertIsNotNone(report["coverage"]["reconnect_cell"])
            loaded = json.loads((Path(directory) / "baseline.json").read_text())
            self.assertEqual(loaded["selected"]["radius_m"], report["selected"]["radius_m"])
