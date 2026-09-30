import json
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from validate_artifacts import validate


class ArtifactValidationTests(unittest.TestCase):
    def test_valid_batch_and_invalid_repetition_count(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "batch.json"
            path.write_text(json.dumps({"suite": "ablation-batch-v1", "repetitions": 1, "runs": [{}], "summary": {"A_fixed_safe": 0, "B_candidate_safe": 1, "C_mean_coverage_ratio": 0.2}}))
            self.assertEqual(validate(path), [])
            path.write_text(json.dumps({"suite": "ablation-batch-v1", "repetitions": 2, "runs": [{}], "summary": {"A_fixed_safe": 0, "B_candidate_safe": 1, "C_mean_coverage_ratio": 0.2}}))
            self.assertTrue(validate(path))
