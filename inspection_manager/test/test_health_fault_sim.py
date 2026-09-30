import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(REPO_ROOT / "inspection_core"))

from inspection_core.health import HealthState  # noqa: E402
from inspection_manager.health_fault_sim_node import HealthFaultSim  # noqa: E402


class HealthFaultSimulationTest(unittest.TestCase):
    def test_sequence_parser_keeps_valid_faults_and_falls_back(self):
        sequence = HealthFaultSim._parse_sequence(
            "READY:2,STALE_POINTCLOUD:1,garbage,LINK_LOST:3"
        )
        self.assertEqual(
            sequence,
            (
                (HealthState.READY, 2.0),
                (HealthState.STALE_POINTCLOUD, 1.0),
                (HealthState.LINK_LOST, 3.0),
            ),
        )
        self.assertEqual(
            HealthFaultSim._parse_sequence("bad"), ((HealthState.READY, 1.0),)
        )

    def test_fault_actions_are_conservative(self):
        self.assertEqual(
            HealthFaultSim._action(HealthState.STALE_POINTCLOUD),
            "PAUSE_AND_REPLAN",
        )
        self.assertEqual(
            HealthFaultSim._action(HealthState.LINK_LOST),
            "HOLD_ONLY_IF_INDEPENDENT_ESTIMATE_VALID",
        )


if __name__ == "__main__":
    unittest.main()
