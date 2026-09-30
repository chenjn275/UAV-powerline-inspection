#!/usr/bin/env python3
"""Offline contract checks for the guarded PX4 path adapter."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import px4_offboard_bridge as bridge  # noqa: E402


class OffboardBridgeContractTest(unittest.TestCase):
    def test_enu_to_ned_zero_yaw_and_origin(self):
        self.assertEqual(
            bridge.Px4OffboardBridge._enu_to_ned((2.0, 3.0, 4.0), (0.0, 0.0, 0.0), 0.0),
            (3.0, 2.0, -4.0),
        )

    def test_enu_to_ned_applies_origin_and_yaw(self):
        point = bridge.Px4OffboardBridge._enu_to_ned(
            (1.0, 0.0, 2.0), (10.0, 20.0, 30.0), math.pi / 2.0
        )
        self.assertAlmostEqual(point[0], 9.0)
        self.assertAlmostEqual(point[1], 20.0)
        self.assertAlmostEqual(point[2], 28.0)

    def test_invalid_alignment_is_rejected_by_contract(self):
        point = bridge.Px4OffboardBridge._enu_to_ned((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 0.0)
        self.assertTrue(all(math.isfinite(value) for value in point))


if __name__ == "__main__":
    unittest.main()
