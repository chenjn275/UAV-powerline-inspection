#!/usr/bin/env python3
"""Offline contracts for the SUPER-to-PX4 adapter boundary."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import super_planner_adapter as adapter  # noqa: E402


class SuperPlannerAdapterContractTest(unittest.TestCase):
    def test_ned_to_enu_is_inverse_of_zero_yaw_bridge_frame(self):
        point = adapter.SuperPlannerAdapter._ned_to_enu(
            (3.0, 2.0, -4.0), (0.0, 0.0, 0.0), 0.0
        )
        self.assertEqual(point, (2.0, 3.0, 4.0))

    def test_ned_to_enu_applies_origin_and_alignment(self):
        point = adapter.SuperPlannerAdapter._ned_to_enu(
            (10.0, 20.0, 28.0), (10.0, 20.0, 30.0), math.pi / 2.0
        )
        self.assertAlmostEqual(point[0], -0.0)
        self.assertAlmostEqual(point[1], 0.0)
        self.assertAlmostEqual(point[2], 2.0)

    def test_ned_to_enu_matches_bridge_rotation_for_nonzero_offset(self):
        enu = (2.0, -1.5, 4.0)
        origin = (1.0, 3.0, 5.0)
        yaw = 0.37
        # The bridge's ENU-to-NED transform and the adapter's inverse must
        # describe the same local frame when alignment is configured.
        north = enu[1]
        east = enu[0]
        c, s = math.cos(yaw), math.sin(yaw)
        ned = (origin[0] + c * north - s * east,
               origin[1] + s * north + c * east,
               origin[2] - enu[2])
        recovered = adapter.SuperPlannerAdapter._ned_to_enu(ned, origin, yaw)
        for actual, expected in zip(recovered, enu):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_path_signature_rounds_only_for_duplicate_republishes(self):
        goals_a = [adapter._Goal((1.0001, 2.0, 3.0), 0.0)]
        goals_b = [adapter._Goal((1.0002, 2.0, 3.0), 0.0)]
        self.assertEqual(
            adapter.SuperPlannerAdapter._path_signature(goals_a),
            adapter.SuperPlannerAdapter._path_signature(goals_b),
        )


if __name__ == "__main__":
    unittest.main()
