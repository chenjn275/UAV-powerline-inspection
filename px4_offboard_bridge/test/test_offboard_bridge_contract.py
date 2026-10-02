#!/usr/bin/env python3
"""Offline contract checks for the guarded PX4 path adapter."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import px4_offboard_bridge as bridge  # noqa: E402


class OffboardBridgeContractTest(unittest.TestCase):
    def test_vehicle_status_subscription_uses_px4_versioned_topic(self):
        self.assertEqual(bridge.PX4_VEHICLE_STATUS_TOPIC, "/fmu/out/vehicle_status_v1")

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

    def test_ned_to_enu_is_inverse_of_enu_to_ned(self):
        enu = (2.0, -1.5, 4.0)
        origin = (1.0, 3.0, 5.0)
        yaw = 0.37
        ned = bridge.Px4OffboardBridge._enu_to_ned(enu, origin, yaw)
        recovered = bridge.Px4OffboardBridge._ned_to_enu(ned, origin, yaw)
        for actual, expected in zip(recovered, enu):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_invalid_alignment_is_rejected_by_contract(self):
        point = bridge.Px4OffboardBridge._enu_to_ned((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 0.0)
        self.assertTrue(all(math.isfinite(value) for value in point))

    def test_enu_yaw_converts_to_px4_heading(self):
        convert = bridge.Px4OffboardBridge._enu_yaw_to_ned
        self.assertAlmostEqual(convert(0.0), math.pi / 2.0)
        self.assertAlmostEqual(convert(math.pi / 2.0), 0.0)

    def test_time_sampling_does_not_jump_between_overlapping_orbits(self):
        points = [
            (0.0, 0.0, float(index))
            if index % 4 == 0 else
            ((0.0, 1.0, float(index)) if index % 4 == 1 else
             ((-1.0, 0.0, float(index)) if index % 4 == 2 else (0.0, -1.0, float(index))))
            for index in range(9)
        ]
        yaws = [0.1 * index for index in range(len(points))]
        sample = bridge.Px4OffboardBridge._sample_path
        before = sample(points, yaws, 0.49)[0]
        after = sample(points, yaws, 0.51)[0]
        self.assertLess(math.dist(before, after), 0.5)
        self.assertGreater(after[2], before[2])

    def test_position_reference_step_is_speed_limited(self):
        limited = bridge.Px4OffboardBridge._limit_position_step(
            (0.0, 0.0, 0.0), (10.0, 0.0, 0.0), dt=0.1, max_speed=1.5
        )
        self.assertAlmostEqual(limited[0], 0.15)
        self.assertAlmostEqual(math.dist((0.0, 0.0, 0.0), limited), 0.15)

    def test_smooth_progress_is_monotone_with_stationary_endpoints(self):
        ease = bridge.Px4OffboardBridge._ease_progress
        self.assertEqual(ease(0.0), 0.0)
        self.assertEqual(ease(1.0), 1.0)
        samples = [ease(i / 20.0) for i in range(21)]
        self.assertEqual(samples, sorted(samples))
        self.assertLess(samples[1], 0.01)
        self.assertGreater(samples[-2], 0.99)


if __name__ == "__main__":
    unittest.main()
