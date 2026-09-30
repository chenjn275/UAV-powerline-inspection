#!/usr/bin/env python3
"""Offline checks for the observation-to-vertical-tower trajectory geometry."""

import math
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
from inspection_manager.path_generation import (  # noqa: E402
    generate_vertical_tower_path,
    plan_observed_tower_scan,
)


class VerticalTowerPathTest(unittest.TestCase):
    def test_two_turn_rising_helix_has_constant_radius(self):
        samples = generate_vertical_tower_path(2.0, -1.0, 1.5, 9.5, 4.0, 2.0, 72)
        self.assertEqual(len(samples), 145)
        self.assertAlmostEqual(samples[0][2], 1.5)
        self.assertAlmostEqual(samples[-1][2], 9.5)
        for x, y, _z, _yaw in samples:
            self.assertAlmostEqual(math.hypot(x - 2.0, y + 1.0), 4.0, places=8)
        self.assertAlmostEqual(samples[0][0], samples[-1][0], places=8)
        self.assertAlmostEqual(samples[0][1], samples[-1][1], places=8)

    def test_partial_request_is_clamped_to_two_turns(self):
        samples = generate_vertical_tower_path(0.0, 0.0, 0.0, 1.0, 1.0, 0.25, 16)
        self.assertEqual(len(samples), 33)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            generate_vertical_tower_path(0.0, 0.0, 3.0, 1.0, 1.0)
        with self.assertRaises(ValueError):
            generate_vertical_tower_path(0.0, 0.0, 0.0, 1.0, 0.0)

    def test_observed_tower_sets_radius_and_vertical_clearances(self):
        plan = plan_observed_tower_scan(
            center_x=4.0, center_y=-1.0, base_z=0.35, top_z=11.75,
            footprint_radius=1.23, minimum_orbit_radius=3.0,
            surface_standoff=2.0, base_clearance=1.5, top_clearance=1.0,
        )
        self.assertAlmostEqual(plan.orbit_radius, 3.23)
        self.assertAlmostEqual(plan.minimum_tower_clearance, 2.0)
        self.assertAlmostEqual(plan.z_min, 1.85)
        self.assertAlmostEqual(plan.z_max, 10.75)
        for x, y, _z, _yaw in plan.samples:
            self.assertAlmostEqual(
                math.hypot(x - plan.center_x, y - plan.center_y) - plan.footprint_radius,
                plan.minimum_tower_clearance,
            )

    def test_observed_tower_rejects_inadequate_height_clearance(self):
        with self.assertRaisesRegex(ValueError, "too short"):
            plan_observed_tower_scan(
                center_x=0.0, center_y=0.0, base_z=0.0, top_z=2.0,
                footprint_radius=0.5, base_clearance=1.5, top_clearance=1.0,
            )


if __name__ == "__main__":
    unittest.main()
