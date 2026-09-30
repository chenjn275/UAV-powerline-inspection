import unittest
from inspection_core import DynamicReplanner, PolylineLineModel, SphereObstacle


class DynamicReplannerTest(unittest.TestCase):
    def test_new_obstacle_forces_replan(self):
        line = PolylineLineModel([(0, 0, 10), (20, 0, 10)])
        replanner = DynamicReplanner(line, radii=(4.0, 5.0, 6.0), line_exclusion_radius=2.0)
        clear = replanner.update(())
        self.assertEqual(clear.reason, "SAFE_CANDIDATE")
        self.assertEqual(clear.sequence, 1)
        blocked = replanner.update((SphereObstacle((10, 0, 6), 1.0, "new_obstacle"),))
        self.assertEqual(blocked.sequence, 2)
        self.assertEqual(blocked.reason, "SAFE_CANDIDATE")
        self.assertNotEqual(blocked.selected.radius, clear.selected.radius)

    def test_no_candidate_is_reported(self):
        line = PolylineLineModel([(0, 0, 10), (20, 0, 10)])
        replanner = DynamicReplanner(line, radii=(4.0,), line_exclusion_radius=2.0, obstacle_margin=1.0)
        result = replanner.update((SphereObstacle((10, 0, 6), 10.0, "blocking"),))
        self.assertEqual(result.reason, "NO_SAFE_CANDIDATE")
        self.assertIsNone(result.selected)


if __name__ == "__main__":
    unittest.main()
