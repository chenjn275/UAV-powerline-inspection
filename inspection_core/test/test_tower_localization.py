import math
import random
import unittest

from inspection_core.tower_localization import estimate_tower_observation


class TowerLocalizationTest(unittest.TestCase):
    def test_robust_center_ignores_sparse_outliers(self):
        rng = random.Random(7)
        points = []
        for _ in range(200):
            points.append((0.2 * rng.uniform(-1, 1), 0.2 * rng.uniform(-1, 1), rng.uniform(0, 10)))
        points.extend((30.0, -20.0, 100.0) for _ in range(3))
        obs = estimate_tower_observation(points)
        self.assertAlmostEqual(obs.center_x, 0.0, delta=0.08)
        self.assertAlmostEqual(obs.center_y, 0.0, delta=0.08)
        self.assertGreater(obs.height_m, 9.0)
        self.assertGreater(obs.uncertainty_m, 0.05)

    def test_rejects_stale_or_degenerate_cloud(self):
        with self.assertRaises(ValueError):
            estimate_tower_observation([(0, 0, 0)] * 10)
        with self.assertRaises(ValueError):
            estimate_tower_observation([(0, 0, 1 + i * 0.001) for i in range(40)])


if __name__ == "__main__":
    unittest.main()
