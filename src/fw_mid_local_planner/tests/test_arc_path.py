"""弧长几何测试：检查点密度、自交窗口及端点曲率的稳定性。"""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fw_mid_local_planner.arc_path import ArcPath
from fw_mid_local_planner.path_processing import PathPoint


class ArcPathTests(unittest.TestCase):
    def test_straight_line_is_independent_of_point_density(self):
        sparse = ArcPath([(0, 0), (3, 0)])
        dense = ArcPath([(index * 0.1, 0) for index in range(31)])
        for path in (sparse, dense):
            self.assertAlmostEqual(path.length, 3.0)
            self.assertAlmostEqual(path.sample(1.23).x, 1.23)
            self.assertEqual(path.sample(1.23).yaw, 0.0)
            self.assertEqual(path.project(1.5, 0.4), (1.5, 0.4))
            self.assertEqual(path.peak_curvature(0, 3), 0.0)

    def test_polyline_interpolation_and_endpoint_clamping(self):
        path = ArcPath([(0, 0), (1, 0), (1, 2)])
        self.assertEqual(path.distances, [0.0, 1.0, 3.0])
        self.assertEqual(path.sample(-1), PathPoint(0, 0, 0))
        self.assertEqual(path.sample(0.5), PathPoint(0.5, 0, 0))
        self.assertEqual(path.sample(1), PathPoint(1, 0, math.pi / 2))
        self.assertEqual(path.sample(2), PathPoint(1, 1, math.pi / 2))
        self.assertEqual(path.sample(10), PathPoint(1, 2, math.pi / 2))
        self.assertEqual(path.project(2, 1, 0.5, 1.5), (1.5, math.sqrt(1.25)))

    def test_return_path_uses_progress_window(self):
        path = ArcPath([(0, 0), (2, 0), (0, 0)])
        self.assertEqual(path.project(1, 0), (1.0, 0.0))
        self.assertEqual(path.project(1, 0, 2.0), (3.0, 0.0))
        self.assertEqual(path.project(1, 0, 0.0, 0.5), (0.5, 0.5))
        self.assertEqual(path.project(1, 0, 3.5, 4.0), (3.5, 0.5))

    def test_self_intersection_does_not_jump_to_later_branch(self):
        path = ArcPath([(-1, -1), (1, 1), (-1, 1), (1, -1)])
        first_cross = math.sqrt(2)
        second_cross = 3 * math.sqrt(2) + 2
        self.assertAlmostEqual(path.project(0, 0)[0], first_cross)
        self.assertAlmostEqual(path.project(0, 0, 0, 2)[0], first_cross)
        self.assertAlmostEqual(path.project(0, 0, 4, path.length)[0], second_cross)

    def test_duplicate_and_single_points(self):
        path = ArcPath([PathPoint(1, 2, 0.5), (1, 2), (1, 2)])
        self.assertEqual(len(path.points), 1)
        self.assertEqual(path.length, 0.0)
        self.assertEqual(path.sample(1), PathPoint(1, 2, 0.5))
        self.assertEqual(path.project(4, 6), (0.0, 5.0))
        self.assertEqual(path.curvature(0), 0.0)
        self.assertEqual(path.peak_curvature(0, 1), 0.0)
        path = ArcPath([(0, 0), (0, 0), (1, 0), (1, 0)])
        self.assertEqual(path.distances, [0.0, 1.0])

    def test_invalid_points_and_queries_are_rejected(self):
        for points in ([], [(math.nan, 0)], [(0, math.inf)], [(0,)],
                       [PathPoint(0, 0, math.nan)], [(1e308, 0), (-1e308, 0)]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                ArcPath(points)
        path = ArcPath([(0, 0), (1, 0)])
        for query in (lambda: path.sample(math.nan),
                      lambda: path.project(0, math.inf),
                      lambda: path.project(0, 0, 0.8, 0.2),
                      lambda: path.curvature(0, 0),
                      lambda: path.peak_curvature(0, 1, 0)):
            with self.assertRaises(ValueError):
                query()

    def test_circular_arc_curvature_has_correct_sign_and_endpoints(self):
        radius = 2.0
        points = [(radius * math.cos(index * 0.005),
                   radius * math.sin(index * 0.005)) for index in range(201)]
        forward, reverse = ArcPath(points), ArcPath(points[::-1])
        for s in (0.0, 0.07, 0.7, forward.length - 0.05, forward.length):
            self.assertAlmostEqual(forward.curvature(s), 1 / radius, delta=0.002)
            self.assertAlmostEqual(reverse.curvature(s), -1 / radius, delta=0.002)

    def test_peak_curvature_includes_corner_between_regular_samples(self):
        path = ArcPath([(0, 0), (0.43, 0), (0.43, 1)])
        peak = path.peak_curvature(0, path.length, step=2.0)
        self.assertGreater(peak, 5.0)
        self.assertAlmostEqual(peak, abs(path.curvature(0.43)))

    def test_short_path_uses_finite_one_sided_curvature(self):
        path = ArcPath([(0, 0), (0.03, 0), (0.03, 0.03)])
        for s in (0.0, 0.03, path.length):
            self.assertAlmostEqual(path.curvature(s), math.sqrt(2) / 0.03)
        straight = ArcPath([(0, 0), (0.01, 0)])
        self.assertEqual(straight.curvature(0), 0.0)


if __name__ == "__main__":
    unittest.main()
