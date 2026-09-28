"""Offline behavioral checks: python -m unittest discover -s .../tests -v."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SRC = Path(__file__).resolve().parents[2]
for package in ("fw_mid_local_planner", "fw_mid_common_utils", "fw_mid_controller"):
    sys.path.insert(0, str(SRC / package))

from fw_mid_common_utils.collision_geometry import CollisionGeometry
from fw_mid_local_planner.footprint_collision import FootprintCollisionChecker
from fw_mid_local_planner.path_processing import PathPoint
from fw_mid_local_planner.potential_field import ArtificialPotentialFieldController


class Layer:
    enabled = True
    timeout = 0.5

    def __init__(self, points=()):
        self.points = list(points)

    def timed_point_snapshot(self):
        return list(self.points)


def follower(**overrides):
    params = dict(
        apf_waypoint_stride=4, apf_near_attraction_gain=1.0, apf_far_attraction_gain=0.35,
        apf_obstacle_influence_distance=0.9, apf_obstacle_radius=CollisionGeometry().obstacle_radius,
        apf_repulsive_gain=0.08, apf_repulsive_max_force=8.0, apf_max_obstacles=300,
        apf_force_epsilon=1e-4, command_max_vx=0.3, command_max_wz=math.radians(30),
        apf_goal_approach_distance=0.8, apf_min_speed_scale=0.2, apf_stop_rotate_yaw=1.45,
        apf_slowdown_yaw=1.0, apf_min_vx=0.04, apf_heading_gain=1.8,
        apf_accel_limit_v=0.25, apf_accel_limit_wz=0.3,
        apf_deadband_v=0.015, apf_deadband_wz=0.025,
        apf_danger_clearance=0.05,
        apf_force_filter_time=0.2, apf_memory_decay_time=2.0, apf_memory_min_weight=0.25,
        geometry=CollisionGeometry(), dynamic_layer=Layer(), now_sec=lambda: 10.0)
    params.update(overrides)
    return SimpleNamespace(**params)


class PotentialFieldTests(unittest.TestCase):
    def setUp(self):
        self.path = [PathPoint(x, 0) for x in (0.4, 0.8, 1.2, 1.6, 2.0)]

    def test_two_attractions_constant_with_distance_and_single_goal_not_doubled(self):
        c = ArtificialPotentialFieldController(follower())
        c.compute((0, 0, 0), self.path, None, 0.1)
        self.assertAlmostEqual(c.last_raw_force[0], 1.35)
        c.compute((-5, 0, 0), self.path, None, 0.1)
        self.assertAlmostEqual(c.last_raw_force[0], 1.35)
        c.compute((0, 0, 0), self.path[-1:], None, 0.1)
        self.assertAlmostEqual(c.last_raw_force[0], 1.0)

    def test_obstacle_distance_does_not_scale_speed_outside_danger_zone(self):
        p = follower(apf_repulsive_gain=0.0001, apf_deadband_v=0)
        speeds = []
        for x in (1.1, 0.9, 0.75, 0.65, 0.52):
            p.dynamic_layer.points = [(x, 0, 10)]
            c = ArtificialPotentialFieldController(p)
            speeds.append(c.compute((0, 0, 0), self.path, None, 0.1)[0])
        self.assertTrue(all(speed > 0 for speed in speeds))
        self.assertAlmostEqual(speeds[0], speeds[-1])
        p.dynamic_layer.points = [(0.42, 0, 10)]
        c = ArtificialPotentialFieldController(p)
        self.assertEqual(c.compute((0, 0, 0), self.path, None, 0.1), (0, 0, 0))
        self.assertEqual(c.status, "danger_zone")

    def test_command_cap_below_minimum_speed_is_respected(self):
        p = follower(command_max_vx=0.02, apf_min_vx=0.1, apf_deadband_v=0)
        c = ArtificialPotentialFieldController(p)
        for _ in range(20):
            self.assertLessEqual(c.compute((0, 0, 0), self.path, None, 0.1)[0], 0.02)

    def test_rectangle_clearance_includes_rotated_sides_and_corners(self):
        g = CollisionGeometry()
        pad_x, pad_y = g.footprint_front + g.footprint_margin, g.footprint_half_width + g.footprint_margin
        for bx, by, expected in ((pad_x + 0.2, 0, 0.2 - g.obstacle_radius),
                                 (0, pad_y + 0.2, 0.2 - g.obstacle_radius),
                                 (pad_x + 0.3, pad_y + 0.4, 0.5 - g.obstacle_radius),
                                 (0, 0, 0)):
            for yaw in (0, math.pi / 2, -1.1):
                p = follower(apf_max_obstacles=1)
                x = 2 + math.cos(yaw) * bx - math.sin(yaw) * by
                y = 3 + math.sin(yaw) * bx + math.cos(yaw) * by
                p.dynamic_layer.points = [(x, y, 10)]
                c = ArtificialPotentialFieldController(p)
                c.compute((2, 3, yaw), self.path, None, 0.1)
                self.assertAlmostEqual(c.nearest_clearance, expected)

    def test_age_decay_keeps_full_memory_clearance_and_collision(self):
        p = follower()
        c = ArtificialPotentialFieldController(p)
        p.dynamic_layer.points = [(0.65, 0.15, 10)]
        c.compute((0, 0, 0), self.path, None, 0.1)
        recent_y, recent_clearance = c.last_raw_force[1], c.nearest_clearance
        p.dynamic_layer.points = [(0.65, 0.15, 0)]
        c.compute((0, 0, 0), self.path, None, 0.1)
        self.assertLess(abs(c.last_raw_force[1]), abs(recent_y))
        self.assertGreater(abs(c.last_raw_force[1]), 0)
        self.assertEqual(c.nearest_clearance, recent_clearance)
        self.assertEqual(c.status, "tracking")
        self.assertEqual(p.dynamic_layer.points, [(0.65, 0.15, 0)])
        result = FootprintCollisionChecker().check((0, 0, 0), 0.3, 0,
            [(x, y) for x, y, _ in p.dynamic_layer.points])
        self.assertFalse(result.safe)

    def test_age_clock_rollback_is_conservative_and_invalid_stamp_stops(self):
        p = follower()
        c = ArtificialPotentialFieldController(p)
        self.assertEqual(c._age_weight(20, 10), 1)
        p.dynamic_layer.points = [(0.8, 0, float("nan"))]
        self.assertEqual(c.compute((0, 0, 0), self.path, None, 0.1), (0, 0, 0))
        self.assertEqual(c.status, "invalid_input")

    def test_vertical_duplicate_points_do_not_multiply_repulsion(self):
        p = follower()
        c = ArtificialPotentialFieldController(p)
        p.dynamic_layer.points = [(0.7, 0.2, 10)]
        c.compute((0, 0, 0), self.path, None, 0.1)
        force = c.last_raw_force
        p.dynamic_layer.points.extend([(0.7, 0.2, 9), (0.7, 0.2, 0)] * 20)
        c.compute((0, 0, 0), self.path, None, 0.1)
        self.assertEqual(c.last_raw_force, force)

    def test_filter_damps_oscillation_and_resets_on_accepted_path(self):
        p = follower()
        c = ArtificialPotentialFieldController(p)
        p.dynamic_layer.points = [(0.7, 0.2, 10)]
        c.compute((0, 0, 0), self.path, None, 0.1)
        p.dynamic_layer.points = [(0.7, -0.2, 10)]
        c.compute((0, 0, 0), list(self.path), None, 0.1)
        self.assertLess(abs(c.last_force[1]), abs(c.last_raw_force[1]))
        new_path = [PathPoint(p.x, p.y) for p in self.path]
        c.compute((0, 0, 0), new_path, None, 0.1)
        self.assertEqual(c.last_force, c.last_raw_force)
        c.reset()
        self.assertIsNone(c.last_force)

    def test_danger_stop_is_not_delayed_by_force_filter(self):
        p = follower(apf_force_filter_time=10.0)
        c = ArtificialPotentialFieldController(p)
        c.compute((0, 0, 0), self.path, None, 0.1)
        c.prev_cmd = (0.3, 0)
        p.dynamic_layer.points = [(0.38, 0, 10)]
        self.assertEqual(c.compute((0, 0, 0), self.path, None, 0.1)[0], 0)
        self.assertEqual(c.status, "danger_zone")
        self.assertIsNone(c.last_force)
        self.assertLess(c.last_raw_force[0], 0)

    def test_filter_handles_angle_wraparound(self):
        c = ArtificialPotentialFieldController(follower())
        path = [PathPoint(-2, 0.01)]
        c.compute((0, 0, math.pi), path, None, 0.1)
        path[0].y = -0.01
        c.compute((0, 0, math.pi), path, None, 0.1)
        self.assertLess(c.last_force[0], -0.99)
        self.assertGreater(abs(c.last_heading), 3.0)

    def test_repulsion_changes_force_without_distance_speed_scaling(self):
        p = follower(apf_repulsive_gain=1.5, apf_deadband_v=0)
        p.dynamic_layer.points = [(0.7, 0, 10)]
        c = ArtificialPotentialFieldController(p)
        command = c.compute((0, 0, 0), self.path, None, 0.1)
        self.assertLess(c.last_raw_force[0], 0)
        self.assertEqual(command[0], 0)
        self.assertEqual(c.status, "tracking")

    def test_startup_escapes_deadbands_at_multiple_rates(self):
        for rate in (10, 20, 50):
            p = follower()
            c = ArtificialPotentialFieldController(p)
            path = [PathPoint(2, 0.4)]
            for _ in range(rate):
                vx, _, wz = c.compute((0, 0, 0), path, None, 1 / rate)
            self.assertGreater(vx, p.apf_deadband_v)
            self.assertGreater(wz, p.apf_deadband_wz)

    def test_downstream_collision_slowdown_resets_ramp_to_published_speed(self):
        c = ArtificialPotentialFieldController(follower(apf_deadband_v=0))
        c.prev_cmd = (0.3, 0)
        c.compute((0, 0, 0), self.path, None, 0.1)
        c.sync_command(0.05, 0)
        vx, _, _ = c.compute((0, 0, 0), self.path, None, 0.1)
        self.assertLessEqual(vx, 0.075 + 1e-9)

    def test_force_cancellation_preserves_reason_and_has_no_direction(self):
        p = follower()
        gap = 0.7 - p.apf_obstacle_radius
        p.apf_repulsive_gain = 1.35 * gap**2 / (1 / gap - 1 / p.apf_obstacle_influence_distance)
        p.dynamic_layer.points = [(0.7, 0, 10)]
        c = ArtificialPotentialFieldController(p)
        self.assertEqual(c.compute((0, 0, 0), self.path, None, 0.1), (0, 0, 0))
        self.assertEqual(c.status, "force_cancelled")
        self.assertIsNone(c.last_force)


if __name__ == "__main__":
    unittest.main()
