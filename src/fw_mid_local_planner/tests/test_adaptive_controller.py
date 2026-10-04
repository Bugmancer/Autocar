"""V3 算法回归：物理前视、速度约束、障碍密度和绕行方向保持。"""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SRC = Path(__file__).resolve().parents[2]
for package in ("fw_mid_local_planner", "fw_mid_common_utils"):
    sys.path.insert(0, str(SRC / package))

from fw_mid_common_utils.collision_geometry import CollisionGeometry
from fw_mid_local_planner.adaptive_controller import AdaptiveController, V3Parameters
from fw_mid_local_planner.footprint_collision import FootprintCollisionChecker
from fw_mid_local_planner.path_processing import PathPoint


class AdaptiveControllerTests(unittest.TestCase):
    def make_controller(self, **parameters):
        follower = SimpleNamespace(geometry=CollisionGeometry(),
            collision_checker=FootprintCollisionChecker(),
            command_max_vx=0.3, command_max_wz=0.6, goal_tolerance=0.15,
            _measured_velocity=(0.0, 0.0))
        return AdaptiveController(follower, V3Parameters(**parameters))

    @staticmethod
    def path(points):
        return [PathPoint(x, y) for x, y in points]

    def test_physical_targets_are_independent_of_straight_path_sampling(self):
        sparse = self.path([(0, 0), (3, 0)])
        dense = self.path([(i / 10, 0) for i in range(31)])
        a, b = self.make_controller(), self.make_controller()
        for i in range(15):
            ca = a.compute((0, 0, 0.2), sparse, 0.05, [], i * 0.05)
            cb = b.compute((0, 0, 0.2), dense, 0.05, [], i * 0.05)
            self.assertEqual(ca, cb)
            self.assertEqual(a.last_force, b.last_force)

    def test_preview_grows_with_speed_and_shrinks_before_corner(self):
        straight, corner = self.make_controller(), self.make_controller()
        for controller in (straight, corner):
            controller.follower._measured_velocity = (0.3, 0)
        straight_path = self.path([(0, 0), (3, 0)])
        corner_path = self.path([(0, 0), (0.8, 0), (0.8, 2)])
        for i in range(10):
            straight.compute((0, 0, 0), straight_path, 0.1, [], i * 0.1)
            corner.compute((0, 0, 0), corner_path, 0.1, [], i * 0.1)
        self.assertGreater(straight.lookahead, 0.5)
        self.assertLess(corner.lookahead, straight.lookahead)
        self.assertGreater(corner.reference_curvature, 1)
        self.assertLess(corner.last_command[0], straight.last_command[0])

    def test_corner_speed_respects_yaw_and_lateral_acceleration_bounds(self):
        c = self.make_controller()
        path = self.path([(0, 0), (0.5, 0), (0.5, 2)])
        for i in range(50):
            velocity, _, _ = c.compute((0, 0, 0), path, 0.1, [], i * 0.1)
        self.assertLessEqual(velocity * c.reference_curvature, c.follower.command_max_wz + 1e-9)
        self.assertLessEqual(velocity * velocity * c.reference_curvature, c.p.lateral_acceleration + 1e-9)

    def test_goal_speed_is_monotonic_without_minimum_creep(self):
        speeds = []
        for distance in (1.0, 0.5, 0.25, 0.1):
            c = self.make_controller()
            path = self.path([(0, 0), (distance, 0)])
            for i in range(40):
                output = c.compute((0, 0, 0), path, 0.1, [], i * 0.1)
            speeds.append(output[0])
        self.assertEqual(speeds, sorted(speeds, reverse=True))
        self.assertLess(speeds[-1], 0.04)

    def test_repeated_cloud_points_do_not_change_repulsion_or_command(self):
        points = [(0.85, y) for y in (-0.3, -0.1, 0.1, 0.3)]
        path = self.path([(0, 0), (3, 0)])
        a, b = self.make_controller(), self.make_controller()
        ca = a.compute((0, 0, 0), path, 0.1, points, 1.0)
        cb = b.compute((0, 0, 0), path, 0.1, points * 100, 1.0)
        self.assertEqual(ca, cb)
        self.assertEqual(a.last_force, b.last_force)
        self.assertEqual(a.nearest_clearance, b.nearest_clearance)

    def test_clearance_uses_rotated_rectangle(self):
        c = self.make_controller()
        front = c.follower.geometry.footprint_front + c.follower.geometry.footprint_margin
        radius = c.follower.geometry.obstacle_radius
        sectors = c.obstacle_sectors((2, 3, math.pi / 2), [(2, 3 + front + radius + 0.08)])
        self.assertAlmostEqual(sectors[0][0], 0.08)
        self.assertAlmostEqual(sectors[0][3][0], 0.0)
        self.assertAlmostEqual(sectors[0][3][1], -1.0)

    def test_bypass_side_survives_sensor_jitter_and_temporary_stop(self):
        c = self.make_controller()
        path = self.path([(0, 0), (3, 0)])
        c.compute((0, 0, 0), path, 0.1, [(0.8, -0.01)], 1.0)
        side = c.bypass_side
        self.assertNotEqual(side, 0)
        c.stop()
        c.compute((0, 0, 0), path, 0.1, [(0.8, 0.01)], 1.1)
        self.assertEqual(c.bypass_side, side)
        c.compute((0, 0, 0), path, 0.1, [], 1.2)
        self.assertEqual(c.bypass_side, side)
        c.compute((0, 0, 0), path, 0.1, [], 2.2)
        self.assertEqual(c.bypass_side, 0)

    def test_stationary_projection_does_not_advance_through_loop(self):
        c = self.make_controller()
        path = self.path([(0, 0), (-1, 0), (-1, -1), (1, -1), (1, 0), (0, 0)])
        for _ in range(100):
            c.update_path_progress((0, 0, 0), path)
        self.assertEqual(c.progress, 0)
        self.assertGreater(c.remaining_distance, 5)

    def test_rear_target_starts_with_rotation(self):
        c = self.make_controller()
        path = self.path([(0, 0), (-1, 0), (-1, 2)])
        output = c.compute((0, 0, 0), path, 0.1, [], 1.0)
        self.assertEqual(output[0], 0)
        self.assertNotEqual(output[2], 0)
        self.assertEqual(c.status, "rotating")

    def test_deadband_ramp_can_start_at_high_control_rates(self):
        for dt in (0.1, 0.05, 0.002):
            with self.subTest(dt=dt):
                c = self.make_controller()
                path = self.path([(0, 0), (3, 0)])
                for i in range(40):
                    output = c.compute((0, 0, 0), path, dt, [], i * dt)
                    c.sync_command((output[0], output[2]), (output[0], output[2]))
                self.assertGreater(output[0], 0)

    def test_downstream_speed_reduction_updates_ramp(self):
        c = self.make_controller()
        c.prev_cmd = (0.3, 0.2)
        c.sync_command((0.05, 0.03), (0.3, 0.2))
        self.assertEqual(c.prev_cmd, (0.05, 0.03))

    def test_late_plan_projects_past_already_travelled_prefix(self):
        c = self.make_controller()
        path = self.path([(0, 0), (1, 0), (2, 0)])
        c.follower._measured_velocity = (0.3, 0)
        velocity, _, turn = c.compute((0.3, 0, 0), path, 0.1, [], 1.0)
        self.assertAlmostEqual(c.progress, 0.3)
        self.assertGreater(velocity, 0)
        self.assertEqual(turn, 0)

    def test_every_candidate_obeys_actual_lateral_acceleration(self):
        c = self.make_controller(lateral_acceleration=0.01)
        path = self.path([(0, 0), (4, 2)])
        pose = (0, 0, 0)
        for i in range(30):
            output = c.compute(pose, path, 0.1, [], i * 0.1)
            self.assertLessEqual(abs(output[0] * output[2]), 0.01 + 1e-12)
            self.assertTrue(c.candidates)
            for velocity, turn in c.candidates:
                self.assertLessEqual(abs(velocity * turn), 0.01 + 1e-12)
            velocity, turn = c.candidates[0]
            c.sync_command((velocity, turn), (output[0], output[2]))
            pose = FootprintCollisionChecker._advance(pose, velocity, turn, 0.1)

    def test_endpoint_projection_keeps_correcting_lateral_goal_error(self):
        c = self.make_controller()
        path = self.path([(0, 0), (2, 0)])
        velocity, _, turn = c.compute((2, 0.5, -math.pi / 2), path, 0.1, [], 1.0)
        self.assertEqual(c.remaining_distance, 0)
        self.assertGreater(velocity, 0)
        self.assertAlmostEqual(turn, 0)

    def test_invalid_parameters_and_inputs_are_rejected(self):
        for name, value in (("lookahead_min", -1), ("lookahead_min", 2),
                            ("obstacle_sectors", 0), ("obstacle_sectors", 8.5),
                            ("trajectory_max_checks", 16), ("accel_v", float("nan")),
                            ("rotate_threshold", math.pi)):
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                V3Parameters.from_getter(lambda key, default: value if key == "v3_" + name else default)
        c = self.make_controller()
        self.assertEqual(c.compute((0, 0, 0), self.path([(1, 0)]), 0, [], 0), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
