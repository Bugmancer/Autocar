"""V3 response with delayed commands and a finite chassis yaw response."""

from collections import deque
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


class AdaptiveYawResponseTests(unittest.TestCase):
    def make_controller(self):
        follower = SimpleNamespace(
            geometry=CollisionGeometry(),
            collision_checker=FootprintCollisionChecker(
                prediction_time=0.8, reaction_time=0.15,
                linear_deceleration=0.2, angular_deceleration=0.4),
            command_max_vx=1.0,
            command_max_wz=math.radians(30.0),
            goal_tolerance=0.28,
            _measured_velocity=(0.0, 0.0),
        )
        return AdaptiveController(follower, V3Parameters(curvature_preview=0.6))

    def follow_delayed(self, controller, points, pose):
        path = [PathPoint(*point) for point in points]
        dt = 0.1
        pending = deque([(0.0, 0.0)] * 4)
        actual = (0.0, 0.0)
        max_offset = 0.0
        max_turn_step = 0.0
        previous = (0.0, 0.0)
        # A 0.4 s transport delay followed by a 0.3 s first-order response.
        response = -math.expm1(-dt / 0.3)
        for tick in range(900):
            controller.follower._measured_velocity = actual
            output = controller.compute(pose, path, dt, [], tick * dt)
            command = controller.candidates[0]
            max_turn_step = max(max_turn_step, abs(command[1] - previous[1]))
            previous = command
            controller.sync_command(command, (output[0], output[2]))
            pending.append(command)
            delayed = pending.popleft()
            actual = tuple(old + response * (target - old)
                           for old, target in zip(actual, delayed))
            pose = FootprintCollisionChecker._advance(pose, *actual, dt)
            _, offset = controller.reference.project(*pose[:2])
            max_offset = max(max_offset, offset)
            tolerance = controller.follower.goal_tolerance
            if (math.hypot(path[-1].x - pose[0], path[-1].y - pose[1]) < tolerance
                    and controller.remaining_distance < tolerance):
                return max_offset, max_turn_step
        self.fail("Delayed chassis did not finish: pose=%r command=%r remaining=%.3f"
                  % (pose, previous, controller.remaining_distance))

    def test_delayed_initial_alignment_limits_lateral_excursion(self):
        controller = self.make_controller()
        offset, turn_step = self.follow_delayed(
            controller, [(0.0, 0.0), (5.0, 0.0)], (0.0, 0.0, math.pi / 2))
        self.assertLess(offset, 0.08)
        self.assertLessEqual(turn_step,
                             controller.p.accel_w * 0.1 + controller.p.deadband_w + 1e-12)

    def test_delayed_continuous_turn_stays_near_reference(self):
        controller = self.make_controller()
        radius = 4.0
        points = [(radius * math.sin(index * math.pi / 160),
                   radius * (1.0 - math.cos(index * math.pi / 160)))
                  for index in range(161)]
        offset, turn_step = self.follow_delayed(controller, points, (0.0, 0.0, 0.0))
        self.assertLess(offset, 0.12)
        self.assertLessEqual(turn_step,
                             controller.p.accel_w * 0.1 + controller.p.deadband_w + 1e-12)

    def test_residual_measured_yaw_prevents_acceleration_for_all_candidates(self):
        path = [PathPoint(0.0, 0.0), PathPoint(5.0, 0.0)]
        for direction in (-1.0, 1.0):
            for away in (False, True):
                with self.subTest(direction=direction, away=away):
                    controller = self.make_controller()
                    controller.prev_cmd = controller.last_command = (0.2, direction * 0.08)
                    controller.follower._measured_velocity = (0.2, direction * 0.7)
                    heading = direction * math.radians(10.0) * (1.0 if away else -1.0)
                    output = controller.compute((0.0, 0.0, heading), path, 0.1, [], 0.0)
                    self.assertLessEqual(output[0], 0.2)
                    self.assertLessEqual(abs(output[2] - direction * 0.08), 0.04 + 1e-12)
                    self.assertTrue(all(vx <= 0.2 + 1e-12 for vx, _ in controller.candidates))

    def test_alignment_brakes_measured_yaw_before_forward_handoff(self):
        controller = self.make_controller()
        path = [PathPoint(0.0, 0.0), PathPoint(5.0, 0.0)]
        controller.compute((0.0, 0.0, -math.pi / 2), path, 0.1, [], 0.0)
        controller.prev_cmd = controller.last_command = (0.0, 0.45)
        controller.follower._measured_velocity = (0.0, 0.7)
        output = controller.compute((0.0, 0.0, -0.3), path, 0.1, [], 0.1)
        self.assertEqual(controller.status, "rotating")
        self.assertEqual(output[0], 0.0)
        self.assertAlmostEqual(output[2], 0.41)


if __name__ == "__main__":
    unittest.main()
