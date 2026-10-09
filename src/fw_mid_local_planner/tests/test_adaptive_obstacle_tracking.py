"""Delayed-actuator regressions for V3 tracking beside observed walls."""

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


class AdaptiveObstacleTrackingTests(unittest.TestCase):
    def make_controller(self, max_vx):
        geometry = CollisionGeometry()
        follower = SimpleNamespace(
            geometry=geometry,
            collision_checker=FootprintCollisionChecker(
                prediction_time=0.8,
                reaction_time=0.15,
                linear_deceleration=0.20,
                angular_deceleration=0.40,
            ),
            command_max_vx=max_vx,
            command_max_wz=math.radians(30.0),
            goal_tolerance=0.28,
            _measured_velocity=(0.0, 0.0),
        )
        return AdaptiveController(follower, V3Parameters(curvature_preview=0.6))

    def follow_wall(self, controller, path, wall, timeout=40.0):
        dt = 0.1
        queued = deque([(0.0, 0.0)] * 5)
        pose = (0.0, 0.0, 0.0)
        offsets, turns, errors = [], [], []
        stationary_turns = 0
        for tick in range(round(timeout / dt)):
            controller.follower._measured_velocity = queued[0]
            output = controller.compute(pose, path, dt, wall, tick * dt)
            self.assertTrue(controller.candidates)
            command = controller.candidates[0]
            controller.sync_command(command, (output[0], output[2]))
            queued.append(command)
            actual = queued.popleft()
            if actual[0] < 0.01 and abs(actual[1]) > 0.01:
                stationary_turns += 1
            if abs(actual[1]) > 0.12:
                turns.append(1 if actual[1] > 0.0 else -1)
            errors.append(abs(controller.heading_error))
            # Check the executed sweep, including actuator lag, rather than only commands.
            for sample in range(1, 6):
                executed = FootprintCollisionChecker._advance(
                    pose, *actual, dt * sample / 5.0)
                result = controller.follower.collision_checker.check(
                    executed, 0.0, 0.0, wall)
                self.assertTrue(result.safe, "Executed footprint hit wall at %r" %
                                (executed,))
            pose = executed
            offsets.append(controller.reference.project(pose[0], pose[1])[1])
            tolerance = controller.follower.goal_tolerance
            if (math.hypot(path[-1].x - pose[0], path[-1].y - pose[1]) <= tolerance
                    and controller.remaining_distance <= tolerance):
                return SimpleNamespace(
                    elapsed=(tick + 1) * dt,
                    stationary_turns=stationary_turns,
                    max_offset=max(offsets),
                    max_heading_error=max(errors),
                    steering_reversals=sum(a != b for a, b in zip(turns, turns[1:])),
                )
        self.fail("Delayed controller did not finish beside the wall: pose=%r remaining=%.3f" %
                  (pose, controller.remaining_distance))

    def test_delayed_curved_wall_tracking_does_not_create_extra_pivot_turns(self):
        path = [PathPoint(index * 0.05, 0.15 * math.sin(4.0 * index * 0.05))
                for index in range(101)]
        wall = [(index * 0.05, 0.8) for index in range(121)]
        for max_vx in (0.4, 1.0):
            with self.subTest(max_vx=max_vx):
                result = self.follow_wall(self.make_controller(max_vx), path, wall)
                self.assertEqual(result.stationary_turns, 0)
                self.assertLess(result.max_heading_error, math.radians(50.0))
                self.assertLess(result.max_offset, 0.17)
                self.assertLessEqual(result.steering_reversals, 8)

    def test_delayed_straight_wall_tracking_settles_without_repeated_corrections(self):
        path = [PathPoint(0.0, 0.0), PathPoint(5.0, 0.0)]
        wall = [(index * 0.05, 0.6) for index in range(121)]
        for max_vx in (0.4, 1.0):
            with self.subTest(max_vx=max_vx):
                result = self.follow_wall(self.make_controller(max_vx), path, wall)
                self.assertEqual(result.stationary_turns, 0)
                self.assertLess(result.max_offset, 0.25)
                self.assertLessEqual(result.steering_reversals, 1)


if __name__ == "__main__":
    unittest.main()
