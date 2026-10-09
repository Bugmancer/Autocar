"""Closed-loop V3 regressions with collision-free differential-drive motion."""

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


class AdaptiveClosedLoopTests(unittest.TestCase):
    def make_controller(self, max_vx=0.4, max_wz_deg=30.0):
        follower = SimpleNamespace(
            geometry=CollisionGeometry(),
            collision_checker=FootprintCollisionChecker(),
            command_max_vx=max_vx,
            command_max_wz=math.radians(max_wz_deg),
            goal_tolerance=0.28,
            _measured_velocity=(0.0, 0.0),
        )
        return AdaptiveController(follower, V3Parameters(curvature_preview=0.6))

    @staticmethod
    def path(points):
        return [PathPoint(x, y) for x, y in points]

    def follow(self, controller, path, pose, timeout=60.0):
        commands = []
        dt = 0.1
        for tick in range(round(timeout / dt)):
            controller.follower._measured_velocity = controller.last_command
            output = controller.compute(pose, path, dt, [], tick * dt)
            self.assertTrue(controller.candidates)
            # All candidates are collision-free here, so the adapter accepts the first.
            command = controller.candidates[0]
            controller.sync_command(command, (output[0], output[2]))
            commands.append(command)
            pose = FootprintCollisionChecker._advance(pose, *command, dt)
            tolerance = controller.follower.goal_tolerance
            if (math.hypot(path[-1].x - pose[0], path[-1].y - pose[1]) <= tolerance
                    and controller.remaining_distance <= tolerance):
                return pose, commands
        self.fail("Controller did not finish the collision-free path: "
                  "pose=%r remaining=%.3f command=%r" %
                  (pose, controller.remaining_distance, commands[-1]))

    def test_safe_forward_command_completes_sharp_corner(self):
        for max_vx, max_wz in ((0.4, 30.0), (1.0, 20.0)):
            with self.subTest(max_vx=max_vx, max_wz_deg=max_wz):
                controller = self.make_controller(max_vx=max_vx, max_wz_deg=max_wz)
                path = self.path([(0, 0), (1, 0), (1, 4)])
                _, commands = self.follow(controller, path, (0, 0, 0))
                self.assertTrue(all(velocity > 0.0 for velocity, _ in commands))

    def test_moderate_heading_error_starts_forward_motion(self):
        controller = self.make_controller()
        path = self.path([(0, 0), (4, 0)])
        velocity, _, _ = controller.compute(
            (0, 0, math.radians(60)), path, 0.1, [], 0.0)
        self.assertGreater(velocity, 0.0)
        self.assertNotEqual(controller.status, "rotating")
        self.follow(controller, path, (0, 0, math.radians(60)))

    def test_large_initial_turn_does_not_reverse_steering_to_correct_overshoot(self):
        for heading in (math.pi / 2, math.pi):
            with self.subTest(heading=heading):
                controller = self.make_controller()
                path = self.path([(0, 0), (5, 0)])
                _, commands = self.follow(controller, path, (0, 0, heading))
                self.assertLessEqual(max(turn for _, turn in commands), 0.03)

    def test_active_replan_preserves_adaptive_preview(self):
        controller = self.make_controller()
        path = self.path([(0, 0), (5, 0)])
        pose = (0, 0, 0)
        for tick in range(40):
            controller.follower._measured_velocity = controller.last_command
            velocity, _, turn = controller.compute(pose, path, 0.1, [], tick * 0.1)
            controller.sync_command((velocity, turn), (velocity, turn))
            pose = FootprintCollisionChecker._advance(pose, velocity, turn, 0.1)
        previous = controller.lookahead
        self.assertGreater(previous, 0.7)
        replanned = self.path([(pose[0], 0), (5, 0)])
        controller.compute(pose, replanned, 0.1, [], 4.0)
        self.assertGreaterEqual(controller.lookahead, previous - 0.01)


if __name__ == "__main__":
    unittest.main()
