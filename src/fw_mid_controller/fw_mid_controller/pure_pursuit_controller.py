#!/usr/bin/env python3
"""Pure-pursuit tracker for the FW-mid ROS1 local planner."""

import math
from typing import Sequence

from fw_mid_common_utils import Pose2D, clamp, normalize_angle

from ._params import get_param


class PurePursuitController:
    def __init__(self, params=None, cruise_speed=None) -> None:
        self.cruise_speed = None if cruise_speed is None else float(cruise_speed)
        if self.cruise_speed is not None and (
                not math.isfinite(self.cruise_speed) or self.cruise_speed < 0):
            raise ValueError("Cruise speed must be finite and nonnegative")
        self.k_v = float(get_param(params, "pp_k_v", 0.55))
        self.min_vx = float(get_param(params, "pp_min_vx", 0.04))
        self.max_vx = float(get_param(params, "pp_max_vx", 0.26))
        if self.cruise_speed is not None:
            self.max_vx = self.cruise_speed
        self.max_wz = float(get_param(params, "pp_max_wz", 0.22))
        self.heading_gain = float(get_param(params, "pp_heading_gain", 0.30))
        self.curvature_slowdown = float(get_param(params, "pp_curvature_slowdown", 0.75))
        self.approach_dist = float(get_param(params, "pp_approach_dist", 0.80))
        self.stop_rotate_yaw = float(get_param(params, "pp_stop_rotate_yaw", 1.85))
        self.accel_limit_v = float(get_param(params, "pp_accel_limit_v", 0.25))
        self.accel_limit_wz = float(get_param(params, "pp_accel_limit_wz", 0.30))
        self.deadband_v = float(get_param(params, "pp_deadband_v", 0.015))
        self.deadband_wz = float(get_param(params, "pp_deadband_wz", 0.025))
        self.reset()

    def reset(self) -> None:
        self.prev_cmd = (0.0, 0.0)

    @staticmethod
    def limit_rate(target: float, previous: float, limit: float, dt: float) -> float:
        delta = max(0.0, limit) * max(float(dt), 1e-3)
        return clamp(target, previous - delta, previous + delta)

    def compute(
        self,
        robot_pose: Pose2D,
        path: Sequence,
        target,
        dt: float,
        obstacle_speed_scale: float = 1.0,
    ):
        if not path or target is None:
            self.reset()
            return 0.0, 0.0, 0.0

        rx, ry, yaw = robot_pose
        dx = float(target.x) - rx
        dy = float(target.y) - ry
        x_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ld2 = max(x_body * x_body + y_body * y_body, 1e-4)
        curvature = 2.0 * y_body / ld2
        goal = path[-1]
        goal_dist = math.hypot(float(goal.x) - rx, float(goal.y) - ry)

        vx = clamp(self.k_v * max(0.0, x_body), 0.0, self.max_vx)
        if self.cruise_speed is not None:
            vx = self.cruise_speed if x_body > 0 else 0.0
        if goal_dist < self.approach_dist:
            vx *= clamp(goal_dist / max(self.approach_dist, 1e-3), 0.15, 1.0)
        curvature_scale = 1.0 / (1.0 + self.curvature_slowdown * abs(curvature))
        vx *= clamp(curvature_scale, 0.20, 1.0)
        vx *= clamp(obstacle_speed_scale, 0.0, 1.0)

        heading_error = normalize_angle(float(target.yaw) - yaw)
        point_heading_error = math.atan2(y_body, max(0.05, x_body))
        if abs(point_heading_error) > self.stop_rotate_yaw:
            vx = 0.0
        if 0.0 < vx < self.min_vx and goal_dist > 0.25:
            vx = self.min_vx

        wz = clamp(vx * curvature + self.heading_gain * heading_error, -self.max_wz, self.max_wz)
        dt = max(float(dt), 1e-3)
        prev_vx, prev_wz = self.prev_cmd
        vx = self.limit_rate(vx, prev_vx, self.accel_limit_v, dt)
        wz = self.limit_rate(wz, prev_wz, self.accel_limit_wz, dt)
        if abs(vx) < self.deadband_v:
            vx = 0.0
        if abs(wz) < self.deadband_wz:
            wz = 0.0
        self.prev_cmd = (vx, wz)
        return vx, 0.0, wz
