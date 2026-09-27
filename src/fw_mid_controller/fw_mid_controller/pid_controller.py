#!/usr/bin/env python3
"""PID path follower retained from the FW-mid ROS2 implementation."""

import math
from typing import Sequence

from fw_mid_common_utils import Pose2D, clamp, normalize_angle

from ._params import get_param


class PIDPathController:
    """PID tracker for the 4T4D drive mode (``vx`` + ``wz`` only).

    The returned angular velocity is in radians per second, matching
    ``geometry_msgs/Twist``.  Conversion to the CAN driver's degree unit is
    deliberately outside this package.
    """

    def __init__(self, params=None, cruise_speed=None) -> None:
        self.cruise_speed = None if cruise_speed is None else float(cruise_speed)
        if self.cruise_speed is not None and (
                not math.isfinite(self.cruise_speed) or self.cruise_speed < 0):
            raise ValueError("Cruise speed must be finite and nonnegative")
        self.kp_yaw = float(get_param(params, "pid_kp_yaw", 1.10))
        self.ki_yaw = float(get_param(params, "pid_ki_yaw", 0.00))
        self.kd_yaw = float(get_param(params, "pid_kd_yaw", 0.25))
        self.kp_lateral = float(get_param(params, "pid_kp_lateral", 0.80))
        self.k_v = float(get_param(params, "pid_k_v", 0.55))
        self.ki_v = float(get_param(params, "pid_ki_v", 0.00))
        self.kd_v = float(get_param(params, "pid_kd_v", 0.00))
        self.min_vx = float(get_param(params, "pid_min_vx", 0.04))
        self.max_vx = float(get_param(params, "pid_max_vx", 0.28))
        if self.cruise_speed is not None:
            self.max_vx = self.cruise_speed
        self.max_wz = float(get_param(params, "pid_max_wz", 0.22))
        self.yaw_deadband = float(get_param(params, "pid_yaw_deadband", 0.035))
        self.integral_limit = float(get_param(params, "pid_integral_limit", 0.4))
        self.slowdown_yaw = float(get_param(params, "pid_slowdown_yaw", 0.75))
        self.stop_rotate_yaw = float(get_param(params, "pid_stop_rotate_yaw", 1.35))
        self.approach_dist = float(get_param(params, "pid_approach_dist", 0.80))
        self.accel_limit_v = float(get_param(params, "pid_accel_limit_v", 0.35))
        self.accel_limit_wz = float(get_param(params, "pid_accel_limit_wz", 0.40))
        self.deadband_v = float(get_param(params, "pid_deadband_v", 0.015))
        self.deadband_wz = float(get_param(params, "pid_deadband_wz", 0.025))
        self.reset()

    def reset(self) -> None:
        self.x_integral = 0.0
        self.prev_x_error = 0.0
        self.yaw_integral = 0.0
        self.prev_yaw_error = 0.0
        self.prev_cmd = (0.0, 0.0)

    @staticmethod
    def limit_rate(target: float, previous: float, limit: float, dt: float) -> float:
        delta = max(0.0, limit) * max(float(dt), 1e-3)
        return clamp(target, previous - delta, previous + delta)

    @staticmethod
    def point_xy(point):
        if hasattr(point, "x") and hasattr(point, "y"):
            return float(point.x), float(point.y)
        return float(point[0]), float(point[1])

    @staticmethod
    def point_yaw(point) -> float:
        return float(getattr(point, "yaw", 0.0))

    def compute(self, robot_pose: Pose2D, global_path: Sequence, target, dt: float):
        if not global_path or target is None:
            self.reset()
            return 0.0, 0.0, 0.0

        rx, ry, yaw = robot_pose
        tx, ty = self.point_xy(target)
        target_yaw = self.point_yaw(target)
        goal_x, goal_y = self.point_xy(global_path[-1])

        dx = tx - rx
        dy = ty - ry
        x_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        x_error = max(0.0, x_body)
        lateral_error = math.atan2(y_body, max(0.05, x_body))
        heading_error = normalize_angle(target_yaw - yaw)
        yaw_error = normalize_angle(heading_error + self.kp_lateral * lateral_error)
        if abs(yaw_error) < self.yaw_deadband:
            yaw_error = 0.0

        dt = max(float(dt), 1e-3)
        self.x_integral = clamp(
            self.x_integral + x_error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        x_derivative = (x_error - self.prev_x_error) / dt
        self.yaw_integral = clamp(
            self.yaw_integral + yaw_error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        yaw_derivative = (yaw_error - self.prev_yaw_error) / dt

        wz = clamp(
            self.kp_yaw * yaw_error
            + self.ki_yaw * self.yaw_integral
            + self.kd_yaw * yaw_derivative,
            -self.max_wz,
            self.max_wz,
        )
        goal_dist = math.hypot(goal_x - rx, goal_y - ry)
        vx = clamp(
            self.k_v * x_error
            + self.ki_v * self.x_integral
            + self.kd_v * x_derivative,
            0.0,
            self.max_vx,
        )
        # A lookahead point guides steering; its spacing must not cap cruise speed.
        if self.cruise_speed is not None:
            vx = self.cruise_speed if x_body > 0 else 0.0
        if goal_dist < self.approach_dist:
            vx *= clamp(goal_dist / max(self.approach_dist, 1e-3), 0.15, 1.0)
        if abs(yaw_error) > self.stop_rotate_yaw:
            vx = 0.0
        else:
            yaw_scale = 1.0 - abs(yaw_error) / max(self.slowdown_yaw, 1e-3)
            vx *= clamp(yaw_scale, 0.20, 1.0)
        if 0.0 < vx < self.min_vx and goal_dist > 0.25:
            vx = self.min_vx

        prev_vx, prev_wz = self.prev_cmd
        vx = self.limit_rate(vx, prev_vx, self.accel_limit_v, dt)
        wz = self.limit_rate(wz, prev_wz, self.accel_limit_wz, dt)
        if abs(vx) < self.deadband_v:
            vx = 0.0
        if abs(wz) < self.deadband_wz:
            wz = 0.0

        self.prev_x_error = x_error
        self.prev_yaw_error = yaw_error
        self.prev_cmd = (vx, wz)
        return vx, 0.0, wz
