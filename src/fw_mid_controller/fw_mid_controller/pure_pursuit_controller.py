#!/usr/bin/env python3
"""供 FW-mid ROS1 局部规划器使用的纯追踪控制器。"""

import math
from typing import Sequence

from fw_mid_common_utils import Pose2D, clamp, normalize_angle

from ._params import get_param


class PurePursuitController:
    """以车体坐标中的前视点计算曲率，只输出前进和转向指令。"""

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
        # 换路径或结束跟踪后清空限速历史，下次从静止状态重新生成候选指令。
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
        """输入位姿与路径须在同一坐标系；输出 (vx, 0, wz) 使用 m/s 和 rad/s。"""
        if not path or target is None:
            self.reset()
            return 0.0, 0.0, 0.0

        rx, ry, yaw = robot_pose
        dx = float(target.x) - rx
        dy = float(target.y) - ry
        # 前视点先转到车体平面；圆弧曲率 k = 2*y/L^2，左侧目标对应正曲率。
        x_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ld2 = max(x_body * x_body + y_body * y_body, 1e-4)
        curvature = 2.0 * y_body / ld2
        goal = path[-1]
        goal_dist = math.hypot(float(goal.x) - rx, float(goal.y) - ry)

        vx = clamp(self.k_v * max(0.0, x_body), 0.0, self.max_vx)
        # 显式巡航速度覆盖按纵向误差计算的速度，再应用终点和曲率减速。
        if self.cruise_speed is not None:
            vx = self.cruise_speed if x_body > 0 else 0.0
        if goal_dist < self.approach_dist:
            vx *= clamp(goal_dist / max(self.approach_dist, 1e-3), 0.15, 1.0)
        curvature_scale = 1.0 / (1.0 + self.curvature_slowdown * abs(curvature))
        # 弯道与障碍接近程度共同调节候选速度，最终停车决策由上层安全检查负责。
        vx *= clamp(curvature_scale, 0.20, 1.0)
        vx *= clamp(obstacle_speed_scale, 0.0, 1.0)

        heading_error = normalize_angle(float(target.yaw) - yaw)
        point_heading_error = math.atan2(y_body, max(0.05, x_body))
        if abs(point_heading_error) > self.stop_rotate_yaw:
            vx = 0.0
        if 0.0 < vx < self.min_vx and goal_dist > 0.25:
            vx = self.min_vx

        wz = clamp(vx * curvature + self.heading_gain * heading_error, -self.max_wz, self.max_wz)
        # v*k 给出圆弧角速度，额外航向项对齐路径切线，再限制每周期指令变化。
        dt = max(float(dt), 1e-3)
        prev_vx, prev_wz = self.prev_cmd
        vx = self.limit_rate(vx, prev_vx, self.accel_limit_v, dt)
        wz = self.limit_rate(wz, prev_wz, self.accel_limit_wz, dt)
        if abs(vx) < self.deadband_v:
            vx = 0.0
        if abs(wz) < self.deadband_wz:
            wz = 0.0
        # 保留实际返回的输出，作为下一周期变化率限制的基准。
        self.prev_cmd = (vx, wz)
        return vx, 0.0, wz
