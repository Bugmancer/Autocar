#!/usr/bin/env python3
"""Small geometry helpers shared by the ROS1 navigation nodes.

The functions in this module deliberately do not import rospy or message
types.  They can therefore be exercised on a development machine without a
running ROS master.

平面位姿约定为 (x, y, yaw)：位置单位为米，角度为弧度；车体坐标
x 向前、y 向左，正 yaw 绕 z 轴逆时针。调用者负责提供同一时刻的位姿。
"""

import math
from typing import Tuple

Point2D = Tuple[float, float]
Pose2D = Tuple[float, float, float]


def clamp(value: float, lo: float, hi: float) -> float:
    """Return *value* constrained to the inclusive interval ``[lo, hi]``."""
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, value))


def normalize_angle(angle: float) -> float:
    """Normalize an angle to ``[-pi, pi]``."""
    # fmod avoids an unbounded loop when a bad sensor value is supplied.
    wrapped = math.fmod(float(angle) + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def yaw_from_quaternion(q) -> float:
    """Extract yaw from an object exposing ``x``, ``y``, ``z`` and ``w``."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from scalar quaternion components."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def body_to_map(robot_pose: Pose2D, point_body: Point2D) -> Point2D:
    """Transform a planar point from the robot frame into the map frame."""
    # p_map = R(yaw) * p_body + t；robot_pose 是车体原点在 map 中的位姿。
    rx, ry, yaw = robot_pose
    bx, by = point_body
    c = math.cos(yaw)
    s = math.sin(yaw)
    return rx + c * bx - s * by, ry + s * bx + c * by


def map_to_body(robot_pose: Pose2D, point_map: Point2D) -> Point2D:
    """Transform a planar point from map coordinates into the robot frame."""
    # 逆变换先减去平移，再乘 R 的转置，得到相对于车头方向的前后/左右距离。
    rx, ry, yaw = robot_pose
    mx, my = point_map
    dx = mx - rx
    dy = my - ry
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
