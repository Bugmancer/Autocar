#!/usr/bin/env python3
"""ROS1 导航节点共用的平面几何工具。

本模块不导入 rospy 或 ROS 消息类型，因此可在没有 ROS 主节点的开发机测试。

平面位姿约定为 (x, y, yaw)：位置单位为米，角度为弧度；车体坐标
x 向前、y 向左，正 yaw 绕 z 轴逆时针。调用者负责提供同一时刻的位姿。
"""

import math
from typing import Tuple

Point2D = Tuple[float, float]
Pose2D = Tuple[float, float, float]


def clamp(value: float, lo: float, hi: float) -> float:
    """将数值限制在闭区间 ``[lo, hi]`` 内，反向边界会先交换。"""
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, value))


def normalize_angle(angle: float) -> float:
    """将有限角度归一化到 ``[-pi, pi)``。"""
    # 使用取余而非反复加减周期，避免极大角度让归一化循环长时间运行。
    wrapped = math.fmod(float(angle) + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def yaw_from_quaternion(q) -> float:
    """从含 ``x/y/z/w`` 属性的四元数对象提取偏航角，调用方负责归一化。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四个标量分量提取偏航角，输入应为已归一化的四元数。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def body_to_map(robot_pose: Pose2D, point_body: Point2D) -> Point2D:
    """将车体坐标系中的平面点转换到地图坐标系。"""
    # p_map = R(yaw) * p_body + t；robot_pose 是车体原点在 map 中的位姿。
    rx, ry, yaw = robot_pose
    bx, by = point_body
    c = math.cos(yaw)
    s = math.sin(yaw)
    return rx + c * bx - s * by, ry + s * bx + c * by


def map_to_body(robot_pose: Pose2D, point_map: Point2D) -> Point2D:
    """将地图坐标系中的平面点转换到车体坐标系。"""
    # 逆变换先减去平移，再乘 R 的转置，得到相对于车头方向的前后/左右距离。
    rx, ry, yaw = robot_pose
    mx, my = point_map
    dx = mx - rx
    dy = my - ry
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
