#!/usr/bin/env python3
"""Path post-processing shared by the local planner and offline tests."""

import math
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Sequence, Tuple


Point2D = Tuple[float, float]
CollisionChecker = Callable[[float, float], bool]


@dataclass
class PathPoint:
    x: float
    y: float
    yaw: float = 0.0


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length(points: Sequence[Point2D]) -> float:
    return sum(distance(points[index - 1], points[index]) for index in range(1, len(points)))


def poses_to_xy(path_msg) -> List[Point2D]:
    return [
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in path_msg.poses
    ]


def segment_collision_free(
    start: Point2D,
    end: Point2D,
    is_occupied: Optional[CollisionChecker],
    step: float,
) -> bool:
    """沿线离散采样；任何采样点占用都否决整段捷径。"""
    if is_occupied is None:
        return True

    length = distance(start, end)
    samples = max(1, int(math.ceil(length / max(float(step), 1e-3))))
    for index in range(samples + 1):
        ratio = float(index) / samples
        x = start[0] + (end[0] - start[0]) * ratio
        y = start[1] + (end[1] - start[1]) * ratio
        if is_occupied(x, y):
            return False
    return True


def path_collision_free(
    points: Sequence[Point2D],
    is_occupied: Optional[CollisionChecker],
    step: float,
) -> bool:
    if is_occupied is None:
        return True
    return all(
        segment_collision_free(points[index - 1], points[index], is_occupied, step)
        for index in range(1, len(points))
    )


def shortcut_path(
    points: Sequence[Point2D],
    is_occupied: Optional[CollisionChecker],
    check_step: float,
    max_skip: int = 80,
) -> List[Point2D]:
    # 从当前点尝试最远可见点，且每次都用占用查询验证整段线段。
    if len(points) <= 2:
        return list(points)

    result = [points[0]]
    current = 0
    while current < len(points) - 1:
        farthest = len(points) - 1
        if max_skip > 0:
            farthest = min(farthest, current + max_skip)
        candidate = farthest
        while candidate > current + 1:
            if segment_collision_free(
                points[current], points[candidate], is_occupied, check_step
            ):
                break
            candidate -= 1
        result.append(points[candidate])
        current = candidate
    return result


def resample_path(points: Sequence[Point2D], spacing: float) -> List[Point2D]:
    """按弧长重采样，使控制器收到稳定间距的路径点。"""
    if len(points) <= 1:
        return list(points)

    spacing = max(float(spacing), 1e-3)
    result = [points[0]]
    remaining = spacing
    segment_start = points[0]

    for segment_end in points[1:]:
        x0, y0 = segment_start
        x1, y1 = segment_end
        segment_length = distance(segment_start, segment_end)
        if segment_length < 1e-9:
            segment_start = segment_end
            continue

        travelled = 0.0
        while travelled + remaining <= segment_length + 1e-12:
            travelled += remaining
            ratio = travelled / segment_length
            point = (x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio)
            if distance(result[-1], point) > 1e-9:
                result.append(point)
            remaining = spacing
        remaining -= segment_length - travelled
        if remaining <= 1e-9:
            remaining = spacing
        segment_start = segment_end

    if distance(result[-1], points[-1]) > 1e-9:
        result.append(points[-1])
    return result


def smooth_path_gradient(
    points: Sequence[Point2D],
    is_occupied: Optional[CollisionChecker],
    check_step: float,
    weight_data: float,
    weight_smooth: float,
    max_iter: int,
    tolerance: float,
) -> List[Point2D]:
    # 平滑点同时受原始路径和相邻点约束；任一新线段碰撞则保留旧点。
    if len(points) <= 2:
        return list(points)

    original = [(float(x), float(y)) for x, y in points]
    smoothed = [list(point) for point in original]
    for _ in range(max(0, int(max_iter))):
        change = 0.0
        for index in range(1, len(smoothed) - 1):
            old_x, old_y = smoothed[index]
            new_x = old_x + weight_data * (original[index][0] - old_x)
            new_y = old_y + weight_data * (original[index][1] - old_y)
            new_x += weight_smooth * (
                smoothed[index - 1][0] + smoothed[index + 1][0] - 2.0 * new_x
            )
            new_y += weight_smooth * (
                smoothed[index - 1][1] + smoothed[index + 1][1] - 2.0 * new_y
            )

            previous = tuple(smoothed[index - 1])
            following = tuple(smoothed[index + 1])
            candidate = (new_x, new_y)
            if not segment_collision_free(previous, candidate, is_occupied, check_step):
                continue
            if not segment_collision_free(candidate, following, is_occupied, check_step):
                continue
            smoothed[index] = [new_x, new_y]
            change += abs(new_x - old_x) + abs(new_y - old_y)
        if change < tolerance:
            break

    result = [(point[0], point[1]) for point in smoothed]
    if not path_collision_free(result, is_occupied, check_step):
        return list(points)
    return result


def compute_center_diff_yaw(points: Sequence[Point2D]) -> List[PathPoint]:
    """用中心差分计算每个路径点的切向 yaw，首尾使用单边差分。"""
    result = []
    for index, (x, y) in enumerate(points):
        if len(points) == 1:
            yaw = 0.0
        elif index == 0:
            yaw = math.atan2(points[1][1] - y, points[1][0] - x)
        elif index == len(points) - 1:
            yaw = math.atan2(y - points[-2][1], x - points[-2][0])
        else:
            yaw = math.atan2(
                points[index + 1][1] - points[index - 1][1],
                points[index + 1][0] - points[index - 1][0],
            )
        result.append(PathPoint(float(x), float(y), yaw))
    return result


def process_path(
    raw_points: Sequence[Point2D],
    *,
    enable_shortcut: bool,
    enable_smoothing: bool,
    resample_ds: float,
    smooth_weight_data: float,
    smooth_weight_smooth: float,
    smooth_max_iter: int,
    smooth_tolerance: float,
    collision_check_step: float,
    is_occupied: Optional[CollisionChecker],
) -> List[PathPoint]:
    """按捷径、重采样、碰撞约束平滑的顺序处理全局路径。"""
    points = [(float(x), float(y)) for x, y in raw_points]
    if len(points) <= 1:
        return compute_center_diff_yaw(points)
    if enable_shortcut and is_occupied is not None:
        points = shortcut_path(points, is_occupied, collision_check_step)
    points = resample_path(points, resample_ds)
    if enable_smoothing and is_occupied is not None:
        points = smooth_path_gradient(
            points,
            is_occupied,
            collision_check_step,
            smooth_weight_data,
            smooth_weight_smooth,
            smooth_max_iter,
            smooth_tolerance,
        )
        points = resample_path(points, resample_ds)
    return compute_center_diff_yaw(points)


def path_to_ros_msg(points: Iterable[PathPoint], frame_id: str, stamp):
    # Path 中的位置和 yaw 都沿用全局 frame；四元数只编码平面航向。
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path

    message = Path()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    for point in points:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x = float(point.x)
        pose.pose.position.y = float(point.y)
        pose.pose.orientation.z = math.sin(0.5 * point.yaw)
        pose.pose.orientation.w = math.cos(0.5 * point.yaw)
        message.poses.append(pose)
    return message
