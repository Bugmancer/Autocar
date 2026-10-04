"""与 ROS 解耦的动态障碍过滤辅助函数，点坐标与距离均使用米。"""

import math
from itertools import product
from typing import Iterable, List, Sequence, Tuple


Point2D = Tuple[float, float]
TimedPoint2D = Tuple[float, float, float]


def supported_obstacle_points(points, radius, min_points):
    """保留同帧三维邻域中具有足够不同回波点支持的点，计数包含点自身。"""
    # 通过同帧邻域支持数抑制孤立噪点；只有保留下来的命中才会进入记忆。
    if not math.isfinite(radius) or radius <= 0 or min_points < 2:
        raise ValueError("Obstacle support requires positive radius and at least two points")
    unique = list(dict.fromkeys(tuple(float(v) for v in p) for p in points))
    buckets = {}
    for point in unique:
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError("Invalid obstacle support point")
        key = tuple(math.floor(v / radius) for v in point)
        buckets.setdefault(key, []).append(point)
    offsets = list(product((-1, 0, 1), repeat=3))
    kept = []
    radius_squared = radius * radius
    for point in unique:
        key = tuple(math.floor(v / radius) for v in point)
        count = 0
        for offset in offsets:
            neighbours = buckets.get(tuple(k + d for k, d in zip(key, offset)), ())
            for other in neighbours:
                if sum((a - b)**2 for a, b in zip(point, other)) <= radius_squared:
                    count += 1
                    if count >= min_points:
                        break
            if count >= min_points:
                kept.append(point)
                break
    return kept


def proximity_speed_scale(points, front, rear, half_width, margin,
                          obstacle_radius, slowdown_distance, minimum_scale):
    """按障碍到带边距车体矩形的净距，同步缩放线速度和角速度。

    此函数仅提供舒适性减速；缩放后的指令与当前实测运动是否安全，仍由后续
    车体扫掠碰撞及制动检查决定，净距为零也不能凭此函数直接放行。
    """
    # 无障碍时保持原速度；任何非有限坐标都返回零缩放系数。
    clearance = float("inf")
    for x, y in points:
        if not math.isfinite(x) or not math.isfinite(y):
            return 0.0
        gap_x = max(-rear - margin - x, x - front - margin, 0.0)
        gap_y = max(abs(y) - half_width - margin, 0.0)
        clearance = min(clearance, max(0.0, math.hypot(gap_x, gap_y) - obstacle_radius))
    fraction = min(1.0, clearance / slowdown_distance)
    return minimum_scale + (1.0 - minimum_scale) * fraction


def largest_cluster_indices(points: Sequence[Point2D], max_distance: float) -> List[int]:
    """返回前方点云中最大连通簇，减少多个噪声簇触发重规划。"""
    if not points:
        return []

    distance_squared = max(0.05, float(max_distance)) ** 2
    visited = [False] * len(points)
    largest = []
    for start_index in range(len(points)):
        if visited[start_index]:
            continue
        stack = [start_index]
        visited[start_index] = True
        cluster = []
        while stack:
            index = stack.pop()
            cluster.append(index)
            px, py = points[index]
            for neighbour, (qx, qy) in enumerate(points):
                if visited[neighbour]:
                    continue
                if (px - qx) ** 2 + (py - qy) ** 2 <= distance_squared:
                    visited[neighbour] = True
                    stack.append(neighbour)
        if len(cluster) > len(largest):
            largest = cluster
    return largest


def select_front_obstacles(
    local_points: Sequence[Point2D],
    map_points: Sequence[Point2D],
    x_min: float,
    x_max: float,
    y_abs: float,
) -> Tuple[List[Point2D], List[Point2D]]:
    """按机器人坐标的前方窗口筛选，并保持局部点与地图点一一对应。"""
    local_result = []
    map_result = []
    for local_point, map_point in zip(local_points, map_points):
        x, y = local_point
        if x_min < x <= x_max and abs(y) <= y_abs:
            local_result.append(local_point)
            map_result.append(map_point)
    return local_result, map_result


def deduplicate_timed_points(
    history: Iterable[TimedPoint2D], resolution: float, limit: int
) -> List[Point2D]:
    """对按时间排序的 ``(stamp, x, y)`` 历史按网格去重，优先保留较新点。"""
    resolution = max(0.02, float(resolution))
    limit = max(1, int(limit))
    seen = set()
    newest_first = []
    for _, x, y in reversed(list(history)):
        key = (int(round(x / resolution)), int(round(y / resolution)))
        if key in seen:
            continue
        seen.add(key)
        newest_first.append((float(x), float(y)))
        if len(newest_first) >= limit:
            break
    newest_first.reverse()
    return newest_first


def trajectory_cost(
    trajectory: Sequence[Tuple[float, float, float]],
    obstacles: Sequence[Point2D],
    collision_radius: float,
    influence_distance: float,
) -> float:
    """按轨迹到最近障碍的距离给代价，进入碰撞半径时返回无穷大。"""
    if not trajectory or not obstacles:
        return 0.0
    minimum = min(
        math.hypot(x - ox, y - oy)
        for x, y, _ in trajectory
        for ox, oy in obstacles
    )
    if minimum <= collision_radius:
        return float("inf")
    if minimum >= influence_distance:
        return 0.0
    return (influence_distance - minimum) / max(
        1e-3, influence_distance - collision_radius
    )
