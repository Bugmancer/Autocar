"""Short local detour generator retained from the ROS2 implementation."""

from typing import List, Optional

from fw_mid_common_utils import Point2D, Pose2D, body_to_map, map_to_body


def _get_param(source, name, default):
    getter = getattr(source, "get_param", None)
    if getter is None:
        return default
    try:
        return getter("~" + name, default)
    except TypeError:
        return getter(name, default)


class LocalAvoidancePlanner:
    """在机器人 body frame 生成短绕行点，再转换回全局地图坐标。"""
    def __init__(self, params=None) -> None:
        self.x_min = float(_get_param(params, "local_replan_trigger_x_min", 0.10))
        self.x_max = float(_get_param(params, "local_replan_trigger_x_max", 1.20))
        self.y_abs = float(_get_param(params, "local_replan_trigger_y_abs", 0.45))
        self.lateral_offset = float(
            _get_param(params, "local_replan_lateral_offset", 0.75)
        )
        self.forward_margin = float(
            _get_param(params, "local_replan_forward_margin", 0.70)
        )
        self.rejoin_distance = float(
            _get_param(params, "local_replan_rejoin_distance", 1.20)
        )
        self.min_points = int(_get_param(params, "local_replan_min_obstacle_points", 8))
        self.prefer_side = str(
            _get_param(params, "local_replan_prefer_side", "auto")
        ).lower()

    def filter_front_obstacles(self, local_points: List[Point2D]) -> List[Point2D]:
        # x 向前、y 向左；只在配置的前方窗口内考虑局部绕行触发。
        return [
            (x, y)
            for x, y in local_points
            if self.x_min <= x <= self.x_max and abs(y) <= self.y_abs
        ]

    def has_front_obstacle(self, local_points: List[Point2D]) -> bool:
        return len(self.filter_front_obstacles(local_points)) >= self.min_points

    def choose_side(self, front_points: List[Point2D], target_body_y: float = 0.0) -> int:
        # 返回 +1 左绕、-1 右绕；优先选择点更少的一侧以保留净空。
        if self.prefer_side == "left":
            return 1
        if self.prefer_side == "right":
            return -1
        positive = sum(1 for _, y in front_points if y > 0.0)
        negative = sum(1 for _, y in front_points if y < 0.0)
        if positive != negative:
            return 1 if positive < negative else -1
        if target_body_y < -0.05:
            return -1
        return 1

    def make_detour(
        self,
        robot_pose: Pose2D,
        local_points: List[Point2D],
        current_target: Optional[Point2D] = None,
    ) -> List[Point2D]:
        # 生成 body frame 的“绕开-越过-回到中心线”三点，再统一转到 map frame。
        front_points = self.filter_front_obstacles(local_points)
        if len(front_points) < self.min_points:
            return []
        target_body_y = 0.0
        if current_target is not None:
            _, target_body_y = map_to_body(robot_pose, current_target)
        side = self.choose_side(front_points, target_body_y)
        minimum_x = min(x for x, _ in front_points)
        maximum_x = max(x for x, _ in front_points)
        body_waypoints = [
            (max(0.20, minimum_x - 0.15), side * self.lateral_offset),
            (maximum_x + self.forward_margin, side * self.lateral_offset),
            (maximum_x + self.rejoin_distance, 0.0),
        ]
        return [body_to_map(robot_pose, point) for point in body_waypoints]
