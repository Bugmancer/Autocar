"""规划膨胀与运动碰撞检查共用的车体尺寸和离散化误差预算。"""

import math
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class CollisionGeometry:
    """统一规划膨胀与运动碰撞检查的车体尺寸，所有长度均以米计。"""

    footprint_front: float = 0.34
    footprint_rear: float = 0.34
    footprint_half_width: float = 0.275
    footprint_margin: float = 0.02
    dynamic_memory_resolution: float = 0.10
    collision_sample_distance: float = 0.025
    planner_tracking_clearance: float = 0.0

    def __post_init__(self):
        # 车体尺寸和采样分辨率必须严格为正；可选安全余量允许为零。
        for field in fields(self):
            value = getattr(self, field.name)
            minimum_ok = value >= 0 if field.name in (
                'footprint_margin', 'planner_tracking_clearance') else value > 0
            if not math.isfinite(value) or not minimum_ok:
                raise ValueError('Invalid collision geometry: ' + field.name)

    @classmethod
    def from_params(cls, get_param):
        return cls(**{field.name: float(get_param(field.name, field.default))
                      for field in fields(cls)})

    @property
    def body_radius(self):
        # 用含安全余量的矩形外接圆覆盖任意车头朝向，供不搜索朝向的 A* 使用。
        return math.hypot(max(self.footprint_front, self.footprint_rear) + self.footprint_margin,
                          self.footprint_half_width + self.footprint_margin)

    @property
    def obstacle_radius(self):
        # 三维体素中心到角点的最大距离，补偿障碍点压缩为体素代表点的误差。
        return math.sqrt(3.0) * self.dynamic_memory_resolution / 2.0

    @staticmethod
    def static_padding(resolution, sample_distance):
        """为静态栅格离散化和沿轨迹采样之间的空隙预留距离。"""
        return math.sqrt(2.0) * resolution + sample_distance

    def planning_radius(self, resolution, dynamic=False):
        """返回 A* 的障碍膨胀半径；执行速度前仍需检查真实矩形车体。"""
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Invalid map resolution')
        padding = self.obstacle_radius + self.collision_sample_distance
        if not dynamic:
            # 已落入静态障碍格的点云体素可能不再叠加，静态膨胀也必须覆盖体素误差。
            padding = max(padding, self.static_padding(resolution, self.collision_sample_distance))
        # 为点到栅格代表位置的偏移预留半个栅格对角线；额外跟踪间隙另行累加。
        return (self.body_radius + padding + math.sqrt(2.0) * resolution / 2.0
                + self.planner_tracking_clearance)
