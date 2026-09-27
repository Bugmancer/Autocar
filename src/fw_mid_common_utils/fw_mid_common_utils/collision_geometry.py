"""Shared vehicle and discretization geometry for planning and motion checks."""

import math
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class CollisionGeometry:
    footprint_front: float = 0.34
    footprint_rear: float = 0.34
    footprint_half_width: float = 0.275
    footprint_margin: float = 0.02
    dynamic_memory_resolution: float = 0.10
    collision_sample_distance: float = 0.025
    planner_tracking_clearance: float = 0.0

    def __post_init__(self):
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
        return math.hypot(max(self.footprint_front, self.footprint_rear) + self.footprint_margin,
                          self.footprint_half_width + self.footprint_margin)

    @property
    def obstacle_radius(self):
        return math.sqrt(3.0) * self.dynamic_memory_resolution / 2.0

    @staticmethod
    def static_padding(resolution, sample_distance):
        return math.sqrt(2.0) * resolution + sample_distance

    def planning_radius(self, resolution, dynamic=False):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Invalid map resolution')
        padding = self.obstacle_radius + self.collision_sample_distance
        if not dynamic:
            # Cloud voxels already covered by a static cell may be omitted from
            # the overlay, so static inflation must also cover their uncertainty.
            padding = max(padding, self.static_padding(resolution, self.collision_sample_distance))
        # A* searches cell centers while raw occupied points may lie at cell edges.
        return (self.body_radius + padding + math.sqrt(2.0) * resolution / 2.0
                + self.planner_tracking_clearance)
