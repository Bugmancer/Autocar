"""Bounded, ROS-independent swept rectangular vehicle collision checks.

All poses and obstacle centers use the same fixed frame.  ``occupied`` is an
optional callable over a RAW occupancy grid; its ``resolution`` attribute must
give the grid cell size in meters.  Unknown/out-of-map policy belongs to that
callable.  A pre-inflated costmap would double-count the vehicle footprint.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from fw_mid_common_utils.collision_geometry import CollisionGeometry


Pose2D = Tuple[float, float, float]
Point2D = Tuple[float, float]


@dataclass(frozen=True)
class CollisionCheckResult:
    safe: bool
    trajectory: List[Pose2D]
    collision_point: Optional[Point2D]
    reason: str


class _ComputationLimit(Exception):
    pass


class FootprintCollisionChecker:
    """Check candidate motion and, separately, stopping the current motion.

    Linear/angular braking share a scale factor so curvature remains constant.
    The slower stopping axis sets the duration; neither deceleration exceeds its
    configured limit.  This permits exact unicycle integration while checking a
    conservative, executable stopping maneuver.
    """
    # 位姿和障碍点必须处于同一固定坐标系；检查结果会同时覆盖候选指令和
    # 当前实测运动的刹车分支，任何输入/计算超限均 fail-closed。

    MAX_POSES = 4096
    MAX_OBSTACLES = 100000
    MAX_POINT_TESTS = 2000000
    MAX_RASTER_TESTS = 2000000
    MAX_OCCUPANCY_QUERIES = 200000

    def __init__(
        self,
        front=CollisionGeometry.footprint_front,
        rear=CollisionGeometry.footprint_rear,
        half_width=CollisionGeometry.footprint_half_width,
        margin=CollisionGeometry.footprint_margin,
        prediction_time=1.0,
        reaction_time=0.2,
        linear_deceleration=0.25,
        angular_deceleration=0.5,
        sample_distance=CollisionGeometry.collision_sample_distance,
        obstacle_radius=math.sqrt(3.0) * CollisionGeometry.dynamic_memory_resolution / 2.0,
    ):
        values = locals().copy()
        values.pop("self")
        for name, value in values.items():
            number = float(value)
            nonnegative = name in ("margin", "reaction_time", "obstacle_radius")
            if not math.isfinite(number) or (
                number < 0.0 if nonnegative else number <= 0.0
            ):
                raise ValueError("%s must be finite and %s" % (
                    name, "nonnegative" if nonnegative else "positive"
                ))
            setattr(self, name, number)
        self._radius = math.hypot(
            max(self.front, self.rear) + self.margin,
            self.half_width + self.margin,
        )
        if not math.isfinite(self._radius):
            raise ValueError("footprint dimensions overflow")

    @staticmethod
    def _numbers(value, size):
        result = tuple(float(component) for component in value)
        if len(result) != size or not all(math.isfinite(v) for v in result):
            raise ValueError("invalid finite vector")
        return result

    def footprint(self, pose, extra_margin=0.0):
        """Four counterclockwise corners, including configured safety margin."""
        x, y, yaw = self._numbers(pose, 3)
        extra_margin = float(extra_margin)
        if not math.isfinite(extra_margin) or extra_margin < 0.0:
            raise ValueError("extra_margin must be finite and nonnegative")
        margin = self.margin + extra_margin
        cosine, sine = math.cos(yaw), math.sin(yaw)
        corners = []
        for bx, by in (
            (self.front + margin, self.half_width + margin),
            (-self.rear - margin, self.half_width + margin),
            (-self.rear - margin, -self.half_width - margin),
            (self.front + margin, -self.half_width - margin),
        ):
            point = (x + cosine * bx - sine * by, y + sine * bx + cosine * by)
            if not all(math.isfinite(value) for value in point):
                raise ValueError("footprint coordinates overflow")
            corners.append(point)
        return corners

    @staticmethod
    def _advance(pose, velocity_x, velocity_yaw, effective_time):
        # 用 unicycle 闭式积分生成弧线，避免小角度时数值除零。
        x, y, yaw = pose
        angle = velocity_yaw * effective_time
        half_angle = angle * 0.5
        sinc = math.sin(half_angle) / half_angle if half_angle else 1.0
        distance = velocity_x * effective_time * sinc
        return (
            x + distance * math.cos(yaw + half_angle),
            y + distance * math.sin(yaw + half_angle),
            math.atan2(math.sin(yaw + angle), math.cos(yaw + angle)),
        )

    def _rollout(self, pose, velocity, hold_time):
        vx, wz = velocity
        braking_time = max(
            abs(vx) / self.linear_deceleration,
            abs(wz) / self.angular_deceleration,
        )
        # With proportional braking, integrating the scale gives T_brake / 2.
        effective_time = hold_time + braking_time * 0.5
        corner_speed_bound = abs(vx) + self._radius * abs(wz)
        swept_distance = effective_time * corner_speed_bound
        samples = swept_distance / self.sample_distance
        if not math.isfinite(samples) or samples > self.MAX_POSES - 1:
            raise _ComputationLimit()
        count = max(1, int(math.ceil(samples)))
        yield pose
        if corner_speed_bound == 0.0:
            return
        # Every material point of the footprint moves <= sample_distance
        # between samples, including the corners during pure rotation.
        for index in range(1, count + 1):
            next_pose = self._advance(pose, vx, wz, effective_time * (index / count))
            if not all(math.isfinite(value) for value in next_pose):
                raise _ComputationLimit()
            yield next_pose

    def _point_collides(self, pose, point, padding):
        dx, dy = point[0] - pose[0], point[1] - pose[1]
        if not math.isfinite(dx) or not math.isfinite(dy):
            raise ValueError("relative obstacle coordinates overflow")
        radius = self._radius + padding
        if abs(dx) > radius or abs(dy) > radius:
            return False
        cosine, sine = math.cos(pose[2]), math.sin(pose[2])
        bx = cosine * dx + sine * dy
        by = -sine * dx + cosine * dy
        gap_x = max(-self.rear - self.margin - bx,
                    bx - self.front - self.margin, 0.0)
        gap_y = max(abs(by) - self.half_width - self.margin, 0.0)
        return math.hypot(gap_x, gap_y) <= padding

    def _static_collision(self, pose, occupied, resolution, cache, budget):
        # Half-cell world lattice hits every cell, including rotated grids.
        # A full cell diagonal covers the offset from a lattice sample inside
        # an occupied cell to its boundary; sampling padding covers motion.
        step = resolution * 0.5
        padding = CollisionGeometry.static_padding(resolution, self.sample_distance)
        corners = self.footprint(pose, padding)
        bounds = (
            min(p[0] for p in corners) / step,
            max(p[0] for p in corners) / step,
            min(p[1] for p in corners) / step,
            max(p[1] for p in corners) / step,
        )
        if not all(math.isfinite(value) for value in bounds):
            raise _ComputationLimit()
        ix0, ix1 = math.floor(bounds[0]), math.ceil(bounds[1])
        iy0, iy1 = math.floor(bounds[2]), math.ceil(bounds[3])
        budget[1] += (ix1 - ix0 + 1) * (iy1 - iy0 + 1)
        if budget[1] > self.MAX_RASTER_TESTS:
            raise _ComputationLimit()
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                key = (ix, iy)
                if cache.get(key) is False:
                    continue
                point = (ix * step, iy * step)
                if not self._point_collides(pose, point, padding):
                    continue
                if key not in cache:
                    if len(cache) >= self.MAX_OCCUPANCY_QUERIES:
                        raise _ComputationLimit()
                    cache[key] = bool(occupied(*point))
                if cache[key]:
                    return point
        return None

    def check(self, pose, velocity_x, velocity_yaw, obstacles,
              current_velocity=None, occupied=None):
        """Return CollisionCheckResult; invalid or excessive work fails closed.

        ``current_velocity`` is an optional measured ``(vx, wz)``.  On success
        trajectory describes the candidate; on collision it is the branch that
        failed, up to its first collision.  ``occupied.resolution`` is required
        when a static map callback is supplied.
        """
        # occupied 是原始栅格查询，不接受已膨胀 costmap，避免重复扩大车体尺寸。
        trajectory = []
        try:
            pose = self._numbers(pose, 3)
            velocity = self._numbers((velocity_x, velocity_yaw), 2)
            actual = None if current_velocity is None else self._numbers(
                current_velocity, 2
            )
            points = []
            for point in obstacles:
                if len(points) >= self.MAX_OBSTACLES:
                    raise _ComputationLimit()
                points.append(self._numbers(point, 2))
            resolution = None
            if occupied is not None:
                resolution = float(occupied.resolution)
                if not callable(occupied) or not math.isfinite(resolution) or resolution <= 0:
                    raise ValueError("occupied.resolution must be finite and positive")
            branches = [("", velocity, max(self.prediction_time, self.reaction_time))]
            if actual is not None and actual != velocity:
                branches.append(("current_motion_", actual, self.reaction_time))
            candidate_trajectory = []
            cache = {}
            budget = [0, 0]
            for prefix, branch_velocity, hold_time in branches:
                trajectory = []
                for sample_pose in self._rollout(pose, branch_velocity, hold_time):
                    trajectory.append(sample_pose)
                    budget[0] += len(points)
                    if budget[0] > self.MAX_POINT_TESTS:
                        raise _ComputationLimit()
                    for point in points:
                        if self._point_collides(
                            sample_pose, point, self.obstacle_radius + self.sample_distance
                        ):
                            return CollisionCheckResult(
                                False, trajectory, point, prefix + "dynamic_obstacle"
                            )
                    if occupied is not None:
                        point = self._static_collision(
                            sample_pose, occupied, resolution, cache, budget
                        )
                        if point is not None:
                            return CollisionCheckResult(
                                False, trajectory, point, prefix + "static_obstacle"
                            )
                if not prefix:
                    candidate_trajectory = trajectory
            return CollisionCheckResult(True, candidate_trajectory, None, "clear")
        except _ComputationLimit:
            return CollisionCheckResult(False, trajectory, None, "computation_limit")
        except (TypeError, ValueError, OverflowError, AttributeError):
            return CollisionCheckResult(False, trajectory, None, "invalid_input")
        except Exception:
            # A broken occupancy provider cannot be interpreted as free space.
            return CollisionCheckResult(False, trajectory, None, "occupancy_error")
