"""与 ROS 解耦、具有计算预算上限的矩形车体扫掠碰撞检查。

位姿与障碍中心必须使用同一个固定坐标系，位置单位为米、航向单位为弧度。
可选的 ``occupied`` 查询原始占用栅格，其 ``resolution`` 属性提供米制分辨率；
未知区域及地图外区域的策略由该查询实现。使用已膨胀代价地图会重复计入车体。
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from fw_mid_common_utils.collision_geometry import CollisionGeometry


Pose2D = Tuple[float, float, float]
Point2D = Tuple[float, float]


@dataclass(frozen=True)
class CollisionCheckResult:
    """包含安全判定、已检查轨迹，以及首次碰撞位置或拒绝原因。"""
    safe: bool
    trajectory: List[Pose2D]
    collision_point: Optional[Point2D]
    reason: str


class _ComputationLimit(Exception):
    pass


class FootprintCollisionChecker:
    """分别检查候选指令和当前实测运动的制动过程。

    线速度和角速度按同一比例制动以保持曲率，较慢停止的轴决定制动时长，
    两轴减速度均不超过配置上限。由此可用独轮车模型闭式积分覆盖该制动轨迹。
    """
    # 位姿和障碍点必须处于同一固定坐标系；检查结果会同时覆盖候选指令和
    # 当前实测运动的刹车分支，任何无效输入或计算超限均按不安全处理。

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
        """返回逆时针排列的四个车体角点，包含配置及本次增加的安全边距。"""
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
        # 用独轮车模型闭式积分生成弧线；半角形式在零角速度时仍可直接得到直线。
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
        # 速度线性同比例降至零，其积分等价于维持原速度行驶半个制动时长。
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
        # 相邻采样之间车体上任一点位移不超过 sample_distance，
        # 包括原地旋转的角点；后续碰撞膨胀会覆盖这段采样间隙。
        for index in range(1, count + 1):
            next_pose = self._advance(pose, vx, wz, effective_time * (index / count))
            if not all(math.isfinite(value) for value in next_pose):
                raise _ComputationLimit()
            yield next_pose

    def _point_collides(self, pose, point, padding):
        # 先做包围半径筛选，再把障碍变换到车体坐标，检查点到矩形的最短距离。
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
        # 世界坐标中半格间距的采样覆盖每个栅格，包括旋转地图。
        # 膨胀包含完整栅格对角线及运动采样间隙，覆盖查询点到占用格边界的偏移。
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
        """返回碰撞检查结果，无效输入或预算超限均拒绝运动。

        ``current_velocity`` 可传入实测 ``(vx, wz)``，单位为米每秒、弧度每秒。
        成功时轨迹属于候选指令；碰撞时返回失败分支截至首次碰撞的轨迹。
        提供静态地图回调时，必须同时提供 ``occupied.resolution``。
        """
        # occupied 是原始栅格查询，不接受已膨胀代价地图，避免重复扩大车体尺寸。
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
            # 候选分支覆盖预测及制动；实测分支额外覆盖响应延迟内的惯性运动。
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
            # 占用查询异常不能被解释为空闲区域。
            return CollisionCheckResult(False, trajectory, None, "occupancy_error")
