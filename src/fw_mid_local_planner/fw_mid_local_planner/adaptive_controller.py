"""V3 纯计算控制器：连续路径参考、车身净距势场和受限短轨迹候选。

所有位置使用地图坐标和米，角度使用弧度；本模块不发布 ROS 消息。
候选轨迹的安全判定复用运行层的 FootprintCollisionChecker。
"""

import math
from dataclasses import dataclass, fields

from fw_mid_common_utils import normalize_angle

from .arc_path import ArcPath
from .footprint_collision import FootprintCollisionChecker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class V3Parameters:
    """仅由 V3 读取的参数；长度为米，时间为秒，加速度为 SI 单位。"""

    lookahead_min: float = 0.45
    lookahead_max: float = 0.80
    lookahead_time: float = 1.5
    lookahead_filter_time: float = 0.30
    curvature_gain: float = 0.70
    curvature_preview: float = 1.20
    lateral_acceleration: float = 0.25
    accel_v: float = 0.25
    accel_w: float = 0.40
    heading_gain: float = 1.80
    yaw_response_time: float = 0.35
    rotate_threshold: float = 1.35
    rotate_exit_threshold: float = 0.45
    rotate_exit_wz: float = 0.15
    goal_approach_distance: float = 0.60
    obstacle_influence: float = 0.65
    repulsive_gain: float = 1.50
    tangent_gain: float = 0.80
    obstacle_heading_limit: float = 0.20
    obstacle_sectors: int = 36
    side_hold_time: float = 1.0
    force_filter_time: float = 0.15
    deadband_v: float = 0.005
    deadband_w: float = 0.010
    trajectory_max_checks: int = 9
    trajectory_time_budget: float = 0.04
    stall_time: float = 4.0
    stall_progress: float = 0.025

    @classmethod
    def from_getter(cls, getter):
        values = {}
        integer_fields = {"obstacle_sectors": (8, 180), "trajectory_max_checks": (1, 15)}
        nonnegative = {"repulsive_gain", "tangent_gain", "curvature_gain",
                       "lookahead_filter_time", "force_filter_time", "deadband_v", "deadband_w",
                       "yaw_response_time", "obstacle_heading_limit"}
        for field in fields(cls):
            value = float(getter("v3_" + field.name, field.default))
            if not math.isfinite(value) or (value < 0 if field.name in nonnegative else value <= 0):
                raise ValueError("Invalid v3_" + field.name)
            if field.name in integer_fields:
                lower, upper = integer_fields[field.name]
                if not value.is_integer() or not lower <= value <= upper:
                    raise ValueError("Invalid v3_" + field.name)
                value = int(value)
            values[field.name] = value
        if (values["lookahead_min"] > values["lookahead_max"]
                or values["rotate_threshold"] >= math.pi
                or values["rotate_exit_threshold"] >= values["rotate_threshold"]
                or values["obstacle_heading_limit"] >= math.pi / 2):
            raise ValueError("Invalid V3 lookahead or rotation bounds")
        return cls(**values)


class AdaptiveController:
    """通过组合接入公共运行层，不继承经典控制器或 APF 控制器。"""

    def __init__(self, follower, parameters):
        self.follower, self.p = follower, parameters
        self.reference = None
        self._path_token = None
        self.progress = self._progress_limit = 0.0
        self.last_pose = None
        self.lookahead = parameters.lookahead_min
        self.bypass_side = 0
        self._last_blocking_time = None
        self.stop()

    def stop(self):
        """停车清掉速度爬升；保留路径进度及绕行侧，避免临时停车后跳段或换边。"""
        self.prev_cmd = self.last_command = (0.0, 0.0)
        self.last_force = None
        self.lookahead = self.p.lookahead_min
        self._rotating = False
        self.heading_error = self.raw_heading_error = 0.0
        self.steering_error = self.obstacle_heading_correction = 0.0
        self.yaw_braking = False
        self.status = "idle"
        self.candidates = []
        self.nearest_clearance = float("inf")
        self.reference_curvature = 0.0

    @property
    def remaining_distance(self):
        return float("inf") if self.reference is None else max(0.0, self.reference.length - self.progress)

    def update_path_progress(self, pose, path):
        """只在有序路径附近投影；投影前进预算随实测位移累积，静止不能跳过回环。"""
        if not path or not all(math.isfinite(v) for v in pose):
            raise ValueError("V3 requires a finite pose and nonempty path")
        token = (id(path[0]), id(path[-1]), len(path))
        if token != self._path_token:
            points = list(path)
            first = points[0]
            start_distance = math.hypot(first.x - pose[0], first.y - pose[1])
            reference = ArcPath(points)
            # 异步重规划的起点可能已经落在身后；先在有限前缀投影，不能直接
            # 拼接“当前位置 -> 旧起点”让正在前行的车辆掉头返走。
            initial_progress, offset = reference.project(pose[0], pose[1], 0.0,
                min(reference.length, 0.20 + 1.5 * start_distance))
            if (reference.length == 0.0 or (initial_progress < 1e-9 and offset > 0.15)) and start_distance > 1e-6:
                points.insert(0, pose[:2])
                reference, initial_progress = ArcPath(points), 0.0
            self.reference = reference
            self._path_token = token
            # 持有端点对象，避免旧路径释放后 Python 重用 id 而误认为同一路径。
            self._path_anchors = (path[0], path[-1])
            self.progress, self._progress_limit = initial_progress, initial_progress + 0.20
            self.bypass_side, self._last_blocking_time = 0, None
            # 同目标重规划只重建进度，保留前视和方向滤波；停车/换目标由 stop 清零。
            self.last_pose = pose
        displacement = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
        self._progress_limit = min(self.reference.length, self._progress_limit + 1.5 * displacement)
        projected, _ = self.reference.project(pose[0], pose[1],
            max(0.0, self.progress - 0.10), self._progress_limit)
        self.progress = max(self.progress, projected)
        self.last_pose = pose

    def _reference_force(self, pose, dt):
        p, ref = self.p, self.reference
        speed = max(abs(self.follower._measured_velocity[0]), abs(self.last_command[0]))
        self.reference_curvature = ref.peak_curvature(
            self.progress, min(ref.length, self.progress + p.curvature_preview))
        desired = clamp((p.lookahead_min + p.lookahead_time * speed)
                        / (1.0 + p.curvature_gain * self.reference_curvature),
                        p.lookahead_min, p.lookahead_max)
        alpha = -math.expm1(-dt / p.lookahead_filter_time) if p.lookahead_filter_time else 1.0
        self.lookahead += alpha * (desired - self.lookahead)
        # 三个目标使用物理弧长；急弯降低远目标权重，避免跨过拐角形成过强吸引。
        force = [0.0, 0.0]
        seen = set()
        bend_scale = 1.0 / (1.0 + p.curvature_gain * self.reference_curvature)
        for factor, weight in ((1.0, 1.0), (1.7, 0.45 * bend_scale), (2.4, 0.20 * bend_scale)):
            distance = min(ref.length, self.progress + self.lookahead * factor)
            if distance in seen:
                continue
            seen.add(distance)
            point = ref.sample(distance)
            dx, dy = point.x - pose[0], point.y - pose[1]
            norm = math.hypot(dx, dy)
            if norm > 1e-9:
                force[0] += weight * dx / norm
                force[1] += weight * dy / norm
        return tuple(force)

    def obstacle_sectors(self, pose, points):
        """每个角扇区仅取车身净距最小的障碍，重复点不放大斥力。

        该压缩只用于引导和评分，碰撞检查仍接收未压缩的完整近场障碍。
        """
        g, p = self.follower.geometry, self.p
        c, s = math.cos(pose[2]), math.sin(pose[2])
        sectors = {}
        radius = g.body_radius + g.obstacle_radius + p.obstacle_influence
        for ox, oy in points:
            if not math.isfinite(ox) or not math.isfinite(oy):
                raise ValueError("Invalid V3 obstacle")
            dx, dy = ox - pose[0], oy - pose[1]
            if math.hypot(dx, dy) > radius:
                continue
            bx, by = c * dx + s * dy, -s * dx + c * dy
            nx = bx - clamp(bx, -g.footprint_rear - g.footprint_margin,
                            g.footprint_front + g.footprint_margin)
            ny = by - clamp(by, -g.footprint_half_width - g.footprint_margin,
                            g.footprint_half_width + g.footprint_margin)
            norm = math.hypot(nx, ny)
            clearance = max(0.0, norm - g.obstacle_radius)
            key = min(p.obstacle_sectors - 1,
                      int((math.atan2(by, bx) + math.pi) / (2 * math.pi) * p.obstacle_sectors))
            normal = ((-c * nx + s * ny) / norm, (-s * nx - c * ny) / norm) if norm > 1e-9 else (0.0, 0.0)
            candidate = (clearance, ox, oy, normal)
            if key not in sectors or candidate[:3] < sectors[key][:3]:
                sectors[key] = candidate
        return [sectors[key] for key in sorted(sectors)]

    def _obstacle_force(self, pose, attraction, sectors, now):
        p = self.p
        self.nearest_clearance = min((row[0] for row in sectors), default=float("inf"))
        result = [0.0, 0.0]
        direction = math.atan2(attraction[1], attraction[0])
        c, s = math.cos(direction), math.sin(direction)
        blocking = []
        for gap, ox, oy, normal in sectors:
            strength = p.repulsive_gain * max(0.0, 1.0 - gap / p.obstacle_influence)**2
            # 扇区积分权重固定；点云加密不会让同一方向的墙壁产生更多份斥力。
            weight = 2.0 * math.pi / p.obstacle_sectors
            result[0] += strength * normal[0] * weight
            result[1] += strength * normal[1] * weight
            dx, dy = ox - pose[0], oy - pose[1]
            forward, lateral = c * dx + s * dy, -s * dx + c * dy
            if (gap < p.obstacle_influence and forward > 0
                    and abs(lateral) < self.follower.geometry.footprint_half_width + 0.20):
                blocking.append((gap, ox, oy, normal))
        if blocking:
            gap, ox, oy, normal = min(blocking)
            self._last_blocking_time = now
            if not self.bypass_side:
                future = self.reference.sample(self.progress + 2.4 * self.lookahead)
                cross = (ox - pose[0]) * (future.y - pose[1]) - (oy - pose[1]) * (future.x - pose[0])
                # 全局路径优先决定左右；完全对称时固定选左，不用随机扰动反复换边。
                self.bypass_side = 1 if cross >= 0 else -1
            tangent = (self.bypass_side * normal[1], -self.bypass_side * normal[0])
            strength = p.tangent_gain * max(0.0, 1.0 - gap / p.obstacle_influence)
            result[0] += strength * tangent[0]
            result[1] += strength * tangent[1]
        elif self._last_blocking_time is not None and now - self._last_blocking_time > p.side_hold_time:
            self.bypass_side = 0
        # A* 已规划绕行；势场只做有限的侧向微调，不抵消路径的前向引导。
        norm = math.hypot(*attraction)
        lateral = clamp(-s * result[0] + c * result[1],
                        -norm * math.tan(p.obstacle_heading_limit),
                        norm * math.tan(p.obstacle_heading_limit))
        self.obstacle_heading_correction = math.atan2(lateral, norm) if norm else 0.0
        return -s * lateral, c * lateral

    def _yaw_limit(self, error):
        deceleration = min(self.p.accel_w, self.follower.collision_checker.angular_deceleration)
        reaction = deceleration * self.p.yaw_response_time
        # 解 w*t_response + w^2/(2*a) <= 剩余角度，接近对齐前即开始转向制动。
        return min(self.follower.command_max_wz,
                   math.sqrt(reaction * reaction + 2.0 * deceleration * abs(error)) - reaction)

    def _stopping_yaw(self, turn):
        deceleration = min(self.p.accel_w, self.follower.collision_checker.angular_deceleration)
        return abs(turn) * self.p.yaw_response_time + turn * turn / (2.0 * deceleration)

    def _speed_reference(self, error):
        p, f = self.p, self.follower
        if self._rotating:
            return 0.0
        steering_curvature = abs(2.0 * math.sin(error) / self.lookahead)
        curvature = max(self.reference_curvature, steering_curvature, 1e-9)
        # 弯道角速度、横向加速度和到点制动共同约束速度，没有强制最小爬行速度。
        goal = self.reference.sample(self.reference.length)
        goal_distance = math.hypot(goal.x - self.last_pose[0], goal.y - self.last_pose[1])
        # 投影先到达路径末端时，仍可能存在横向误差；不能因剩余弧长为零而
        # 永久停在目标容差外。回环路径则继续使用更长的剩余弧长限制提前到点。
        distance = max(0.0, max(self.remaining_distance, goal_distance) - f.goal_tolerance * 0.5)
        speed = min(f.command_max_vx, f.command_max_wz / curvature,
                    math.sqrt(p.lateral_acceleration / curvature),
                    math.sqrt(2.0 * f.collision_checker.linear_deceleration * distance))
        speed *= min(1.0, distance / p.goal_approach_distance) * max(0.0, math.cos(error))
        # if self.nearest_clearance < p.obstacle_influence:
        #     speed *= clamp(self.nearest_clearance / p.obstacle_influence, 0.0, 1.0)
        return speed

    def compute(self, pose, path, dt, obstacles, now):
        if (not path or not all(math.isfinite(v) for v in (*pose, dt, now,
                *self.follower._measured_velocity)) or dt <= 0):
            self.stop()
            self.status = "invalid_input"
            return 0.0, 0.0, 0.0
        self.update_path_progress(pose, path)
        attraction = self._reference_force(pose, dt)
        sectors = self.obstacle_sectors(pose, obstacles)
        repulsion = self._obstacle_force(pose, attraction, sectors, now)
        raw = (attraction[0] + repulsion[0], attraction[1] + repulsion[1])
        if math.hypot(*raw) < 1e-8:
            self.stop()
            self.status = "force_cancelled"
            return 0.0, 0.0, 0.0
        alpha = -math.expm1(-dt / self.p.force_filter_time) if self.p.force_filter_time else 1.0
        self.last_force = raw if self.last_force is None else tuple(
            old + alpha * (new - old) for old, new in zip(self.last_force, raw))
        heading = math.atan2(self.last_force[1], self.last_force[0])
        error = normalize_angle(heading - pose[2])
        raw_error = normalize_angle(math.atan2(raw[1], raw[0]) - pose[2])
        self.heading_error, self.raw_heading_error = error, raw_error
        measured_w = self.follower._measured_velocity[1]
        old_v, old_w = self.prev_cmd
        self.steering_error = normalize_angle(error - measured_w * self.p.yaw_response_time)
        turning_error = max(abs(error), abs(raw_error))
        if self._rotating:
            if (turning_error <= self.p.rotate_exit_threshold
                    and max(abs(old_w), abs(measured_w)) <= self.p.rotate_exit_wz):
                self._rotating = False
        elif turning_error >= self.p.rotate_threshold:
            self._rotating = True
        # 新出现的急转要求立即参与限速，不能被方向滤波延后。
        speed = min(self._speed_reference(error), self._speed_reference(raw_error))
        f, p = self.follower, self.p
        self.yaw_braking = any(abs(turn) > p.deadband_w and (
            turn * error <= 0.0 or self._stopping_yaw(turn) >= abs(error))
            for turn in (old_w, measured_w))
        if self.yaw_braking:
            speed = min(speed, old_v)
        vx = clamp(speed, max(0.0, old_v - f.collision_checker.linear_deceleration * dt), old_v + p.accel_v * dt)
        turn_limit = self._yaw_limit(error)
        if self._rotating:
            target_w = (0.0 if turning_error <= p.rotate_exit_threshold else
                        clamp(p.heading_gain * self.steering_error, -turn_limit, turn_limit))
        else:
            # 实测转速预估短时航向变化，给底盘转向滞后留出制动时间。
            target_w = clamp(vx * 2.0 * math.sin(self.steering_error) / self.lookahead,
                             -turn_limit, turn_limit)
        # 转速窗先与最低可达车速相容，避免随后横向约束造成超出制动斜率的急减速。
        low_v = max(0.0, old_v - f.collision_checker.linear_deceleration * dt)
        reachable_w = p.lateral_acceleration / max(low_v, 1e-9)
        wz = clamp(target_w, max(-reachable_w, old_w - p.accel_w * dt),
                   min(reachable_w, old_w + p.accel_w * dt))
        vx, wz = min(vx, f.command_max_vx), clamp(wz, -f.command_max_wz, f.command_max_wz)
        # 实际角速度含航向反馈，不能只用路径曲率替代真实 v*w 横向加速度。
        vx = min(vx, p.lateral_acceleration / max(abs(wz), 1e-9))
        self.prev_cmd = (vx, wz)
        command = (0.0 if vx < p.deadband_v else vx,
                   0.0 if abs(wz) < p.deadband_w else wz)
        self.status = "rotating" if self._rotating else "tracking"
        self.candidates = self._candidates(pose, command, speed, heading, dt, sectors, (old_v, old_w))
        self.last_command = command
        return command[0], 0.0, command[1]

    def _candidates(self, pose, nominal, speed, heading, dt, sectors, previous):
        """优先检查连续跟踪命令；仅在受阻时使用可达速度窗内的备选短弧线。"""
        f, p = self.follower, self.p
        horizon = f.collision_checker.prediction_time
        low_v = max(0.0, previous[0] - f.collision_checker.linear_deceleration * dt)
        high_v = min(f.command_max_vx, previous[0] + p.accel_v * dt)
        if self.yaw_braking:
            high_v = min(high_v, previous[0])
        reachable_w = p.lateral_acceleration / max(low_v, 1e-9)
        low_w = max(-f.command_max_wz, previous[1] - p.accel_w * dt, -reachable_w)
        high_w = min(f.command_max_wz, previous[1] + p.accel_w * dt, reachable_w)
        # 包含原始命令和减速候选，转向两侧都保留，以免绕行偏好成为硬性方向约束。
        velocities = {nominal[0], clamp(speed * 0.5, low_v, high_v), low_v}
        turns = {nominal[1], low_w, high_w, clamp(0.0, low_w, high_w)}
        ranked = []
        for velocity in sorted(velocities, reverse=True):
            for turn in sorted(turns):
                velocity = 0.0 if velocity < p.deadband_v else velocity
                turn = 0.0 if abs(turn) < p.deadband_w else turn
                if velocity * abs(turn) > p.lateral_acceleration + 1e-12:
                    continue
                end = FootprintCollisionChecker._advance(pose, velocity, turn, horizon)
                progress, offset = self.reference.project(end[0], end[1],
                    max(0.0, self.progress - 0.10), min(self.reference.length,
                    self.progress + abs(velocity) * horizon + self.lookahead))
                gain = (progress - self.progress) / max(f.command_max_vx * horizon, 0.02)
                score = (2.0 * offset / self.lookahead + 1.5 * abs(normalize_angle(heading - end[2]))
                         + 0.40 * abs(velocity - speed) / max(f.command_max_vx, 0.01)
                         + 0.08 * abs(turn - previous[1]) / max(f.command_max_wz, 0.01) - 2.0 * gain)
                if sectors:
                    clearance = min((row[0] for row in self.obstacle_sectors(end,
                        [(row[1], row[2]) for row in sectors])), default=p.obstacle_influence)
                    score += 0.20 * max(0.0, 1.0 - clearance / p.obstacle_influence)
                ranked.append((score, velocity, turn))
        # 相同输出的死区候选只检一次，限制昂贵的车身栅格查询次数。
        # 安全的正常跟踪不应被评分换成停车或更强转向，导致下一周期再次纠偏。
        commands = [nominal]
        for _, velocity, turn in sorted(ranked):
            if (velocity, turn) not in commands:
                commands.append((velocity, turn))
        return commands

    def sync_command(self, actual, requested):
        """同步真正发布的速度；控制器自身死区归零时保留连续的爬升状态。"""
        self.prev_cmd = tuple(self.prev_cmd[i] if abs(requested[i]) < 1e-12
                             and abs(actual[i]) < 1e-12 else actual[i] for i in (0, 1))
        self.last_command = tuple(actual)
