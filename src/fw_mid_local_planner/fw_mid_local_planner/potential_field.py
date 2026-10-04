"""与 ROS 解耦的 v1 跟踪策略：计算人工势场合力与速度请求。"""

import heapq
import math

from fw_mid_common_utils import normalize_angle


class ArtificialPotentialFieldController:
    """叠加最多三个定幅吸引力及按观测年龄衰减的障碍斥力。

    参数和障碍快照由所属跟踪器提供。合力使用地图坐标；净距按带边距的车体
    矩形计算，危险区降低速度请求。最终是否允许运动，仍由运行时的车体扫掠
    碰撞检查与实测速度制动检查决定。
    """

    def __init__(self, follower):
        self.follower = follower
        self.reset()

    def reset(self):
        self.prev_cmd = (0.0, 0.0)
        self.last_command = (0.0, 0.0)
        self.last_force = None
        self.last_raw_force = None
        self.last_heading = None
        self.nearest_clearance = float("inf")
        self.obstacle_count = 0
        self.status = "idle"
        self._path_end = None

    @staticmethod
    def _point_xy(point):
        if hasattr(point, "x"):
            return float(point.x), float(point.y)
        return float(point[0]), float(point[1])

    @staticmethod
    def _limit_rate(target, previous, limit, dt):
        delta = limit * dt
        return max(previous - delta, min(previous + delta, target))

    def sync_command(self, vx, wz, requested_vx=None, requested_wz=None):
        """同步下游限速结果，并保留尚未越过输出死区的连续速度爬升状态。"""
        requested = (requested_vx, requested_wz)
        actual = (vx, wz)
        ramp = self.prev_cmd
        synced = []
        for index in (0, 1):
            # 控制器死区导致的零输出不重置连续限加速度状态，避免永远无法起步；
            # 下游显式停车则应同步零速度，下一次从停止状态重新爬升。
            if (requested[index] is not None
                    and abs(requested[index]) < (self.follower.apf_deadband_v
                        if index == 0 else self.follower.apf_deadband_wz)
                    and abs(actual[index]) < 1e-9):
                synced.append(ramp[index])
            else:
                synced.append(actual[index])
        self.prev_cmd = tuple(synced)
        self.last_command = (vx, wz)

    def _targets(self, path, robot_pose):
        """在路径前部搜索近目标，并按索引步长选择最多三个不同的吸引点。"""
        if not path:
            return None, None, None

        rx, ry, yaw = robot_pose
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        # 寻找第一个合适的近点：在车辆前方且不需要大幅回头
        near_idx = 0
        for i in range(min(len(path), self.follower.apf_waypoint_stride * 2)):
            px, py = self._point_xy(path[i])
            dx, dy = px - rx, py - ry
            forward = dx * cos_yaw + dy * sin_yaw
            distance = math.hypot(dx, dy)

            # 如果点在车辆后方或需要大幅回头，且还有更前方的点，则跳过
            if i < len(path) - 1 and (forward < -0.3 or (forward < 0 and distance > 0.5)):
                continue

            near_idx = i
            break

        far_idx = min(near_idx + self.follower.apf_waypoint_stride, len(path) - 1)
        farther_idx = min(far_idx + self.follower.apf_waypoint_stride, len(path) - 1)

        near = path[near_idx]
        far = path[far_idx] if far_idx > near_idx else None
        farther = path[farther_idx] if farther_idx > far_idx else None

        return near, far, farther

    def _age_weight(self, stamp, now):
        # 新鲜度窗口内不衰减，之后按秒指数衰减并保留非零权重下限。
        p = self.follower
        age = max(0.0, now - stamp - p.dynamic_layer.timeout)
        return max(p.apf_memory_min_weight,
                   math.exp(-age / p.apf_memory_decay_time))

    def _nearby_obstacles(self, robot_pose, now):
        """保持点与时间戳对应，先计算全部候选点净距，再限制参与合力的点数。"""
        p = self.follower
        self.nearest_clearance = float("inf")
        if not p.dynamic_layer.enabled:
            return []
        rx, ry, yaw = robot_pose
        c, s = math.cos(yaw), math.sin(yaw)
        g = p.geometry
        # 搜索范围覆盖车体每个角的危险带，以及以车体中心计距的完整斥力影响带。
        search_radius = max(
            p.apf_obstacle_influence_distance + p.apf_obstacle_radius,
            g.body_radius + p.apf_obstacle_radius + p.apf_danger_clearance)
        cells = {}
        for x, y, stamp in p.dynamic_layer.timed_point_snapshot():
            if not all(math.isfinite(v) for v in (x, y, stamp)):
                raise ValueError("invalid timed obstacle")
            dx, dy = x - rx, y - ry
            if math.hypot(dx, dy) > search_radius:
                continue
            # 同一 XY 位置的竖直体素列只产生一份平面斥力，采用最新命中时间。
            key = (x, y)
            cells[key] = max(stamp, cells.get(key, stamp))
        candidates = []
        for (x, y), stamp in cells.items():
            dx, dy = x - rx, y - ry
            bx, by = c * dx + s * dy, -s * dx + c * dy
            gap_x = max(-g.footprint_rear - g.footprint_margin - bx,
                        0.0, bx - g.footprint_front - g.footprint_margin)
            gap_y = max(0.0, abs(by) - g.footprint_half_width - g.footprint_margin)
            clearance = max(0.0, math.hypot(gap_x, gap_y) - p.apf_obstacle_radius)
            self.nearest_clearance = min(self.nearest_clearance, clearance)
            candidates.append((clearance, x, y, stamp))
        return heapq.nsmallest(p.apf_max_obstacles, candidates)

    def _zero(self, status):
        self.prev_cmd = self.last_command = (0.0, 0.0)
        self.last_force = self.last_heading = None
        self.status = status
        return 0.0, 0.0, 0.0

    def compute(self, robot_pose, path, _target, dt):
        """从米制地图位姿和路径计算 ``(vx, 0, wz)``，dt 使用秒。"""
        if not path:
            self.reset()
            return self._zero("no_path")
        p = self.follower
        rx, ry, yaw = robot_pose
        now = p.now_sec()
        if (not all(math.isfinite(v) for v in (rx, ry, yaw, dt, now)) or dt <= 0):
            return self._zero("invalid_input")
        if self._path_end is not path[-1]:
            # 运行时复制或裁剪列表时保留 PathPoint 对象；收到新的 A* 路径后，
            # 即使终点坐标相同也会换终点对象。仅发起重规划请求不重置滤波器。
            self.last_force = self.last_heading = None
            self._path_end = path[-1]
        near, far, farther = self._targets(path, robot_pose)
        ax = ay = 0.0
        # 每个目标只贡献指定幅值的单位方向吸引力，距离不放大该目标的权重。
        for target, gain in ((near, p.apf_near_attraction_gain),
                             (far, p.apf_far_attraction_gain),
                             (farther, p.apf_farther_attraction_gain)):
            if target is None:
                continue
            tx, ty = self._point_xy(target)
            distance = math.hypot(tx - rx, ty - ry)
            if not math.isfinite(distance):
                return self._zero("invalid_input")
            if distance > 1e-9:
                ax += gain * (tx - rx) / distance
                ay += gain * (ty - ry) / distance

        try:
            obstacles = self._nearby_obstacles(robot_pose, now)
        except (ValueError, OverflowError):
            return self._zero("invalid_input")
        self.obstacle_count = len(obstacles)
        rep_x = rep_y = 0.0
        influence = p.apf_obstacle_influence_distance
        for _, ox, oy, stamp in obstacles:
            dx, dy = rx - ox, ry - oy
            distance = math.hypot(dx, dy)
            if distance < 1e-9:
                # 障碍与车体中心重合时斥力方向未定义；此处跳过方向计算，
                # 后续净距逻辑会降速，运行时车体碰撞检查负责拒绝重叠运动。
                continue
            gap = max(distance - p.apf_obstacle_radius, 1e-3)
            if gap >= influence:
                continue
            strength = min(p.apf_repulsive_max_force,
                           p.apf_repulsive_gain * (1.0 / gap - 1.0 / influence) / gap**2)
            strength *= self._age_weight(stamp, now)
            rep_x += strength * dx / distance
            rep_y += strength * dy / distance

        raw = (ax + rep_x, ay + rep_y)
        self.last_raw_force = raw
        norm = math.hypot(*raw)
        if not math.isfinite(norm):
            return self._zero("invalid_force")

        # 危险区域降低速度请求；安全放行仍取决于下游碰撞及制动检查。
        danger_speed_scale = 1.0
        if self.nearest_clearance <= p.apf_danger_clearance:
            self.status = "danger_zone"
            # 按净距比例缩放速度，最低保留 10% 请求，避免该层直接将速度归零。
            danger_speed_scale = max(0.10, self.nearest_clearance / max(0.01, p.apf_danger_clearance))
            # 仅当合力接近零时退回纯吸引力方向；正常危险区仍保留障碍斥力。
            if norm < p.apf_force_epsilon:
                # 如果合力为零，则使用纯吸引力方向
                raw = (ax, ay)
                norm = math.hypot(*raw)
                if norm < p.apf_force_epsilon:
                    # 连吸引力都没有，才真正停止
                    self.last_force = self.last_heading = None
                    self.prev_cmd = self.last_command = (0.0, 0.0)
                    return 0.0, 0.0, 0.0
        elif norm < p.apf_force_epsilon:
            return self._zero("force_cancelled")

        # 对向量滤波而非航向角滤波，避免正负 pi 交界处的不连续跳变。
        tau = p.apf_force_filter_time
        alpha = -math.expm1(-dt / tau) if tau > 0.0 else 1.0
        force = raw if self.last_force is None else tuple(
            old + alpha * (new - old) for old, new in zip(self.last_force, raw))
        if math.hypot(*force) < p.apf_force_epsilon:
            force = raw
        self.last_force = force
        self.last_heading = math.atan2(force[1], force[0])
        error = normalize_angle(self.last_heading - yaw)
        raw_error = normalize_angle(math.atan2(raw[1], raw[0]) - yaw)
        # 转弯减速同时参考未滤波方向，避免滤波滞后使急转弯时仍保持高速度。
        turn_error = max(abs(error), abs(raw_error))

        gx, gy = self._point_xy(path[-1])
        goal_distance = math.hypot(gx - rx, gy - ry)
        speed = p.command_max_vx

        if goal_distance < p.apf_goal_approach_distance:
            speed *= max(p.apf_min_speed_scale, goal_distance / p.apf_goal_approach_distance)

        if turn_error >= p.apf_stop_rotate_yaw:
            speed *= p.apf_min_speed_scale  # 保持最低速度而不是完全停止
        else:
            speed *= max(p.apf_min_speed_scale, 1.0 - turn_error / p.apf_slowdown_yaw)

        if 0.0 < speed < p.apf_min_vx and goal_distance > 0.25:
            speed = min(p.command_max_vx, p.apf_min_vx)

        # 应用危险区域降速系数
        speed *= danger_speed_scale

        vx = self._limit_rate(speed, self.prev_cmd[0], p.apf_accel_limit_v, dt)
        wz = max(-p.command_max_wz, min(p.command_max_wz, p.apf_heading_gain * error))
        wz = self._limit_rate(wz, self.prev_cmd[1], p.apf_accel_limit_wz, dt)
        wz = max(-p.command_max_wz, min(p.command_max_wz, wz))
        self.prev_cmd = (vx, wz)

        # 保存死区处理前的连续速度，避免较短控制周期使限速器一直困在输出死区。
        out_v = 0.0 if vx < p.apf_deadband_v else vx
        out_w = 0.0 if abs(wz) < p.apf_deadband_wz else wz

        self.last_command = (out_v, out_w)
        self.status = "tracking"
        return out_v, 0.0, out_w
