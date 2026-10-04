"""V3 策略适配：候选检查、发布同步和按实际路径进展触发的重规划。"""

import math
import time

import rospy

from .adaptive_controller import AdaptiveController, V3Parameters
from .path_processing import PathPoint


class AdaptiveTracking:
    """独立组合策略，共用运行层接口但不继承其他导航节点或跟踪策略。"""

    name = "v3"
    manages_path_progress = True

    def __init__(self, follower):
        self.follower = follower
        self.parameters = V3Parameters.from_getter(follower.param)
        self.controller = AdaptiveController(follower, self.parameters)
        self._epoch = 0
        self._candidate_epoch = 0
        self._obstacles = []
        self._progress_sample = None

    def update_path_progress(self, pose, path):
        self.controller.update_path_progress(pose, path)

    @property
    def remaining_distance(self):
        return self.controller.remaining_distance

    def compute(self, pose, path, _target, dt):
        _, self._obstacles = self.follower.dynamic_layer.point_snapshot()
        self._candidate_epoch = self._epoch
        try:
            return self.controller.compute(pose, path, dt, self._obstacles, time.monotonic())
        except (ValueError, OverflowError):
            self.controller.stop()
            self.controller.status = "invalid_input"
            return 0.0, 0.0, 0.0

    def refine_command(self, pose, command, generation):
        """在运行层锁外检查短轨迹，定位失效及取消目标不必等待候选计算。"""
        f, c, p = self.follower, self.controller, self.parameters
        with f._lock:
            epoch = self._candidate_epoch
            candidates = list(c.candidates)
            obstacles = list(self._obstacles)
            measured = f._measured_velocity
            occupied = f.raw_map_checker()
        if f.collision_require_static_map and occupied is None:
            return 0.0, 0.0, 0.0
        checker = f.collision_checker
        # 仅丢弃不可能进入任何候选/制动扫掠的远点，不能用势场扇区压缩代替碰撞输入。
        speed = max(f.command_max_vx, abs(measured[0]))
        braking = max(speed / checker.linear_deceleration,
                      max(f.command_max_wz, abs(measured[1])) / checker.angular_deceleration)
        radius = speed * (checker.prediction_time + checker.reaction_time + braking * 0.5) + f.geometry.body_radius + checker.obstacle_radius + 1.0
        obstacles = [(x, y) for x, y in obstacles
                     if not math.isfinite(x) or not math.isfinite(y)
                     or math.hypot(x - pose[0], y - pose[1]) <= radius]
        started = time.monotonic()
        selected = None
        for index, candidate in enumerate(candidates[:p.trajectory_max_checks]):
            with f._lock:
                if epoch != self._epoch or generation != f._plan_generation:
                    return 0.0, 0.0, 0.0
            if index and time.monotonic() - started >= p.trajectory_time_budget:
                break
            result = checker.check(pose, *candidate, obstacles,
                                   current_velocity=measured, occupied=occupied)
            if result.safe:
                selected = candidate
                break
            if result.reason not in ("dynamic_obstacle", "static_obstacle"):
                break
        with f._lock:
            if epoch != self._epoch or generation != f._plan_generation:
                return 0.0, 0.0, 0.0
            if selected is None:
                c.status = "trajectory_blocked"
                selected = (0.0, 0.0)
                c.prev_cmd = selected
            elif selected != (command[0], command[2]):
                c.prev_cmd = selected
        return selected[0], 0.0, selected[1]

    def reset(self):
        self._epoch += 1
        self.controller.stop()

    def before_recovery(self):
        self.controller.last_force = None
        self._progress_sample = None

    def finish_recovery(self, command):
        self.controller.prev_cmd = self.controller.last_command = tuple(command)
        self._progress_sample = None

    def after_control_cycle(self):
        """连续缺少路径进展时重规划；正常原地转向用实际航向变化豁免。"""
        f, c, p = self.follower, self.controller, self.parameters
        if (not f.global_path or f.waiting_for_plan or f.start_recovery is not None
                or f.dynamic_avoidance_mode != "astar_replan"
                or not f.dynamic_layer.is_fresh() or c.last_pose is None
                or (f.require_localization and (not f.localization_valid
                    or f.localization_received is None
                    or time.monotonic() - f.localization_received > f.pose_timeout))):
            self._progress_sample = None
            return
        now = time.monotonic()
        sample = (now, c._path_token, c.progress, c.last_pose[2])
        previous = self._progress_sample
        if previous is None or previous[1] != c._path_token:
            self._progress_sample = sample
            return
        moved = c.progress - previous[2] >= p.stall_progress
        turning = c.status == "rotating" and abs(math.atan2(
            math.sin(c.last_pose[2] - previous[3]), math.cos(c.last_pose[2] - previous[3]))) >= 0.10
        if moved or turning:
            self._progress_sample = sample
        elif now - previous[0] >= p.stall_time and f.dynamic_replan_due(None):
            kind = "avoidance" if f.dynamic_history else "normal"
            if f.request_plan(c.last_pose, kind):
                f.last_dynamic_replan_time = f.now_sec()
                self._progress_sample = sample
                rospy.logwarn("V3 path progress stalled; requesting A* plan")

    def command_diagnostics(self):
        c = self.controller
        return c.status, c.progress, c.lookahead, c.nearest_clearance, c.bypass_side

    def command_published(self, accepted, requested_vx, requested_wz, diagnostics):
        if accepted:
            self.controller.sync_command(self.follower._last_command, (requested_vx, requested_wz))
        rospy.loginfo_throttle(1.0, "V3: state=%s progress=%.3f lookahead=%.3f clearance=%.3f side=%d accepted=%s" % (*diagnostics, accepted))

    def desired_direction(self, pose, target, active):
        force = self.controller.last_force
        if self.follower.start_recovery is not None:
            return target, active
        if force is None or math.hypot(*force) < 1e-9:
            return None, False
        norm = math.hypot(*force)
        return PathPoint(pose[0] + force[0] / norm, pose[1] + force[1] / norm), active
