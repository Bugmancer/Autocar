"""APF 策略与运行层之间的适配：参数、复位、交接、停滞重规划和诊断。"""

import math
import time

import rospy

from .path_processing import PathPoint
from .potential_field import ArtificialPotentialFieldController


class APFTracking:
    """组合势场控制器；ROS 收发与最终安全检查仍由公共运行层负责。"""

    name = "apf"

    def __init__(self, follower):
        self.follower = follower
        self._stall_since = None
        if follower.obstacle_slowdown_enabled:
            rospy.logwarn("APF v1 ignores obstacle_slowdown_enabled; using APF speed control")
        follower.obstacle_slowdown_enabled = False
        self._load_parameters()
        self.controller = ArtificialPotentialFieldController(follower)
        rospy.loginfo(
            "APF v1 tracking enabled: stride=%d near_gain=%.3f far_gain=%.3f "
            "repulsive_gain=%.3f influence=%.2f m",
            follower.apf_waypoint_stride,
            follower.apf_near_attraction_gain,
            follower.apf_far_attraction_gain,
            follower.apf_repulsive_gain,
            follower.apf_obstacle_influence_distance,
        )

    def _load_parameters(self):
        # 分组声明取值约束；第三吸引力允许为零，用于禁用更远目标的贡献。
        # 参数存到运行层供纯计算控制器读取，不能把 ROS 调用引入势场计算模块。
        p = self.follower
        positive = {
            "near_attraction_gain": 1.0,
            "far_attraction_gain": 0.35,
            "repulsive_gain": 0.08,
            "obstacle_influence_distance": 0.90,
            "obstacle_radius": p.geometry.obstacle_radius,
            "repulsive_max_force": 8.0,
            "heading_gain": 1.8,
            "stop_rotate_yaw": 1.45,
            "slowdown_yaw": 1.0,
            "goal_approach_distance": 0.80,
            "min_vx": 0.04,
            "force_epsilon": 1e-4,
            "accel_limit_v": 0.25,
            "accel_limit_wz": 0.30,
            "memory_decay_time": 2.0,
            "stall_replan_time": 2.0,
        }
        nonnegative = {
            "farther_attraction_gain": 0.20,
            "danger_clearance": 0.05,
            "force_filter_time": 0.20,
            "deadband_v": 0.015,
            "deadband_wz": 0.025,
        }
        defaults = dict(positive, **nonnegative)
        defaults.update(min_speed_scale=0.20, memory_min_weight=0.25)
        values = {name: float(p.param("apf_" + name, default))
                  for name, default in defaults.items()}
        p.apf_waypoint_stride = max(1, int(p.param("apf_waypoint_stride", 4)))
        p.apf_max_obstacles = max(1, int(p.param("apf_max_obstacles", 300)))
        if (not all(math.isfinite(values[name]) and values[name] > 0.0
                    for name in positive)
                or not all(math.isfinite(values[name]) and values[name] >= 0.0
                           for name in nonnegative)
                or not math.isfinite(values["memory_min_weight"])
                or not 0.0 < values["memory_min_weight"] <= 1.0
                or not math.isfinite(values["min_speed_scale"])
                or not 0.0 <= values["min_speed_scale"] <= 1.0
                or values["near_attraction_gain"] < values["far_attraction_gain"]):
            raise ValueError("Invalid APF parameters")
        for name, value in values.items():
            setattr(p, "apf_" + name, value)

    def compute(self, pose, path, target, dt):
        return self.controller.compute(pose, path, target, dt)

    def reset(self):
        # 停车/换路径时一并清理合力滤波、加速度状态和停滞计时，防止恢复后沿用旧方向。
        self.controller.reset()
        self._stall_since = None

    def before_recovery(self):
        # 恢复段方向已由车身碰撞检查确认，不能混入恢复前保存的 APF 合力方向。
        self.controller.last_force = self.controller.last_heading = None

    def finish_recovery(self, command):
        self.controller.prev_cmd = command

    def after_control_cycle(self):
        """持续合力抵消时尝试重新规划；由运行层持状态锁调用。"""
        p = self.follower
        if (self.controller.status not in ("force_cancelled", "danger_zone")
                or not p.global_path or p.goal_pose is None
                or p.dynamic_avoidance_mode != "astar_replan"):
            self._stall_since = None
            return
        now = time.monotonic()
        if self._stall_since is None:
            self._stall_since = now
        if (p.waiting_for_plan or now - self._stall_since < p.apf_stall_replan_time
                or not p.dynamic_replan_due(None)):
            return
        pose = p.get_robot_pose()
        if pose is None:
            return
        # 零速可以持续通过碰撞检查，因此势场停滞必须另行触发重规划。
        kind = "avoidance" if p.dynamic_history else "normal"
        if p.request_plan(pose, kind):
            p.last_dynamic_replan_time = p.now_sec()
            self._stall_since = now
            rospy.logwarn("APF force cancelled; requesting a new A* path")

    def command_diagnostics(self):
        # 必须在最终检查之前采样，否则停车复位会抹掉导致拒绝发布的力和状态。
        if self.follower.start_recovery is not None:
            return None
        c = self.controller
        return (c.last_raw_force, c.last_force, c.nearest_clearance,
                c.obstacle_count, c.status)

    def command_published(self, accepted, requested_vx, requested_wz, diagnostics):
        # 限幅/碰撞降速可能改变候选值；用实际输出同步加速度状态，避免下周期跳回高速。
        command = self.follower._last_command
        if accepted:
            self.controller.sync_command(*command, requested_vx, requested_wz)
        if diagnostics is not None:
            raw, filtered, clearance, count, status = diagnostics
            rospy.loginfo_throttle(1.0,
                "APF: raw=%s filtered=%s clearance=%.3f m points=%d state=%s "
                "accepted=%s vx=%.3f wz=%.3f"
                % (raw, filtered, clearance, count, status, accepted, *command))

    def desired_direction(self, robot_pose, target, active):
        """显示滤波后的合力方向；恢复阶段仍显示已经验证的恢复目标。"""
        if not active or self.follower.start_recovery is not None:
            return target, active
        force = self.controller.last_force
        norm = math.hypot(*force) if force is not None else 0.0
        if norm < self.follower.apf_force_epsilon:
            return None, False
        return PathPoint(robot_pose[0] + force[0] / norm,
                         robot_pose[1] + force[1] / norm), True
