#!/usr/bin/env python3
"""人工势场路径跟踪器。

``PathFollower`` 负责目标接收、A* 服务调用、点云障碍记忆、动态障碍
重规划以及最终车身碰撞检查。本模块只替换路径跟踪阶段：从 A* 返回的
离散路径中取两个尚未到达的目标点，用两个吸引力和实时障碍斥力计算
合力，再把合力方向转换为 ``cmd_vel`` 的线速度和角速度。
"""

import math
import time

import rospy

from .path_follower_node import PathFollower
from .path_processing import PathPoint
from .potential_field import ArtificialPotentialFieldController


class APFPathFollower(PathFollower):
    """Existing local planner with APF path tracking enabled."""

    def __init__(self) -> None:
        # The base starts its timer during construction. Hold motion until
        # APF parameters and the replacement controller have been installed.
        self._apf_ready = False
        self._apf_stall_since = None
        super().__init__()
        if self.obstacle_slowdown_enabled:
            rospy.logwarn("APF v1 ignores obstacle_slowdown_enabled; danger zone parks directly")
        self.obstacle_slowdown_enabled = False
        self._load_apf_parameters()
        self.apf_controller = ArtificialPotentialFieldController(self)
        # The base loop selects one of these based on tracking_controller.
        # Point both choices at APF so a stale config cannot silently switch
        # this v1 node back to PID or pure pursuit.
        self.pid_controller = self.apf_controller
        self.pure_pursuit_controller = self.apf_controller
        self.tracking_controller = "pid"
        self._apf_ready = True
        rospy.loginfo(
            "APF v1 tracking enabled: stride=%d near_gain=%.3f far_gain=%.3f "
            "repulsive_gain=%.3f influence=%.2f m",
            self.apf_waypoint_stride,
            self.apf_near_attraction_gain,
            self.apf_far_attraction_gain,
            self.apf_repulsive_gain,
            self.apf_obstacle_influence_distance,
        )

    def _load_apf_parameters(self) -> None:
        self.apf_waypoint_stride = max(1, int(self.param("apf_waypoint_stride", 4)))
        self.apf_near_attraction_gain = float(
            self.param("apf_near_attraction_gain", 1.0))
        self.apf_far_attraction_gain = float(
            self.param("apf_far_attraction_gain", 0.35))
        self.apf_farther_attraction_gain = float(
            self.param("apf_farther_attraction_gain", 0.20))
        self.apf_repulsive_gain = float(self.param("apf_repulsive_gain", 0.08))
        self.apf_obstacle_influence_distance = float(
            self.param("apf_obstacle_influence_distance", 0.90))
        self.apf_obstacle_radius = float(
            self.param("apf_obstacle_radius", self.geometry.obstacle_radius))
        self.apf_repulsive_max_force = float(
            self.param("apf_repulsive_max_force", 8.0))
        self.apf_max_obstacles = max(1, int(self.param("apf_max_obstacles", 300)))
        self.apf_heading_gain = float(self.param("apf_heading_gain", 1.8))
        self.apf_stop_rotate_yaw = float(self.param("apf_stop_rotate_yaw", 1.45))
        self.apf_slowdown_yaw = float(self.param("apf_slowdown_yaw", 1.0))
        self.apf_goal_approach_distance = float(
            self.param("apf_goal_approach_distance", 0.80))
        self.apf_min_speed_scale = float(self.param("apf_min_speed_scale", 0.20))
        self.apf_min_vx = float(self.param("apf_min_vx", 0.04))
        self.apf_force_epsilon = float(self.param("apf_force_epsilon", 1e-4))
        self.apf_accel_limit_v = float(self.param("apf_accel_limit_v", 0.25))
        self.apf_accel_limit_wz = float(self.param("apf_accel_limit_wz", 0.30))
        self.apf_deadband_v = float(self.param("apf_deadband_v", 0.015))
        self.apf_deadband_wz = float(self.param("apf_deadband_wz", 0.025))
        self.apf_danger_clearance = float(
            self.param("apf_danger_clearance", 0.05))
        self.apf_force_filter_time = float(self.param("apf_force_filter_time", 0.20))
        self.apf_memory_decay_time = float(self.param("apf_memory_decay_time", 2.0))
        self.apf_memory_min_weight = float(self.param("apf_memory_min_weight", 0.25))
        self.apf_stall_replan_time = float(self.param("apf_stall_replan_time", 2.0))
        positive = (
            self.apf_near_attraction_gain,
            self.apf_far_attraction_gain,
            self.apf_repulsive_gain,
            self.apf_obstacle_influence_distance,
            self.apf_obstacle_radius,
            self.apf_repulsive_max_force,
            self.apf_heading_gain,
            self.apf_stop_rotate_yaw,
            self.apf_slowdown_yaw,
            self.apf_goal_approach_distance,
            self.apf_min_vx,
            self.apf_force_epsilon,
            self.apf_accel_limit_v,
            self.apf_accel_limit_wz,
            self.apf_memory_decay_time,
            self.apf_stall_replan_time,
        )
        nonnegative = (self.apf_danger_clearance, self.apf_force_filter_time)
        if (not all(math.isfinite(value) and value > 0.0 for value in positive)
                or not all(math.isfinite(value) and value >= 0.0 for value in nonnegative)
                or not math.isfinite(self.apf_memory_min_weight)
                or not 0.0 < self.apf_memory_min_weight <= 1.0
                or not math.isfinite(self.apf_min_speed_scale)
                or not 0.0 <= self.apf_min_speed_scale <= 1.0
                or not math.isfinite(self.apf_deadband_v)
                or not math.isfinite(self.apf_deadband_wz)
                or self.apf_deadband_v < 0.0
                or self.apf_deadband_wz < 0.0
                or self.apf_near_attraction_gain < self.apf_far_attraction_gain):
            raise ValueError("Invalid APF parameters")

    def _control_loop(self, event) -> None:
        if not self._apf_ready:
            self.publish_stop()
            return
        super()._control_loop(event)
        with self._lock:
            if (self.apf_controller.status not in ("force_cancelled", "danger_zone")
                    or not self.global_path
                    or self.goal_pose is None or self.dynamic_avoidance_mode != "astar_replan"):
                self._apf_stall_since = None
                return
            now = time.monotonic()
            if self._apf_stall_since is None:
                self._apf_stall_since = now
            if (self.waiting_for_plan or now - self._apf_stall_since < self.apf_stall_replan_time
                    or not self.dynamic_replan_due(None)):
                return
            pose = self.get_robot_pose()
            if pose is None:
                return
            # A zero command can pass the collision check indefinitely. Ask
            # A* explicitly, also when no dynamic overlay points are present.
            kind = "avoidance" if self.dynamic_history else "normal"
            if self.request_plan(pose, kind):
                self.last_dynamic_replan_time = self.now_sec()
                self._apf_stall_since = now
                rospy.logwarn("APF force cancelled; requesting a new A* path")

    def reset_controllers(self):
        if not self._apf_ready:
            return super().reset_controllers()
        self.apf_controller.reset()
        self._apf_stall_since = None

    def follow_start_recovery(self, robot_pose):
        # The base hands off at the last published velocity, while APF must
        # not keep a direction from before the checked recovery segment.
        self.apf_controller.last_force = self.apf_controller.last_heading = None
        return super().follow_start_recovery(robot_pose)

    def publish_cmd(self, velocity_x, velocity_y, velocity_yaw, robot_pose=None,
                    generation=None, allow_stale_dynamic=False):
        with self._lock:
            controller = self.apf_controller if self._apf_ready else None
            diagnostics = None
            if controller is not None and self.start_recovery is None:
                diagnostics = (controller.last_raw_force, controller.last_force,
                    controller.nearest_clearance, controller.obstacle_count,
                    controller.status)
            accepted = super().publish_cmd(velocity_x, velocity_y, velocity_yaw,
                robot_pose, generation, allow_stale_dynamic)
            if controller is not None and accepted:
                controller.sync_command(self._last_command[0], self._last_command[1],
                                        velocity_x, velocity_yaw)
            if diagnostics is not None:
                raw, filtered, clearance, count, status = diagnostics
                rospy.loginfo_throttle(1.0,
                    "APF: raw=%s filtered=%s clearance=%.3f m points=%d state=%s "
                    "accepted=%s vx=%.3f wz=%.3f"
                    % (raw, filtered, clearance, count,
                       status, accepted, self._last_command[0], self._last_command[1]))
            return accepted

    def publish_desired_direction(self, robot_pose, target, velocity_x, active=True):
        """Show the filtered resultant; clear the arrow for undefined forces."""
        if not self._apf_ready or not active or self.start_recovery is not None:
            return super().publish_desired_direction(
                robot_pose, target, velocity_x, active=active)
        force = self.apf_controller.last_force
        norm = math.hypot(*force) if force is not None else 0.0
        if norm < self.apf_force_epsilon:
            return super().publish_desired_direction(robot_pose, None, 0.0, active=False)
        endpoint = PathPoint(robot_pose[0] + force[0] / norm,
                             robot_pose[1] + force[1] / norm)
        return super().publish_desired_direction(robot_pose, endpoint, velocity_x, active=True)


def main() -> None:
    # Keep the original node name/private topics so existing launch files,
    # services and RViz configurations continue to address this follower.
    rospy.init_node("path_follower_node")
    APFPathFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
