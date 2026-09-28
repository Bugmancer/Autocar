#!/usr/bin/env python3
"""ROS1 path follower with dynamic-obstacle A* replanning.

The node deliberately ends at ``/cmd_vel``.  It does not import the chassis
driver and cannot publish ``/fw_mid/command_dict`` by itself.
"""

import math
import threading
import time
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Bool, Time as TimeMessage
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from fw_mid_common_utils import Point2D, Pose2D, yaw_from_quaternion
from fw_mid_common_utils.collision_geometry import CollisionGeometry
from fw_mid_controller import PIDPathController, PurePursuitController

from .dynamic_obstacle_layer import DynamicObstacleLayer
from .footprint_collision import FootprintCollisionChecker, CollisionCheckResult
from .obstacle_processing import proximity_speed_scale, select_front_obstacles
from .path_processing import PathPoint, path_to_ros_msg, poses_to_xy, process_path, path_collision_free
from .start_recovery import forward_exit


class PathFollower:
    """局部跟踪主节点：接收 map 路径，经过 TF/障碍门控后发布 ``cmd_vel``。

    全局路径使用 ``global_frame``；动态记忆保存在 ``memory_frame`` 后投影
    到该帧。控制器读取机器人在全局帧的位姿，输出仅限线速度 x 与角速度 z。
    任何定位、动态点云、静态地图
    或扫掠碰撞检查不满足条件时，统一走 ``publish_stop``。
    """
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._plan_generation = 0
        self._memory_generation = 0
        self.require_localization = bool(self.param("require_localization", False))
        self.localization_valid = False
        self.localization_received = None
        self.pose_timeout = float(self.param("pose_timeout", 0.5))
        if not math.isfinite(self.pose_timeout) or self.pose_timeout <= 0:
            raise ValueError("pose_timeout must be finite and positive")

        self.global_frame = self._frame(self.param("global_frame", "map"))
        self.robot_frame = self._frame(self.param("robot_frame", "base_link"))
        self.planning_frame = self._frame(
            self.param("planning_frame", self.robot_frame)
        )
        self.cmd_vel_topic = str(self.param("cmd_vel_topic", "/cmd_vel"))
        self.plan_topic = str(
            self.param("plan_topic", "/local_planner/global_plan")
        )
        self.planner_service = str(
            self.param("planner_service", "/astar_planner_node/get_plan")
        )
        self.goal_topic = str(
            self.param("goal_topic", "/move_base_simple/goal")
        )
        self.legacy_goal_topic = str(self.param("legacy_goal_topic", "/goal_pose"))
        self.costmap_topic = str(
            self.param("costmap_topic", "/astar_planner_node/costmap")
        )
        self.static_map_topic = str(
            self.param("static_map_topic", "/astar_planner_node/map")
        )
        self.control_rate = max(1.0, float(self.param("control_rate", 10.0)))
        self.control_dt = 1.0 / self.control_rate
        self.command_max_vx = float(self.param("command_max_vx", 0.30))
        self.command_max_wz = math.radians(float(self.param("command_max_wz_deg", 30.0)))
        if not all(math.isfinite(value) and value >= 0
                   for value in (self.command_max_vx, self.command_max_wz)):
            raise ValueError("Command limits must be finite and nonnegative")
        self.lookahead_distance = float(self.param("lookahead_distance", 0.5))
        self.waypoint_tolerance = float(self.param("waypoint_tolerance", 0.25))
        self.goal_tolerance = float(self.param("goal_tolerance", 0.35))
        self.tracking_controller = str(
            self.param("tracking_controller", "pid")
        ).strip().lower()
        if self.tracking_controller not in ("pid", "pure_pursuit"):
            rospy.logwarn(
                "Unknown tracking_controller=%s; using pid",
                self.tracking_controller,
            )
            self.tracking_controller = "pid"

        self.path_enable_shortcut = bool(self.param("path_enable_shortcut", True))
        self.path_enable_smoothing = bool(
            self.param("path_enable_smoothing", True)
        )
        self.path_resample_ds = float(self.param("path_resample_ds", 0.10))
        self.path_smooth_weight_data = float(
            self.param("path_smooth_weight_data", 0.20)
        )
        self.path_smooth_weight_smooth = float(
            self.param("path_smooth_weight_smooth", 0.35)
        )
        self.path_smooth_max_iter = int(self.param("path_smooth_max_iter", 80))
        self.path_smooth_tolerance = float(
            self.param("path_smooth_tolerance", 1e-4)
        )
        self.path_collision_check_step = float(
            self.param("path_collision_check_step", 0.05)
        )
        self.path_occupied_threshold = int(
            self.param("path_occupied_threshold", 50)
        )

        self.dynamic_avoidance_mode = str(
            self.param("dynamic_avoidance_mode", "astar_replan")
        ).strip().lower()
        if self.dynamic_avoidance_mode not in ("off", "stop", "astar_replan"):
            rospy.logwarn(
                "Unknown dynamic_avoidance_mode=%s; using stop",
                self.dynamic_avoidance_mode,
            )
            self.dynamic_avoidance_mode = "stop"
        self.obstacle_slowdown_enabled = bool(self.param("obstacle_slowdown_enabled", False))
        self.obstacle_slowdown_distance = float(self.param("obstacle_slowdown_distance", 0.20))
        self.obstacle_min_speed_scale = float(self.param("obstacle_min_speed_scale", 0.20))
        if (not math.isfinite(self.obstacle_slowdown_distance) or self.obstacle_slowdown_distance <= 0
                or not math.isfinite(self.obstacle_min_speed_scale)
                or not 0 < self.obstacle_min_speed_scale <= 1):
            raise ValueError("Invalid obstacle slowdown distance or minimum speed scale")
        self.dynamic_degraded_motion_enabled = bool(
            self.param("dynamic_degraded_motion_enabled", True))
        self.dynamic_degraded_timeout = float(
            self.param("dynamic_degraded_timeout", 0.5))
        self.dynamic_degraded_speed_scale = float(
            self.param("dynamic_degraded_speed_scale", 0.25))
        if (not math.isfinite(self.dynamic_degraded_timeout)
                or self.dynamic_degraded_timeout < 0
                or not math.isfinite(self.dynamic_degraded_speed_scale)
                or not 0 < self.dynamic_degraded_speed_scale <= 1):
            raise ValueError("Invalid degraded radar motion parameters")
        self.dynamic_detect_x_min = float(
            self.param("dynamic_detect_x_min", 0.0)
        )
        self.dynamic_detect_x_max = float(
            self.param("dynamic_detect_x_max", 2.0)
        )
        self.dynamic_detect_y_abs = float(
            self.param("dynamic_detect_y_abs", 0.7)
        )
        self.dynamic_min_obstacle_points = int(
            self.param("dynamic_min_obstacle_points", 5)
        )
        self.dynamic_confirm_frames = max(
            1, int(self.param("dynamic_confirm_frames", 3))
        )
        self.dynamic_center_stable_dist = float(
            self.param("dynamic_center_stable_dist", 0.25)
        )
        self.dynamic_exit_clearance = float(
            self.param("dynamic_exit_clearance", 0.30)
        )
        self.dynamic_static_filter_enabled = bool(
            self.param("dynamic_static_filter_enabled", True)
        )
        self.dynamic_static_filter_radius = float(
            self.param("dynamic_static_filter_radius", 0.0)
        )
        self.dynamic_static_filter_threshold = int(
            self.param("dynamic_static_filter_threshold", 50)
        )
        self.dynamic_history_publish_resolution = float(
            self.param("dynamic_history_publish_resolution", 0.10)
        )
        if (not math.isfinite(self.dynamic_history_publish_resolution)
                or self.dynamic_history_publish_resolution <= 0):
            raise ValueError("dynamic_history_publish_resolution must be finite and positive")
        self.dynamic_astar_replan_period = float(
            self.param("dynamic_astar_replan_period", 1.0)
        )
        self.dynamic_obstacle_update_dist = float(
            self.param("dynamic_obstacle_update_dist", 0.30)
        )
        self.dynamic_obstacle_points_topic = str(
            self.param(
                "dynamic_obstacle_points_topic",
                "/local_planner/dynamic_obstacle_points",
            )
        )
        self.dynamic_keep_moving_during_replan = bool(
            self.param("dynamic_keep_moving_during_replan", True)
        )
        self.planner_service_timeout = float(
            self.param("planner_service_timeout", 0.5)
        )
        self.planner_overlay_settle_time = max(
            0.0, float(self.param("planner_overlay_settle_time", 0.03))
        )
        self.require_overlay_ack = bool(self.param("require_overlay_ack", True))
        self.overlay_applied_stamp = rospy.Time()
        self.overlay_sent_stamp = rospy.Time()
        self._last_overlay_wall = 0.0
        self.collision_check_enabled = bool(self.param("collision_check_enabled", True))
        self.collision_speed_reduction_enabled = bool(self.param("collision_speed_reduction_enabled", True))
        self.collision_require_static_map = bool(self.param("collision_require_static_map", True))
        self.start_recovery_enabled = bool(self.param("start_recovery_enabled", True))
        self.start_recovery_max_distance = float(self.param("start_recovery_max_distance", 1.2))
        if not 0 < self.start_recovery_max_distance <= 2.0:
            raise ValueError("Invalid bounded start recovery distance")
        self.start_recovery = None
        self.recovery_suffix = []
        self.recovery_planning = False
        self._last_motion_pose = None
        self._measured_velocity = (0.0, 0.0)
        self._last_command = (0.0, 0.0)

        self.costmap = None
        self.static_map = None
        self.global_path: List[PathPoint] = []
        self.goal_pose: Optional[PoseStamped] = None
        self.waiting_for_plan = False
        self.pending_plan_kind = "normal"
        self.avoidance_state = "normal"
        self.dynamic_history: List[Tuple[float, float, float]] = []
        self.last_dynamic_replan_time: Optional[float] = None
        self.last_dynamic_obstacle_center: Optional[Point2D] = None
        self.candidate_obstacle_center: Optional[Point2D] = None
        self.candidate_obstacle_count = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.dynamic_layer = DynamicObstacleLayer(self.tf_buffer, self.robot_frame, self.global_frame)
        self.geometry = CollisionGeometry.from_params(self.param)
        self.collision_checker = FootprintCollisionChecker(
            front=self.geometry.footprint_front,
            rear=self.geometry.footprint_rear,
            half_width=self.geometry.footprint_half_width,
            margin=self.geometry.footprint_margin,
            prediction_time=float(self.param("collision_prediction_time", 1.0)),
            reaction_time=float(self.param("collision_reaction_time", 0.2)),
            linear_deceleration=float(self.param("collision_linear_deceleration", 0.25)),
            angular_deceleration=float(self.param("collision_angular_deceleration", 0.5)),
            sample_distance=self.geometry.collision_sample_distance,
            obstacle_radius=self.geometry.obstacle_radius,
        )
        self.pid_controller = PIDPathController(rospy, cruise_speed=self.command_max_vx)
        self.pure_pursuit_controller = PurePursuitController(rospy, cruise_speed=self.command_max_vx)

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=10
        )
        # The arrow is the controller's current lookahead direction in the
        # global frame.  It is separate from the processed path so RViz can
        # show what the controller is trying to do at this instant.
        self.desired_direction_topic = str(
            self.param(
                "desired_direction_topic",
                "/local_planner/desired_direction",
            )
        )
        self.desired_direction_pub = rospy.Publisher(
            self.desired_direction_topic, Marker, queue_size=1
        )
        self.plan_pub = rospy.Publisher(
            self.plan_topic, Path, queue_size=1, latch=True
        )
        self.dynamic_points_pub = rospy.Publisher(
            self.dynamic_obstacle_points_topic,
            PoseArray,
            queue_size=1,
            latch=True,
        )
        self.collision_marker_pub = rospy.Publisher(
            "/local_planner/collision_check", MarkerArray, queue_size=1, latch=True)
        self.collision_blocked_pub = rospy.Publisher(
            "/local_planner/collision_blocked", Bool, queue_size=1, latch=True)
        self.overlay_ack_sub = rospy.Subscriber(
            str(self.param("dynamic_overlay_ack_topic", "/astar_planner_node/dynamic_overlay_stamp")),
            TimeMessage, self.overlay_ack_cb, queue_size=1)
        self.clear_memory_service = rospy.Service(
            "~clear_obstacle_memory", Trigger, self.clear_memory_cb)
        self.goal_subscriber = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_cb, queue_size=1
        )
        self.localization_subscriber = rospy.Subscriber(
            "/localizer/localization_valid", Bool, self.localization_cb, queue_size=1
        )
        self.legacy_goal_subscriber = None
        if self.legacy_goal_topic and self.legacy_goal_topic != self.goal_topic:
            self.legacy_goal_subscriber = rospy.Subscriber(
                self.legacy_goal_topic, PoseStamped, self.goal_cb, queue_size=1
            )
        self.costmap_subscriber = rospy.Subscriber(
            self.costmap_topic, OccupancyGrid, self.costmap_cb, queue_size=1
        )
        self.static_map_subscriber = rospy.Subscriber(
            self.static_map_topic, OccupancyGrid, self.static_map_cb, queue_size=1
        )
        self.control_timer = rospy.Timer(
            rospy.Duration(self.control_dt), self.control_loop
        )
        rospy.on_shutdown(self.shutdown)
        self.publish_dynamic_obstacle_overlay()
        self.publish_stop()
        rospy.loginfo(
            "Path follower ready: TF %s -> %s, cmd=%s, planner=%s, "
            "dynamic_mode=%s (no CAN output)",
            self.global_frame,
            self.robot_frame,
            self.cmd_vel_topic,
            self.planner_service,
            self.dynamic_avoidance_mode,
        )

    @staticmethod
    def param(name, default):
        return rospy.get_param("~" + name, default)

    @staticmethod
    def _frame(frame) -> str:
        return str(frame).strip().strip("/")

    @staticmethod
    def now_sec() -> float:
        return rospy.Time.now().to_sec()

    def get_frame_pose(self, frame: str) -> Optional[Pose2D]:
        # TF 查询目标固定为 global_frame；时间戳过期或未来数据都不能用于控制。
        if self.require_localization:
            with self._lock:
                ready = (self.localization_valid and self.localization_received is not None
                         and time.monotonic() - self.localization_received <= self.pose_timeout)
            if not ready:
                rospy.logwarn_throttle(1.0, "Waiting for fresh, converged localization")
                return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0,
                "No TF %s -> %s: %s"
                % (self.global_frame, frame, exc),
            )
            return None
        age = (rospy.Time.now() - transform.header.stamp).to_sec()
        if not math.isfinite(age) or age < -0.1 or age > self.pose_timeout:
            rospy.logwarn_throttle(1.0, "Robot TF is stale or future-dated")
            return None
        translation = transform.transform.translation
        pose = (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(transform.transform.rotation),
        )
        return pose if all(math.isfinite(value) for value in pose) else None

    def localization_cb(self, message):
        with self._lock:
            self.localization_valid = message.data
            self.localization_received = time.monotonic()

    def overlay_ack_cb(self, message):
        with self._lock:
            self.overlay_applied_stamp = message.data

    def clear_memory_cb(self, _request):
        # Explicit reset also cancels the current goal, so observing an empty
        # layer cannot restart the old trajectory after an operator reset.
        with self._lock:
            self._plan_generation += 1
            self._memory_generation += 1
            self.start_recovery = None
            self.global_path = []
            self.goal_pose = None
            self.waiting_for_plan = False
            self.avoidance_state = "normal"
            self.dynamic_layer.clear()
            self.dynamic_history = []
            self.reset_avoidance_tracking()
            self.publish_stop()
            self.publish_path_points([])
            self.publish_dynamic_obstacle_overlay()
        return TriggerResponse(success=True, message=
            "Session obstacle memory cleared and goal cancelled; waiting for fresh point clouds")

    def get_robot_pose(self) -> Optional[Pose2D]:
        return self.get_frame_pose(self.robot_frame)

    def get_planning_pose(
        self, current_robot_pose: Optional[Pose2D] = None
    ) -> Optional[Pose2D]:
        if self.planning_frame == self.robot_frame and current_robot_pose is not None:
            return current_robot_pose
        return self.get_frame_pose(self.planning_frame)

    def goal_cb(self, message: PoseStamped) -> None:
        frame = self._frame(message.header.frame_id or self.global_frame)
        if frame != self.global_frame:
            rospy.logerr(
                "Goal frame '%s' does not match global frame '%s'; goal ignored",
                frame,
                self.global_frame,
            )
            self.publish_stop()
            return
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            rospy.logwarn("Goal received but robot pose is unavailable")
            self.publish_stop()
            return
        message.header.frame_id = self.global_frame
        with self._lock:
            self._plan_generation += 1
            self.waiting_for_plan = False
            self.goal_pose = message
            self.start_recovery = None
            self.recovery_suffix = []
            self.recovery_planning = False
            self.pending_plan_kind = "normal"
            self.global_path = []
            self.avoidance_state = "normal"
            self.reset_avoidance_tracking()
            self.publish_stop()
            self.publish_path_points([])
        self.request_plan(robot_pose, "normal")
        rospy.loginfo(
            "Goal accepted: (%.2f, %.2f)",
            message.pose.position.x,
            message.pose.position.y,
        )

    def request_plan(self, robot_pose: Pose2D, kind: str) -> bool:
        # 每次规划递增 generation，旧线程返回时会被丢弃，避免旧路径覆盖新目标。
        planning_pose = self.get_planning_pose(robot_pose)
        if planning_pose is None:
            rospy.logwarn("Cannot plan: planning frame '%s' unavailable", self.planning_frame)
            return False
        with self._lock:
            if self.waiting_for_plan:
                return False
            if self.goal_pose is None:
                rospy.logwarn("Cannot plan without a goal")
                return False
            self._plan_generation += 1
            generation = self._plan_generation
            goal = self.goal_pose
            self.waiting_for_plan = True
            self.start_recovery = None
            self.recovery_planning = (self.costmap is not None
                and self.is_costmap_occupied(planning_pose[0], planning_pose[1]))
            self.pending_plan_kind = kind
            if self.recovery_planning:
                if self.is_dynamic_replan_pending():
                    rospy.loginfo("Inflated start: planning recovery while tracking with live collision checks")
                else:
                    rospy.loginfo("Navigation stop: inflated start; waiting for checked recovery plan")
                    self.publish_stop()
            overlay_stamp = self.publish_dynamic_obstacle_overlay()

        request = GetPlanRequest()
        request.start.header.frame_id = self.global_frame
        request.start.header.stamp = rospy.Time.now()
        request.start.pose.position.x = planning_pose[0]
        request.start.pose.position.y = planning_pose[1]
        request.start.pose.orientation.z = math.sin(planning_pose[2] / 2.0)
        request.start.pose.orientation.w = math.cos(planning_pose[2] / 2.0)
        request.goal = goal
        request.tolerance = self.goal_tolerance
        worker = threading.Thread(
            target=self._plan_worker,
            args=(request, kind, generation, overlay_stamp),
            name="fw_mid_plan_request",
        )
        worker.daemon = True
        worker.start()
        return True

    def _plan_worker(self, request, kind: str, generation: int, overlay_stamp) -> None:
        # 先等待 A* 确认同一份动态障碍快照，再接受服务结果。
        recovery = None
        try:
            if self.require_overlay_ack:
                deadline = time.monotonic() + self.planner_service_timeout
                while not rospy.is_shutdown():
                    with self._lock:
                        if generation != self._plan_generation:
                            return
                        applied = self.overlay_applied_stamp >= overlay_stamp
                    if applied:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("A* has not acknowledged the obstacle memory snapshot")
                    time.sleep(0.01)
            elif self.planner_overlay_settle_time > 0.0:
                rospy.sleep(self.planner_overlay_settle_time)
            rospy.wait_for_service(
                self.planner_service, timeout=self.planner_service_timeout
            )
            client = rospy.ServiceProxy(self.planner_service, GetPlan)
            response = client(request)
            path = response.plan
            if not path.poses:
                path, recovery = self.plan_start_recovery(client, request, generation)
            error = None
        except Exception as exc:
            path = None
            error = exc

        if error is not None:
            with self._lock:
                if generation != self._plan_generation:
                    return
                self.waiting_for_plan = False
                if kind == "avoidance":
                    self.avoidance_state = "wait"
                else:
                    self.global_path = []
                self.publish_stop()
            rospy.logerr("Planning failed (%s): %s", kind, error)
            return
        if path is None or not path.poses:
            with self._lock:
                if generation != self._plan_generation:
                    return
                self.waiting_for_plan = False
                if kind == "avoidance":
                    self.avoidance_state = "wait"
                else:
                    self.global_path = []
                self.publish_stop()
            rospy.logwarn("Planner returned an empty %s path", kind)
            return

        raw_points = poses_to_xy(path)
        processed = self.process_planned_path(raw_points)
        if not processed:
            with self._lock:
                if generation != self._plan_generation:
                    return
                self.waiting_for_plan = False
                if kind == "avoidance":
                    self.avoidance_state = "wait"
                else:
                    self.global_path = []
                self.publish_stop()
            rospy.logwarn("Path post-processing returned no points")
            return
        with self._lock:
            if generation != self._plan_generation:
                return
            self.global_path = processed
            self.start_recovery = recovery
            self.recovery_suffix = processed if recovery is not None else []
            if recovery is not None:
                # Preserve the checked prefix. Smoothing/shortcutting is only
                # applied to the normal A* suffix, outside the inflated region.
                self.global_path = recovery.points + processed
            self.avoidance_state = "avoiding" if kind == "avoidance" else "normal"
            self.waiting_for_plan = False
            self.publish_path_points(self.global_path)
            if kind != "avoidance" or not self.dynamic_keep_moving_during_replan:
                self.reset_controllers()
        rospy.loginfo(
            "%s A* path accepted: raw=%d processed=%d",
            "Dynamic" if kind == "avoidance" else "Static",
            len(raw_points),
            len(processed),
        )

    def plan_start_recovery(self, client, request, generation):
        if (not self.start_recovery_enabled or not self.collision_check_enabled
                or self.planning_frame != self.robot_frame or self.costmap is None
                or not self.dynamic_layer.is_fresh()):
            return None, None
        with self._lock:
            if generation != self._plan_generation:
                return None, None
            self.recovery_planning = True
            if not self.is_dynamic_replan_pending():
                self.publish_stop()
        pose = self.get_robot_pose()
        if pose is None:
            return None, None
        costmap = self.costmap
        _, obstacles = self.dynamic_layer.point_snapshot()
        radius = self.geometry.planning_radius(costmap.info.resolution, dynamic=True)
        def inflated(x, y):
            grid = self.world_to_grid(costmap, x, y)
            return (grid is None or not self.grid_in_bounds(costmap, *grid)
                or costmap.data[grid[1]*costmap.info.width+grid[0]] != 0
                or any((x-ox)**2 + (y-oy)**2 <= radius**2 for ox, oy in obstacles))
        diagnostics = {}
        recovery = forward_exit(pose, self.collision_checker, obstacles,
            self.raw_map_checker(), inflated, self._measured_velocity,
            self.start_recovery_max_distance,
            speed=self.command_max_vx, diagnostics=diagnostics)
        if recovery is None:
            rospy.logwarn("Blocked start: no safe forward exit within %.2f m; %s",
                          self.start_recovery_max_distance, diagnostics)
            return None, None
        exit_request = GetPlanRequest()
        exit_request.start.header = request.start.header
        exit_request.start.pose.position.x = recovery.end.x
        exit_request.start.pose.position.y = recovery.end.y
        exit_request.start.pose.orientation.z = math.sin(pose[2] / 2.0)
        exit_request.start.pose.orientation.w = math.cos(pose[2] / 2.0)
        exit_request.goal = request.goal
        exit_request.tolerance = request.tolerance
        path = client(exit_request).plan
        if not path.poses:
            rospy.logwarn("Blocked start: forward exit has no A* route to goal")
            return None, None
        rospy.loginfo("Blocked start recovery: checked %.2f m forward at %.3f m/s, then A* (%d poses); buffered_exit=%s",
                      recovery.distance, recovery.speed, len(path.poses), recovery.buffered_exit)
        return path, recovery

    def process_planned_path(self, raw_points: List[Point2D]) -> List[PathPoint]:
        # 后处理沿用 global_frame，并把会话内动态记忆纳入每一次碰撞查询。
        # Include the local memory snapshot even before a costmap callback
        # arrives, so shortcuts cannot erase a freshly planned obstacle detour.
        obstacle_points = self.dedup_dynamic_history_points()
        resolution = self.static_map.info.resolution if self.static_map is not None else 0.05
        radius = self.geometry.planning_radius(resolution, dynamic=True)
        def checker(x, y):
            if self.costmap is not None and self.is_costmap_occupied(x, y):
                return True
            return any((x-ox)**2 + (y-oy)**2 <= radius**2 for ox, oy in obstacle_points)
        return process_path(
            raw_points,
            enable_shortcut=self.path_enable_shortcut,
            enable_smoothing=self.path_enable_smoothing,
            resample_ds=self.path_resample_ds,
            smooth_weight_data=self.path_smooth_weight_data,
            smooth_weight_smooth=self.path_smooth_weight_smooth,
            smooth_max_iter=self.path_smooth_max_iter,
            smooth_tolerance=self.path_smooth_tolerance,
            collision_check_step=self.path_collision_check_step,
            is_occupied=checker,
        )

    def publish_path_points(self, points: List[PathPoint]) -> None:
        self.plan_pub.publish(
            path_to_ros_msg(points, self.global_frame, rospy.Time.now())
        )

    @staticmethod
    def choose_target_from_path(path, robot_x, robot_y, lookahead):
        for point in path:
            if math.hypot(point.x - robot_x, point.y - robot_y) >= lookahead:
                return point
        return path[-1] if path else None

    def costmap_cb(self, message: OccupancyGrid) -> None:
        if self._frame(message.header.frame_id) == self.global_frame:
            self.costmap = message

    def static_map_cb(self, message: OccupancyGrid) -> None:
        frame = self._frame(message.header.frame_id)
        if frame and frame != self.global_frame:
            rospy.logwarn_throttle(
                2.0,
                "Static map frame is '%s', expected '%s'; map ignored"
                % (frame, self.global_frame),
            )
            return
        self.static_map = message

    @staticmethod
    def world_to_grid(message, world_x, world_y):
        if message is None or message.info.resolution <= 0.0:
            return None
        yaw = yaw_from_quaternion(message.info.origin.orientation)
        dx, dy = world_x - message.info.origin.position.x, world_y - message.info.origin.position.y
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return (math.floor((cosine*dx + sine*dy) / message.info.resolution),
                math.floor((-sine*dx + cosine*dy) / message.info.resolution))

    @staticmethod
    def grid_in_bounds(message, grid_x, grid_y):
        return (
            message is not None
            and 0 <= grid_x < int(message.info.width)
            and 0 <= grid_y < int(message.info.height)
        )

    def is_costmap_occupied(self, world_x: float, world_y: float) -> bool:
        message = self.costmap
        grid = self.world_to_grid(message, world_x, world_y)
        if grid is None or not self.grid_in_bounds(message, *grid):
            return True
        index = grid[1] * int(message.info.width) + grid[0]
        if index < 0 or index >= len(message.data):
            return True
        return int(message.data[index]) < 0 or int(message.data[index]) >= self.path_occupied_threshold

    def is_static_map_occupied(self, world_x: float, world_y: float) -> bool:
        message = self.static_map
        if not self.dynamic_static_filter_enabled or message is None:
            return False
        grid = self.world_to_grid(message, world_x, world_y)
        if grid is None or not self.grid_in_bounds(message, *grid):
            return False
        resolution = max(float(message.info.resolution), 1e-3)
        radius = int(math.ceil(self.dynamic_static_filter_radius / resolution))
        width = int(message.info.width)
        height = int(message.info.height)
        for grid_y in range(max(0, grid[1] - radius), min(height, grid[1] + radius + 1)):
            for grid_x in range(max(0, grid[0] - radius), min(width, grid[0] + radius + 1)):
                index = grid_y * width + grid_x
                if (
                    index < len(message.data)
                    and int(message.data[index])
                    >= self.dynamic_static_filter_threshold
                ):
                    return True
        return False

    def get_front_obstacle_info(self):
        if not self.dynamic_layer.enabled or not self.dynamic_layer.is_fresh():
            return False, float("inf"), None, []
        local_points, map_points = self.dynamic_layer.point_snapshot()
        front_local, front_map = select_front_obstacles(
            local_points,
            map_points,
            self.dynamic_detect_x_min,
            self.dynamic_detect_x_max,
            self.dynamic_detect_y_abs,
        )
        filtered = [
            (local_point, map_point)
            for local_point, map_point in zip(front_local, front_map)
            if not self.is_static_map_occupied(*map_point)
        ]
        front_local = [pair[0] for pair in filtered]
        front_map = [pair[1] for pair in filtered]
        cluster = self.dynamic_layer.largest_cluster_indices(front_local)
        if cluster:
            front_local = [front_local[index] for index in cluster]
            front_map = [front_map[index] for index in cluster]
        if len(front_local) < self.dynamic_min_obstacle_points:
            return False, float("inf"), None, []
        center = (
            sum(point[0] for point in front_map) / len(front_map),
            sum(point[1] for point in front_map) / len(front_map),
        )
        return True, min(point[0] for point in front_local), center, front_map

    def update_obstacle_confirmation(self, has_obstacle, center) -> bool:
        if not has_obstacle or center is None:
            self.candidate_obstacle_center = None
            self.candidate_obstacle_count = 0
            return False
        if self.candidate_obstacle_center is None:
            self.candidate_obstacle_center = center
            self.candidate_obstacle_count = 1
        else:
            distance = math.hypot(
                center[0] - self.candidate_obstacle_center[0],
                center[1] - self.candidate_obstacle_center[1],
            )
            if distance <= self.dynamic_center_stable_dist:
                alpha = 0.35
                self.candidate_obstacle_center = (
                    (1.0 - alpha) * self.candidate_obstacle_center[0]
                    + alpha * center[0],
                    (1.0 - alpha) * self.candidate_obstacle_center[1]
                    + alpha * center[1],
                )
                self.candidate_obstacle_count += 1
            else:
                self.candidate_obstacle_center = center
                self.candidate_obstacle_count = 1
        return self.candidate_obstacle_count >= self.dynamic_confirm_frames

    def update_dynamic_history(self) -> None:
        # This is a projection of session memory, not a timed front-only history.
        with self._lock:
            generation = self._memory_generation
            _, memory_points = self.dynamic_layer.point_snapshot()
        resolution = self.dynamic_history_publish_resolution
        cells = {}
        static_cache = {}
        for x, y in memory_points:
            key = (math.floor(x/resolution), math.floor(y/resolution))
            if key in cells:
                continue
            map_key = self.world_to_grid(self.static_map, x, y)
            if map_key not in static_cache:
                static_cache[map_key] = self.is_static_map_occupied(x, y)
            if not static_cache[map_key]:
                cells[key] = (x, y)
        with self._lock:
            if generation == self._memory_generation:
                self.dynamic_history = [(self.now_sec(), x, y) for x, y in cells.values()]

    def dedup_dynamic_history_points(self) -> List[Point2D]:
        with self._lock:
            return [(x, y) for _, x, y in self.dynamic_history]

    def publish_dynamic_obstacle_overlay(self):
        with self._lock:
            message = PoseArray()
            message.header.frame_id = self.global_frame
            message.header.stamp = rospy.Time.now()
            for _, x, y in self.dynamic_history:
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.orientation.w = 1.0
                message.poses.append(pose)
            self.overlay_sent_stamp = message.header.stamp
            self._last_overlay_wall = time.monotonic()
            self.dynamic_points_pub.publish(message)
            return message.header.stamp

    def reset_avoidance_tracking(self) -> None:
        with self._lock:
            self.last_dynamic_replan_time = None
            self.last_dynamic_obstacle_center = None
            self.candidate_obstacle_center = None
            self.candidate_obstacle_count = 0

    def dynamic_replan_due(self, center) -> bool:
        if self.last_dynamic_replan_time is None:
            return True
        if self.now_sec() - self.last_dynamic_replan_time < self.dynamic_astar_replan_period:
            return False
        if center is None or self.last_dynamic_obstacle_center is None:
            return True
        return math.hypot(
            center[0] - self.last_dynamic_obstacle_center[0],
            center[1] - self.last_dynamic_obstacle_center[1],
        ) >= self.dynamic_obstacle_update_dist

    def request_dynamic_replan(self, robot_pose, center, force=False) -> bool:
        if not self.dynamic_history:
            return False
        if not force and not self.dynamic_replan_due(center):
            return False
        if not self.request_plan(robot_pose, "avoidance"):
            return False
        self.last_dynamic_replan_time = self.now_sec()
        if center is not None:
            self.last_dynamic_obstacle_center = center
        return True

    def should_exit_avoidance(self, robot_pose, current_has_obstacle) -> bool:
        if current_has_obstacle:
            return False
        if not self.dynamic_history:
            return True
        rx, ry, yaw = robot_pose
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        points = [(x, y) for _, x, y in self.dynamic_history
                  if math.hypot(x-rx, y-ry) <= self.dynamic_detect_x_max + 1.0]
        if not points:
            return True
        maximum_body_x = max(
            cosine * (x - rx) + sine * (y - ry) for x, y in points
        )
        return maximum_body_x < -self.dynamic_exit_clearance

    def is_dynamic_replan_pending(self) -> bool:
        with self._lock:
            return (
                self.waiting_for_plan
                # Inflation alone must not interrupt a physically safe command.
                # Continuing the old path requires the live guard during recovery.
                and (not self.recovery_planning or self.collision_check_enabled)
                and self.dynamic_keep_moving_during_replan
                and bool(self.global_path)
                and self.pending_plan_kind == "avoidance"
            )

    def control_loop(self, _event) -> None:
        started = time.monotonic()
        try:
            self._control_loop(_event)
        finally:
            elapsed = time.monotonic() - started
            if elapsed > 0.3:
                rospy.logwarn_throttle(2.0,
                    "Navigation control cycle took %.3f s (target %.3f s); "
                    "delayed cmd_vel may trigger the downstream watchdog"
                    % (elapsed, self.control_dt))

    def _control_loop(self, _event) -> None:
        # 控制周期的安全顺序：位姿/数据新鲜度 -> 紧急包络 -> 重规划状态
        # -> 路径跟踪 -> publish_cmd 内的扫掠和实测制动复核。
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.dynamic_layer.publish_markers()
            self.publish_stop()
            return
        self.update_measured_velocity()
        self.dynamic_layer.update_map_points(robot_pose)
        if self.dynamic_layer.projection_valid:
            self.update_dynamic_history()
            if time.monotonic() - self._last_overlay_wall >= 0.5:
                self.publish_dynamic_obstacle_overlay()
        if self.dynamic_layer.enabled and not self.dynamic_layer.is_fresh():
            if self.publish_degraded_radar_cmd(robot_pose):
                rospy.logwarn_throttle(
                    1.0,
                    "Radar input temporarily stale; continuing at %.0f%% speed"
                    % (100.0 * self.dynamic_degraded_speed_scale),
                )
                return
            self.publish_collision_preview(robot_pose, CollisionCheckResult(
                False, [robot_pose], None, "obstacle input unavailable, stale or memory full"))
            self.publish_stop()
            return
        dynamic_replan_pending = self.is_dynamic_replan_pending()
        with self._lock:
            waiting = self.waiting_for_plan
            has_path = bool(self.global_path)
        if waiting and not dynamic_replan_pending:
            self.publish_stop()
            return
        if not has_path:
            self.publish_collision_preview(robot_pose, None)
            self.publish_stop()
            return
        if self.dynamic_layer.should_emergency_stop():
            self.avoidance_state = "wait"
            self.publish_collision_preview(robot_pose, CollisionCheckResult(
                False, [robot_pose], None, "emergency stop envelope"))
            self.publish_stop()
            if self.dynamic_avoidance_mode == "astar_replan" and self.dynamic_replan_due(None):
                self.request_dynamic_replan(robot_pose, None, force=True)
            rospy.logwarn_throttle(1.0, "Dynamic obstacle emergency stop")
            return

        if self.start_recovery is not None and self.follow_start_recovery(robot_pose):
            return

        if self.dynamic_avoidance_mode != "off":
            has_obstacle, minimum_x, center, front_points = (
                self.get_front_obstacle_info()
            )
        else:
            has_obstacle, minimum_x, center, front_points = (
                False,
                float("inf"),
                None,
                [],
            )

        if self.dynamic_avoidance_mode == "off":
            if self.avoidance_state != "normal":
                self.avoidance_state = "normal"
                self.reset_avoidance_tracking()
        elif self.dynamic_avoidance_mode == "stop":
            if has_obstacle:
                self.publish_stop()
                rospy.logwarn_throttle(1.0, "Dynamic obstacle detected; stop mode active")
                return
        elif self.dynamic_avoidance_mode == "astar_replan":
            confirmed = self.update_obstacle_confirmation(has_obstacle, center)
            if self.avoidance_state in ("avoiding", "wait") and self.should_exit_avoidance(
                robot_pose, has_obstacle
            ):
                self.avoidance_state = "normal"
                self.reset_avoidance_tracking()
                self.request_plan(robot_pose, "normal")
                return
            # A stopped vehicle must retry even when the remembered obstacle
            # center is unchanged. Proximity changes speed, not plan eligibility.
            recovering = self.avoidance_state == "wait"
            if confirmed or recovering:
                replan_center = None if recovering else center
                if self.request_dynamic_replan(robot_pose, replan_center):
                    dynamic_replan_pending = self.is_dynamic_replan_pending()
                    if not dynamic_replan_pending:
                        self.publish_stop()
                        return
                if recovering and not dynamic_replan_pending:
                    self.publish_stop()
                    return

        rx, ry, _ = robot_pose
        with self._lock:
            while len(self.global_path) > 1:
                first = self.global_path[0]
                if math.hypot(first.x - rx, first.y - ry) >= self.waypoint_tolerance:
                    break
                self.global_path.pop(0)
            path = list(self.global_path)
            generation = self._plan_generation
        if not path:
            self.publish_stop()
            return
        final = path[-1]
        if math.hypot(final.x - rx, final.y - ry) < self.goal_tolerance:
            with self._lock:
                if generation != self._plan_generation:
                    return
                self._plan_generation += 1
                self.global_path = []
                self.goal_pose = None
                self.waiting_for_plan = False
                self.start_recovery = None
                self.recovery_suffix = []
                self.recovery_planning = False
                self.pending_plan_kind = "normal"
                self.avoidance_state = "normal"
                self.reset_avoidance_tracking()
                self.publish_stop()
                self.publish_path_points([])
            rospy.loginfo("Goal reached; waiting for the next goal")
            return

        target = self.choose_target_from_path(
            path, rx, ry, self.lookahead_distance
        )
        with self._lock:
            if generation != self._plan_generation:
                return
            self.publish_path_points(path)
            controller = (self.pure_pursuit_controller if self.tracking_controller == "pure_pursuit"
                          else self.pid_controller)
            velocity_x, _, velocity_yaw = controller.compute(robot_pose, path, target, self.control_dt)
        if self.publish_cmd(velocity_x, 0.0, velocity_yaw, robot_pose, generation):
            self.publish_desired_direction(robot_pose, target, velocity_x, active=True)

    def follow_start_recovery(self, robot_pose):
        with self._lock:
            recovery = self.start_recovery
            generation = self._plan_generation
        if recovery is None:
            return False
        x, y, yaw = recovery.start
        dx, dy = robot_pose[0]-x, robot_pose[1]-y
        progress = dx*math.cos(yaw) + dy*math.sin(yaw)
        lateral = -dx*math.sin(yaw) + dy*math.cos(yaw)
        angle = math.atan2(math.sin(yaw-robot_pose[2]), math.cos(yaw-robot_pose[2]))
        remaining = recovery.distance - progress
        if (remaining <= 0.05 and abs(lateral) <= 0.08 and abs(angle) <= 0.25
                and self.recovery_suffix_clear(robot_pose)):
            with self._lock:
                if generation != self._plan_generation:
                    return True
                self.global_path = list(self.recovery_suffix)
                self.recovery_suffix = []
                self.start_recovery = None
                self.avoidance_state = "normal"
                self.reset_avoidance_tracking()
                # Controllers were idle during recovery; resume rate limiting
                # from the last guarded command rather than stale pre-recovery state.
                self.pid_controller.prev_cmd = self._last_command
                self.pure_pursuit_controller.prev_cmd = self._last_command
                self.publish_path_points(self.global_path)
            rospy.loginfo("Recovery handoff: current pose and A* suffix revalidated; continuing tracking")
            # Continue the same control cycle; the new command still passes
            # the current footprint and measured-braking checks in publish_cmd.
            return False
        if remaining <= 0.0 or abs(lateral) > 0.08 or abs(angle) > 0.25:
            with self._lock:
                if generation != self._plan_generation:
                    return True
                self.start_recovery = None
                rospy.loginfo("Navigation stop: recovery %s; replanning from measured pose",
                    "suffix unavailable or blocked" if remaining <= 0.0 else "tracking deviation")
                self.publish_stop()
                # Replan from the measured exit rather than jumping to an
                # old suffix after localization drift or a changing overlay.
                self.avoidance_state = "wait"
                self.request_plan(robot_pose, "avoidance")
            return True
        speed = recovery.speed
        turn = max(-0.08, min(0.08, angle - 0.8*lateral))
        if self.publish_cmd(speed, 0, turn, robot_pose, generation):
            self.publish_desired_direction(robot_pose, recovery.end, speed, active=True)
        return True

    def recovery_suffix_clear(self, robot_pose):
        suffix = list(self.recovery_suffix)
        costmap = self.costmap
        occupied = self.raw_map_checker()
        if not suffix or costmap is None or occupied is None or not self.dynamic_layer.is_fresh():
            return False
        _, obstacles = self.dynamic_layer.point_snapshot()
        radius = self.geometry.planning_radius(costmap.info.resolution, dynamic=True)
        def blocked(x, y):
            grid = self.world_to_grid(costmap, x, y)
            return (grid is None or not self.grid_in_bounds(costmap, *grid)
                or costmap.data[grid[1]*costmap.info.width+grid[0]] != 0
                or any((x-ox)**2 + (y-oy)**2 <= radius**2 for ox, oy in obstacles))
        points = [robot_pose[:2]] + [(p.x, p.y) for p in suffix]
        if not path_collision_free(points, blocked, min(0.025, costmap.info.resolution / 2)):
            return False
        return self.collision_checker.check(robot_pose, 0, 0, obstacles,
            current_velocity=self._measured_velocity, occupied=occupied).safe

    def publish_degraded_radar_cmd(self, robot_pose):
        """Continue briefly on the last path command during a radar dropout.

        This is bounded by a short age window and still runs the static map,
        remembered-obstacle and measured-braking collision checks. It never
        runs before a valid cloud has been received and never bypasses the
        emergency envelope.
        """
        if (not self.dynamic_degraded_motion_enabled
                or self.dynamic_layer.stale_age()
                    > self.dynamic_layer.timeout + self.dynamic_degraded_timeout):
            return False
        if self.dynamic_layer.should_emergency_stop():
            return False
        with self._lock:
            velocity_x, velocity_yaw = self._last_command
            generation = self._plan_generation
            if not self.global_path or (abs(velocity_x) < 1e-6
                                        and abs(velocity_yaw) < 1e-6):
                return False
        return self.publish_cmd(
            velocity_x * self.dynamic_degraded_speed_scale,
            0.0,
            velocity_yaw * self.dynamic_degraded_speed_scale,
            robot_pose,
            generation,
            allow_stale_dynamic=True,
        )

    def publish_cmd(self, velocity_x, velocity_y, velocity_yaw, robot_pose=None,
                    generation=None, allow_stale_dynamic=False) -> bool:
        # 所有最终指令必须经过定位、动态新鲜度、静态地图和车辆 footprint 检查。
        if (not all(math.isfinite(value) for value in (velocity_x, velocity_y, velocity_yaw))
                or abs(velocity_y) > 1e-9):
            self.publish_stop()
            return False
        # Independent downstream axis clamps change the turning radius. Check
        # and publish the same bounded command that the chassis will receive.
        velocity_x = max(-self.command_max_vx, min(self.command_max_vx, velocity_x))
        velocity_yaw = max(-self.command_max_wz, min(self.command_max_wz, velocity_yaw))
        robot_pose = robot_pose or self.get_robot_pose()
        if (robot_pose is None or
                (self.dynamic_layer.enabled and not self.dynamic_layer.is_fresh()
                 and not allow_stale_dynamic)):
            self.publish_stop()
            return False
        if self.obstacle_slowdown_enabled and self.dynamic_layer.enabled:
            local_points, _ = self.dynamic_layer.point_snapshot()
            checker = self.collision_checker
            scale = proximity_speed_scale(local_points, checker.front, checker.rear,
                checker.half_width, checker.margin, checker.obstacle_radius,
                self.obstacle_slowdown_distance, self.obstacle_min_speed_scale)
            velocity_x *= scale
            velocity_yaw *= scale
        if self.collision_check_enabled:
            _, points = self.dynamic_layer.point_snapshot()
            # Full memory stays in the map; only nearby points can intersect
            # the bounded prediction and braking trajectories checked here.
            motions = ((velocity_x, velocity_yaw), self._measured_velocity)
            travel = max(abs(v) * (self.collision_checker.prediction_time
                + self.collision_checker.reaction_time
                + 0.5 * max(abs(v)/self.collision_checker.linear_deceleration,
                            abs(w)/self.collision_checker.angular_deceleration))
                for v, w in motions)
            radius = travel + math.hypot(max(self.collision_checker.front, self.collision_checker.rear),
                                         self.collision_checker.half_width) + 1.0
            points = [(x, y) for x, y in points
                      if math.hypot(x-robot_pose[0], y-robot_pose[1]) <= radius]
            occupied = self.raw_map_checker()
            if self.collision_require_static_map and occupied is None:
                result = CollisionCheckResult(False, [robot_pose], None, "waiting for static map")
            else:
                result = self.collision_checker.check(robot_pose, velocity_x, velocity_yaw, points,
                    current_velocity=self._measured_velocity, occupied=occupied)
                if (self.collision_speed_reduction_enabled and not result.safe
                        and result.reason in ('dynamic_obstacle', 'static_obstacle')
                        and len(result.trajectory) > 1):
                    # Keep curvature and always check actual braking as well.
                    # Initial overlap, sensor failures and unsafe current motion
                    # cannot be solved by merely lowering a requested speed.
                    for scale in (0.75, 0.5, 0.25):
                        slower = self.collision_checker.check(robot_pose,
                            velocity_x * scale, velocity_yaw * scale, points,
                            current_velocity=self._measured_velocity, occupied=occupied)
                        if slower.safe:
                            rospy.logwarn_throttle(2.0,
                                "Collision speed reduction: %s at %s; vx %.3f -> %.3f m/s"
                                % (result.reason, result.collision_point,
                                   velocity_x, velocity_x * scale))
                            velocity_x *= scale
                            velocity_yaw *= scale
                            result = slower
                            break
            self.publish_collision_preview(robot_pose, result)
            if not result.safe:
                with self._lock:
                    if generation is not None and generation != self._plan_generation:
                        return False
                    self.publish_stop()
                    if self.dynamic_avoidance_mode == "astar_replan" and self.dynamic_replan_due(None):
                        self.avoidance_state = "wait"
                        self.request_dynamic_replan(robot_pose, result.collision_point, force=True)
                rospy.logwarn_throttle(1.0,
                    "Vehicle footprint blocked: %s; obstacle_map_xy=%s; checked_vx=%.3f wz=%.3f"
                    % (result.reason, result.collision_point, velocity_x, velocity_yaw))
                return False
        message = Twist()
        message.linear.x = float(velocity_x)
        message.linear.y = float(velocity_y)
        message.angular.z = float(velocity_yaw)
        with self._lock:
            localization_ready = (not self.require_localization or
                (self.localization_valid and self.localization_received is not None
                 and time.monotonic() - self.localization_received <= self.pose_timeout))
            if (not self.global_path or not localization_ready
                    or (generation is not None and generation != self._plan_generation)
                    or (self.dynamic_layer.enabled and not self.dynamic_layer.is_fresh()
                        and not allow_stale_dynamic)):
                self.publish_stop()
                return False
            self._last_command = (velocity_x, velocity_yaw)
            self.cmd_pub.publish(message)
        return True

    def update_measured_velocity(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.dynamic_layer.memory_frame,
                self.robot_frame, rospy.Time(0), rospy.Duration(0.02))
            p = transform.transform.translation
            pose = (p.x, p.y, yaw_from_quaternion(transform.transform.rotation))
            stamp = transform.header.stamp.to_sec()
            previous = self._last_motion_pose
            if previous is not None and 0 < stamp - previous[0] <= self.pose_timeout:
                dt = stamp - previous[0]
                old = previous[1]
                vx = ((pose[0]-old[0])*math.cos(old[2]) + (pose[1]-old[1])*math.sin(old[2])) / dt
                wz = math.atan2(math.sin(pose[2]-old[2]), math.cos(pose[2]-old[2])) / dt
                self._measured_velocity = (vx, wz)
            elif previous is None or stamp < previous[0] or stamp - previous[0] > self.pose_timeout:
                self._measured_velocity = self._last_command
            self._last_motion_pose = (stamp, pose)
        except Exception:
            self._measured_velocity = self._last_command

    def raw_map_checker(self):
        message = self.static_map
        if message is None:
            return None
        def occupied(x, y):
            grid = self.world_to_grid(message, x, y)
            if grid is None or not self.grid_in_bounds(message, *grid):
                return True
            index = grid[1] * message.info.width + grid[0]
            return (index >= len(message.data) or message.data[index] < 0
                    or message.data[index] >= self.path_occupied_threshold)
        occupied.resolution = float(message.info.resolution)
        return occupied

    def publish_collision_preview(self, pose, result):
        message = MarkerArray()
        poses = result.trajectory if result is not None else [pose]
        for marker_id, (name, samples) in enumerate((
                ("vehicle_footprint", [pose]), ("predicted_footprints", poses))):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns, marker.id = name, marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.018 if marker_id == 0 else 0.01
            marker.color.a = 1.0 if marker_id == 0 else 0.6
            if marker_id == 0:
                marker.color.b = 1.0
                marker.color.g = 0.6
            elif result is not None and not result.safe:
                marker.color.r = 1.0
            else:
                marker.color.g = 1.0
            for sample in samples:
                corners = self.collision_checker.footprint(sample)
                for i in range(4):
                    for x, y in (corners[i], corners[(i+1) % 4]):
                        marker.points.append(Point(x=x, y=y, z=0.12))
            marker.lifetime = rospy.Duration(0.5)
            message.markers.append(marker)
        obstacle = Marker()
        obstacle.header.frame_id = self.global_frame
        obstacle.header.stamp = rospy.Time.now()
        obstacle.ns, obstacle.id = "blocking_obstacle", 2
        obstacle.type = Marker.SPHERE
        obstacle.pose.orientation.w = 1.0
        obstacle.scale.x = obstacle.scale.y = obstacle.scale.z = 0.14
        obstacle.color.r, obstacle.color.a = 1.0, 1.0
        if result is not None and result.collision_point is not None:
            obstacle.action = Marker.ADD
            obstacle.pose.position.x, obstacle.pose.position.y = result.collision_point
            obstacle.pose.position.z = 0.15
        else:
            obstacle.action = Marker.DELETE
        obstacle.lifetime = rospy.Duration(0.5)
        message.markers.append(obstacle)
        self.collision_marker_pub.publish(message)
        self.collision_blocked_pub.publish(Bool(data=result is not None and not result.safe))

    def publish_desired_direction(
        self,
        robot_pose: Pose2D,
        target: Optional[PathPoint],
        velocity_x: float,
        active: bool = True,
    ) -> None:
        """Publish the current lookahead direction as an RViz arrow.

        The arrow follows the direction from the current robot pose to the
        controller's lookahead target.  Its colour distinguishes a forward
        command (green) from a reverse command (orange); an inactive command
        deletes the previous arrow so a stopped vehicle is not shown as
        moving.
        """
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "desired_motion"
        marker.id = 1
        if not active or target is None:
            marker.action = Marker.DELETE
            self.desired_direction_pub.publish(marker)
            return

        rx, ry, _ = robot_pose
        dx = float(target.x) - rx
        dy = float(target.y) - ry
        distance = math.hypot(dx, dy)
        if not math.isfinite(distance) or distance < 1e-3:
            marker.action = Marker.DELETE
            self.desired_direction_pub.publish(marker)
            return

        # Keep the visual readable for both a near endpoint and a long path.
        arrow_length = max(0.35, min(1.20, distance))
        scale = arrow_length / distance
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06  # shaft diameter
        marker.scale.y = 0.12  # head diameter
        marker.scale.z = 0.16  # head length
        marker.color.r = 1.0 if velocity_x < 0.0 else 0.1
        marker.color.g = 0.55 if velocity_x < 0.0 else 1.0
        marker.color.b = 0.05 if velocity_x < 0.0 else 0.1
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(max(0.2, 2.0 * self.control_dt))
        marker.points = [
            Point(x=rx, y=ry, z=0.16),
            Point(x=rx + dx * scale, y=ry + dy * scale, z=0.16),
        ]
        self.desired_direction_pub.publish(marker)

    def reset_controllers(self) -> None:
        self.pid_controller.reset()
        self.pure_pursuit_controller.reset()

    def publish_stop(self) -> None:
        with self._lock:
            self.reset_controllers()
            self._last_command = (0.0, 0.0)
            self.cmd_pub.publish(Twist())
            self.publish_desired_direction((0.0, 0.0, 0.0), None, 0.0, active=False)

    def shutdown(self) -> None:
        try:
            self.publish_stop()
            self.dynamic_layer.clear()
            self.dynamic_history = []
            self.publish_dynamic_obstacle_overlay()
        except Exception:
            pass


def main() -> None:
    rospy.init_node("path_follower_node")
    PathFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
