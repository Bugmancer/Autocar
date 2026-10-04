#!/usr/bin/env python3
"""两种跟踪策略共用的 ROS1 导航运行层，负责目标、重规划和运动安全检查。

本层只输出 ``/cmd_vel``；速度单位转换、底盘协议和通信看门狗由底盘包负责。
新增跟踪策略应通过组合接入，不能绕过本层的最终命令检查。
"""

import math
import threading
import time
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Bool, Time as TimeMessage
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from fw_mid_common_utils import Point2D, Pose2D, yaw_from_quaternion
from fw_mid_common_utils.collision_geometry import CollisionGeometry

from .dynamic_obstacle_layer import DynamicObstacleLayer
from .follower_parameters import load_parameters
from .follower_visualization import publish_collision_preview, publish_desired_direction
from .footprint_collision import FootprintCollisionChecker, CollisionCheckResult
from .obstacle_processing import proximity_speed_scale, select_front_obstacles
from .path_processing import PathPoint, path_to_ros_msg, poses_to_xy, process_path, path_collision_free
from .start_recovery import forward_exit


class FollowerRuntime:
    """局部跟踪主节点：接收 map 路径，经过 TF/障碍门控后发布 ``cmd_vel``。

    全局路径使用 ``global_frame``；动态记忆保存在 ``memory_frame`` 后投影
    到该帧。控制器读取机器人在全局帧的位姿，输出仅限线速度 x 与角速度 z。
    定位、静态地图或扫掠碰撞检查失败时统一停车；点云过期仅允许进入
    显式配置的短时降速窗口，该窗口仍使用已有障碍记忆检查制动过程。
    """
    def __init__(self, tracking_factory) -> None:
        # ROS 回调、控制定时器和规划线程共享状态；耗时计算不长期占用此锁。
        # 规划代次隔离旧请求，记忆代次隔离人工清空前的旧投影结果。
        self._lock = threading.RLock()
        self._plan_generation = 0
        self._memory_generation = 0
        load_parameters(self)
        self.localization_valid = False
        self.localization_received = None
        self.overlay_applied_stamp = rospy.Time()
        self.overlay_sent_stamp = rospy.Time()
        self._last_overlay_wall = 0.0
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
        # 策略必须先完整构造，再注册点云/目标回调，最后才启动控制定时器。
        self.tracking = tracking_factory(self)
        self.dynamic_layer = DynamicObstacleLayer(self.tf_buffer, self.robot_frame, self.global_frame)

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=10
        )
        # 方向箭头与路径分开发布：经典策略显示前视方向，APF 显示滤波后的合力方向。
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
        rospy.on_shutdown(self.shutdown)
        self.publish_dynamic_obstacle_overlay()
        self.publish_stop()
        self.control_timer = rospy.Timer(
            rospy.Duration(self.control_dt), self.control_loop
        )
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
        # 使用单调墙钟判断消息是否持续到达，避免 ROS 仿真时钟暂停掩盖输入中断。
        with self._lock:
            self.localization_valid = message.data
            self.localization_received = time.monotonic()

    def overlay_ack_cb(self, message):
        with self._lock:
            self.overlay_applied_stamp = message.data

    def clear_memory_cb(self, _request):
        # 清空记忆同时取消目标并使旧工作线程失效，空障碍层不能让旧轨迹自动续行。
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
        """只接收全局坐标系目标；替换目标时先停车，再异步请求新路径。"""
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
        """启动一次规划请求；返回值表示请求已发出，不代表已经得到可行路径。"""
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
        """后台获取路径，处理返回结果前核对代次，防止覆盖新目标或人工取消。"""
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
            # 上述超时只限制服务出现时间，不限制同步调用返回；服务阻塞会占用本线程。
            # 若增加请求截止时间，还必须用代次丢弃迟到响应，不能接受超时后的旧路径。
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
                # 已验证的恢复前缀保持原样；仅对膨胀区外的普通 A* 后缀做平滑和捷径处理。
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
        """起点位于膨胀区时，寻找受检的前行出口，再验证出口到目标的完整路径。"""
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
        # 即使最新代价图尚未回调，也加入本地记忆快照，避免捷径重新穿过刚绕开的障碍。
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
        # 原始静态地图用于车身检查，不能误用已按车身半径膨胀的代价图重复扩大障碍。
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
        """先逆旋转地图原点再取格索引；负坐标也需使用 floor，不能直接截断为整数。"""
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
        """识别静态地图已有的障碍，用于动态重规划去重，不替代最终车身占用检查。"""
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
        """取前方最大有效障碍簇；局部点和全局点必须成对过滤，保持索引一致。"""
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
        """连续观测到空间位置稳定的障碍后才触发常规重规划；紧急停车不等确认。"""
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
        # 此处保存整个会话记忆的投影；不能按观测年龄或是否位于车前方直接删除。
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
        """发布完整障碍快照及当前 ROS 时间戳，供 A* 回传动态层应用确认。"""
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
        # 周期门限限制请求频率；障碍中心无变化时可跳过常规请求，停车恢复另行强制重试。
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
        """当前前方无障碍，且附近记忆都已落到车后方时，才退出本次避障状态。"""
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
                # 仅落入保守膨胀区不必立即中断安全运动；恢复规划中续行必须启用实时碰撞检查。
                and (not self.recovery_planning or self.collision_check_enabled)
                and self.dynamic_keep_moving_during_replan
                and bool(self.global_path)
                and self.pending_plan_kind == "avoidance"
            )

    def control_loop(self, _event) -> None:
        """执行公共控制周期，再调用策略附加处理；耗时日志用于发现看门狗触发原因。"""
        started = time.monotonic()
        try:
            self._control_loop(_event)
            with self._lock:
                self.tracking.after_control_cycle()
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
            # 等待避障的停车状态也需重试；不能因为记忆障碍中心没变化而永久停止重规划。
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

        rx, ry, yaw = robot_pose
        with self._lock:
            # 保留最后一个目标点，舍去已接近或位于当前车身朝向后方的路径点。
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            while len(self.global_path) > 1:
                first = self.global_path[0]
                dx = first.x - rx
                dy = first.y - ry
                distance = math.hypot(dx, dy)
                forward_projection = dx * cos_yaw + dy * sin_yaw
                if forward_projection < 0 or distance < self.waypoint_tolerance:
                    self.global_path.pop(0)
                else:
                    break
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
            velocity_x, _, velocity_yaw = self.tracking.compute(
                robot_pose, path, target, self.control_dt)
        if self.publish_cmd(velocity_x, 0.0, velocity_yaw, robot_pose, generation):
            self.publish_desired_direction(robot_pose, target, velocity_x, active=True)

    def follow_start_recovery(self, robot_pose):
        """执行受限前行恢复；返回 False 时可在同一周期转入普通跟踪。"""
        self.tracking.before_recovery()
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
                # 恢复期间跟踪器未参与控制；从实际发布速度接续限加速度，避免沿用旧状态跳变。
                self.tracking.finish_recovery(self._last_command)
                self.publish_path_points(self.global_path)
            rospy.loginfo("Recovery handoff: current pose and A* suffix revalidated; continuing tracking")
            # 同一周期继续跟踪，新命令仍须检查当前车身占用和实测速度的制动轨迹。
            return False
        if remaining <= 0.0 or abs(lateral) > 0.12 or abs(angle) > 0.35:
            with self._lock:
                if generation != self._plan_generation:
                    return True
                self.start_recovery = None
                rospy.loginfo("Navigation stop: recovery %s; replanning from measured pose",
                    "suffix unavailable or blocked" if remaining <= 0.0 else "tracking deviation")
                self.publish_stop()
                # 定位漂移或障碍变化后从实测位置重规划，不能直接跳入之前保存的后缀。
                self.avoidance_state = "wait"
                self.request_plan(robot_pose, "avoidance")
            return True
        speed = recovery.speed
        turn = max(-0.08, min(0.08, angle - 0.8*lateral))
        if self.publish_cmd(speed, 0, turn, robot_pose, generation):
            self.publish_desired_direction(robot_pose, recovery.end, speed, active=True)
        return True

    def recovery_suffix_clear(self, robot_pose):
        """恢复交接前复查当前位置到后缀的连接段，以及当前运动能否安全制动。"""
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
        """点云短时中断时，在限定时间内缩小上一条已发布的路径跟踪命令。

        必须已有有效观测和路径；静态地图、记忆障碍及实测制动检查仍然执行。
        注意动态层的紧急包络当前只检查新鲜数据，过期时返回 False；因此本分支
        实际依赖最终车身检查，不能宣称完整保留了额外配置的紧急停车距离。
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
        """采集策略诊断、执行公共检查，并以最终发布值同步策略限速状态。"""
        with self._lock:
            diagnostics = self.tracking.command_diagnostics()
        # 碰撞预测期间必须允许定位失效和目标取消回调执行。
        # 最终发布前在锁内重新检查状态，不能把整段耗时预测包进此锁。
        accepted = self._publish_guarded_cmd(
            velocity_x, velocity_y, velocity_yaw, robot_pose,
            generation, allow_stale_dynamic)
        with self._lock:
            self.tracking.command_published(accepted, velocity_x, velocity_yaw, diagnostics)
            return accepted

    def _publish_guarded_cmd(self, velocity_x, velocity_y, velocity_yaw, robot_pose=None,
                             generation=None, allow_stale_dynamic=False) -> bool:
        # 所有最终指令必须经过定位、动态新鲜度、静态地图和车辆 footprint 检查。
        if (not all(math.isfinite(value) for value in (velocity_x, velocity_y, velocity_yaw))
                or abs(velocity_y) > 1e-9):
            self.publish_stop()
            return False
        # 分别限制线速度/角速度会改变转弯半径；预测必须使用底盘最终收到的限幅值。
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
            # 全部记忆仍保留；这里只裁剪不可能进入本次预测/制动扫掠范围的远点以降低开销。
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
                    # 同比例降速保持曲率，且每个候选都用原实测速度重验制动。
                    # 当前已重叠、输入失效或当前运动无法安全制动时，降低候选速度不能放行。
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
        """在连续里程计坐标系内差分估速，避免全局重定位跳变被误判成车速。

        差分不可用时回退到上一条发布命令；这只是估计，不能替代真实轮速反馈。
        """
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
        """固定本次静态地图快照；越界、未知和非法索引均视为占用。"""
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
        publish_collision_preview(self, pose, result)

    def publish_desired_direction(self, robot_pose, target, velocity_x, active=True):
        target, active = self.tracking.desired_direction(robot_pose, target, active)
        publish_desired_direction(self, robot_pose, target, velocity_x, active)

    def reset_controllers(self) -> None:
        self.tracking.reset()

    def publish_stop(self) -> None:
        # 同时清除控制器积分/速度/方向状态；只发零速而不复位会影响之后的起步。
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
