#!/usr/bin/env python3
"""Live 3D obstacle observations and session-only memory, independent of A*."""

import math
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from .obstacle_memory import ObstacleMemory
from .obstacle_processing import largest_cluster_indices, supported_obstacle_points, trajectory_cost


class DynamicObstacleLayer:
    """把点云投影到机器人/记忆/地图坐标，并维护会话级障碍物。

    点云的原始 frame 只用于 TF 查询；``memory_frame`` 是固定的连续世界
    坐标，障碍记忆在该坐标中累积，再分别投影到 ``robot_frame`` 做近场
    判断、投影到 ``global_frame`` 供 A* 和 RViz 使用。输入过期、TF 失败
    或内存溢出时由上层安全联锁停止运动。
    """
    def __init__(self, tf_buffer, robot_frame="base_link", global_frame="map"):
        self.tf_buffer = tf_buffer
        self.robot_frame = self._normalise_frame(robot_frame)
        self.global_frame = self._normalise_frame(global_frame)
        self.enabled = bool(rospy.get_param("~enable_dynamic_obstacles", False))
        self.cloud_topic = str(rospy.get_param("~dynamic_cloud_topic", "/fastlio2/body_cloud"))
        # FAST-LIO's frame named lidar is its continuous local world, not the sensor.
        self.memory_frame = self._normalise_frame(rospy.get_param("~dynamic_memory_frame", "lidar"))
        self.timeout = float(rospy.get_param("~dynamic_obstacle_timeout", 0.5))
        self.x_min = float(rospy.get_param("~dynamic_x_min", -1.5))
        self.x_max = float(rospy.get_param("~dynamic_x_max", 2.5))
        self.y_abs = float(rospy.get_param("~dynamic_y_abs", 1.5))
        self.z_min = float(rospy.get_param("~dynamic_z_min", -0.3))
        self.z_max = float(rospy.get_param("~dynamic_z_max", 1.5))
        self.ground_filter_enabled = bool(rospy.get_param("~dynamic_ground_filter_enabled", True))
        self.ground_z_max = float(rospy.get_param("~dynamic_ground_z_max", 0.12))
        self.point_step = max(1, int(rospy.get_param("~dynamic_point_step", 2)))
        self.max_points = max(10, int(rospy.get_param("~dynamic_max_points", 2400)))
        self.max_clear_rays = max(1, int(rospy.get_param("~dynamic_memory_max_clear_rays", 600)))
        self.max_range = float(rospy.get_param("~dynamic_max_range", 8.0))
        self.sensor_origin = np.asarray(rospy.get_param(
            "~dynamic_sensor_origin", [-0.011, -0.02329, 0.04412]), dtype=float)
        self.front = float(rospy.get_param("~footprint_front", 0.34))
        self.rear = float(rospy.get_param("~footprint_rear", 0.34))
        self.half_width = float(rospy.get_param("~footprint_half_width", 0.275))
        self.self_z_max = float(rospy.get_param("~dynamic_self_filter_z_max", 0.65))
        self.cluster_distance = float(rospy.get_param("~dynamic_cluster_distance", 0.35))
        self.support_radius = float(rospy.get_param("~dynamic_obstacle_support_radius", 0.15))
        self.support_min_points = int(rospy.get_param("~dynamic_obstacle_support_min_points", 3))
        self.collision_radius = float(rospy.get_param("~dynamic_collision_radius", 0.42))
        self.influence_distance = float(rospy.get_param("~dynamic_influence_distance", 0.85))
        self.use_emergency_stop = bool(rospy.get_param("~dynamic_emergency_stop", True))
        self.emergency_stop_mode = str(rospy.get_param("~emergency_stop_mode", "rectangle"))
        self.center_stop_radius = float(rospy.get_param("~dynamic_center_stop_radius", 0.40))
        self.emergency_x_min = float(rospy.get_param("~emergency_x_min", 0.0))
        self.emergency_x_max = float(rospy.get_param("~emergency_x_max", 0.45))
        self.emergency_y_abs = float(rospy.get_param("~emergency_y_abs", 0.38))
        self.resolution = float(rospy.get_param("~dynamic_memory_resolution", 0.10))
        self.clear_enabled = bool(rospy.get_param("~dynamic_memory_clear_enabled", True))
        self.memory = ObstacleMemory(
            resolution=self.resolution,
            max_voxels=int(rospy.get_param("~dynamic_memory_max_voxels", 60000)),
            clear_confirmations=int(rospy.get_param("~dynamic_memory_clear_confirmations", 1)),
            endpoint_margin=float(rospy.get_param("~dynamic_memory_endpoint_margin", 0.15)),
            free_confirmation_window=float(rospy.get_param("~dynamic_memory_free_confirmation_window", 1.0)),
            clear_neighbor_radius=float(rospy.get_param("~dynamic_memory_clear_neighbor_radius", 0.15)),
        )
        parameters = (self.timeout, self.max_range, self.front, self.rear, self.half_width,
                      self.support_radius)
        bounds = (self.x_min, self.x_max, self.y_abs, self.z_min, self.z_max,
                  self.ground_z_max, self.self_z_max)
        if (not all(math.isfinite(value) and value > 0 for value in parameters)
                or not all(math.isfinite(value) for value in bounds)
                or self.x_min >= self.x_max or self.y_abs <= 0 or self.z_min >= self.z_max
                or self.sensor_origin.shape != (3,) or not np.isfinite(self.sensor_origin).all()
                or not self.memory_frame):
            raise ValueError("Invalid obstacle memory geometry or timing parameters")
        if self.support_min_points < 2 or self.support_min_points > self.max_points:
            raise ValueError("Obstacle support count must be between 2 and dynamic_max_points")

        self._lock = threading.RLock()
        self._clear_generation = 0
        self.local_points = []
        self.map_points = []
        self.last_update_time = None
        self.last_received = None
        self.projection_valid = False
        self.input_error = None
        self._last_marker_wall = 0.0
        self._last_processed_stamp = rospy.Time()
        self.marker_pub = rospy.Publisher(
            str(rospy.get_param("~dynamic_marker_topic", "/dynamic_obstacles_markers")),
            MarkerArray, queue_size=1, latch=True)
        self.subscriber = None
        self.publish_markers(force=True)
        if self.enabled:
            self.subscriber = rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb,
                                               queue_size=1, buff_size=8 * 1024 * 1024)
        rospy.loginfo("Obstacle memory: enabled=%s frame=%s resolution=%.2f m (RAM only)",
                      self.enabled, self.memory_frame, self.resolution)

    @staticmethod
    def _normalise_frame(frame):
        return str(frame).strip().strip("/")

    def lookup(self, target, source, stamp):
        # tf2 的 lookup_transform(target, source) 返回 source 点在 target 中的表示。
        if target == source:
            return None
        return self.tf_buffer.lookup_transform(target, source, stamp, rospy.Duration(0.05))

    @staticmethod
    def transform_points(points, transform):
        # 四元数旋转后再加平移；无效四元数/结果一律抛错，不能默认为零障碍。
        points = np.asarray(points, dtype=float).reshape((-1, 3))
        if transform is None:
            return points.copy()
        q = transform.transform.rotation
        quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(norm) or norm < 1e-9:
            raise ValueError("Invalid TF quaternion")
        quaternion /= norm
        uv = np.cross(quaternion[:3], points)
        uuv = np.cross(quaternion[:3], uv)
        t = transform.transform.translation
        result = points + 2.0 * (quaternion[3] * uv + uuv) + np.array([t.x, t.y, t.z])
        if not np.isfinite(result).all():
            raise ValueError("Invalid TF translation")
        return result

    def cloud_cb(self, message):
        started = time.monotonic()
        # 只接收时间窗内的新扫描；时间倒退会锁存输入错误，等待人工清空会话。
        frame = self._normalise_frame(message.header.frame_id)
        stamp = message.header.stamp
        age = (rospy.Time.now() - stamp).to_sec()
        if not frame or stamp.is_zero() or not -0.05 <= age <= self.timeout:
            return
        with self._lock:
            generation = self._clear_generation
            if stamp < self._last_processed_stamp:
                self.input_error = "point cloud clock moved backwards; restart LIO and navigation"
                return
            if stamp == self._last_processed_stamp or self.input_error:
                return
        try:
            to_robot = self.lookup(self.robot_frame, frame, stamp)
            to_memory = self.lookup(self.memory_frame, frame, stamp)
            points = np.asarray(list(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True)), dtype=float).reshape((-1, 3))
            points = points[np.isfinite(points).all(axis=1)][::self.point_step]
            if not len(points):
                return
            distances = np.linalg.norm(points - self.sensor_origin, axis=1)
            points = points[(distances > 0.10) & (distances <= self.max_range)]
            if not len(points):
                return
            robot_points = self.transform_points(points, to_robot)
            # 先在机器人坐标中过滤车体自身、地面和感兴趣区域，再写入全局记忆。
            inside_body = ((robot_points[:, 0] >= -self.rear)
                           & (robot_points[:, 0] <= self.front)
                           & (np.abs(robot_points[:, 1]) <= self.half_width)
                           & (robot_points[:, 2] <= self.self_z_max))
            points = points[~inside_body]
            robot_points = robot_points[~inside_body]
            memory_points = self.transform_points(points, to_memory)
            origin = self.transform_points([self.sensor_origin], to_memory)[0]
            obstacle = ((robot_points[:, 0] >= self.x_min) & (robot_points[:, 0] <= self.x_max)
                        & (np.abs(robot_points[:, 1]) <= self.y_abs)
                        & (robot_points[:, 2] >= self.z_min) & (robot_points[:, 2] <= self.z_max))
            if self.ground_filter_enabled:
                obstacle &= robot_points[:, 2] > self.ground_z_max
            # Crop before bounding obstacle work, so distant floor returns
            # cannot consume the sampling budget for nearby obstacles.
            hits = memory_points[obstacle]
            if len(hits) > self.max_points:
                hits = hits[np.linspace(0, len(hits) - 1, self.max_points, dtype=int)]
            # Filter new occupancy only. All actual returns still bound clearing rays.
            hits = supported_obstacle_points(hits, self.support_radius, self.support_min_points)
            # Outside endpoints never mark obstacles, but their real rays can
            # still prove that a previously observed obstacle has disappeared.
            rays = memory_points
            if len(rays) > self.max_clear_rays:
                rays = rays[np.linspace(0, len(rays) - 1, self.max_clear_rays, dtype=int)]
            with self._lock:
                if generation != self._clear_generation:
                    return
                if not self.memory.observe(origin, rays if self.clear_enabled else [],
                                           hits, stamp.to_sec(), protected_endpoints_xyz=memory_points):
                    return
                self.last_update_time = stamp
                self._last_processed_stamp = stamp
                self.last_received = time.monotonic()
            elapsed = time.monotonic() - started
            if elapsed > self.timeout * .5:
                rospy.logwarn_throttle(2.0,
                    "Obstacle cloud processing took %.3f s; completed scan age=%.3f s (timeout=%.3f s)"
                    % (elapsed, (rospy.Time.now()-stamp).to_sec(), self.timeout))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Obstacle observation rejected: %s" % exc)

    def is_fresh(self):
        """只有最近接收时间和消息时间都未超时，动态数据才可放行运动。"""
        with self._lock:
            if (not self.enabled or self.last_update_time is None or self.last_received is None
                    or self.input_error or self.memory.overflowed or not self.projection_valid):
                return False
            receipt_age = time.monotonic() - self.last_received
            scan_age = (rospy.Time.now() - self.last_update_time).to_sec()
            fresh = receipt_age <= self.timeout and -0.05 <= scan_age <= self.timeout
            if not fresh:
                rospy.logwarn_throttle(1.0,
                    "Obstacle input stale: scan_age=%.3f s receipt_age=%.3f s limit=%.3f s; motion stopped"
                    % (scan_age, receipt_age, self.timeout))
            return fresh

    def stale_age(self):
        """Age of the newest valid cloud, or infinity before first reception."""
        with self._lock:
            if (self.last_update_time is None or self.last_received is None
                    or self.input_error or self.memory.overflowed
                    or not self.projection_valid):
                return float("inf")
            receipt_age = time.monotonic() - self.last_received
            scan_age = (rospy.Time.now() - self.last_update_time).to_sec()
            return max(receipt_age, scan_age)

    def point_snapshot(self):
        with self._lock:
            return list(self.local_points), list(self.map_points)

    def update_map_points(self, robot_pose=None):
        if not self.enabled:
            return
        with self._lock:
            generation = self._clear_generation
            snapshot = self.memory.snapshot()
        try:
            to_map = self.lookup(self.global_frame, self.memory_frame, rospy.Time(0))
            to_robot = self.lookup(self.robot_frame, self.memory_frame, rospy.Time(0))
            points = np.asarray([row[:3] for row in snapshot], dtype=float).reshape((-1, 3))
            mapped = self.transform_points(points, to_map)
            local = self.transform_points(points, to_robot)
            with self._lock:
                if generation != self._clear_generation:
                    return
                self.local_points = [(float(x), float(y)) for x, y, _ in local]
                self.map_points = [(float(x), float(y)) for x, y, _ in mapped]
                self.projection_valid = True
        except Exception as exc:
            with self._lock:
                if generation != self._clear_generation:
                    return
                self.projection_valid = False
            rospy.logwarn_throttle(2.0, "Obstacle memory transform unavailable: %s" % exc)
        self.publish_markers()

    def clear(self):
        with self._lock:
            # Work already doing TF or point processing belongs to the old session.
            self._clear_generation += 1
            self.memory.clear()
            self.local_points = []
            self.map_points = []
            self.last_update_time = None
            self.last_received = None
            self.projection_valid = False
            self.input_error = None
            self._last_processed_stamp = rospy.Time()
        self.publish_markers(force=True)

    def publish_markers(self, force=False):
        with self._lock:
            if not force and time.monotonic() - self._last_marker_wall < 0.2:
                return
            generation = self._clear_generation
            snapshot = self.memory.snapshot()
            overflow = self.memory.overflowed
        stamp = rospy.Time.now()
        recent, remembered = [], []
        for x, y, z, seen in snapshot:
            target = recent if 0 <= stamp.to_sec() - seen <= self.timeout else remembered
            target.append(Point(x=x, y=y, z=z))
        message = MarkerArray()
        message.markers.append(self.observation_box_marker(stamp))
        for marker_id, (name, points, color) in enumerate((
                ("observed_obstacles", recent, (0.15, 0.95, 0.35)),
                ("remembered_obstacles", remembered, (1.0, 0.55, 0.05)))):
            marker = Marker()
            marker.header.frame_id = self.memory_frame
            marker.header.stamp = stamp
            marker.ns, marker.id = name, marker_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD if points else Marker.DELETE
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = self.resolution
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.8
            marker.points = points
            message.markers.append(marker)
        with self._lock:
            if generation != self._clear_generation:
                return
            self._last_marker_wall = time.monotonic()
            self.marker_pub.publish(message)
        if overflow:
            rospy.logerr_throttle(2.0, "Obstacle memory full: motion blocked; inspect scene before explicit reset")

    def observation_box_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = stamp
        marker.ns = "observation_box"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD if self.enabled else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 0.8, 1.0, 0.7)
        bottom = max(self.z_min, self.ground_z_max) if self.ground_filter_enabled else self.z_min
        corners = [(x, y, z) for z in (bottom, self.z_max)
                   for x, y in ((self.x_min, -self.y_abs), (self.x_max, -self.y_abs),
                                (self.x_max, self.y_abs), (self.x_min, self.y_abs))]
        for a, b in ((0, 1), (1, 2), (2, 3), (3, 0),
                     (4, 5), (5, 6), (6, 7), (7, 4),
                     (0, 4), (1, 5), (2, 6), (3, 7)):
            for index in (a, b):
                x, y, z = corners[index]
                marker.points.append(Point(x=x, y=y, z=z))
        return marker

    def should_emergency_stop(self):
        """检查机器人前方紧急包络；数据不新鲜时由控制器执行停机策略。"""
        if not self.enabled or not self.use_emergency_stop or not self.is_fresh():
            return False
        points, _ = self.point_snapshot()
        if self.emergency_stop_mode in ("center_envelope", "center", "point"):
            return any(x >= self.emergency_x_min and x*x + y*y <= self.center_stop_radius**2
                       for x, y in points)
        return any(self.emergency_x_min <= x <= self.emergency_x_max
                   and abs(y) <= self.emergency_y_abs for x, y in points)

    def largest_cluster_indices(self, points):
        return largest_cluster_indices(points, self.cluster_distance)

    def trajectory_cost(self, trajectory):
        _, points = self.point_snapshot()
        return trajectory_cost(trajectory, points, self.collision_radius, self.influence_distance)
