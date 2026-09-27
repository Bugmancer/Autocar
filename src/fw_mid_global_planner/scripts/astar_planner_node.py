#!/usr/bin/env python3
"""ROS1 A* global planner with a dynamic obstacle overlay.

This node is a rospy port of the FW-mid ROS2 ``global_planner`` node.  The
algorithm and private interface names are intentionally kept compatible so a
ROS1 local planner can call ``/astar_planner_node/get_plan`` without an
additional bridge.
"""

import heapq
import math
import os
import threading

import cv2
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from std_msgs.msg import Time as TimeMessage
from visualization_msgs.msg import Marker, MarkerArray
from fw_mid_common_utils.collision_geometry import CollisionGeometry


class AStarPlanner:
    """Load a static map and expose A* planning through a ROS1 service."""

    def __init__(self):
        self.map_yaml_path = str(rospy.get_param("~map_yaml_path", ""))
        self.map_frame = self._normalise_frame(
            rospy.get_param("~map_frame", "map")
        )
        self.allow_unknown = bool(rospy.get_param("~allow_unknown", False))

        self.geometry = CollisionGeometry.from_params(lambda name, default: rospy.get_param('~' + name, default))

        self.cluster_px = int(rospy.get_param("~cluster_radius_px", 15))
        self.enable_forbidden_zones = bool(
            rospy.get_param("~enable_forbidden_zones", False)
        )
        self.min_cluster_area_px = int(
            rospy.get_param("~min_cluster_area_px", 300)
        )
        self.max_cluster_area_px = int(
            rospy.get_param("~max_cluster_area_px", 3000)
        )
        self.max_forbidden_width_m = float(
            rospy.get_param("~max_forbidden_width_m", 2.0)
        )
        self.max_forbidden_height_m = float(
            rospy.get_param("~max_forbidden_height_m", 2.0)
        )

        self.enable_dynamic_overlay = bool(
            rospy.get_param("~enable_dynamic_overlay", True)
        )
        self.dynamic_obstacle_topic = str(
            rospy.get_param(
                "~dynamic_obstacle_topic",
                "/local_planner/dynamic_obstacle_points",
            )
        )
        self.dynamic_overlay_timeout = float(
            rospy.get_param("~dynamic_overlay_timeout", 0.0)
        )

        self.map_pub = rospy.Publisher(
            "~map", OccupancyGrid, queue_size=1, latch=True
        )
        self.costmap_pub = rospy.Publisher(
            "~costmap", OccupancyGrid, queue_size=10, latch=True
        )
        self.path_pub = rospy.Publisher("~visual_plan", Path, queue_size=1)
        self.overlay_stamp_pub = rospy.Publisher("~dynamic_overlay_stamp", TimeMessage,
                                                 queue_size=1, latch=True)
        self.zone_pub = rospy.Publisher(
            "~forbidden_zones", MarkerArray, queue_size=1
        )

        self.forbidden_rects = []
        self.dynamic_points_map = []
        self.last_dynamic_overlay_time = None
        self._dynamic_lock = threading.RLock()

        self.grid_map = None
        self.occupancy_map = None
        self.static_costmap = None
        self.current_costmap = None
        self.resolution = None
        self.origin = [0.0, 0.0, 0.0]
        self.width = 0
        self.height = 0

        # Match the ROS2 node: fail startup when the configured map is invalid.
        self.load_and_process_map(self.map_yaml_path)

        self.plan_srv = rospy.Service("~get_plan", GetPlan, self.plan_cb)
        self.dynamic_points_sub = rospy.Subscriber(
            self.dynamic_obstacle_topic, PoseArray, self.dynamic_points_cb, queue_size=1)

        rospy.Timer(rospy.Duration(2.0), self.publish_visuals)
        rospy.loginfo(
            "A* planner ready: dynamic_overlay=%s topic=%s "
            "dynamic inflation=%.3f m static inflation=%.3f m timeout=%.2f s",
            self.enable_dynamic_overlay,
            self.dynamic_obstacle_topic,
            self.dynamic_inflation_radius,
            self.static_inflation_radius,
            self.dynamic_overlay_timeout,
        )

    @staticmethod
    def _normalise_frame(frame):
        frame = str(frame).strip().strip("/")
        return frame or "map"

    # ------------------------------------------------------------------
    # Map loading and costmap construction
    # ------------------------------------------------------------------

    def load_and_process_map(self, yaml_path):
        if not yaml_path:
            raise RuntimeError("~map_yaml_path is empty")
        if not os.path.isfile(yaml_path):
            raise RuntimeError("Map YAML does not exist: {}".format(yaml_path))

        with open(yaml_path, "r", encoding="utf-8") as map_file:
            map_data = yaml.safe_load(map_file)
        if not isinstance(map_data, dict):
            raise RuntimeError("Map YAML must contain a mapping")

        try:
            self.resolution = float(map_data["resolution"])
            self.origin = list(map_data["origin"])
            image_name = str(map_data["image"])
            negate = int(map_data.get("negate", 0))
            occupied_thresh = float(map_data.get("occupied_thresh", 0.65))
            free_thresh = float(map_data.get("free_thresh", 0.196))
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("Map YAML has invalid map metadata") from exc

        if self.resolution <= 0.0 or len(self.origin) < 2:
            raise RuntimeError("Map resolution/origin is invalid")
        if negate not in (0, 1):
            raise RuntimeError("Map negate must be 0 or 1")
        if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
            raise RuntimeError("Map free/occupied thresholds are invalid")
        if len(self.origin) < 3:
            self.origin.append(0.0)

        image_path = os.path.join(os.path.dirname(yaml_path), image_name)
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError("Failed to read map image: {}".format(image_path))

        # OccupancyGrid and the source planner use a bottom-left grid origin.
        image = cv2.flip(image, 0)
        self.height, self.width = image.shape
        image_probability = image.astype(np.float32) / 255.0
        occupancy_probability = (
            image_probability if negate else 1.0 - image_probability
        )
        occupied = occupancy_probability > occupied_thresh
        free = occupancy_probability < free_thresh
        unknown = ~(occupied | free)

        self.occupancy_map = np.zeros_like(image, dtype=np.int8)
        self.occupancy_map[occupied] = 100
        self.occupancy_map[unknown] = -1

        # Unknown space is blocked by default. It may be exposed only through
        # an explicit parameter for maps that were intentionally cropped.
        self.grid_map = np.zeros_like(image, dtype=np.uint8)
        self.grid_map[occupied] = 100
        if not self.allow_unknown:
            self.grid_map[unknown] = 100

        if self.enable_forbidden_zones:
            self._find_forbidden_zones()

        self.static_inflation_radius = self.geometry.planning_radius(self.resolution)
        self.dynamic_inflation_radius = self.geometry.planning_radius(self.resolution, dynamic=True)
        kernel = self.inflation_kernel(self.static_inflation_radius, self.resolution)
        self.static_costmap = cv2.dilate(self.grid_map, kernel,
            borderType=cv2.BORDER_CONSTANT, borderValue=100)
        self.current_costmap = self.static_costmap.copy()

    @staticmethod
    def inflation_kernel(radius, resolution):
        cells = int(math.ceil(radius / resolution))
        y, x = np.ogrid[-cells:cells+1, -cells:cells+1]
        # Round only the array extent; geometry already includes cell uncertainty.
        return (x*x + y*y <= (radius / resolution)**2 + 1e-12).astype(np.uint8)

    def _find_forbidden_zones(self):
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (max(1, self.cluster_px), max(1, self.cluster_px)),
        )
        dilated = cv2.dilate(self.grid_map, kernel)
        contours, _ = cv2.findContours(
            dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area_px = cv2.contourArea(contour)
            if area_px < self.min_cluster_area_px:
                continue
            if area_px > self.max_cluster_area_px:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            width_m = width * self.resolution
            height_m = height * self.resolution
            if width_m > self.max_forbidden_width_m:
                continue
            if height_m > self.max_forbidden_height_m:
                continue

            wx_min, wy_min = self.grid_to_world(x, y)
            wx_max, wy_max = self.grid_to_world(x + width, y + height)
            self.forbidden_rects.append(
                (
                    min(wx_min, wx_max),
                    min(wy_min, wy_max),
                    max(wx_min, wx_max),
                    max(wy_min, wy_max),
                )
            )

    # ------------------------------------------------------------------
    # Dynamic obstacle overlay
    # ------------------------------------------------------------------

    def dynamic_points_cb(self, msg):
        frame = self._normalise_frame(msg.header.frame_id)
        if frame != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "Dynamic obstacle frame is '{}'; expected '{}'. Ignoring.".format(
                    frame, self.map_frame
                ),
            )
            return

        points = [
            (float(pose.position.x), float(pose.position.y))
            for pose in msg.poses
            if math.isfinite(pose.position.x) and math.isfinite(pose.position.y)
        ]
        with self._dynamic_lock:
            self.dynamic_points_map = points
            self.last_dynamic_overlay_time = rospy.Time.now()
            self.current_costmap = self.build_working_costmap()
            costmap = self._create_grid_msg(self.current_costmap)

        # The acknowledgment follows applying the complete snapshot. Local
        # planning waits for it instead of guessing delivery time with a sleep.
        self.costmap_pub.publish(costmap)
        self.overlay_stamp_pub.publish(TimeMessage(data=msg.header.stamp))

        if points:
            rospy.loginfo_throttle(
                1.0,
                "Received {} dynamic obstacle points.".format(len(points)),
            )
        else:
            rospy.loginfo_throttle(1.0, "Dynamic obstacle overlay cleared.")

    def expire_dynamic_overlay_if_needed(self):
        with self._dynamic_lock:
            if not self.dynamic_points_map:
                return
            if self.dynamic_overlay_timeout <= 0.0:
                return
            if self.last_dynamic_overlay_time is None:
                self.dynamic_points_map = []
                return

            age = (
                rospy.Time.now() - self.last_dynamic_overlay_time
            ).to_sec()
            if age <= self.dynamic_overlay_timeout:
                return

            self.dynamic_points_map = []
            if self.static_costmap is not None:
                self.current_costmap = self.static_costmap.copy()
            rospy.logwarn_throttle(
                1.0, "Dynamic obstacle overlay expired and was cleared."
            )

    def build_working_costmap(self):
        """Return a fresh static costmap with current dynamic points added."""
        if self.static_costmap is None:
            return None

        self.expire_dynamic_overlay_if_needed()
        working = self.static_costmap.copy()
        if not self.enable_dynamic_overlay:
            return working

        with self._dynamic_lock:
            points = list(self.dynamic_points_map)
        if not points:
            return working

        seeds = np.zeros_like(working)
        for world_x, world_y in points:
            grid_x, grid_y = self.world_to_grid(world_x, world_y)
            if not self.grid_in_bounds(grid_x, grid_y):
                continue
            seeds[grid_y, grid_x] = 100
        kernel = self.inflation_kernel(self.dynamic_inflation_radius, self.resolution)
        return np.maximum(working, cv2.dilate(seeds, kernel))

    # ------------------------------------------------------------------
    # ROS service and A* implementation
    # ------------------------------------------------------------------

    def plan_cb(self, request):
        response = GetPlanResponse()
        start_frame = self._normalise_frame(request.start.header.frame_id)
        goal_frame = self._normalise_frame(request.goal.header.frame_id)
        if start_frame != self.map_frame or goal_frame != self.map_frame:
            rospy.logwarn(
                "A* requires start/goal in frame '%s' (got '%s'/'%s')",
                self.map_frame,
                start_frame,
                goal_frame,
            )
            return response
        start_x = request.start.pose.position.x
        start_y = request.start.pose.position.y
        goal_x = request.goal.pose.position.x
        goal_y = request.goal.pose.position.y
        rospy.loginfo(
            "Plan request: start=(%.2f, %.2f) goal=(%.2f, %.2f)",
            start_x,
            start_y,
            goal_x,
            goal_y,
        )

        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        if not self.grid_in_bounds(*start_grid):
            rospy.logwarn("A* start is outside the map")
            return response
        if not self.grid_in_bounds(*goal_grid):
            rospy.logwarn("A* goal is outside the map")
            return response

        costmap = self.build_working_costmap()
        if costmap is None:
            rospy.logwarn("A* costmap is not ready")
            return response
        self.current_costmap = costmap

        for label, grid, x, y in (
            ("start", start_grid, start_x, start_y),
            ("goal", goal_grid, goal_x, goal_y),
        ):
            if self._cell_blocked(grid, costmap):
                rospy.logwarn(
                    "A* %s (%.2f, %.2f) is blocked in the inflated costmap; "
                    "check the static map, robot clearance and dynamic obstacles",
                    label,
                    x,
                    y,
                )
                return response

        path_grid = self.a_star(start_grid, goal_grid, costmap)
        if not path_grid:
            rospy.logwarn("A* failed: no path found")
            return response

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = rospy.Time.now()

        # Keep obstacle corners; the follower shortcuts only after checking the
        # whole segment. Blind four-cell downsampling could cut into inflation.
        for grid_x, grid_y in path_grid:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y = self.grid_to_world(
                grid_x, grid_y
            )
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        response.plan = path_msg
        self.path_pub.publish(path_msg)
        rospy.loginfo("A* success: %d path poses", len(path_msg.poses))
        return response

    def a_star(self, start, goal, costmap):
        if self._cell_blocked(start, costmap) or self._cell_blocked(goal, costmap):
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        motions = (
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        )

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for delta_x, delta_y in motions:
                neighbor = (
                    current[0] + delta_x,
                    current[1] + delta_y,
                )
                if not self.grid_in_bounds(*neighbor):
                    continue
                if self._cell_blocked(neighbor, costmap):
                    continue

                # A diagonal step is valid only when both adjacent cardinal
                # cells are clear; otherwise a footprint could cut a corner.
                if delta_x and delta_y:
                    side_x = (current[0] + delta_x, current[1])
                    side_y = (current[0], current[1] + delta_y)
                    if self._cell_blocked(side_x, costmap):
                        continue
                    if self._cell_blocked(side_y, costmap):
                        continue

                world_x, world_y = self.grid_to_world(*neighbor)
                if self.is_in_forbidden_zone(world_x, world_y):
                    continue

                new_cost = g_score[current] + math.hypot(delta_x, delta_y)
                if neighbor not in g_score or new_cost < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = new_cost
                    heuristic = math.hypot(
                        goal[0] - neighbor[0], goal[1] - neighbor[1]
                    )
                    heapq.heappush(
                        open_set, (new_cost + heuristic, neighbor)
                    )
        return None

    def _cell_blocked(self, cell, costmap):
        grid_x, grid_y = cell
        if not self.grid_in_bounds(grid_x, grid_y):
            return True
        if costmap[grid_y, grid_x] > 50:
            return True
        world_x, world_y = self.grid_to_world(grid_x, grid_y)
        return self.is_in_forbidden_zone(world_x, world_y)

    # ------------------------------------------------------------------
    # Coordinate and message helpers
    # ------------------------------------------------------------------

    def grid_in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height

    def grid_to_world(self, grid_x, grid_y):
        yaw = float(self.origin[2]) if len(self.origin) > 2 else 0.0
        local_x = grid_x * self.resolution
        local_y = grid_y * self.resolution
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            self.origin[0] + cos_yaw * local_x - sin_yaw * local_y,
            self.origin[1] + sin_yaw * local_x + cos_yaw * local_y,
        )

    def world_to_grid(self, world_x, world_y):
        yaw = float(self.origin[2]) if len(self.origin) > 2 else 0.0
        delta_x = world_x - self.origin[0]
        delta_y = world_y - self.origin[1]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = cos_yaw * delta_x + sin_yaw * delta_y
        local_y = -sin_yaw * delta_x + cos_yaw * delta_y
        return (
            int(math.floor(local_x / self.resolution)),
            int(math.floor(local_y / self.resolution)),
        )

    def is_in_forbidden_zone(self, world_x, world_y):
        return any(
            x_min <= world_x <= x_max and y_min <= world_y <= y_max
            for x_min, y_min, x_max, y_max in self.forbidden_rects
        )

    def _create_grid_msg(self, data_array):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)
        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        origin_yaw = float(self.origin[2]) if len(self.origin) > 2 else 0.0
        msg.info.origin.orientation.z = math.sin(origin_yaw / 2.0)
        msg.info.origin.orientation.w = math.cos(origin_yaw / 2.0)
        msg.data = np.asarray(data_array, dtype=np.int8).flatten().tolist()
        return msg

    def publish_visuals(self, _event=None):
        marker_array = MarkerArray()
        stamp = rospy.Time.now()
        for marker_id, (x_min, y_min, x_max, y_max) in enumerate(
            self.forbidden_rects
        ):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = "forbidden_zones"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (x_min + x_max) / 2.0
            marker.pose.position.y = (y_min + y_max) / 2.0
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = abs(x_max - x_min)
            marker.scale.y = abs(y_max - y_min)
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.a = 0.4
            marker_array.markers.append(marker)

        self.zone_pub.publish(marker_array)
        if self.occupancy_map is not None:
            self.map_pub.publish(self._create_grid_msg(self.occupancy_map))
        if self.current_costmap is not None:
            self.costmap_pub.publish(self._create_grid_msg(self.current_costmap))


def main():
    rospy.init_node("astar_planner_node")
    try:
        AStarPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
