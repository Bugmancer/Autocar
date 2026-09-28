"""ROS-independent force and speed calculations for the v1 follower."""

import heapq
import math

from fw_mid_common_utils import normalize_angle


class ArtificialPotentialFieldController:
    """Two constant attractions, age-weighted repulsion and danger stop.

    Parameters and obstacle snapshots are supplied by the owning follower.
    Forces use map coordinates. Clearance uses the padded vehicle rectangle;
    entering the configured danger clearance returns a zero command. The
    parent still performs its complete swept collision check.
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
        """Follow downstream reductions without losing sub-deadband ramps."""
        requested = (requested_vx, requested_wz)
        actual = (vx, wz)
        ramp = self.prev_cmd
        synced = []
        for index in (0, 1):
            # A zero output below the controller deadband is intentional and
            # must not reset the continuous limiter state. A zero output for
            # an explicit stop must reset it.
            if (requested[index] is not None
                    and abs(requested[index]) < (self.follower.apf_deadband_v
                        if index == 0 else self.follower.apf_deadband_wz)
                    and abs(actual[index]) < 1e-9):
                synced.append(ramp[index])
            else:
                synced.append(actual[index])
        self.prev_cmd = tuple(synced)
        self.last_command = (vx, wz)

    def _targets(self, path):
        if not path:
            return None, None
        second = min(self.follower.apf_waypoint_stride, len(path) - 1)
        # The parent already removes reached points. Do not count a lone
        # endpoint twice: its attraction stays at the near-goal strength.
        return path[0], path[second] if second else None

    def _age_weight(self, stamp, now):
        p = self.follower
        age = max(0.0, now - stamp - p.dynamic_layer.timeout)
        return max(p.apf_memory_min_weight,
                   math.exp(-age / p.apf_memory_decay_time))

    def _nearby_obstacles(self, robot_pose, now):
        """Keep timestamps aligned; compute clearance before the force budget."""
        p = self.follower
        self.nearest_clearance = float("inf")
        if not p.dynamic_layer.enabled:
            return []
        rx, ry, yaw = robot_pose
        c, s = math.cos(yaw), math.sin(yaw)
        g = p.geometry
        # Include the danger band around every corner of the body,
        # as well as the complete center-based repulsive influence band.
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
            # Vertical stacks share one XY force, using their latest hit.
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
        if not path:
            self.reset()
            return self._zero("no_path")
        p = self.follower
        rx, ry, yaw = robot_pose
        now = p.now_sec()
        if (not all(math.isfinite(v) for v in (rx, ry, yaw, dt, now)) or dt <= 0):
            return self._zero("invalid_input")
        if self._path_end is not path[-1]:
            # Parent copies/prunes lists but retains PathPoint objects. A new
            # accepted A* path (even at the same coordinates) has a new end
            # object. A pending request alone must not reset the filter.
            self.last_force = self.last_heading = None
            self._path_end = path[-1]
        near, far = self._targets(path)
        ax = ay = 0.0
        for target, gain in ((near, p.apf_near_attraction_gain),
                             (far, p.apf_far_attraction_gain)):
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
                # Direction is undefined at the center; clearance below will
                # command a stop, and the footprint check rejects overlap.
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
        if self.nearest_clearance <= p.apf_danger_clearance:
            self.status = "danger_zone"
            self.last_force = self.last_heading = None
            self.prev_cmd = self.last_command = (0.0, 0.0)
            return 0.0, 0.0, 0.0
        if norm < p.apf_force_epsilon:
            return self._zero("force_cancelled")

        # Filter vectors, not angles, to avoid discontinuity at +/- pi.
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
        # A newly reversed force immediately removes forward motion, even
        # while the filtered direction/turn rate still catches up.
        turn_error = max(abs(error), abs(raw_error))

        gx, gy = self._point_xy(path[-1])
        goal_distance = math.hypot(gx - rx, gy - ry)
        speed = p.command_max_vx
        if goal_distance < p.apf_goal_approach_distance:
            speed *= max(p.apf_min_speed_scale, goal_distance / p.apf_goal_approach_distance)
        if turn_error >= p.apf_stop_rotate_yaw:
            speed = 0.0
        else:
            speed *= max(p.apf_min_speed_scale, 1.0 - turn_error / p.apf_slowdown_yaw)
        if 0.0 < speed < p.apf_min_vx and goal_distance > 0.25:
            speed = min(p.command_max_vx, p.apf_min_vx)
        vx = self._limit_rate(speed, self.prev_cmd[0], p.apf_accel_limit_v, dt)
        wz = max(-p.command_max_wz, min(p.command_max_wz, p.apf_heading_gain * error))
        wz = self._limit_rate(wz, self.prev_cmd[1], p.apf_accel_limit_wz, dt)
        wz = max(-p.command_max_wz, min(p.command_max_wz, wz))
        self.prev_cmd = (vx, wz)
        # Preserve the continuous ramp, so a short timestep cannot keep the
        # limiter permanently below the output deadband.
        out_v = 0.0 if vx < p.apf_deadband_v else vx
        out_w = 0.0 if abs(wz) < p.apf_deadband_wz else wz
        self.last_command = (out_v, out_w)
        self.status = "tracking"
        return out_v, 0.0, out_w
