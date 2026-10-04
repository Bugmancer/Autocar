"""两种策略共用的导航参数与校验；参数只在节点启动时读取。"""

import math

import rospy


def load_parameters(node):
    """保持 ROS 私有参数名稳定；新增参数应同时更新 YAML 和操作说明。"""
    # 定位有效性与位姿新鲜度是两个独立条件，不能只检查能否查询到 TF。
    node.require_localization = bool(node.param("require_localization", False))
    node.pose_timeout = float(node.param("pose_timeout", 0.5))
    if not math.isfinite(node.pose_timeout) or node.pose_timeout <= 0:
        raise ValueError("pose_timeout must be finite and positive")

    # 路径、目标和障碍投影统一使用 global_frame，车身碰撞模型以 robot_frame 为原点。
    node.global_frame = node._frame(node.param("global_frame", "map"))
    node.robot_frame = node._frame(node.param("robot_frame", "base_link"))
    node.planning_frame = node._frame(
        node.param("planning_frame", node.robot_frame)
    )
    node.cmd_vel_topic = str(node.param("cmd_vel_topic", "/cmd_vel"))
    node.plan_topic = str(node.param("plan_topic", "/local_planner/global_plan"))
    node.planner_service = str(node.param("planner_service", "/astar_planner_node/get_plan"))
    node.goal_topic = str(node.param("goal_topic", "/move_base_simple/goal"))
    node.legacy_goal_topic = str(node.param("legacy_goal_topic", "/goal_pose"))
    node.costmap_topic = str(node.param("costmap_topic", "/astar_planner_node/costmap"))
    node.static_map_topic = str(node.param("static_map_topic", "/astar_planner_node/map"))
    # 外部角速度上限用度/秒配置，运行层和控制器内部统一使用弧度/秒。
    node.control_rate = max(1.0, float(node.param("control_rate", 10.0)))
    node.control_dt = 1.0 / node.control_rate
    node.command_max_vx = float(node.param("command_max_vx", 0.30))
    node.command_max_wz = math.radians(float(node.param("command_max_wz_deg", 30.0)))
    if not all(math.isfinite(value) and value >= 0
               for value in (node.command_max_vx, node.command_max_wz)):
        raise ValueError("Command limits must be finite and nonnegative")
    node.lookahead_distance = float(node.param("lookahead_distance", 0.5))
    node.waypoint_tolerance = float(node.param("waypoint_tolerance", 0.25))
    node.goal_tolerance = float(node.param("goal_tolerance", 0.35))

    # 路径处理可以修改点列，但其采样与平滑不能绕过占用检查。
    node.path_enable_shortcut = bool(node.param("path_enable_shortcut", True))
    node.path_enable_smoothing = bool(node.param("path_enable_smoothing", True))
    node.path_resample_ds = float(node.param("path_resample_ds", 0.10))
    node.path_smooth_weight_data = float(node.param("path_smooth_weight_data", 0.20))
    node.path_smooth_weight_smooth = float(node.param("path_smooth_weight_smooth", 0.35))
    node.path_smooth_max_iter = int(node.param("path_smooth_max_iter", 80))
    node.path_smooth_tolerance = float(node.param("path_smooth_tolerance", 1e-4))
    node.path_collision_check_step = float(node.param("path_collision_check_step", 0.05))
    node.path_occupied_threshold = int(node.param("path_occupied_threshold", 50))

    # 避障模式控制重规划决策；紧急包络和最终车身检查由各自开关独立控制。
    node.dynamic_avoidance_mode = str(node.param("dynamic_avoidance_mode", "astar_replan")).strip().lower()
    if node.dynamic_avoidance_mode not in ("off", "stop", "astar_replan"):
        rospy.logwarn(
            "Unknown dynamic_avoidance_mode=%s; using stop",
            node.dynamic_avoidance_mode,
        )
        node.dynamic_avoidance_mode = "stop"
    node.obstacle_slowdown_enabled = bool(node.param("obstacle_slowdown_enabled", False))
    node.obstacle_slowdown_distance = float(node.param("obstacle_slowdown_distance", 0.20))
    node.obstacle_min_speed_scale = float(node.param("obstacle_min_speed_scale", 0.20))
    if (not math.isfinite(node.obstacle_slowdown_distance) or node.obstacle_slowdown_distance <= 0
            or not math.isfinite(node.obstacle_min_speed_scale)
            or not 0 < node.obstacle_min_speed_scale <= 1):
        raise ValueError("Invalid obstacle slowdown distance or minimum speed scale")
    # 降级窗口只应覆盖短时点云中断；延长窗口会增加新障碍无法被及时感知的时间。
    node.dynamic_degraded_motion_enabled = bool(
        node.param("dynamic_degraded_motion_enabled", True))
    node.dynamic_degraded_timeout = float(
        node.param("dynamic_degraded_timeout", 0.5))
    node.dynamic_degraded_speed_scale = float(
        node.param("dynamic_degraded_speed_scale", 0.25))
    if (not math.isfinite(node.dynamic_degraded_timeout)
            or node.dynamic_degraded_timeout < 0
            or not math.isfinite(node.dynamic_degraded_speed_scale)
            or not 0 < node.dynamic_degraded_speed_scale <= 1):
        raise ValueError("Invalid degraded radar motion parameters")
    node.dynamic_detect_x_min = float(node.param("dynamic_detect_x_min", 0.0))
    node.dynamic_detect_x_max = float(node.param("dynamic_detect_x_max", 2.0))
    node.dynamic_detect_y_abs = float(node.param("dynamic_detect_y_abs", 0.7))
    node.dynamic_min_obstacle_points = int(node.param("dynamic_min_obstacle_points", 5))
    node.dynamic_confirm_frames = max(
        1, int(node.param("dynamic_confirm_frames", 3))
    )
    node.dynamic_center_stable_dist = float(node.param("dynamic_center_stable_dist", 0.25))
    node.dynamic_exit_clearance = float(node.param("dynamic_exit_clearance", 0.30))
    node.dynamic_static_filter_enabled = bool(node.param("dynamic_static_filter_enabled", True))
    node.dynamic_static_filter_radius = float(node.param("dynamic_static_filter_radius", 0.0))
    node.dynamic_static_filter_threshold = int(node.param("dynamic_static_filter_threshold", 50))
    # 此分辨率用于给全局规划器的二维去重，不代表三维记忆的体素尺寸或保存期限。
    node.dynamic_history_publish_resolution = float(node.param("dynamic_history_publish_resolution", 0.10))
    if (not math.isfinite(node.dynamic_history_publish_resolution)
            or node.dynamic_history_publish_resolution <= 0):
        raise ValueError("dynamic_history_publish_resolution must be finite and positive")
    node.dynamic_astar_replan_period = float(node.param("dynamic_astar_replan_period", 1.0))
    node.dynamic_obstacle_update_dist = float(node.param("dynamic_obstacle_update_dist", 0.30))
    node.dynamic_obstacle_points_topic = str(
        node.param(
            "dynamic_obstacle_points_topic",
            "/local_planner/dynamic_obstacle_points",
        )
    )
    node.dynamic_keep_moving_during_replan = bool(node.param("dynamic_keep_moving_during_replan", True))
    node.planner_service_timeout = float(node.param("planner_service_timeout", 0.5))
    node.planner_overlay_settle_time = max(
        0.0, float(node.param("planner_overlay_settle_time", 0.03))
    )
    node.require_overlay_ack = bool(node.param("require_overlay_ack", True))
    # 恢复只允许在受限距离内前行，并要求车身、制动与出口后缀均通过检查。
    node.collision_check_enabled = bool(node.param("collision_check_enabled", True))
    node.collision_speed_reduction_enabled = bool(node.param("collision_speed_reduction_enabled", True))
    node.collision_require_static_map = bool(node.param("collision_require_static_map", True))
    node.start_recovery_enabled = bool(node.param("start_recovery_enabled", True))
    node.start_recovery_max_distance = float(node.param("start_recovery_max_distance", 1.2))
    if not 0 < node.start_recovery_max_distance <= 2.0:
        raise ValueError("Invalid bounded start recovery distance")
