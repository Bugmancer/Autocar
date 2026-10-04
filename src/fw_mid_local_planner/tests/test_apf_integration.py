"""离线适配 ROS 传输层，直接验证生产代码的跟踪策略和安全流程。"""

import importlib
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

# 先加载 NumPy 扩展，避免临时 ROS 模块替身影响其导入。
import numpy

SRC = Path(__file__).resolve().parents[2]
for package in ("fw_mid_local_planner", "fw_mid_common_utils", "fw_mid_controller"):
    sys.path.insert(0, str(SRC / package))


class Stamp:
    def __init__(self, seconds=0):
        self.seconds = seconds

    @classmethod
    def now(cls):
        return cls(10.0)

    def to_sec(self):
        return self.seconds

    def is_zero(self):
        return self.seconds == 0

    def __sub__(self, other):
        return Stamp(self.seconds - other.seconds)

    def __eq__(self, other):
        return isinstance(other, Stamp) and self.seconds == other.seconds

    def __lt__(self, other):
        return self.seconds < other.seconds


def vector():
    return SimpleNamespace(x=0.0, y=0.0, z=0.0)


def pose():
    return SimpleNamespace(position=vector(), orientation=SimpleNamespace(x=0, y=0, z=0, w=0))


class Message:
    """只替代消息字段存储；坐标变换与安全判断仍执行生产代码。"""

    ADD, DELETE, DELETEALL = 0, 2, 3
    ARROW, SPHERE, LINE_LIST, CUBE_LIST = 0, 2, 5, 6

    def __init__(self, **kwargs):
        self.header = SimpleNamespace(frame_id="", stamp=Stamp())
        self.pose = pose()
        self.position, self.orientation = self.pose.position, self.pose.orientation
        self.linear, self.angular, self.scale = vector(), vector(), vector()
        self.color = SimpleNamespace(r=0, g=0, b=0, a=0)
        self.info = SimpleNamespace(resolution=0, width=0, height=0, origin=pose())
        self.points, self.poses, self.markers, self.data = [], [], [], []
        self.x = self.y = self.z = 0.0
        self.__dict__.update(kwargs)


class Publisher:
    def __init__(self, *_args, **_kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def load_ros_adapters():
    modules = {}
    for name, symbols in {
        "rospy": (), "tf2_ros": (),
        "geometry_msgs.msg": ("Point", "Pose", "PoseArray", "PoseStamped", "Twist"),
        "nav_msgs.msg": ("OccupancyGrid", "Path"),
        "nav_msgs.srv": ("GetPlan", "GetPlanRequest"),
        "std_msgs.msg": ("Bool", "Time"),
        "std_srvs.srv": ("Trigger", "TriggerResponse"),
        "visualization_msgs.msg": ("Marker", "MarkerArray"),
        "sensor_msgs.msg": ("PointCloud2",), "sensor_msgs.point_cloud2": (),
    }.items():
        module = modules[name] = ModuleType(name)
        for symbol in symbols:
            setattr(module, symbol, type(symbol, (Message,), {}))
        if "." in name:
            parent, child = name.rsplit(".", 1)
            parent_module = modules.setdefault(parent, ModuleType(parent))
            setattr(parent_module, child, module)
    ros = modules["rospy"]
    ros.Time = ros.Duration = Stamp
    ros.Publisher = Publisher
    for name in ("get_param", "Subscriber", "Service", "Timer", "on_shutdown",
                 "init_node", "spin", "logwarn", "loginfo", "logerr",
                 "logwarn_throttle", "loginfo_throttle", "logerr_throttle"):
        setattr(ros, name, Mock())
    modules["tf2_ros"].Buffer = Mock(side_effect=lambda **_: SimpleNamespace(
        lookup_transform=Mock(side_effect=LookupError("TF unavailable"))))
    modules["tf2_ros"].TransformListener = Mock()
    with patch.dict(sys.modules, modules):
        imported = [importlib.import_module("fw_mid_local_planner." + name) for name in (
            "follower_runtime", "dynamic_obstacle_layer", "apf_tracking", "tracking",
            "path_follower_node", "path_follower_node_v1")]
    return (ros, *imported, modules)


ROS, RUNTIME, LAYER, APF, CLASSIC, NODE, NODE_V1, ROS_MODULES = load_ros_adapters()
from fw_mid_local_planner.path_processing import PathPoint


class FollowerIntegrationTests(unittest.TestCase):
    def setUp(self):
        modules = patch.dict(sys.modules, ROS_MODULES)
        modules.start()
        self.addCleanup(modules.stop)
        for value in vars(ROS).values():
            if isinstance(value, Mock):
                value.reset_mock(return_value=True, side_effect=True)
        ROS.get_param.side_effect = lambda _name, default: default
        clock = patch.object(RUNTIME.time, "monotonic", return_value=100.0)
        self.clock = clock.start()
        self.addCleanup(clock.stop)

    def make_node(self, strategy=APF.APFTracking, **parameters):
        configured = {"~enable_dynamic_obstacles": True, "~require_localization": True}
        configured.update({"~" + name: value for name, value in parameters.items()})
        ROS.get_param.side_effect = lambda name, default: configured.get(name, default)
        node = RUNTIME.FollowerRuntime(strategy)
        node.global_path = [PathPoint(2, 0)]
        node.goal_pose = Message()
        node.localization_valid = True
        node.localization_received = 100.0
        node.dynamic_layer.last_received = 100.0
        node.dynamic_layer.last_update_time = Stamp.now()
        node.dynamic_layer.projection_valid = True
        node.static_map = Message()
        node.static_map.info = SimpleNamespace(
            width=100, height=100, resolution=0.1,
            origin=SimpleNamespace(position=SimpleNamespace(x=-5, y=-5, z=0),
                orientation=SimpleNamespace(x=0, y=0, z=0, w=1)))
        node.static_map.data = [0] * 10000
        node.costmap = node.static_map
        node.cmd_pub.messages.clear()
        return node

    def assert_stopped(self, node):
        self.assertTrue(node.cmd_pub.messages)
        for message in node.cmd_pub.messages:
            self.assertEqual((message.linear.x, message.linear.y, message.angular.z), (0, 0, 0))
        self.assertEqual(node._last_command, (0, 0))
        self.assertEqual(node.tracking.controller.prev_cmd, (0, 0))

    def test_entrypoints_select_independent_strategies(self):
        for module, strategy in ((NODE, CLASSIC.ClassicTracking), (NODE_V1, APF.APFTracking)):
            with self.subTest(module=module.__name__), patch.object(module, "FollowerRuntime") as runtime:
                ROS.init_node.reset_mock()
                ROS.spin.reset_mock()
                module.main()
                runtime.assert_called_once_with(strategy)
                ROS.init_node.assert_called_once_with("path_follower_node")
                ROS.spin.assert_called_once_with()
        self.assertNotIn(RUNTIME.FollowerRuntime, APF.APFTracking.__mro__)
        self.assertNotIn(CLASSIC.ClassicTracking, APF.APFTracking.__mro__)

    def test_strategy_is_ready_before_any_subscription_or_timer(self):
        for strategy in (CLASSIC.ClassicTracking, APF.APFTracking):
            with self.subTest(strategy=strategy.__name__):
                initialized = []
                callbacks = []

                def factory(node):
                    tracking = strategy(node)
                    initialized.append(tracking)
                    return tracking

                def subscribe(topic, _message_type, callback, **_kwargs):
                    self.assertEqual(len(initialized), 1)
                    self.assertIsNotNone(initialized[0].controller)
                    callbacks.append(topic)
                    if topic == "/localizer/localization_valid":
                        callback(Message(data=False))
                    return Mock()

                def start_timer(_duration, callback):
                    self.assertIs(callback.__self__.tracking, initialized[0])
                    callback(None)
                    return Mock()

                with patch.object(ROS, "Subscriber", side_effect=subscribe), \
                        patch.object(ROS, "Timer", side_effect=start_timer):
                    node = self.make_node(factory)
                self.assertIn(node.dynamic_layer.cloud_topic, callbacks)
                self.assertIn(node.goal_topic, callbacks)

    def test_classic_instantiates_only_selected_controller_with_unknown_fallback(self):
        for requested, expected in (("pid", "pid"), ("pure_pursuit", "pure_pursuit"),
                                    ("unknown", "pid")):
            with self.subTest(requested=requested), \
                    patch.object(CLASSIC, "PIDPathController", wraps=CLASSIC.PIDPathController) as pid, \
                    patch.object(CLASSIC, "PurePursuitController", wraps=CLASSIC.PurePursuitController) as pp:
                node = self.make_node(CLASSIC.ClassicTracking, tracking_controller=requested)
                self.assertEqual(node.tracking.name, expected)
                self.assertEqual(pid.call_count, int(expected == "pid"))
                self.assertEqual(pp.call_count, int(expected == "pure_pursuit"))

    def test_both_strategies_publish_valid_guarded_motion(self):
        for strategy in (CLASSIC.ClassicTracking, APF.APFTracking):
            with self.subTest(strategy=strategy.__name__):
                node = self.make_node(strategy)
                command = node.tracking.compute((0, 0, 0), node.global_path,
                    node.global_path[0], node.control_dt)
                self.assertGreater(command[0], 0)
                self.assertTrue(node.publish_cmd(*command, robot_pose=(0, 0, 0), generation=0))
                self.assertEqual(len(node.cmd_pub.messages), 1)
                self.assertGreater(node.cmd_pub.messages[0].linear.x, 0)

    def test_shared_safety_gates_never_publish_motion(self):
        for strategy in (CLASSIC.ClassicTracking, APF.APFTracking):
            for failure in ("nan", "infinity", "lateral", "generation", "localization",
                            "localization_stale", "dynamic_stale", "static_missing",
                            "collision", "no_path"):
                with self.subTest(strategy=strategy.__name__, failure=failure):
                    node = self.make_node(strategy)
                    node._last_command = node.tracking.controller.prev_cmd = (0.2, 0.1)
                    command, generation = (0.2, 0, 0.1), 0
                    if failure == "nan":
                        command = (float("nan"), 0, 0.1)
                    elif failure == "infinity":
                        command = (0.2, 0, float("inf"))
                    elif failure == "lateral":
                        command = (0.2, 0.1, 0.1)
                    elif failure == "generation":
                        generation = 1
                    elif failure == "localization":
                        node.localization_valid = False
                    elif failure == "localization_stale":
                        node.localization_received = 90.0
                    elif failure == "dynamic_stale":
                        node.dynamic_layer.last_received = 90.0
                    elif failure == "static_missing":
                        node.static_map = None
                    elif failure == "collision":
                        node.dynamic_layer.map_points = [(0, 0)]
                    elif failure == "no_path":
                        node.global_path = []
                    self.assertFalse(node.publish_cmd(*command, robot_pose=(0, 0, 0), generation=generation))
                    self.assert_stopped(node)

    def test_safety_callbacks_can_cancel_command_during_collision_check(self):
        for strategy in (CLASSIC.ClassicTracking, APF.APFTracking):
            for callback_name in ("localization", "clear_memory"):
                with self.subTest(strategy=strategy.__name__, callback=callback_name):
                    node = self.make_node(strategy)
                    node._last_command = node.tracking.controller.prev_cmd = (0.2, 0.1)
                    checking, release, callback_done = (threading.Event() for _ in range(3))
                    results, errors = [], []
                    real_check = node.collision_checker.check

                    def blocked_check(*args, **kwargs):
                        checking.set()
                        if not release.wait(3.0):
                            raise TimeoutError("collision check was not released")
                        return real_check(*args, **kwargs)

                    def publish():
                        try:
                            results.append(node.publish_cmd(0.2, 0, 0.1,
                                robot_pose=(0, 0, 0), generation=0))
                        except Exception as error:
                            errors.append(error)

                    def cancel():
                        try:
                            if callback_name == "localization":
                                node.localization_cb(Message(data=False))
                            else:
                                node.clear_memory_cb(None)
                        except Exception as error:
                            errors.append(error)
                        finally:
                            callback_done.set()

                    worker = threading.Thread(target=publish, daemon=True)
                    callback_worker = threading.Thread(target=cancel, daemon=True)
                    with patch.object(node.collision_checker, "check", side_effect=blocked_check):
                        worker.start()
                        try:
                            self.assertTrue(checking.wait(1.0), "collision check never started")
                            callback_worker.start()
                            self.assertTrue(callback_done.wait(1.0),
                                "safety callback blocked behind collision computation")
                        finally:
                            release.set()
                            worker.join(3.0)
                            if callback_worker.ident is not None:
                                callback_worker.join(3.0)
                    self.assertFalse(worker.is_alive())
                    self.assertFalse(callback_worker.is_alive())
                    self.assertEqual(errors, [])
                    self.assertEqual(results, [False])
                    self.assert_stopped(node)

    def test_final_published_speed_is_synced_after_real_collision_reduction(self):
        node = self.make_node()
        node.dynamic_layer.map_points = [(0.8, 0)]
        controller = node.tracking.controller
        controller.prev_cmd = controller.last_command = (0.3, 0)
        self.assertTrue(node.publish_cmd(0.3, 0, 0, robot_pose=(0, 0, 0), generation=0))
        self.assertGreater(node._last_command[0], 0)
        self.assertLess(node._last_command[0], 0.3)
        self.assertEqual(controller.prev_cmd, node._last_command)

    def test_rejected_command_preserves_diagnostics_before_reset(self):
        node = self.make_node()
        controller = node.tracking.controller
        controller.last_raw_force = controller.last_force = (1, 2)
        controller.status = "force_cancelled"
        self.assertFalse(node.publish_cmd(float("nan"), 0, 0, robot_pose=(0, 0, 0)))
        self.assertIsNone(controller.last_force)
        text = ROS.loginfo_throttle.call_args.args[1]
        self.assertIn("raw=(1, 2)", text)
        self.assertIn("state=force_cancelled", text)
        self.assertIn("accepted=False", text)

    def test_recovery_handoff_uses_last_guarded_command_and_stop_resets(self):
        for strategy in (CLASSIC.ClassicTracking, APF.APFTracking):
            with self.subTest(strategy=strategy.__name__):
                node = self.make_node(strategy)
                node.start_recovery = SimpleNamespace(start=(0, 0, 0), distance=0.5)
                node.recovery_suffix = [PathPoint(1, 0), PathPoint(2, 0)]
                node._last_command = (0.05, 0.01)
                controller = node.tracking.controller
                if strategy is APF.APFTracking:
                    controller.last_force = (1, 2)
                self.assertFalse(node.follow_start_recovery((0.5, 0, 0)))
                self.assertIsNone(node.start_recovery)
                self.assertEqual(controller.prev_cmd, node._last_command)
                self.assertEqual(len(node.global_path), 2)
                if strategy is APF.APFTracking:
                    self.assertIsNone(controller.last_force)
                    node.tracking._stall_since = 3
                node.publish_stop()
                self.assert_stopped(node)
                if strategy is APF.APFTracking:
                    self.assertIsNone(node.tracking._stall_since)

    def test_apf_direction_uses_filtered_force_and_recovery_target(self):
        node = self.make_node()
        node.tracking.controller.last_force = (3, 4)
        node.publish_desired_direction((1, 2, 0), PathPoint(10, 2), 0.1)
        arrow = node.desired_direction_pub.messages[-1]
        self.assertAlmostEqual(arrow.points[-1].x, 1.6)
        self.assertAlmostEqual(arrow.points[-1].y, 2.8)
        node.tracking.controller.last_force = None
        node.publish_desired_direction((1, 2, 0), PathPoint(10, 2), 0.1)
        self.assertEqual(node.desired_direction_pub.messages[-1].action, Message.DELETE)
        node.start_recovery = object()
        node.publish_desired_direction((1, 2, 0), PathPoint(2, 2), 0.1)
        self.assertEqual(node.desired_direction_pub.messages[-1].points[-1].x, 2)

    def test_cancelled_force_requests_astar_after_timeout_without_dynamic_overlay(self):
        node = self.make_node()
        node.tracking.controller.status = "force_cancelled"
        node.get_robot_pose = Mock(return_value=(0, 0, 0))
        node.request_plan = Mock(return_value=True)
        with patch.object(node, "_control_loop"):
            for stamp in (0, 0.1, 1.9):
                self.clock.return_value = stamp
                node.control_loop(None)
            node.request_plan.assert_not_called()
            self.clock.return_value = 2.1
            node.control_loop(None)
            node.request_plan.assert_called_once_with((0, 0, 0), "normal")
            self.clock.return_value = 2.2
            node.control_loop(None)
            self.assertEqual(node.request_plan.call_count, 1)
            node.waiting_for_plan = True
            self.clock.return_value = 20
            node.control_loop(None)
            self.assertEqual(node.request_plan.call_count, 1)

    def test_dynamic_cancelled_force_uses_overlay_and_respects_mode(self):
        node = self.make_node()
        node.tracking.controller.status = "force_cancelled"
        node.dynamic_history = [(10, 1, 0)]
        node.tracking._stall_since = 0
        node.get_robot_pose = Mock(return_value=(0, 0, 0))
        node.request_plan = Mock(return_value=True)
        self.clock.return_value = 3
        with patch.object(node, "_control_loop"):
            node.control_loop(None)
            node.request_plan.assert_called_once_with((0, 0, 0), "avoidance")
            node.request_plan.reset_mock()
            node.dynamic_avoidance_mode = "stop"
            node.tracking._stall_since = 0
            node.control_loop(None)
            node.request_plan.assert_not_called()

    def test_bad_apf_parameters_are_rejected(self):
        for key, value in (("apf_force_filter_time", -1), ("apf_memory_decay_time", 0),
                           ("apf_memory_min_weight", 0), ("apf_memory_min_weight", 1.1),
                           ("apf_danger_clearance", -1), ("apf_danger_clearance", float("nan")),
                           ("apf_farther_attraction_gain", -1),
                           ("apf_farther_attraction_gain", float("nan")),
                           ("apf_farther_attraction_gain", float("inf"))):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.make_node(**{key: value})

    def test_zero_farther_gain_disables_third_attraction(self):
        node = self.make_node(apf_farther_attraction_gain=0)
        path = [PathPoint(x * 0.2, 0) for x in range(1, 11)]
        node.tracking.compute((0, 0, 0), path, None, 0.1)
        self.assertAlmostEqual(node.tracking.controller.last_raw_force[0], 1.35)

    def test_projection_snapshot_keeps_real_stamps_through_transform_and_clear(self):
        layer = LAYER.DynamicObstacleLayer.__new__(LAYER.DynamicObstacleLayer)
        layer._lock = threading.RLock()
        layer._clear_generation = 0
        layer.enabled = True
        layer.global_frame, layer.memory_frame, layer.robot_frame = "map", "lidar", "base_link"
        points = [(1, 2, 0.5, 3.0), (2, 3, 0.5, 7.0)]
        layer.memory = Mock()
        layer.memory.snapshot.side_effect = lambda: list(points)
        layer.memory.clear.side_effect = points.clear
        transform = SimpleNamespace(transform=SimpleNamespace(
            rotation=SimpleNamespace(x=0, y=0, z=0, w=1),
            translation=SimpleNamespace(x=10, y=0, z=0)))
        layer.lookup = lambda target, source, stamp: transform if target == "map" else None
        layer.publish_markers = Mock()
        layer.update_map_points()
        self.assertEqual(layer.timed_point_snapshot(), [(11, 2, 3), (12, 3, 7)])
        self.assertEqual(layer.point_snapshot(), ([(1, 2), (2, 3)], [(11, 2), (12, 3)]))
        transform.transform.translation.x = 20
        layer.update_map_points()
        self.assertEqual(layer.timed_point_snapshot(), [(21, 2, 3), (22, 3, 7)])
        layer.clear()
        self.assertEqual(layer.timed_point_snapshot(), [])
        self.assertEqual(layer.point_snapshot(), ([], []))
        self.assertFalse(layer.projection_valid)


if __name__ == "__main__":
    unittest.main()
