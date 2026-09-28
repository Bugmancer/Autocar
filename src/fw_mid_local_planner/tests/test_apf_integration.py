"""Offline ROS adapters; real memory projection and follower overrides."""

import importlib
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

# Keep NumPy's extension modules outside the temporary ROS import stubs.
import numpy

SRC = Path(__file__).resolve().parents[2]
for package in ("fw_mid_local_planner", "fw_mid_common_utils", "fw_mid_controller"):
    sys.path.insert(0, str(SRC / package))


def load_ros_adapters():
    # Stub the transport and message imports only. Production Python code is
    # imported unmodified; these tests do not claim to exercise ROS transport.
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
            setattr(module, symbol, type(symbol, (), {}))
        if "." in name:
            parent, child = name.rsplit(".", 1)
            parent_module = modules.setdefault(parent, ModuleType(parent))
            setattr(parent_module, child, module)
    ros = modules["rospy"]
    ros.Time = Mock(return_value=0)
    ros.logwarn = Mock()
    ros.logwarn_throttle = Mock()
    ros.loginfo_throttle = Mock()
    with patch.dict(sys.modules, modules):
        node = importlib.import_module("fw_mid_local_planner.path_follower_node_v1")
        layer = importlib.import_module("fw_mid_local_planner.dynamic_obstacle_layer")
    return node, layer


NODE, LAYER = load_ros_adapters()
from fw_mid_common_utils.collision_geometry import CollisionGeometry
from fw_mid_local_planner.path_processing import PathPoint
from fw_mid_local_planner.potential_field import ArtificialPotentialFieldController


class APFIntegrationTests(unittest.TestCase):
    def make_node(self):
        node = NODE.APFPathFollower.__new__(NODE.APFPathFollower)
        node.geometry = CollisionGeometry()
        node.param = lambda key, default: default
        node._load_apf_parameters()
        node._lock = threading.RLock()
        node._apf_ready = True
        node._apf_stall_since = None
        node.apf_controller = ArtificialPotentialFieldController(node)
        node.start_recovery = None
        node.global_path = [PathPoint(2, 0)]
        node.goal_pose = object()
        node.dynamic_avoidance_mode = "astar_replan"
        node.dynamic_history = []
        node.waiting_for_plan = False
        node.dynamic_replan_due = Mock(return_value=True)
        node.get_robot_pose = Mock(return_value=(0, 0, 0))
        node.now_sec = lambda: 10.0
        node.request_plan = Mock(return_value=True)
        return node

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
        # Reprojection is not a new occupied observation.
        transform.transform.translation.x = 20
        layer.update_map_points()
        self.assertEqual(layer.timed_point_snapshot(), [(21, 2, 3), (22, 3, 7)])
        layer.clear()
        self.assertEqual(layer.timed_point_snapshot(), [])
        self.assertEqual(layer.point_snapshot(), ([], []))
        self.assertFalse(layer.projection_valid)

    def test_stop_during_base_constructor_uses_original_reset(self):
        node = NODE.APFPathFollower.__new__(NODE.APFPathFollower)
        node._apf_ready = False
        with patch.object(NODE.PathFollower, "reset_controllers") as base:
            node.reset_controllers()
            base.assert_called_once()

    def test_final_published_speed_is_synced_after_downstream_guard(self):
        node = self.make_node()
        c = node.apf_controller
        c.prev_cmd = c.last_command = (0.3, 0.2)
        def guarded(*args, **kwargs):
            node._last_command = (0.075, 0.05)
            return True
        with patch.object(NODE.PathFollower, "publish_cmd", side_effect=guarded):
            self.assertTrue(node.publish_cmd(0.3, 0, 0.2))
        self.assertEqual(c.prev_cmd, node._last_command)

    def test_reset_and_recovery_handoff_clear_direction(self):
        node = self.make_node()
        c = node.apf_controller
        c.last_force = (1, 2)
        c.prev_cmd = (0.05, 0.01)
        with patch.object(NODE.PathFollower, "follow_start_recovery", return_value=False):
            self.assertFalse(node.follow_start_recovery((0, 0, 0)))
        self.assertIsNone(c.last_force)
        self.assertEqual(c.prev_cmd, (0.05, 0.01))
        node._apf_stall_since = 3
        node.reset_controllers()
        self.assertEqual(c.prev_cmd, (0, 0))
        self.assertIsNone(node._apf_stall_since)

    def test_cancelled_force_requests_astar_after_timeout_without_dynamic_overlay(self):
        node = self.make_node()
        c = node.apf_controller
        c.status = "force_cancelled"
        with patch.object(NODE.PathFollower, "_control_loop"), patch.object(NODE.time, "monotonic") as clock:
            for stamp in (0, 0.1, 1.9):
                clock.return_value = stamp
                node._control_loop(None)
            node.request_plan.assert_not_called()
            clock.return_value = 2.1
            node._control_loop(None)
            node.request_plan.assert_called_once_with((0, 0, 0), "normal")
            clock.return_value = 2.2
            node._control_loop(None)
            self.assertEqual(node.request_plan.call_count, 1)
            node.waiting_for_plan = True
            clock.return_value = 20
            node._control_loop(None)
            self.assertEqual(node.request_plan.call_count, 1)

    def test_dynamic_cancelled_force_uses_overlay_and_respects_mode(self):
        node = self.make_node()
        node.apf_controller.status = "force_cancelled"
        node.dynamic_history = [(10, 1, 0)]
        node._apf_stall_since = 0
        with patch.object(NODE.PathFollower, "_control_loop"), patch.object(NODE.time, "monotonic", return_value=3):
            node._control_loop(None)
            node.request_plan.assert_called_once_with((0, 0, 0), "avoidance")
            node.request_plan.reset_mock()
            node.dynamic_avoidance_mode = "stop"
            node._apf_stall_since = 0
            node._control_loop(None)
            node.request_plan.assert_not_called()

    def test_bad_filter_age_and_clearance_parameters_are_rejected(self):
        for key, value in (("apf_force_filter_time", -1), ("apf_memory_decay_time", 0),
                           ("apf_memory_min_weight", 0), ("apf_memory_min_weight", 1.1),
                           ("apf_danger_clearance", -1),
                           ("apf_danger_clearance", float("nan"))):
            with self.subTest(key=key, value=value):
                node = self.make_node()
                node.param = lambda name, default: value if name == key else default
                with self.assertRaises(ValueError):
                    node._load_apf_parameters()


if __name__ == "__main__":
    unittest.main()
