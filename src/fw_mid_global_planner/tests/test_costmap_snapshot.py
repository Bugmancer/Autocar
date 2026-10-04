"""验证动态障碍更新与规划请求并发时地图快照归属的离线回归测试。"""

import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np


SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / "fw_mid_common_utils"))


def pose():
    return NS(position=NS(x=0.0, y=0.0, z=0.0),
              orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))


def load_planner():
    # 本测试只验证快照归属，使用替身隔离 ROS 通信和地图加载。
    modules = {name: ModuleType(name) for name in (
        "rospy", "cv2", "geometry_msgs.msg", "nav_msgs.msg", "nav_msgs.srv",
        "std_msgs.msg", "visualization_msgs.msg")}
    ros = modules["rospy"]
    ros.Time = NS(now=lambda: 10.0)
    for name in ("logwarn", "loginfo", "logwarn_throttle", "loginfo_throttle"):
        setattr(ros, name, Mock())
    modules["geometry_msgs.msg"].PoseArray = object
    modules["geometry_msgs.msg"].PoseStamped = lambda: NS(header=NS(), pose=pose())
    modules["nav_msgs.msg"].Path = lambda: NS(header=NS(), poses=[])
    modules["nav_msgs.msg"].OccupancyGrid = lambda: NS(
        header=NS(), info=NS(origin=pose()), data=[])
    modules["nav_msgs.srv"].GetPlan = object
    modules["nav_msgs.srv"].GetPlanResponse = lambda: NS(plan=NS(poses=[]))
    modules["std_msgs.msg"].Time = lambda **kwargs: NS(**kwargs)
    modules["visualization_msgs.msg"].Marker = object
    modules["visualization_msgs.msg"].MarkerArray = lambda: NS(markers=[])
    spec = importlib.util.spec_from_file_location(
        "astar_snapshot_test_node", SRC / "fw_mid_global_planner/scripts/astar_planner_node.py")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.AStarPlanner


AStarPlanner = load_planner()


class CostmapSnapshotTests(unittest.TestCase):
    def test_overlay_update_is_not_replaced_by_older_plan_snapshot(self):
        for old_value, new_value in ((0, 100), (100, 0)):
            with self.subTest(old=old_value, new=new_value):
                planner = AStarPlanner.__new__(AStarPlanner)
                planner.map_frame = "map"
                planner.origin = [0.0, 0.0, 0.0]
                planner.resolution = 1.0
                planner.width = planner.height = 5
                planner.forbidden_rects = []
                planner.occupancy_map = np.zeros((5, 5), dtype=np.int8)
                planner.current_costmap = planner.occupancy_map.copy()
                planner.current_costmap[2, 2] = old_value
                latest = planner.occupancy_map.copy()
                latest[2, 2] = new_value
                planner._dynamic_lock = threading.RLock()
                for name in ("costmap_pub", "overlay_stamp_pub", "path_pub", "map_pub", "zone_pub"):
                    setattr(planner, name, Mock())

                snapshot_taken, resume = threading.Event(), threading.Event()
                responses, errors = [], []

                def build_snapshot():
                    if threading.current_thread() is worker:
                        # 暂停已取得旧地图的规划线程，让障碍回调先提交新地图。
                        snapshot = planner.current_costmap.copy()
                        snapshot_taken.set()
                        if not resume.wait(3.0):
                            raise TimeoutError("overlay update did not complete")
                        return snapshot
                    return latest

                planner.build_working_costmap = build_snapshot
                start = NS(header=NS(frame_id="map"), pose=pose())
                goal = NS(header=NS(frame_id="map"), pose=pose())
                goal.pose.position.x = 4.0

                def plan():
                    try:
                        responses.append(planner.plan_cb(NS(start=start, goal=goal)))
                    except Exception as error:
                        errors.append(error)

                worker = threading.Thread(target=plan, daemon=True)
                worker.start()
                try:
                    self.assertTrue(snapshot_taken.wait(3.0))
                    obstacle = pose()
                    obstacle.position.x = obstacle.position.y = 2.0
                    planner.dynamic_points_cb(NS(
                        header=NS(frame_id="map", stamp=11.0),
                        poses=[obstacle] if new_value else []))
                    self.assertEqual(planner.overlay_stamp_pub.publish.call_args.args[0].data, 11.0)
                finally:
                    resume.set()
                    worker.join(3.0)

                self.assertFalse(worker.is_alive())
                self.assertEqual(errors, [])
                self.assertEqual(len(responses[0].plan.poses), 5)
                planner.path_pub.publish.assert_called_once()
                self.assertIs(planner.current_costmap, latest)
                planner.publish_visuals()
                published = planner.costmap_pub.publish.call_args.args[0]
                self.assertEqual(published.data[2 * planner.width + 2], new_value)


if __name__ == "__main__":
    unittest.main()
