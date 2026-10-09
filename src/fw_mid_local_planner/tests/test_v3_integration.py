"""复用 ROS 传输替身，验证 V3 独立入口和公共安全流程的完整接入。"""

import importlib
import math
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_apf_integration as adapters


class V3IntegrationTests(unittest.TestCase):
    def setUp(self):
        # 只借用替身环境，不继承旧测试类，避免重复执行其他策略的整套测试。
        self.adapters = adapters.FollowerIntegrationTests()
        self.adapters.setUp()
        self.addCleanup(self.adapters.doCleanups)
        with patch.dict("sys.modules", adapters.ROS_MODULES):
            self.strategy_module = importlib.import_module(
                "fw_mid_local_planner.adaptive_tracking")
            self.entry_module = importlib.import_module(
                "fw_mid_local_planner.path_follower_node_v3")
        self.strategy = self.strategy_module.AdaptiveTracking

    def make_node(self, **parameters):
        return self.adapters.make_node(self.strategy, **parameters)

    def prepare_control_cycle(self, node, pose=(0, 0, 0)):
        # 测试聚焦控制周期与回调交错；已有 TF 和点云投影各自由原测试覆盖。
        node.get_robot_pose = Mock(return_value=pose)
        node.dynamic_layer.update_map_points = Mock()
        node.update_measured_velocity = Mock()

    def test_entrypoint_selects_independent_composed_strategy(self):
        with patch.object(self.entry_module, "FollowerRuntime") as runtime:
            self.entry_module.main()
        runtime.assert_called_once_with(self.strategy)
        adapters.ROS.init_node.assert_called_once_with("path_follower_node")
        adapters.ROS.spin.assert_called_once_with()
        for other in (adapters.RUNTIME.FollowerRuntime,
                      adapters.CLASSIC.ClassicTracking, adapters.APF.APFTracking):
            self.assertNotIn(other, self.strategy.__mro__)

    def test_strategy_is_ready_before_subscriptions_and_timer(self):
        initialized = []

        def factory(node):
            tracking = self.strategy(node)
            initialized.append(tracking)
            return tracking

        def subscribe(topic, _message_type, callback, **_kwargs):
            self.assertEqual(len(initialized), 1)
            self.assertIsNotNone(initialized[0].controller)
            if topic == "/localizer/localization_valid":
                callback(adapters.Message(data=False))
            return Mock()

        def start_timer(_duration, callback):
            self.assertIs(callback.__self__.tracking, initialized[0])
            callback(None)
            return Mock()

        with patch.object(adapters.ROS, "Subscriber", side_effect=subscribe), \
                patch.object(adapters.ROS, "Timer", side_effect=start_timer):
            self.adapters.make_node(factory)

    def test_complete_control_cycle_publishes_guarded_motion(self):
        node = self.make_node()
        self.prepare_control_cycle(node)
        node.control_loop(None)
        self.assertTrue(node.cmd_pub.messages)
        self.assertGreater(node.cmd_pub.messages[-1].linear.x, 0)
        self.assertEqual(node.tracking.controller.prev_cmd, node._last_command)

    def test_invalid_path_stops_complete_control_cycle(self):
        node = self.make_node(dynamic_avoidance_mode="off")
        self.prepare_control_cycle(node)
        node._last_command = node.tracking.controller.prev_cmd = (0.2, 0.1)
        node.global_path = [adapters.PathPoint(float("nan"), 0)]
        node.control_loop(None)
        self.adapters.assert_stopped(node)

    def test_candidate_collision_checks_keep_all_nearby_obstacles(self):
        node = self.make_node()
        pose = (0, 0, 0)
        points = [(0.8, 0.10 + index * 0.001) for index in range(30)]
        node.dynamic_layer.map_points = list(points)
        command = node.tracking.compute(pose, node.global_path,
                                        node.global_path[-1], node.control_dt)
        sectors = node.tracking.controller.obstacle_sectors(pose, points)
        self.assertLess(len(sectors), len(points))
        with patch.object(node.collision_checker, "check",
                          wraps=node.collision_checker.check) as check:
            node.tracking.refine_command(pose, command, 0)
        self.assertTrue(check.call_args_list)
        # 扇区聚合只参与方向引导，不能删除交给车身扫掠检查的近场障碍。
        for call in check.call_args_list:
            self.assertEqual(call.args[3], points)

    def test_rejected_nominal_falls_back_only_after_collision_check(self):
        node = self.make_node(dynamic_avoidance_mode="off")
        pose = (0, 0, 0)
        command = node.tracking.compute(pose, node.global_path,
                                        node.global_path[-1], node.control_dt)
        nominal = (command[0], command[2])
        collision_check = node.collision_checker.check

        def reject_nominal(checked_pose, vx, wz, *args, **kwargs):
            if (vx, wz) == nominal:
                return adapters.RUNTIME.CollisionCheckResult(
                    False, [pose, (0.01, 0, 0)], (0.5, 0), "dynamic_obstacle")
            return collision_check(checked_pose, vx, wz, *args, **kwargs)

        with patch.object(node.collision_checker, "check", side_effect=reject_nominal) as check:
            selected = node.tracking.refine_command(pose, command, 0)
            self.assertEqual(check.call_count, 2)
            self.assertEqual(check.call_args_list[0].args[1:3], nominal)
            self.assertEqual(check.call_args_list[1].args[1:3], (selected[0], selected[2]))
            self.assertNotEqual((selected[0], selected[2]), nominal)
            self.assertTrue(node.publish_cmd(*selected, robot_pose=pose, generation=0))
            self.assertEqual(check.call_count, 3)
            self.assertEqual(check.call_args_list[-1].args[1:3], node._last_command)

    def test_diagnostics_preserve_heading_errors_and_report_published_command(self):
        for accepted in (False, True):
            with self.subTest(accepted=accepted):
                node = self.make_node(dynamic_avoidance_mode="off", command_max_vx=0.4)
                controller = node.tracking.controller
                controller.status = "tracking"
                controller.heading_error = math.radians(60)
                controller.raw_heading_error = math.radians(-70)
                vx = 0.6 if accepted else float("nan")
                self.assertEqual(node.publish_cmd(vx, 0, 1.2,
                    robot_pose=(0, 0, 0), generation=0), accepted)
                logged = adapters.ROS.loginfo_throttle.call_args.args[1]
                self.assertIn("state=tracking", logged)
                self.assertIn("accepted=%s" % accepted, logged)
                self.assertIn("heading_error_deg=60.0", logged)
                self.assertIn("raw_heading_error_deg=-70.0", logged)
                self.assertIn("vx=%.3f" % node._last_command[0], logged)
                self.assertIn("wz_deg=%.1f" % math.degrees(node._last_command[1]), logged)
                self.assertEqual(node._last_command[0], 0.4 if accepted else 0.0)

    def test_required_rear_detour_is_not_trimmed_by_body_heading(self):
        node = self.make_node(dynamic_avoidance_mode="off")
        self.prepare_control_cycle(node)
        path = [adapters.PathPoint(x, y) for x, y in
                ((0, 0), (-0.3, 0), (-0.6, 0), (-0.6, 0.8), (1.5, 0.8))]
        node.global_path = list(path)
        node.control_loop(None)
        # 路径尚未走过，车头朝向不能用于跳过向后的合法绕行段。
        self.assertIn(path[1], node.global_path)
        self.assertIn(path[2], node.global_path)

    def test_nearby_loop_endpoint_does_not_finish_untravelled_route(self):
        node = self.make_node(dynamic_avoidance_mode="off")
        self.prepare_control_cycle(node)
        node.global_path = [adapters.PathPoint(x, y) for x, y in
                            ((0, 0), (1, 0), (1, 1), (0, 1), (0.05, 0))]
        goal = node.goal_pose
        node.control_loop(None)
        # 回环终点虽在到点半径内，但完整路径尚未执行，不能提前宣布到达。
        self.assertIs(node.goal_pose, goal)
        self.assertEqual(len(node.global_path), 5)
        self.assertGreater(node.tracking.remaining_distance, 3.0)

    def test_new_goal_resets_controller_before_requesting_new_path(self):
        node = self.make_node()
        self.prepare_control_cycle(node)
        node.control_loop(None)
        self.assertGreater(node.tracking.controller.prev_cmd[0], 0)
        node.tracking.update_path_progress((0.4, 0, 0), node.global_path)
        self.assertGreater(node.tracking.controller.progress, 0)
        node.tracking.controller.bypass_side = 1
        self.prepare_control_cycle(node, (0.4, 0, 0))
        node.request_plan = Mock(return_value=True)
        next_goal = adapters.Message()
        next_goal.header.frame_id = "map"
        next_goal.pose.position.x = 0.4
        next_goal.pose.position.y = 2.0
        node.cmd_pub.messages.clear()
        with patch.object(node.tracking.controller, "stop",
                          wraps=node.tracking.controller.stop) as stop:
            node.goal_cb(next_goal)
        stop.assert_called()
        self.assertIs(node.goal_pose, next_goal)
        self.assertEqual(node.global_path, [])
        self.assertIsNone(node.tracking.controller.last_force)
        node.request_plan.assert_called_once_with((0.4, 0, 0), "normal")
        self.adapters.assert_stopped(node)
        # 临时停车保留旧路径进度；新的规划路径接入时必须重置进度及绕行侧。
        node.global_path = [adapters.PathPoint(0.4, 0), adapters.PathPoint(0.4, 2)]
        node.control_loop(None)
        self.assertAlmostEqual(node.tracking.controller.progress, 0.0)
        self.assertEqual(node.tracking.controller.bypass_side, 0)

    def test_resume_path_preserves_motion_only_for_continuous_v3(self):
        for strategy in (self.strategy, adapters.APF.APFTracking, adapters.CLASSIC.ClassicTracking):
            for keep_moving in (False, True):
                with self.subTest(strategy=strategy.__name__, keep_moving=keep_moving):
                    node = self.adapters.make_node(strategy,
                        dynamic_keep_moving_during_replan=keep_moving,
                        require_overlay_ack=False, planner_overlay_settle_time=0.0,
                        command_max_vx=0.4, command_max_wz_deg=30.0)
                    self.prepare_control_cycle(node)
                    published = (0.3, 0.1)
                    node._last_command = node._measured_velocity = published
                    node.tracking.controller.prev_cmd = published
                    node.waiting_for_plan = True
                    node.pending_plan_kind = "resume"
                    endpoint = adapters.Message()
                    endpoint.pose.position.x = 2.0
                    response = SimpleNamespace(plan=adapters.Message(poses=[endpoint]))
                    with patch.object(adapters.ROS, "wait_for_service", create=True), \
                            patch.object(adapters.ROS, "ServiceProxy", create=True,
                                         return_value=Mock(return_value=response)):
                        node._plan_worker(adapters.Message(), "resume", node._plan_generation, None)
                    continuous = strategy is self.strategy and keep_moving
                    self.assertEqual(node.tracking.controller.prev_cmd,
                                     published if continuous else (0.0, 0.0))
                    self.assertFalse(node.waiting_for_plan)
                    self.assertTrue(node.global_path)
                    if continuous:
                        node.control_loop(None)
                        self.assertGreaterEqual(node.cmd_pub.messages[-1].linear.x,
                            published[0] - node.collision_checker.linear_deceleration * node.control_dt)
                        self.assertEqual(node.tracking.controller.prev_cmd, node._last_command)

    def test_failed_resume_plan_still_stops_continuous_v3(self):
        for failure in ("service", "empty", "processing"):
            with self.subTest(failure=failure):
                node = self.make_node(require_overlay_ack=False,
                    planner_overlay_settle_time=0.0, start_recovery_enabled=False)
                node._last_command = node.tracking.controller.prev_cmd = (0.3, 0.1)
                node.waiting_for_plan = True
                node.pending_plan_kind = "resume"
                endpoint = adapters.Message()
                endpoint.pose.position.x = 2.0
                response = SimpleNamespace(plan=adapters.Message(
                    poses=[] if failure == "empty" else [endpoint]))
                client = Mock(return_value=response,
                              side_effect=RuntimeError("planner unavailable") if failure == "service" else None)
                if failure == "processing":
                    node.process_planned_path = Mock(return_value=[])
                with patch.object(adapters.ROS, "wait_for_service", create=True), \
                        patch.object(adapters.ROS, "ServiceProxy", create=True, return_value=client):
                    node._plan_worker(adapters.Message(), "resume", node._plan_generation, None)
                self.assertFalse(node.waiting_for_plan)
                self.adapters.assert_stopped(node)

    def test_final_safety_gates_reject_invalid_motion(self):
        failures = ("nan", "infinity", "lateral", "generation", "localization",
                    "localization_stale", "dynamic_stale", "static_missing",
                    "collision", "no_path")
        for failure in failures:
            with self.subTest(failure=failure):
                node = self.make_node()
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
                self.assertFalse(node.publish_cmd(*command, robot_pose=(0, 0, 0),
                                                  generation=generation))
                self.adapters.assert_stopped(node)

    def test_callbacks_cancel_command_during_trajectory_selection(self):
        for callback_name in ("localization", "clear_memory"):
            with self.subTest(callback=callback_name):
                node = self.make_node()
                self.prepare_control_cycle(node)
                checking, release, callback_done = (threading.Event() for _ in range(3))
                refining = threading.Event()
                errors = []
                refine = node.tracking.refine_command
                collision_check = node.collision_checker.check

                def observed_refine(*args, **kwargs):
                    refining.set()
                    try:
                        return refine(*args, **kwargs)
                    finally:
                        refining.clear()

                def blocked_check(*args, **kwargs):
                    # 阻塞真实候选轨迹检查内部，验证适配层没有重新包住运行层状态锁。
                    if refining.is_set():
                        checking.set()
                        if not release.wait(3.0):
                            raise TimeoutError("候选轨迹计算没有及时释放")
                    return collision_check(*args, **kwargs)

                def control():
                    try:
                        node.control_loop(None)
                    except Exception as error:
                        errors.append(error)

                def cancel():
                    try:
                        if callback_name == "localization":
                            node.localization_cb(adapters.Message(data=False))
                        else:
                            node.clear_memory_cb(None)
                    except Exception as error:
                        errors.append(error)
                    finally:
                        callback_done.set()

                worker = threading.Thread(target=control, daemon=True)
                callback_worker = threading.Thread(target=cancel, daemon=True)
                with patch.object(node.tracking, "refine_command", side_effect=observed_refine), \
                        patch.object(node.collision_checker, "check", side_effect=blocked_check):
                    worker.start()
                    try:
                        self.assertTrue(checking.wait(1.0), "未进入候选轨迹计算")
                        callback_worker.start()
                        self.assertTrue(callback_done.wait(1.0), "安全回调被候选轨迹计算阻塞")
                    finally:
                        release.set()
                        worker.join(3.0)
                        if callback_worker.ident is not None:
                            callback_worker.join(3.0)
                self.assertFalse(worker.is_alive())
                self.assertFalse(callback_worker.is_alive())
                self.assertEqual(errors, [])
                self.adapters.assert_stopped(node)

    def test_recovery_handoff_syncs_last_published_command(self):
        node = self.make_node()
        node.start_recovery = SimpleNamespace(start=(0, 0, 0), distance=0.5)
        node.recovery_suffix = [adapters.PathPoint(1, 0), adapters.PathPoint(2, 0)]
        node._last_command = (0.05, 0.01)
        node.tracking.controller.last_force = (1, 2)
        self.assertFalse(node.follow_start_recovery((0.5, 0, 0)))
        self.assertIsNone(node.start_recovery)
        self.assertEqual(node.tracking.controller.prev_cmd, node._last_command)
        self.assertIsNone(node.tracking.controller.last_force)
        self.assertEqual(len(node.global_path), 2)
        node.publish_stop()
        self.adapters.assert_stopped(node)

    def test_exhausted_candidate_budgets_publish_only_zero(self):
        for budget in ("count", "time"):
            with self.subTest(budget=budget):
                self.adapters.clock.return_value = 100.0
                node = self.make_node(dynamic_avoidance_mode="off",
                                      v3_trajectory_max_checks=2 if budget == "count" else 9)
                command = node.tracking.compute((0, 0, 0), node.global_path,
                                                node.global_path[-1], node.control_dt)
                self.assertGreater(len(node.tracking.controller.candidates), 2)

                def reject_candidate(*_args, **_kwargs):
                    if budget == "time":
                        self.adapters.clock.return_value += 0.05
                    return adapters.RUNTIME.CollisionCheckResult(
                        False, [(0, 0, 0), (0.01, 0, 0)], (0.3, 0), "dynamic_obstacle")

                with patch.object(node.collision_checker, "check",
                                  side_effect=reject_candidate) as check:
                    selected = node.tracking.refine_command((0, 0, 0), command, 0)
                self.assertEqual(check.call_count, 2 if budget == "count" else 1)
                self.assertEqual(selected, (0, 0, 0))
                self.assertEqual(node.tracking.controller.status, "trajectory_blocked")
                # 无候选获准时，不得回退到未经验证的原始非零命令。
                self.assertTrue(node.publish_cmd(*selected, robot_pose=(0, 0, 0), generation=0))
                self.adapters.assert_stopped(node)

    def observe_progress(self, node, stamp, pose, status="tracking", fresh_localization=True):
        self.adapters.clock.return_value = stamp
        node.dynamic_layer.last_received = stamp
        if fresh_localization:
            node.localization_received = stamp
        node.tracking.update_path_progress(pose, node.global_path)
        node.tracking.controller.status = status
        node.tracking.after_control_cycle()

    def test_stall_detection_uses_path_progress_instead_of_lateral_jitter(self):
        for has_progress in (False, True):
            with self.subTest(has_progress=has_progress):
                node = self.make_node()
                node.request_plan = Mock(return_value=True)
                self.observe_progress(node, 100.0, (0, 0, 0))
                self.observe_progress(node, 103.9, (0, 0, 0))
                node.request_plan.assert_not_called()
                pose = (0.05, 0, 0) if has_progress else (0, 0.05, 0)
                self.observe_progress(node, 104.1, pose)
                if has_progress:
                    node.request_plan.assert_not_called()
                else:
                    node.request_plan.assert_called_once_with(pose, "normal")

    def test_real_rotation_progress_defers_stall_replan(self):
        node = self.make_node()
        node.request_plan = Mock(return_value=True)
        self.observe_progress(node, 100.0, (0, 0, 0), "rotating")
        self.observe_progress(node, 104.1, (0, 0, 0.3), "rotating")
        self.observe_progress(node, 108.2, (0, 0, 0.6), "rotating")
        node.request_plan.assert_not_called()
        self.assertAlmostEqual(node.tracking.controller.progress, 0.0)
        # 转向指令本身不是进展；连续四秒没有真实航向变化仍应尝试恢复。
        self.observe_progress(node, 112.3, (0, 0, 0.6), "rotating")
        node.request_plan.assert_called_once_with((0, 0, 0.6), "normal")

    def test_stale_localization_does_not_trigger_stall_replan(self):
        node = self.make_node()
        node.request_plan = Mock(return_value=True)
        self.observe_progress(node, 100.0, (0, 0, 0))
        self.observe_progress(node, 104.1, (0, 0, 0), fresh_localization=False)
        self.assertTrue(node.localization_valid)
        self.assertTrue(node.dynamic_layer.is_fresh())
        node.request_plan.assert_not_called()


if __name__ == "__main__":
    unittest.main()
