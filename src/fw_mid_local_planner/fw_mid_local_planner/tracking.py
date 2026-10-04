"""经典跟踪策略：为公共导航运行层提供 PID 或纯追踪控制器。"""

import rospy

from fw_mid_controller import PIDPathController, PurePursuitController


class ClassicTracking:
    """只创建选中的控制器，并管理停车复位及恢复交接状态。"""

    def __init__(self, follower):
        self.follower = follower
        self.name = str(follower.param("tracking_controller", "pid")).strip().lower()
        controllers = {"pid": PIDPathController, "pure_pursuit": PurePursuitController}
        if self.name not in controllers:
            rospy.logwarn("Unknown tracking_controller=%s; using pid", self.name)
            self.name = "pid"
        follower.tracking_controller = self.name
        self.controller = controllers[self.name](rospy, cruise_speed=follower.command_max_vx)

    def compute(self, pose, path, target, dt):
        """输出候选车身速度，仍须由运行层执行限幅、碰撞和定位检查。"""
        return self.controller.compute(pose, path, target, dt)

    def reset(self):
        self.controller.reset()

    def before_recovery(self):
        # 经典控制器没有独立方向滤波状态；保留钩子以统一策略调用协议。
        pass

    def finish_recovery(self, command):
        # 恢复段由运行层直接控制，从最后实际发布的速度继续做变化率限制。
        self.controller.prev_cmd = command

    def after_control_cycle(self):
        # 经典策略的避障与重规划全部由公共运行层处理，无额外周期任务。
        pass

    def command_diagnostics(self):
        return None

    def command_published(self, accepted, requested_vx, requested_wz, diagnostics):
        # 保持经典控制器原有状态更新约定，不在这里额外改写其积分或滤波状态。
        pass

    def desired_direction(self, robot_pose, target, active):
        return target, active
