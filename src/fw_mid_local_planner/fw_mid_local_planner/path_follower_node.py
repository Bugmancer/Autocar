#!/usr/bin/env python3
"""PID/纯追踪导航的独立入口；与 APF 入口二选一启动。"""

import rospy

from .follower_runtime import FollowerRuntime
from .tracking import ClassicTracking


def main() -> None:
    # 入口只选择策略；公共运行层统一持有回调、规划状态和安全检查。
    rospy.init_node("path_follower_node")
    FollowerRuntime(ClassicTracking)
    rospy.spin()


if __name__ == "__main__":
    main()
