#!/usr/bin/env python3
"""自适应导航的独立入口；通过组合选择策略，不继承已有导航节点。"""

import rospy

from .adaptive_tracking import AdaptiveTracking
from .follower_runtime import FollowerRuntime


def main() -> None:
    # 三个入口互斥使用同一节点名，保持私有服务和 RViz 话题地址稳定。
    rospy.init_node("path_follower_node")
    FollowerRuntime(AdaptiveTracking)
    rospy.spin()


if __name__ == "__main__":
    main()
