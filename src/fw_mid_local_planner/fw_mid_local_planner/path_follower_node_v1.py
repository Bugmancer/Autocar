#!/usr/bin/env python3
"""人工势场导航的独立入口；通过组合选择策略，不继承经典导航节点。"""

import rospy

from .apf_tracking import APFTracking
from .follower_runtime import FollowerRuntime


def main() -> None:
    # 三个入口互斥使用同一节点名，保持已有私有服务和 RViz 话题地址稳定。
    rospy.init_node("path_follower_node")
    FollowerRuntime(APFTracking)
    rospy.spin()


if __name__ == "__main__":
    main()
