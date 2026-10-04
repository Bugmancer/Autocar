"""三种跟踪策略共用的 RViz 输出；仅显示诊断，不参与运动放行决策。"""

import math
from typing import Optional

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from fw_mid_common_utils import Pose2D
from .path_processing import PathPoint


def publish_collision_preview(node, pose, result):
    """显示当前车身、预测轨迹及碰撞点；无碰撞点时显式删除旧标记。"""
    message = MarkerArray()
    poses = result.trajectory if result is not None else [pose]
    for marker_id, (name, samples) in enumerate((
            ("vehicle_footprint", [pose]), ("predicted_footprints", poses))):
        marker = Marker()
        marker.header.frame_id = node.global_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns, marker.id = name, marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.018 if marker_id == 0 else 0.01
        marker.color.a = 1.0 if marker_id == 0 else 0.6
        if marker_id == 0:
            marker.color.b = 1.0
            marker.color.g = 0.6
        elif result is not None and not result.safe:
            marker.color.r = 1.0
        else:
            marker.color.g = 1.0
        for sample in samples:
            corners = node.collision_checker.footprint(sample)
            for i in range(4):
                for x, y in (corners[i], corners[(i+1) % 4]):
                    marker.points.append(Point(x=x, y=y, z=0.12))
        marker.lifetime = rospy.Duration(0.5)
        message.markers.append(marker)
    obstacle = Marker()
    obstacle.header.frame_id = node.global_frame
    obstacle.header.stamp = rospy.Time.now()
    obstacle.ns, obstacle.id = "blocking_obstacle", 2
    obstacle.type = Marker.SPHERE
    obstacle.pose.orientation.w = 1.0
    obstacle.scale.x = obstacle.scale.y = obstacle.scale.z = 0.14
    obstacle.color.r, obstacle.color.a = 1.0, 1.0
    if result is not None and result.collision_point is not None:
        obstacle.action = Marker.ADD
        obstacle.pose.position.x, obstacle.pose.position.y = result.collision_point
        obstacle.pose.position.z = 0.15
    else:
        obstacle.action = Marker.DELETE
    obstacle.lifetime = rospy.Duration(0.5)
    message.markers.append(obstacle)
    node.collision_marker_pub.publish(message)
    node.collision_blocked_pub.publish(Bool(data=result is not None and not result.safe))

def publish_desired_direction(
    node,
    robot_pose: Pose2D,
    target: Optional[PathPoint],
    velocity_x: float,
    active: bool = True,
) -> None:
    """将所选策略的期望方向画成箭头；非活动状态删除旧箭头，避免误显示运动。"""
    marker = Marker()
    marker.header.frame_id = node.global_frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "desired_motion"
    marker.id = 1
    if not active or target is None:
        marker.action = Marker.DELETE
        node.desired_direction_pub.publish(marker)
        return

    rx, ry, _ = robot_pose
    dx = float(target.x) - rx
    dy = float(target.y) - ry
    distance = math.hypot(dx, dy)
    if not math.isfinite(distance) or distance < 1e-3:
        marker.action = Marker.DELETE
        node.desired_direction_pub.publish(marker)
        return

    # 只约束显示长度，不改变真实目标；过近目标和长路径都保持箭头可辨认。
    arrow_length = max(0.35, min(1.20, distance))
    scale = arrow_length / distance
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.06  # 箭杆直径，单位米。
    marker.scale.y = 0.12  # 箭头直径，单位米。
    marker.scale.z = 0.16  # 箭头长度，单位米。
    marker.color.r = 1.0 if velocity_x < 0.0 else 0.1
    marker.color.g = 0.55 if velocity_x < 0.0 else 1.0
    marker.color.b = 0.05 if velocity_x < 0.0 else 0.1
    marker.color.a = 0.95
    marker.lifetime = rospy.Duration(max(0.2, 2.0 * node.control_dt))
    marker.points = [
        Point(x=rx, y=ry, z=0.16),
        Point(x=rx + dx * scale, y=ry + dy * scale, z=0.16),
    ]
    node.desired_direction_pub.publish(marker)
