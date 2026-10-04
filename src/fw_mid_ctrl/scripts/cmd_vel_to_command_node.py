#!/usr/bin/env python3
"""将 ROS Twist 转换为 FW-mid 底盘 JSON 指令。

``Twist`` 使用 m/s 和 rad/s；发送给 CAN 驱动前，保持线速度单位不变，
仅将角速度转换为底盘协议要求的 deg/s。
"""

import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def build_command(
    vx,
    vy,
    wz_rad,
    *,
    drive_gear=6,
    stop_gear=1,
    max_vx=0.3,
    max_vy=0.0,
    max_wz_deg=30.0,
):
    """检查速度是否有限并按底盘单位限幅，静止时选择驻车档。"""
    # ROS Twist 的角速度为 rad/s，底盘 JSON 使用 deg/s；线速度仍为 m/s。
    values = (float(vx), float(vy), float(wz_rad))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Twist command contains a non-finite value")

    vx_cmd = clamp(values[0], -abs(max_vx), abs(max_vx))
    vy_cmd = clamp(values[1], -abs(max_vy), abs(max_vy))
    wz_cmd = clamp(math.degrees(values[2]), -abs(max_wz_deg), abs(max_wz_deg))
    moving = any(abs(value) > 1e-9 for value in (vx_cmd, vy_cmd, wz_cmd))
    return {
        "gear": int(drive_gear if moving else stop_gear),
        "vx": vx_cmd,
        "vy": vy_cmd,
        "wz": wz_cmd,
    }


class CmdVelToCommand:
    """把导航 Twist 转成限幅后的底盘命令，未收到输入或输入超时则发送驻车。"""
    def __init__(self):
        # ROS 定时器周期发布，单调时钟判断输入过期；下游 CAN 驱动还会独立超时停车。
        self.drive_gear = int(rospy.get_param("~drive_gear", 6))
        self.stop_gear = int(rospy.get_param("~stop_gear", 1))
        self.max_vx = float(rospy.get_param("~max_vx", 0.3))
        self.max_vy = float(rospy.get_param("~max_vy", 0.0))
        self.max_wz_deg = float(rospy.get_param("~max_wz_deg", 30.0))
        self.cmd_timeout = max(0.0, float(rospy.get_param("~cmd_timeout", 0.5)))
        self.publish_rate = max(1.0, float(rospy.get_param("~publish_rate", 20.0)))
        input_topic = str(rospy.get_param("~input_topic", "/cmd_vel"))
        output_topic = str(
            rospy.get_param("~output_topic", "/fw_mid/command_dict")
        )

        self._lock = threading.Lock()
        self._last_command = self._stop_command()
        self._last_input_monotonic = None

        self._publisher = rospy.Publisher(output_topic, String, queue_size=10)
        self._subscriber = rospy.Subscriber(
            input_topic, Twist, self._twist_cb, queue_size=10
        )
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self._timer_cb
        )
        rospy.loginfo(
            "Twist adapter ready: %s -> %s, timeout=%.2f s",
            input_topic,
            output_topic,
            self.cmd_timeout,
        )

    def _stop_command(self):
        return {"gear": self.stop_gear, "vx": 0.0, "vy": 0.0, "wz": 0.0}

    def _twist_cb(self, message):
        try:
            command = build_command(
                message.linear.x,
                message.linear.y,
                message.angular.z,
                drive_gear=self.drive_gear,
                stop_gear=self.stop_gear,
                max_vx=self.max_vx,
                max_vy=self.max_vy,
                max_wz_deg=self.max_wz_deg,
            )
        except (TypeError, ValueError) as error:
            # 非法速度覆盖为停车，避免上一次合法运动指令继续被定时重发。
            rospy.logerr_throttle(1.0, "Rejected /cmd_vel: %s", error)
            command = self._stop_command()

        with self._lock:
            self._last_command = command
            self._last_input_monotonic = time.monotonic()

    def _timer_cb(self, _event):
        # 用单调时钟衡量命令年龄，避免 ROS 仿真时间跳变影响超时判断。
        now = time.monotonic()
        with self._lock:
            # 复制命令与时间戳后释放锁，JSON 编码和 ROS 发布不阻塞输入回调。
            command = dict(self._last_command)
            last_input = self._last_input_monotonic

        if last_input is None or now - last_input > self.cmd_timeout:
            command = self._stop_command()
            if last_input is not None:
                rospy.logwarn_throttle(1.0, "Twist input timed out; publishing stop")

        message = String()
        message.data = json.dumps(command, separators=(",", ":"), allow_nan=False)
        self._publisher.publish(message)


def main():
    rospy.init_node("cmd_vel_to_command")
    CmdVelToCommand()
    rospy.spin()


if __name__ == "__main__":
    main()
