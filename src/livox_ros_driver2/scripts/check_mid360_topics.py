#!/usr/bin/env python3
"""Fail the launch when the single MID360 does not supply fresh lidar and IMU data."""

from collections import deque
import math
import threading
import time

import rospy
from sensor_msgs.msg import Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg


class TopicWindow:
    def __init__(self, seconds=3.0):
        self.seconds = seconds
        self.samples = deque(maxlen=20000)
        self.last_stamp = None
        self.lock = threading.Lock()

    def add(self, stamp, now):
        if not math.isfinite(stamp) or stamp <= 0:
            return
        with self.lock:
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return
            self.last_stamp = stamp
            self.samples.append(now)

    def rate(self, now):
        with self.lock:
            while self.samples and now - self.samples[0] > self.seconds:
                self.samples.popleft()
            if len(self.samples) < 2 or now - self.samples[-1] > 1.0:
                return 0.0
            span = self.samples[-1] - self.samples[0]
            return (len(self.samples) - 1) / span if span >= self.seconds * 0.8 else 0.0


def monitor():
    rospy.init_node("mid360_topic_check")
    timeout = float(rospy.get_param("~startup_timeout", 45.0))
    lidar_min = float(rospy.get_param("~min_lidar_hz", 5.0))
    imu_min = float(rospy.get_param("~min_imu_hz", 50.0))
    window = float(rospy.get_param("~window_seconds", 3.0))
    if not all(math.isfinite(v) and v > 0 for v in (timeout, lidar_min, imu_min, window)):
        raise ValueError("Topic timeouts, window and minimum frequencies must be finite and positive")
    lidar, imu = TopicWindow(window), TopicWindow(window)
    cloud_type = CustomMsg if int(rospy.get_param("~xfer_format", 1)) == 1 else PointCloud2

    def lidar_cb(msg):
        count = min(msg.point_num, len(msg.points)) if cloud_type is CustomMsg else msg.width * msg.height
        if count > 0:
            lidar.add(msg.header.stamp.to_sec(), time.monotonic())

    def imu_cb(msg):
        vectors = (msg.linear_acceleration, msg.angular_velocity)
        if all(math.isfinite(value) for vector in vectors for value in (vector.x, vector.y, vector.z)):
            imu.add(msg.header.stamp.to_sec(), time.monotonic())

    lidar_sub = rospy.Subscriber("/livox/lidar", cloud_type, lidar_cb, queue_size=1)
    imu_sub = rospy.Subscriber("/livox/imu", Imu, imu_cb, queue_size=100)
    deadline = time.monotonic() + timeout
    passed = False
    bad_since = None
    while not rospy.is_shutdown():
        now = time.monotonic()
        rates = lidar.rate(now), imu.rate(now)
        connections = lidar_sub.get_num_connections(), imu_sub.get_num_connections()
        if any(count > 1 for count in connections):
            rospy.logfatal("Multiple MID360 topic publishers detected; refusing duplicate drivers")
            return 1
        healthy = connections == (1, 1) and rates[0] >= lidar_min and rates[1] >= imu_min
        if healthy:
            bad_since = None
            if not passed:
                rospy.loginfo("MID360 topics ready: lidar=%.1f Hz, imu=%.1f Hz", *rates)
                passed = True
        elif passed:
            if bad_since is None:
                bad_since = now
            if now - bad_since > window:
                rospy.logfatal("MID360 stream lost/too slow: lidar=%.1f Hz, imu=%.1f Hz", *rates)
                return 1
        elif now > deadline:
            rospy.logfatal("MID360 startup timed out: lidar=%.1f Hz, imu=%.1f Hz; "
                           "check mid360_startup.log, topic types and network", *rates)
            return 1
        # Use wall time, including when a ROS simulated clock is paused.
        time.sleep(0.2)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(monitor())
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("MID360 topic check failed: %s", error)
        raise SystemExit(1)
