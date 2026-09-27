#!/usr/bin/env python3
"""Save a fresh localized map-to-body pose as this project's next startup seed."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


def pose_to_initial_pose(message):
    if message.header.frame_id.lstrip('/') != 'map':
        raise ValueError('Expected /localizer/pose in the map frame')
    position = message.pose.position
    orientation = message.pose.orientation
    xyz = [position.x, position.y, position.z]
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    if not all(math.isfinite(value) for value in xyz + quaternion):
        raise ValueError('Pose contains NaN or infinity')
    norm = math.hypot(*quaternion)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError('Pose has an invalid quaternion')
    quaternion = [value / norm for value in quaternion]
    # The Relocalize service composes Rz(yaw) * Rx(roll) * Ry(pitch).
    yaw, roll, pitch = euler_from_quaternion(quaternion, axes='rzxy')
    return [round(value, 6) for value in xyz + [yaw, pitch, roll]]


class PoseCapture:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None
        self.pose_received = 0.0
        self.valid_received = 0.0
        self.valid_count = 0

    def pose_callback(self, message):
        with self.lock:
            self.pose = message
            self.pose_received = time.monotonic()

    def valid_callback(self, message):
        with self.lock:
            self.valid_received = time.monotonic()
            if message.data:
                self.valid_count += 1
            else:
                self.valid_count = 0
                self.pose = None

    def current(self, max_age):
        with self.lock:
            # Require another status update after the possible latched message.
            if self.pose is None or self.valid_count < 2:
                return None
            now = time.monotonic()
            if now - self.pose_received > max_age or now - self.valid_received > max_age:
                return None
            stamp = self.pose.header.stamp
            age = (rospy.Time.now() - stamp).to_sec()
            if stamp.is_zero() or age < -0.05 or age > max_age:
                return None
            return self.pose


def save_seed(path, seed):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(seed, separators=(',', ':'), allow_nan=False)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='ascii', dir=str(path.parent),
                                         prefix='.' + path.name + '.', delete=False) as stream:
            temporary_path = stream.name
            stream.write(text + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, str(path))
        temporary_path = None
    finally:
        if temporary_path is not None:
            os.unlink(temporary_path)
    return path, text


def positive_seconds(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError('must be finite and greater than zero')
    return seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1] / '.ros' / 'initial_pose.txt',
                        help='destination for [x,y,z,yaw,pitch,roll] (default: .ros/initial_pose.txt)')
    parser.add_argument('--timeout', type=positive_seconds, default=10.0,
                        help='maximum wall-clock wait in seconds (default: 10)')
    parser.add_argument('--max-age', type=positive_seconds, default=0.5,
                        help='maximum pose/status age in seconds (default: 0.5)')
    args = parser.parse_args(rospy.myargv()[1:])
    reason = ('No fresh, valid localization within %.1f s. Start localization and '
              'wait for /localizer/localization_valid=true; keep the vehicle stationary.'
              % args.timeout)
    deadline = time.monotonic() + args.timeout
    timer = threading.Timer(args.timeout, lambda: rospy.signal_shutdown(reason))
    timer.daemon = True
    timer.start()
    subscribers = []
    try:
        rospy.init_node('save_initial_pose', anonymous=True, disable_rosout=True)
        capture = PoseCapture()
        subscribers = [
            rospy.Subscriber('/localizer/pose', PoseStamped, capture.pose_callback, queue_size=1),
            rospy.Subscriber('/localizer/localization_valid', Bool,
                             capture.valid_callback, queue_size=1),
        ]
        while not rospy.is_shutdown():
            message = capture.current(args.max_age)
            if message is not None:
                seed = pose_to_initial_pose(message)
                path, text = save_seed(args.output, seed)
                print('initial_pose:="%s"' % text)
                print('Saved map -> body seed to %s' % path, file=sys.stderr)
                return 0
            time.sleep(0.02)
        raise RuntimeError(reason)
    except (rospy.ROSException, RuntimeError, ValueError, OSError) as error:
        detail = reason if time.monotonic() >= deadline else str(error)
        if isinstance(error, rospy.ROSException):
            detail += '; start ROS localization before capturing a pose'
        print('Failed to save initial pose: %s' % detail, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        timer.cancel()
        for subscriber in subscribers:
            subscriber.unregister()
        rospy.signal_shutdown('Pose capture finished')


if __name__ == '__main__':
    sys.exit(main())
