#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Project-owned ROS/CAN adapter for the FW-mid chassis.

The bundled ``can`` package under this directory is an imported transport
implementation and is intentionally left unchanged.  This node validates the
JSON command contract, applies the watchdog, packs the chassis bit layout,
and decodes feedback into ROS topics.
"""

import json
import math
import os
import struct
import sys
import threading
import time
from typing import Tuple

# Catkin's devel wrapper does not add the source script directory to sys.path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from can.interfaces.socketcan import SocketcanBus
import rospy
from std_msgs.msg import String, Float32MultiArray


def reject_constant(value):
    raise ValueError('Non-finite JSON constant: ' + value)


def finite_number(value):
    if isinstance(value, bool):
        raise ValueError('Boolean is not a command number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('Command number must be finite')
    return number


def parse_command(data, max_vx, max_vy, max_wz, drive_gear=6, stop_gear=1):
    """校验 JSON 并限幅；停车类档位强制速度为零，wz 的输入单位为 deg/s。"""
    cmd = json.loads(data, parse_constant=reject_constant)
    if not isinstance(cmd, dict):
        raise ValueError('Command must be a JSON object')
    gear = finite_number(cmd.get('gear', drive_gear))
    if gear not in (0, 1, 2, 6, 8):
        raise ValueError('Unsupported gear')
    speeds = []
    for key, limit in zip(('vx', 'vy', 'wz'), (max_vx, max_vy, max_wz)):
        value = finite_number(cmd.get(key, 0.0))
        speeds.append(max(-limit, min(limit, value)))
    if gear in (0, 1, 2):
        speeds = [0.0, 0.0, 0.0]
    elif not any(speeds):
        gear = stop_gear
    return (int(gear), *speeds)


class FwMidCanDriver:
    """底盘 CAN 收发边界：周期发送受看门狗保护的指令，并解码车辆反馈。"""
    def __init__(self) -> None:
        can_iface = rospy.get_param('~can_interface', 'can0')

        self.max_vx = float(rospy.get_param('~max_vx', 0.6))          # m/s
        self.max_vy = float(rospy.get_param('~max_vy', 0.6))          # m/s
        self.max_wz = float(rospy.get_param('~max_wz', 60.0))         # degree/s
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        self.send_rate = float(rospy.get_param('~send_rate', 20.0))
        self.drive_gear = int(rospy.get_param('~drive_gear', 6))
        self.stop_gear = int(rospy.get_param('~stop_gear', 1))
        if not all(math.isfinite(v) and v >= 0 for v in
                   (self.max_vx, self.max_vy, self.max_wz)):
            raise ValueError('Velocity limits must be finite and non-negative')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.cmd_timeout, self.send_rate)):
            raise ValueError('cmd_timeout and send_rate must be finite and positive')
        if self.drive_gear not in (6, 8) or self.stop_gear != 1:
            raise ValueError('drive_gear must be 6 or 8; stop_gear must be 1')

        self.alive_counter = 0
        self.last_cmd_time = None
        self.last_cmd: Tuple[int, float, float, float] = (self.stop_gear, 0.0, 0.0, 0.0)
        self.lock = threading.Lock()
        self._shutdown = threading.Event()

        try:
            self.bus = SocketcanBus(channel=can_iface)
            rospy.loginfo('Connected to CAN interface: %s', can_iface)
        except Exception as exc:
            rospy.logerr('Cannot open CAN interface %s: %s', can_iface, exc)
            raise

        self.fb_vel_pub = rospy.Publisher('/fw_mid/feedback/velocity', Float32MultiArray, queue_size=10)
        self.fb_bms_pub = rospy.Publisher('/fw_mid/feedback/bms', Float32MultiArray, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/fw_mid/command_dict', String, self.cmd_cb, queue_size=1)
        self.receive_thread = threading.Thread(target=self.receive_can_messages, daemon=True)
        self.tx_thread = threading.Thread(target=self.transmit_loop, daemon=True)
        rospy.on_shutdown(self.shutdown)
        self.receive_thread.start()
        self.tx_thread.start()

        rospy.loginfo('FW-mid CAN driver ready. send_rate=%.1f Hz, cmd_timeout=%.2f s', self.send_rate, self.cmd_timeout)

    def cmd_cb(self, msg: String) -> None:
        try:
            command = parse_command(msg.data, self.max_vx, self.max_vy,
                                    self.max_wz, self.drive_gear, self.stop_gear)
            received = time.monotonic()
        except (TypeError, ValueError, OverflowError) as exc:
            rospy.logerr_throttle(1.0, 'Rejected command; stopping: %s', exc)
            command = (self.stop_gear, 0.0, 0.0, 0.0)
            received = None
        with self.lock:
            self.last_cmd = command
            self.last_cmd_time = received

    def transmit_loop(self) -> None:
        # 独立线程按墙钟周期发送，ROS /clock 暂停也不会停掉硬件看门狗。
        # The hardware watchdog must keep running even if ROS /clock pauses.
        while not self._shutdown.is_set():
            self.tx_timer_cb(None)
            self._shutdown.wait(1.0 / self.send_rate)

    def tx_timer_cb(self, _event) -> None:
        if self.bus is None:
            return

        now = time.monotonic()
        with self.lock:
            age = now - self.last_cmd_time if self.last_cmd_time is not None else float('inf')
            if age > self.cmd_timeout:
                gear, vx, vy, wz = self.stop_gear, 0.0, 0.0, 0.0
                rospy.logwarn_throttle(1.0, 'Command timeout %.2f s. Send stop command.', age)
            else:
                gear, vx, vy, wz = self.last_cmd

            self.send_can_ctrl_msg(gear, vx, vy, wz)

    def shutdown(self) -> None:
        if self._shutdown.is_set():
            return
        self._shutdown.set()
        self.tx_thread.join(timeout=1.0)
        with self.lock:
            self.send_can_ctrl_msg(self.stop_gear, 0.0, 0.0, 0.0)
        self.receive_thread.join(timeout=1.0)
        self.bus.shutdown()

    def send_can_ctrl_msg(self, gear: int, vx: float, vy: float, wz: float) -> None:
        """Pack physical values into FW-mid CAN control frame."""
        vx_raw = int(vx / 0.001)    # 0.001 m/s/bit
        vy_raw = int(vy / 0.001)    # 0.001 m/s/bit
        wz_raw = int(wz / 0.01)     # 0.01 degree/s/bit

        vx_raw = max(-32768, min(32767, vx_raw))
        vy_raw = max(-32768, min(32767, vy_raw))
        wz_raw = max(-32768, min(32767, wz_raw))

        # 有符号速度先转为 16 位补码，再按协议位偏移拼入 64 位小端载荷。
        vx_u16 = vx_raw & 0xFFFF
        vy_u16 = vy_raw & 0xFFFF
        wz_u16 = wz_raw & 0xFFFF

        payload = 0
        payload |= (gear & 0x0F) << 0
        payload |= (vx_u16 & 0xFFFF) << 4
        payload |= (wz_u16 & 0xFFFF) << 20
        payload |= (vy_u16 & 0xFFFF) << 36
        payload |= (self.alive_counter & 0x0F) << 52

        # 前七字节异或生成 BCC；4 位 alive_counter 只在成功发送后递增。
        data = bytearray(struct.pack('<Q', payload))
        bcc = 0
        for i in range(7):
            bcc ^= data[i]
        data[7] = bcc

        msg = can.Message(arbitration_id=0x18C4D1D0, data=data, is_extended_id=True)
        try:
            self.bus.send(msg, timeout=0.05)
            self.alive_counter = (self.alive_counter + 1) % 16
        except can.CanError as exc:
            rospy.logerr_throttle(1.0, 'CAN send failed: %s', exc)

    def receive_can_messages(self) -> None:
        # 仅解析长度正确且 BCC 通过的扩展帧，避免损坏帧污染速度/电池反馈。
        while not self._shutdown.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                if not msg.is_extended_id or len(msg.data) != 8:
                    continue

                if len(msg.data) == 8:
                    calc_bcc = 0
                    for i in range(7):
                        calc_bcc ^= msg.data[i]
                    if calc_bcc != msg.data[7]:
                        rospy.logwarn('Frame %s BCC check failed.', hex(msg.arbitration_id))
                        continue

                if msg.arbitration_id == 0x18C4D1EF:
                    self.parse_ctrl_fb(msg.data)
                elif msg.arbitration_id == 0x18C4E1EF:
                    self.parse_bms_fb(msg.data)
                elif msg.arbitration_id == 0x18C4DAEF:
                    self.parse_io_fb(msg.data)

            except (can.CanError, OSError) as exc:
                rospy.logerr_throttle(1.0, 'CAN receive failed: %s', exc)
                self._shutdown.wait(0.1)

    def parse_ctrl_fb(self, data: bytes) -> None:
        # 反馈沿用控制帧的位域，提取后需恢复有符号速度及物理单位。
        payload = struct.unpack('<Q', data)[0]
        gear_raw = (payload >> 0) & 0x0F
        vx_raw = (payload >> 4) & 0xFFFF
        wz_raw = (payload >> 20) & 0xFFFF
        vy_raw = (payload >> 36) & 0xFFFF

        if vx_raw & 0x8000:
            vx_raw -= 0x10000
        if vy_raw & 0x8000:
            vy_raw -= 0x10000
        if wz_raw & 0x8000:
            wz_raw -= 0x10000

        vx = vx_raw * 0.001
        vy = vy_raw * 0.001
        wz = wz_raw * 0.01

        gear_dict = {0: 'Disable', 1: '驻车', 2: '空档', 6: '4T4D', 8: '横移'}
        gear_str = gear_dict.get(gear_raw, '未知')
        rospy.loginfo_throttle(0.5, '[运动反馈] 档位: %s, Vx: %.3f m/s, Vy: %.3f m/s, Wz: %.2f deg/s', gear_str, vx, vy, wz)

        msg = Float32MultiArray()
        msg.data = [float(gear_raw), vx, vy, wz]
        self.fb_vel_pub.publish(msg)

    def parse_bms_fb(self, data: bytes) -> None:
        payload = struct.unpack('<Q', data)[0]
        vol_raw = (payload >> 0) & 0xFFFF
        cur_raw = (payload >> 16) & 0xFFFF
        cap_raw = (payload >> 32) & 0xFFFF

        if cur_raw & 0x8000:
            cur_raw -= 0x10000

        voltage = vol_raw * 0.01
        current = cur_raw * 0.01
        capacity = cap_raw * 0.01

        rospy.loginfo_throttle(2.0, '[电池反馈] 电压: %.2f V, 电流: %.2f A, 剩余容量: %.2f Ah', voltage, current, capacity)

        msg = Float32MultiArray()
        msg.data = [voltage, current, capacity]
        self.fb_bms_pub.publish(msg)

    def parse_io_fb(self, data: bytes) -> None:
        payload = struct.unpack('<Q', data)[0]
        estop_raw = (payload >> 40) & 0x01
        rc_status_raw = (payload >> 41) & 0x01

        estop_str = '被按下 (停车)' if estop_raw == 1 else '已释放 (正常)'
        rc_str = '遥控器控制' if rc_status_raw == 1 else '指令(CAN)控制'
        rospy.loginfo_throttle(2.0, '[IO反馈] 急停状态: %s | 控制权: %s', estop_str, rc_str)


if __name__ == '__main__':
    try:
        rospy.init_node('fw_mid_can_driver')
        node = FwMidCanDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
