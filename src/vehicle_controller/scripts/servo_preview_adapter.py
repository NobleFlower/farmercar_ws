#!/usr/bin/env python3
"""Convert chassis Twist to existing EE protocol for inspection ONLY.

No serial module, device access, or hardware command publisher exists here.
"""
import json
import math
import struct
import threading
import time

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, UInt8MultiArray


def encode_preview(v, omega, wheelbase, max_angle_deg, neutral_deg, sign, speed_units):
    values = (v, omega, wheelbase, max_angle_deg, neutral_deg, sign, speed_units)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('non-finite command or configuration')
    if wheelbase <= 0 or speed_units <= 0 or sign not in (-1, 1):
        raise ValueError('invalid calibration')
    if v < 0:
        raise ValueError('reverse disabled for tag following')
    if abs(v) < 1e-6:
        if abs(omega) > 1e-6:
            raise ValueError('Ackermann chassis cannot rotate in place')
        angle = 0.0
    else:
        angle = math.degrees(math.atan(wheelbase * omega / v))
    if abs(angle) > max_angle_deg + 1e-6:
        raise ValueError('steering exceeds chassis limit')
    protocol_deg = neutral_deg + sign * angle
    speed_raw = round(v * speed_units * 100)
    steering_raw = round(protocol_deg * 10)
    if not 0 <= speed_raw <= 32767 or not 0 <= steering_raw <= 1800:
        raise ValueError('protocol range exceeded')
    payload = bytes([0xEE, 0x01, 0x04]) + struct.pack('<hH', speed_raw, steering_raw)
    packet = list(payload + bytes([sum(payload) & 0xFF]))
    return packet, angle, protocol_deg


class PreviewAdapter:
    def __init__(self):
        self.lock = threading.Lock()
        self.cmd = None
        self.received = 0.0
        self.wheelbase = float(rospy.get_param('~wheelbase', 0.839))
        self.max_angle = float(rospy.get_param('~max_steering_deg', 50.0))
        self.max_speed = float(rospy.get_param('~max_speed', 0.15))
        self.neutral = float(rospy.get_param('~steering_neutral_deg', 90.0))
        self.sign = float(rospy.get_param('~steering_sign', 1.0))
        self.speed_units = float(rospy.get_param('~protocol_speed_units_per_mps', 1.0))
        self.timeout = float(rospy.get_param('~command_timeout', 0.5))
        self.packet_pub = rospy.Publisher('/vehicle_controller/preview/motion_packet', UInt8MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher('/vehicle_controller/preview/status', String, queue_size=1)
        rospy.Subscriber(rospy.get_param('~command_topic', '/apriltag_servo/preview/cmd_vel'), TwistStamped, self.callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.logwarn('Protocol preview only: no serial port is opened. Speed units / steering sign need calibration.')

    def callback(self, msg):
        with self.lock:
            self.cmd = msg
            self.received = time.monotonic()

    def tick(self, event):
        with self.lock:
            msg, received = self.cmd, self.received
        reason = 'ok'
        v, omega = 0.0, 0.0
        try:
            if msg is None or time.monotonic() - received > self.timeout:
                raise ValueError('command_timeout')
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if msg.header.stamp.is_zero() or age > self.timeout or age < -0.05:
                raise ValueError('invalid_command_stamp')
            if msg.header.frame_id != 'base_link':
                raise ValueError('expected base_link frame')
            t = msg.twist
            unsupported = (t.linear.y, t.linear.z, t.angular.x, t.angular.y)
            if any(not math.isfinite(x) or abs(x) > 1e-6 for x in unsupported):
                raise ValueError('nonplanar_command')
            v, omega = t.linear.x, t.angular.z
            if not math.isfinite(v) or v > self.max_speed + 1e-6:
                raise ValueError('speed_limit')
            packet, angle, protocol_angle = encode_preview(v, omega, self.wheelbase, self.max_angle, self.neutral, self.sign, self.speed_units)
        except ValueError as exc:
            reason = str(exc)
            v, omega = 0.0, 0.0
            packet, angle, protocol_angle = encode_preview(0, 0, self.wheelbase, self.max_angle, self.neutral, self.sign, self.speed_units)
        self.packet_pub.publish(UInt8MultiArray(data=packet))
        self.status_pub.publish(String(data=json.dumps(dict(preview_only=True, calibration_confirmed=False, reason=reason, speed_mps=v, yaw_rate=omega, steering_deg=angle, protocol_steering_deg=protocol_angle, packet_hex=' '.join('{:02X}'.format(x) for x in packet)))))


if __name__ == '__main__':
    rospy.init_node('servo_preview_adapter')
    PreviewAdapter()
    rospy.spin()
