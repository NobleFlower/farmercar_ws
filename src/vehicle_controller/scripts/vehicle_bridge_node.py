#!/usr/bin/env python3
"""Chassis command -> gated serial TX; serial RX -> truthful ROS feedback."""
import copy
import json
import math
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt8MultiArray
from tricycle_controller.msg import ChassisCommand
from vehicle_controller.msg import VehicleFeedback
from vehicle_controller.wire import Calibration, FeedbackParser, decode_feedback, encode_motion
from vehicle_controller.transport import SerialTransport


class VehicleBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.cal = Calibration(float(rospy.get_param('~protocol_speed_units_per_mps', 1)),
                               float(rospy.get_param('~steering_neutral_deg', 90)),
                               int(rospy.get_param('~steering_sign', 1)),
                               float(rospy.get_param('~max_speed', .15)),
                               float(rospy.get_param('~max_steering_deg', 50)))
        self.cal.validate()
        self.version = rospy.get_param('~protocol_version', 'legacy')
        if self.version not in ('legacy', 'extended'):
            raise ValueError('protocol_version must be legacy or extended')
        self.serial_enabled = bool(rospy.get_param('~serial_enabled', False))
        self.timeout = float(rospy.get_param('~command_timeout', .5))
        self.feedback_timeout = float(rospy.get_param('~feedback_timeout', .5))
        self.wheelbase = float(rospy.get_param('~wheelbase', .839))
        rate = float(rospy.get_param('~output_rate', 20))
        if not all(math.isfinite(x) and x > 0 for x in (self.timeout, self.feedback_timeout, self.wheelbase, rate)):
            raise ValueError('invalid positive timing/geometry parameter')
        self.require_feedback = bool(rospy.get_param('~require_feedback', True))
        self.allow_legacy = bool(rospy.get_param('~allow_legacy_actuation', False))
        self.require_ack = bool(rospy.get_param('~require_ack', False))
        self.max_ack_lag = int(rospy.get_param('~max_ack_lag', 10))
        if not 1 <= self.max_ack_lag <= 32767:
            raise ValueError('invalid max_ack_lag')
        self.legacy_speed_measured = bool(rospy.get_param('~legacy_speed_is_measured', False))
        self.legacy_steer_measured = bool(rospy.get_param('~legacy_steering_is_measured', False))
        self.publish_simulated_joints = bool(rospy.get_param('~publish_simulated_joint_states', False))
        self.armed = False
        self.follow_enabled = False
        self.estop = False
        self.command = None
        self.feedback = None
        self.last_joint_stamp = None
        self.sequence = 0
        self.io_fault = None
        self.parser = FeedbackParser()
        self.transport = None
        if self.serial_enabled:
            state = rospy.get_master().getSystemState()[2]
            nodes = {name for group in state for _, names in group for name in names}
            if any(name.rsplit('/', 1)[-1] == 'vehicle_controller_node' for name in nodes):
                raise ValueError('legacy vehicle_controller_node is already running; use one serial owner')
            self.transport = SerialTransport(rospy.get_param('~port', '/dev/ttyACM0'),
                                             int(rospy.get_param('~baudrate', 115200)))
        self.tx_pub = rospy.Publisher('/vehicle_controller/tx_packet', UInt8MultiArray, queue_size=1)
        self.desired_pub = rospy.Publisher('/vehicle_controller/desired_packet', UInt8MultiArray, queue_size=1)
        self.feedback_pub = rospy.Publisher('/vehicle_controller/feedback', VehicleFeedback, queue_size=1)
        self.status_pub = rospy.Publisher('/vehicle_controller/status', String, queue_size=1)
        self.joint_pub = rospy.Publisher('/vehicle/steering_joint_states', JointState, queue_size=1)
        self.cmd_sub = rospy.Subscriber('chassis_command', ChassisCommand, self.receive_command, queue_size=1)
        self.arm_sub = rospy.Subscriber('/vehicle_controller/arm', Bool, self.set_arm, queue_size=1)
        self.estop_sub = rospy.Subscriber('/vehicle_controller/emergency_stop', Bool, self.set_estop, queue_size=1)
        self.enable_sub = rospy.Subscriber('/apriltag_servo/enable', Bool, self.set_follow, queue_size=1)
        if not self.serial_enabled:
            # Diagnostic injection is never subscribed when a real serial port is enabled.
            self.inject_sub = rospy.Subscriber('/vehicle_controller/rx_inject', UInt8MultiArray,
                                              self.inject, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1/rate), self.tick)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('Vehicle bridge serial_enabled=%s protocol=%s; starts disarmed.', self.serial_enabled, self.version)

    def receive_command(self, message):
        with self.lock:
            self.command = copy.deepcopy(message), time.monotonic()

    def set_arm(self, message):
        with self.lock:
            self.armed = bool(message.data) and not self.estop and self.io_fault is None
            if self.feedback and (self.feedback[0]['emergency_stop'] or self.feedback[0]['fault_flags'] or
                                  (self.feedback[0]['status_authoritative'] and self.feedback[0]['control_mode'] == 1)):
                self.armed = False

    def set_estop(self, message):
        with self.lock:
            self.estop = bool(message.data)
            if self.estop:
                self.armed = False

    def set_follow(self, message):
        with self.lock:
            self.follow_enabled = bool(message.data)

    def inject(self, message):
        with self.lock:
            self.receive_bytes(bytes(message.data), simulated=True)

    def receive_bytes(self, data, simulated):
        for frame in self.parser.feed(data):
            try:
                decoded = decode_feedback(frame, self.cal, self.legacy_speed_measured, self.legacy_steer_measured)
            except ValueError:
                self.parser.errors += 1
                continue
            self.feedback = decoded, time.monotonic(), rospy.Time.now(), simulated
            if (decoded['emergency_stop'] or decoded['fault_flags'] or
                    (decoded['status_authoritative'] and decoded['control_mode'] == 1)):
                self.armed = False

    def validate_command(self, now):
        if self.command is None:
            return 0, 0, 'waiting_for_command'
        msg, received = self.command
        if (time.monotonic()-received > self.timeout or msg.header.stamp == rospy.Time(0) or
                (now-msg.header.stamp).to_sec() > self.timeout or (now-msg.header.stamp).to_sec() < -.05):
            return 0, 0, 'command_timeout'
        if msg.header.frame_id != 'base_link' or not msg.accepted:
            return 0, 0, 'command_not_accepted'
        if not all(math.isfinite(x) for x in (msg.speed, msg.steering_angle, msg.yaw_rate)):
            return 0, 0, 'nonfinite_command'
        if abs(msg.speed) > self.cal.max_speed+1e-6 or abs(math.degrees(msg.steering_angle)) > self.cal.max_steering_deg+1e-6:
            return 0, 0, 'command_exceeds_limits'
        if abs(msg.yaw_rate-msg.speed*math.tan(msg.steering_angle)/self.wheelbase) > 1e-5:
            return 0, 0, 'inconsistent_kinematics'
        return msg.speed, msg.steering_angle, 'ok'

    def gate_reason(self, command_reason):
        if self.io_fault:
            return 'serial_io_fault'
        if self.estop:
            return 'host_emergency_stop'
        if not self.follow_enabled:
            return 'follow_disabled'
        if not self.armed:
            return 'driver_disarmed'
        if command_reason != 'ok':
            return command_reason
        if self.require_feedback:
            if self.feedback is None or time.monotonic()-self.feedback[1] > self.feedback_timeout:
                return 'feedback_timeout'
        if self.feedback:
            state = self.feedback[0]
            if state['status_authoritative']:
                if state['emergency_stop'] or state['fault_flags']:
                    return 'controller_fault_or_emergency_stop'
                if not state['controller_ready'] or state['control_mode'] != 2:
                    return 'controller_not_ready_for_auto'
            elif not self.allow_legacy:
                return 'legacy_feedback_requires_explicit_opt_in'
            if self.require_ack and (not state['ack_valid'] or
                    ((self.sequence-1-state['ack_sequence']) & 65535) > self.max_ack_lag):
                return 'acknowledgement_missing_or_late'
        elif self.require_ack:
            return 'acknowledgement_missing_or_late'
        return 'ok'

    def publish_feedback(self):
        msg = VehicleFeedback()
        msg.header.frame_id = 'base_link'
        if self.feedback:
            state, received, stamp, simulated = self.feedback
            msg.header.stamp = stamp  # Never turn an old measurement into a fresh one.
            msg.received = True
            msg.fresh = time.monotonic()-received <= self.feedback_timeout
            msg.simulated = simulated
            for key, value in state.items():
                setattr(msg, key, value)
            if (msg.fresh and msg.steering_is_measured and stamp != self.last_joint_stamp
                    and (not simulated or self.publish_simulated_joints)):
                joint = JointState()
                joint.header.stamp = stamp
                joint.name = ['steering_joint']
                joint.position = [msg.steering_angle]
                self.joint_pub.publish(joint)
                self.last_joint_stamp = stamp
        self.feedback_pub.publish(msg)

    def tick(self, event):
        with self.lock:
            if self.transport and self.io_fault is None:
                try:
                    self.receive_bytes(self.transport.read(), simulated=False)
                except (OSError, ValueError) as exc:
                    self.io_fault = str(exc)
                    self.armed = False
            now = rospy.Time.now()
            desired_speed, desired_steer, command_reason = self.validate_command(now)
            desired = encode_motion(desired_speed, desired_steer, self.cal, self.version,
                                    command_reason == 'ok', self.sequence, auto_requested=True)
            reason = self.gate_reason(command_reason)
            permitted = reason == 'ok'
            speed, steer = (desired_speed, desired_steer) if permitted else (0, 0)
            auto_request = self.follow_enabled and self.armed and not self.estop and self.io_fault is None
            tx = encode_motion(speed, steer, self.cal, self.version, permitted, self.sequence,
                               auto_requested=auto_request)
            written = False
            if self.transport and self.io_fault is None:
                try:
                    self.transport.write(tx)
                    written = True
                except (OSError, TimeoutError) as exc:
                    self.io_fault = str(exc)
                    self.armed = False
                    reason = 'serial_io_fault'
                    permitted = False
            self.sequence = (self.sequence+1) & 65535
            self.desired_pub.publish(UInt8MultiArray(data=list(desired)))
            self.tx_pub.publish(UInt8MultiArray(data=list(tx)))
            self.publish_feedback()
            self.status_pub.publish(String(data=json.dumps(dict(
                serial_enabled=self.serial_enabled, packet_written=written,
                actuation_permitted=permitted and self.serial_enabled, dry_run=not self.serial_enabled,
                armed=self.armed, follow_enabled=self.follow_enabled, host_emergency_stop=self.estop,
                reason=reason, command_reason=command_reason,
                desired_speed_mps=desired_speed, desired_steering_rad=desired_steer,
                output_speed_mps=speed, output_steering_rad=steer,
                feedback_received=self.feedback is not None, parser_errors=self.parser.errors,
                protocol_version=self.version, io_fault=self.io_fault,
                desired_hex=desired.hex(' '), tx_hex=tx.hex(' ')), allow_nan=False)))

    def shutdown(self):
        with self.lock:
            self.armed = False
            stop = encode_motion(0, 0, self.cal, self.version, False, self.sequence)
            self.tx_pub.publish(UInt8MultiArray(data=list(stop)))
            if self.transport:
                try:
                    self.transport.write(stop)
                except (OSError, TimeoutError):
                    pass
                self.transport.close()
                self.transport = None


if __name__ == '__main__':
    rospy.init_node('vehicle_controller_bridge')
    try:
        bridge = VehicleBridge()
        rospy.spin()
    except (ValueError, OSError) as exc:
        rospy.logfatal('Cannot start vehicle bridge: %s', exc)
        raise
