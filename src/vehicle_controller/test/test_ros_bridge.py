#!/usr/bin/env python3
"""ROS gating/feedback tests with serial disabled; fake frames only."""
import json
import math
import struct
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Bool, String, UInt8MultiArray
from tricycle_controller.msg import ChassisCommand
from vehicle_controller.msg import VehicleFeedback
from vehicle_controller.wire import packet


class BridgeTests(unittest.TestCase):
    def test_bridge_fields_and_gates(self):
        self.status = None
        self.feedback = None
        self.tx = None
        self.joints = []
        from sensor_msgs.msg import JointState
        subs = [rospy.Subscriber('/vehicle_controller/status', String, lambda m:setattr(self,'status',json.loads(m.data))),
                rospy.Subscriber('/vehicle_controller/feedback', VehicleFeedback, lambda m:setattr(self,'feedback',m)),
                rospy.Subscriber('/vehicle_controller/tx_packet', UInt8MultiArray, lambda m:setattr(self,'tx',list(m.data))),
                rospy.Subscriber('/vehicle/steering_joint_states',JointState,lambda m:self.joints.append(m))]
        cmd_pub = rospy.Publisher('/bridge_test/command', ChassisCommand, queue_size=1)
        rx_pub = rospy.Publisher('/vehicle_controller/rx_inject', UInt8MultiArray, queue_size=1)
        arm_pub = rospy.Publisher('/vehicle_controller/arm', Bool, queue_size=1)
        enable_pub = rospy.Publisher('/apriltag_servo/enable', Bool, queue_size=1)
        estop_pub = rospy.Publisher('/vehicle_controller/emergency_stop', Bool, queue_size=1)
        deadline = time.monotonic()+5
        while time.monotonic()<deadline and any(p.get_num_connections()==0 for p in (cmd_pub,rx_pub,arm_pub,enable_pub,estop_pub)):
            time.sleep(.02)

        def command():
            m = ChassisCommand()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = 'base_link'
            m.source_stamp = m.header.stamp
            m.accepted = True
            m.speed = .1
            m.steering_angle = math.radians(10)
            m.yaw_rate = .1*math.tan(m.steering_angle)/.839
            return m

        def feedback(kind=2, mode=2, flags=13, faults=0):
            payload = struct.pack('<hH', 8, 1000)
            if kind == 2:
                payload += struct.pack('<BBHH', mode, flags, faults, 0)
            return UInt8MultiArray(data=list(packet(0xEF, kind, payload)))

        def wait(predicate, rx=True, cmd=True, timeout=2):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                if cmd:
                    cmd_pub.publish(command())
                if rx:
                    rx_pub.publish(feedback())
                time.sleep(.03)
                if self.status and predicate():
                    return
            self.fail(str(self.status))

        wait(lambda:self.status['reason']=='follow_disabled')
        self.assertEqual(self.status['output_speed_mps'], 0)
        enable_pub.publish(Bool(data=True))
        wait(lambda:self.status['reason']=='driver_disarmed')
        arm_pub.publish(Bool(data=True))
        wait(lambda:self.status['reason']=='ok' and self.feedback is not None and self.feedback.fresh and self.tx is not None and self.tx[3]==10)
        self.assertFalse(self.status['serial_enabled'])
        self.assertFalse(self.status['packet_written'])
        self.assertFalse(self.status['actuation_permitted'])
        self.assertAlmostEqual(self.feedback.speed, .08)
        self.assertAlmostEqual(self.feedback.steering_angle, math.radians(10))
        self.assertTrue(self.feedback.speed_is_measured)
        self.assertTrue(self.feedback.simulated)
        self.assertEqual(self.joints, [])
        self.assertEqual(struct.unpack('<hHBH', bytes(self.tx[3:-1]))[:3], (10, 1000, 5))
        self.assertEqual(self.tx[-1], sum(self.tx[:-1]) & 255)
        # Retain fresh commands but stop feedback: receiving an old feedback is not refreshed.
        time.sleep(.1)
        stamp = self.feedback.header.stamp
        wait(lambda:self.status['reason']=='feedback_timeout',rx=False,timeout=2)
        self.assertFalse(self.feedback.fresh)
        self.assertEqual(self.feedback.header.stamp,stamp)
        self.assertEqual(self.status['output_speed_mps'],0)
        wait(lambda:self.status['reason']=='ok')
        wait(lambda:self.status['command_reason']=='command_timeout',cmd=False,timeout=2)
        self.assertEqual(self.status['output_speed_mps'],0)
        wait(lambda:self.status['reason']=='ok')
        estop_pub.publish(Bool(data=True))
        wait(lambda:self.status['reason']=='host_emergency_stop')
        self.assertFalse(self.status['armed'])
        estop_pub.publish(Bool(data=False))
        wait(lambda:self.status['reason']=='driver_disarmed')
        arm_pub.publish(Bool(data=True))
        wait(lambda:self.status['reason']=='ok')
        # Firmware fault disarms; healthy feedback alone cannot resume.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            rx_pub.publish(feedback(faults=1))
            cmd_pub.publish(command())
            time.sleep(.03)
            if not self.status['armed']:
                break
        self.assertFalse(self.status['armed'])
        wait(lambda:not self.status['armed'])
        self.assertEqual(self.status['output_speed_mps'],0)
        arm_pub.publish(Bool(data=True))
        wait(lambda:self.status['reason']=='ok')
        # Manual mode takes priority and latches the driver disarmed.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            rx_pub.publish(feedback(mode=1))
            cmd_pub.publish(command())
            time.sleep(.03)
            if not self.status['armed']:
                break
        self.assertFalse(self.status['armed'])
        wait(lambda:self.status['reason']=='driver_disarmed')
        arm_pub.publish(Bool(data=True))
        wait(lambda:self.status['reason']=='ok')
        # Legacy setting frames cannot satisfy the default ready/auto handshake.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            rx_pub.publish(feedback(kind=1))
            cmd_pub.publish(command())
            time.sleep(.03)
            if (self.status['reason']=='legacy_feedback_requires_explicit_opt_in' and
                    self.feedback is not None and self.feedback.protocol_version==1 and
                    not self.feedback.status_authoritative):
                break
        self.assertEqual(self.status['reason'],'legacy_feedback_requires_explicit_opt_in')
        self.assertFalse(self.feedback.status_authoritative)
        self.assertFalse(self.feedback.speed_is_measured)
        self.assertEqual(self.status['output_speed_mps'],0)
        wait(lambda:self.status['reason']=='ok')
        enable_pub.publish(Bool(data=False))
        wait(lambda:self.status['reason']=='follow_disabled')
        self.assertEqual(self.status['output_speed_mps'],0)


if __name__ == '__main__':
    rospy.init_node('vehicle_bridge_test')
    rostest.rosrun('vehicle_controller','bridge',BridgeTests)
