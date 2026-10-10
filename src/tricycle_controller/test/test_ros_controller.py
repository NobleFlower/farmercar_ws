#!/usr/bin/env python3
"""Isolated ROS tests of both velocity input formats. No driver or hardware."""
import math
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, TwistStamped
from tricycle_controller.msg import ChassisCommand


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.commands = {}
        self.subs = []
        for kind in ('plain', 'stamped'):
            self.subs.append(rospy.Subscriber('/tricycle_test/'+kind+'_command', ChassisCommand,
                                             lambda msg, key=kind: self.commands.__setitem__(key, (msg, time.monotonic()))))
        self.plain = rospy.Publisher('/tricycle_test/plain_input', Twist, queue_size=1)
        self.stamped = rospy.Publisher('/tricycle_test/stamped_input', TwistStamped, queue_size=1)
        deadline = time.monotonic()+5
        while time.monotonic()<deadline and (self.plain.get_num_connections()==0 or self.stamped.get_num_connections()==0):
            time.sleep(.02)
        self.assertGreater(self.plain.get_num_connections(), 0)
        self.assertGreater(self.stamped.get_num_connections(), 0)

    @staticmethod
    def twist(v=.1, vy=0, w=.05):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = v, vy, w
        return msg

    def send_until(self, kind, make, predicate):
        pub = self.plain if kind == 'plain' else self.stamped
        started = time.monotonic()
        while time.monotonic()-started < 2:
            pub.publish(make())
            time.sleep(.03)
            if kind in self.commands:
                cmd, received = self.commands[kind]
                if received > started and predicate(cmd):
                    return cmd
        self.fail('Expected output missing for '+kind+': '+str(self.commands.get(kind)))

    def assert_zero(self, command):
        self.assertFalse(command.accepted)
        self.assertEqual((command.speed, command.steering_angle, command.yaw_rate), (0, 0, 0))

    def stamped_msg(self, age=0, frame='base_link', zero_stamp=False, v=.1):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time(0) if zero_stamp else rospy.Time.now()-rospy.Duration(age)
        msg.header.frame_id = frame
        msg.twist = self.twist(v=v)
        return msg

    def test_plain_and_stamped_inputs(self):
        cmd = self.send_until('plain', lambda:self.twist(), lambda c:c.accepted and not c.saturated)
        self.assertAlmostEqual(cmd.speed, .1)
        self.assertAlmostEqual(cmd.steering_angle, math.atan(.839*.05/.1))
        self.assertAlmostEqual(cmd.yaw_rate, .05)
        self.assertTrue(cmd.source_stamp.is_zero())
        cmd = self.send_until('plain', lambda:self.twist(v=-.1), lambda c:c.accepted and c.speed<0)
        self.assertLess(cmd.steering_angle, 0)
        self.assertGreater(cmd.yaw_rate, 0)
        cmd = self.send_until('plain', lambda:self.twist(v=1, w=5), lambda c:c.saturated)
        self.assertAlmostEqual(cmd.speed, .15)
        self.assertAlmostEqual(cmd.steering_angle, math.radians(50))
        self.assertAlmostEqual(cmd.yaw_rate, .15*math.tan(math.radians(50))/.839)
        for message, reason in ((self.twist(vy=.1), 'lateral_velocity_not_supported'),
                                (self.twist(v=0), 'in_place_rotation_not_supported'),
                                (self.twist(v=float('nan')), 'nonfinite_velocity')):
            cmd = self.send_until('plain', lambda m=message:m, lambda c:c.reason==reason)
            self.assert_zero(cmd)
        invalid = self.twist()
        invalid.angular.x = .1
        cmd = self.send_until('plain', lambda:invalid, lambda c:c.reason=='nonplanar_velocity_not_supported')
        self.assert_zero(cmd)
        self.send_until('plain', lambda:self.twist(), lambda c:c.accepted)
        time.sleep(.7)
        self.assertEqual(self.commands['plain'][0].reason, 'command_timeout')
        self.assert_zero(self.commands['plain'][0])
        self.send_until('stamped', lambda:self.stamped_msg(), lambda c:c.accepted)
        self.assertFalse(self.commands['stamped'][0].source_stamp.is_zero())
        for make, reason in ((lambda:self.stamped_msg(age=2), 'invalid_or_stale_stamp'),
                              (lambda:self.stamped_msg(age=-2), 'invalid_or_stale_stamp'),
                              (lambda:self.stamped_msg(zero_stamp=True), 'invalid_or_stale_stamp'),
                              (lambda:self.stamped_msg(frame='map'), 'unexpected_command_frame')):
            cmd = self.send_until('stamped', make, lambda c:c.reason==reason)
            self.assert_zero(cmd)
        # Repeating an old stamped message must not bypass the stamp watchdog.
        old = self.stamped_msg()
        self.send_until('stamped', lambda:old, lambda c:c.accepted)
        time.sleep(.6)
        cmd = self.send_until('stamped', lambda:old, lambda c:c.reason=='invalid_or_stale_stamp')
        self.assert_zero(cmd)
        self.send_until('stamped', lambda:self.stamped_msg(), lambda c:c.accepted)


if __name__ == '__main__':
    rospy.init_node('tricycle_controller_test')
    rostest.rosrun('tricycle_controller', 'controller', ControllerTests)
