#!/usr/bin/env python3
"""Velocity bridge + chassis software test. No motor/serial processes."""
import json
import math
import time
import unittest
import rospy
import rostest
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from tricycle_controller.msg import ChassisCommand


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.status = {}
        self.command = {}
        self.mapped = {}
        self.subs = []
        for kind in ('plain', 'stamped'):
            self.subs.append(rospy.Subscriber('/adapter_test/'+kind+'_status', String,
                lambda m, k=kind:self.status.__setitem__(k, (json.loads(m.data), time.monotonic()))))
            self.subs.append(rospy.Subscriber('/adapter_test/'+kind+'_command', ChassisCommand,
                lambda m, k=kind:self.command.__setitem__(k, (m, time.monotonic()))))
            self.subs.append(rospy.Subscriber('/adapter_test/'+kind+'_mapped', TwistStamped,
                lambda m, k=kind:self.mapped.__setitem__(k, (m, time.monotonic()))))
        self.plain = rospy.Publisher('/adapter_test/plain_input', Twist, queue_size=1)
        self.stamped = rospy.Publisher('/adapter_test/stamped_input', TwistStamped, queue_size=1)
        deadline = time.monotonic()+5
        while time.monotonic()<deadline and (self.plain.get_num_connections()==0 or self.stamped.get_num_connections()==0):
            time.sleep(.02)
        self.assertGreater(self.plain.get_num_connections(), 0)
        self.assertGreater(self.stamped.get_num_connections(), 0)

    @staticmethod
    def twist(vx=.1, vy=0, w=.05):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = vx, vy, w
        return m

    def stamped_msg(self, age=0, frame='base_link'):
        m = TwistStamped()
        m.header.stamp = rospy.Time.now()-rospy.Duration(age)
        m.header.frame_id = frame
        m.twist = self.twist(vy=.05)
        return m

    def send_until(self, kind, make, predicate):
        pub = self.plain if kind == 'plain' else self.stamped
        started = time.monotonic()
        while time.monotonic()-started<2:
            pub.publish(make())
            time.sleep(.04)
            if all(kind in x and x[kind][1]>started for x in (self.status,self.command,self.mapped)):
                status, cmd, mapped = self.status[kind][0], self.command[kind][0], self.mapped[kind][0]
                if predicate(status, cmd, mapped):
                    return status, cmd, mapped
        self.fail(str(self.status.get(kind))+' '+str(self.command.get(kind)))

    def test_differential_omni_and_watchdogs(self):
        status, cmd, mapped = self.send_until('plain', lambda:self.twist(),
            lambda s,c,m:s['reason']=='differential_passthrough' and abs(c.speed-.1)<1e-9)
        self.assertFalse(status['approximate'])
        self.assertAlmostEqual(cmd.steering_angle, math.atan(.839*.05/.1))
        self.assertAlmostEqual(cmd.yaw_rate, .05)
        for vx, vy, expected_sign in ((.1,.1,1),(0,.1,1),(0,-.1,-1),(-.1,.1,-1)):
            status, cmd, mapped = self.send_until('plain', lambda x=vx,y=vy:self.twist(x,y,0),
                lambda s,c,m:s['approximate'] and c.accepted and
                math.copysign(1,c.yaw_rate)==expected_sign and
                abs(m.twist.linear.x-math.copysign(math.hypot(vx,vy),-1 if vx<0 else 1))<1e-9)
            self.assertEqual(mapped.twist.linear.y, 0)
            self.assertLessEqual(abs(cmd.steering_angle), math.radians(50))
            self.assertLessEqual(abs(cmd.speed), .15)
            self.assertAlmostEqual(cmd.yaw_rate, cmd.speed*math.tan(cmd.steering_angle)/.839)
        for make, reason in ((lambda:self.twist(0,0,.2),'pure_rotation_unreachable'),
                              (lambda:self.twist(float('nan'),0,0),'nonfinite_velocity')):
            status, cmd, mapped = self.send_until('plain', make,
                lambda s,c,m:s['reason']==reason and c.speed==0 and c.yaw_rate==0)
            self.assertFalse(status['accepted'])
            self.assertEqual(mapped.twist.linear.x, 0)
        self.send_until('plain', lambda:self.twist(vy=.1), lambda s,c,m:s['accepted'] and c.speed>0)
        time.sleep(.7)
        self.assertEqual(self.status['plain'][0]['reason'], 'command_timeout')
        self.assertEqual(self.command['plain'][0].speed, 0)
        self.send_until('stamped', lambda:self.stamped_msg(), lambda s,c,m:s['accepted'] and c.speed>0)
        for make, reason in ((lambda:self.stamped_msg(age=2),'invalid_or_stale_stamp'),
                              (lambda:self.stamped_msg(age=-2),'invalid_or_stale_stamp'),
                              (lambda:self.stamped_msg(frame='map'),'unexpected_command_frame')):
            self.send_until('stamped', make, lambda s,c,m:s['reason']==reason and c.speed==0)
        old = self.stamped_msg()
        self.send_until('stamped',lambda:old,lambda s,c,m:s['accepted'] and c.speed>0)
        time.sleep(.6)
        self.send_until('stamped',lambda:old,lambda s,c,m:s['reason']=='invalid_or_stale_stamp' and c.speed==0)
        self.send_until('stamped',lambda:self.stamped_msg(),lambda s,c,m:s['accepted'] and c.speed>0)


if __name__ == '__main__':
    rospy.init_node('velocity_adapter_test')
    rostest.rosrun('velocity_adapter', 'adapter', AdapterTests)
