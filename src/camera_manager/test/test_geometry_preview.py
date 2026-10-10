#!/usr/bin/env python3
"""Isolated rostest: simulated steering only; no hardware process is launched."""
import json
import math
import time
import unittest
import rospy
import rostest
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


class GeometryTests(unittest.TestCase):
    def test_geometry_and_preview_watchdog(self):
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        pub = rospy.Publisher('/vehicle/steering_joint_states', JointState, queue_size=1)
        cmd = rospy.Publisher('/apriltag_servo/preview/cmd_vel', TwistStamped, queue_size=1)
        self.status = None
        sub = rospy.Subscriber('/vehicle_controller/preview/status', String, lambda msg: setattr(self, 'status', json.loads(msg.data)))
        deadline = time.monotonic()+5
        while (pub.get_num_connections() == 0 or cmd.get_num_connections() == 0) and time.monotonic()<deadline:
            time.sleep(.03)
        self.assertGreater(pub.get_num_connections(), 0)
        for angle in (0.0, math.radians(20), math.radians(-20)):
            for _ in range(8):
                joint = JointState()
                joint.header.stamp = rospy.Time.now()
                joint.name = ['steering_joint']
                joint.position = [angle]
                pub.publish(joint)
                time.sleep(.04)
            tf = buffer.lookup_transform('base_link', 'camera_optical_frame', rospy.Time(0), rospy.Duration(2))
            t, q = tf.transform.translation, tf.transform.rotation
            self.assertAlmostEqual(t.x, 0, places=5)
            self.assertAlmostEqual(t.y, 0, places=5)
            self.assertAlmostEqual(t.z, 0, places=5)
            # Optical z axis is the third rotation-matrix column.
            forward = (2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y))
            self.assertAlmostEqual(forward[0], math.cos(angle), places=5)
            self.assertAlmostEqual(forward[1], math.sin(angle), places=5)
            self.assertAlmostEqual(forward[2], 0, places=5)
        last_stamp = tf.header.stamp
        time.sleep(.7)
        after = buffer.lookup_transform('base_link', 'camera_optical_frame', rospy.Time(0), rospy.Duration(1))
        self.assertEqual(after.header.stamp, last_stamp)
        for _ in range(8):
            message = TwistStamped()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'base_link'
            message.twist.linear.x = .1
            cmd.publish(message)
            time.sleep(.03)
        self.assertEqual(self.status['reason'], 'ok')
        self.assertAlmostEqual(self.status['speed_mps'], .1)
        self.assertTrue(self.status['preview_only'])
        time.sleep(.7)
        self.assertEqual(self.status['reason'], 'command_timeout')
        self.assertEqual(self.status['speed_mps'], 0)
        for _ in range(5):
            message.header.stamp = rospy.Time.now()
            message.twist.linear.x = 0
            message.twist.angular.z = .1
            cmd.publish(message)
            time.sleep(.05)
        self.assertIn('cannot rotate in place', self.status['reason'])
        self.assertEqual(self.status['speed_mps'], 0)


if __name__ == '__main__':
    rospy.init_node('geometry_preview_test')
    rostest.rosrun('camera_manager', 'geometry_preview', GeometryTests)
