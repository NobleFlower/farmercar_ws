#!/usr/bin/env python3
"""Run with rostest on an isolated ROS master. Publishes synthetic detections only."""
import json
import time
import unittest

import rospy
import rostest
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String


class FaultTests(unittest.TestCase):
    def setUp(self):
        self.cmd = None
        self.status = None
        self.cmd_time = 0
        self.status_time = 0
        self.sub_cmd = rospy.Subscriber('/servo_test_preview/cmd_vel', TwistStamped, self.command)
        self.sub_status = rospy.Subscriber('/servo_test_preview/status', String, self.state)
        self.pub = rospy.Publisher('/servo_test_detections', AprilTagDetectionArray, queue_size=1)
        self.pose = None
        self.sub_pose = rospy.Subscriber('/servo_test_preview/target_pose', PoseStamped,
                                         lambda msg: setattr(self, 'pose', msg))
        deadline = time.monotonic() + 5
        while self.pub.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertGreater(self.pub.get_num_connections(), 0)
        time.sleep(.2)

    def command(self, message):
        self.cmd = message
        self.cmd_time = time.monotonic()

    def state(self, message):
        self.status = json.loads(message.data)
        self.status_time = time.monotonic()

    def detection(self, x=3.0, y=0.0, up=0.0, frame='servo_test_camera', age=0, empty=False):
        msg = AprilTagDetectionArray()
        msg.header.stamp = rospy.Time.now() - rospy.Duration(age)
        msg.header.frame_id = frame
        if not empty:
            d = AprilTagDetection()
            d.id = [0]
            d.size = [0.1]
            d.pose.header = msg.header
            d.pose.pose.pose.position.x = -y
            d.pose.pose.pose.position.y = -up
            d.pose.pose.pose.position.z = x
            d.pose.pose.pose.orientation.w = 1.0
            msg.detections = [d]
        return msg

    def send_until(self, make_message, predicate, timeout=2):
        started = time.monotonic()
        deadline = started + timeout
        while time.monotonic() < deadline:
            self.pub.publish(make_message())
            time.sleep(.03)
            if (self.status_time > started and self.cmd_time > started
                    and self.status is not None and self.cmd is not None and predicate()):
                return
        self.fail('Expected state missing: %s' % self.status)

    def assert_stopped(self):
        self.assertEqual(self.cmd.twist.linear.x, 0.0)
        self.assertEqual(self.cmd.twist.angular.z, 0.0)
        self.assertFalse(self.status['actuator_allowed'])

    def test_faults_and_recovery(self):
        self.send_until(lambda: self.detection(y=.5), lambda: self.cmd.twist.linear.x > 0)
        self.assertGreater(self.cmd.twist.angular.z, 0)
        self.assertEqual(self.status['mode'], 'SOFTWARE_COMMAND')
        self.assertFalse(self.status['mounting_compensation'])
        self.assertFalse(self.status['steering_compensation'])
        self.assertEqual(self.pose.header.frame_id, 'servo_test_axes')
        self.assertEqual(self.pose.pose.position.x, 3.0)
        self.assertEqual(self.pose.pose.position.y, .5)
        self.assertEqual(self.cmd.header.frame_id, 'base_link')
        scenarios = [
            (lambda: self.detection(empty=True), 'target_not_detected'),
            (lambda: self.detection(age=2), 'invalid_or_stale_detection_stamp'),
            (lambda: self.detection(age=-2), 'invalid_or_stale_detection_stamp'),
            (lambda: self.detection(frame='servo_test_missing'), 'unexpected_camera_frame'),
            (lambda: self.detection(x=float('nan')), 'invalid_detection_pose'),
            (lambda: self.detection(x=-3), 'invalid_geometry'),
            (lambda: self.detection(x=.9, y=2, up=4), 'desired_distance_reached'),
        ]
        for make_message, reason in scenarios:
            self.send_until(make_message, lambda: self.status['reason'] == reason
                            and self.cmd.twist.linear.x == 0.0
                            and self.cmd.twist.angular.z == 0.0)
            self.assert_stopped()
        self.send_until(lambda: self.detection(), lambda: self.cmd.twist.linear.x > 0)
        time.sleep(.7)
        self.assertEqual(self.status['reason'], 'target_timeout_or_clock_jump')
        self.assert_stopped()
        self.send_until(lambda: self.detection(), lambda: self.cmd.twist.linear.x > 0)
        self.assertLessEqual(self.cmd.twist.linear.x, .15)


if __name__ == '__main__':
    rospy.init_node('servo_fault_test')
    rostest.rosrun('apriltag_servo', 'servo_faults', FaultTests)
