#!/usr/bin/env python3
"""ROS integration: voice bytes -> AprilTag state -> command -> gated TX."""
import math
import struct
import time
import unittest

import rospy
import rostest
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from std_msgs.msg import Bool, UInt8MultiArray
from tricycle_controller.msg import ChassisCommand
from vehicle_controller.wire import packet


class VoiceNavigationIntegrationTests(unittest.TestCase):
    def test_voice_crossing_reaches_gated_packet(self):
        self.command = None
        self.tx = None
        rospy.Subscriber("/voice_test/command", ChassisCommand,
                         lambda msg: setattr(self, "command", msg), queue_size=1)
        rospy.Subscriber("/vehicle_controller/tx_packet", UInt8MultiArray,
                         lambda msg: setattr(self, "tx", list(msg.data)), queue_size=1)
        detections_pub = rospy.Publisher(
            "/tag_detections", AprilTagDetectionArray, queue_size=1)
        voice_pub = rospy.Publisher(
            "/voice_apriltag_navigation/rx_inject",
            UInt8MultiArray, queue_size=1)
        feedback_pub = rospy.Publisher(
            "/vehicle_controller/rx_inject", UInt8MultiArray, queue_size=1)
        arm_pub = rospy.Publisher(
            "/vehicle_controller/arm", Bool, queue_size=1)

        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and any(
                pub.get_num_connections() == 0 for pub in
                (detections_pub, voice_pub, feedback_pub, arm_pub)):
            time.sleep(0.03)
        self.assertTrue(all(pub.get_num_connections() for pub in
                            (detections_pub, voice_pub, feedback_pub, arm_pub)))

        feedback = UInt8MultiArray(data=list(packet(
            0xEF, 2, struct.pack("<hHBBHH", 0, 900, 2, 13, 0, 0))))

        def detection(tag_id, x=0.0, z=2.0):
            msg = AprilTagDetectionArray()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera_optical_frame"
            item = AprilTagDetection()
            item.id = [tag_id]
            item.size = [0.154]
            item.pose.header = msg.header
            item.pose.pose.pose.position.x = x
            item.pose.pose.pose.position.z = z
            item.pose.pose.pose.orientation.w = 1.0
            msg.detections = [item]
            return msg

        def wait_until(make_detection, predicate, timeout=4.0):
            started = time.monotonic()
            while time.monotonic() - started < timeout:
                detections_pub.publish(make_detection())
                feedback_pub.publish(feedback)
                time.sleep(0.03)
                if self.command is not None and self.tx is not None and predicate():
                    return
            self.fail("command=%r tx=%r" % (self.command, self.tx))

        arm_pub.publish(Bool(data=True))
        voice_pub.publish(UInt8MultiArray(data=[0xAB, 0xAB, 0xAB, 0x01]))
        wait_until(lambda: detection(11, x=-0.2, z=2.0),
                   lambda: abs(self.command.speed - 0.3) < 1e-6 and
                   self.command.steering_angle > 0 and self.tx[3] == 30)

        voice_pub.publish(UInt8MultiArray(data=[0xAB, 0xAB, 0xAB, 0x00]))
        wait_until(lambda: detection(11, x=0.0, z=2.0),
                   lambda: self.command.speed == 0 and
                   self.command.steering_angle == 0 and self.tx[3:7] == [0, 0, 0x84, 3])

        voice_pub.publish(UInt8MultiArray(data=[0xAD, 0xAD, 0xAD, 0x02]))
        wait_until(lambda: detection(13, z=1.0),
                   lambda: abs(self.command.speed - 1.0) < 1e-6 and
                   abs(math.degrees(self.command.steering_angle) - 45.0) < 1e-6 and
                   self.tx[3:5] == [100, 0] and self.tx[5:7] == [70, 5])

        wait_until(lambda: detection(22, z=0.5),
                   lambda: self.command.speed == 0 and self.command.steering_angle == 0)


if __name__ == "__main__":
    rospy.init_node("voice_navigation_integration_test")
    rostest.rosrun("apriltag_navigation", "voice_navigation",
                   VoiceNavigationIntegrationTests)
