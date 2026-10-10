#!/usr/bin/env python3
"""Minimal direction-only base, steering and camera frame tree."""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler


def make_transform(parent, child, xyz, rpy, stamp):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = xyz[0]
    transform.transform.translation.y = xyz[1]
    transform.transform.translation.z = xyz[2]
    quaternion = quaternion_from_euler(*rpy)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    return transform


class VehicleCameraTF:
    def __init__(self):
        self.lock = threading.Lock()
        self.steering_angle = None
        self.steering_stamp = None
        self.last_published_stamp = None
        self.received_feedback = False
        self.joint = rospy.get_param("~steering_joint", "steering_joint")
        self.assume_zero = rospy.get_param("~assume_zero_steering", True)
        self.timeout = self.number("steering_timeout", 0.5)
        self.max_angle = self.number("max_steering_angle", math.radians(50.0))
        rate = self.number("publish_rate", 30.0)
        if self.timeout <= 0.0 or self.max_angle <= 0.0 or rate <= 0.0:
            raise ValueError("timeout, max_steering_angle and publish_rate must be positive")
        if self.assume_zero:
            rospy.logwarn("assume_zero_steering=true: before steering feedback arrives, "
                          "publish a DEBUG zero steering angle; do not move the vehicle.")
        self.dynamic = tf2_ros.TransformBroadcaster()
        self.static = tf2_ros.StaticTransformBroadcaster()
        self.publish_fixed_frames()
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~steering_topic", "/vehicle/steering_joint_states"),
            JointState, self.receive_steering, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.publish_steering)

    @staticmethod
    def number(name, default):
        value = float(rospy.get_param("~" + name, default))
        if not math.isfinite(value):
            raise ValueError("Nonfinite geometry parameter: " + name)
        return value

    def publish_fixed_frames(self):
        # Co-located origins: only axis conventions and steering are represented.
        # No wheel, axle, mounting offset or height compensation is modeled.
        stamp = rospy.Time.now()
        self.static.sendTransform([
            make_transform("steering_link", "camera_link", (0, 0, 0), (0, 0, 0), stamp),
            make_transform("camera_link", "camera_optical_frame", (0, 0, 0),
                           (-math.pi / 2, 0, -math.pi / 2), stamp),
        ])

    def receive_steering(self, message):
        with self.lock:
            # Once feedback is seen, loss or malformed feedback must not fall back to debug zero.
            self.received_feedback = True
            self.steering_angle = None
            self.steering_stamp = None
            try:
                index = message.name.index(self.joint)
                angle = message.position[index]
            except (ValueError, IndexError):
                rospy.logwarn_throttle(5.0, "Steering feedback lacks the required joint/position.")
                return
            if not math.isfinite(angle) or abs(angle) > self.max_angle + 1e-6:
                rospy.logwarn_throttle(5.0, "Steering feedback is nonfinite or beyond the configured limit.")
                return
            age = (rospy.Time.now() - message.header.stamp).to_sec()
            if message.header.stamp == rospy.Time(0) or age < -0.05 or age > self.timeout:
                rospy.logwarn_throttle(5.0, "Steering feedback has a zero, future or stale timestamp.")
                return
            self.steering_angle = angle
            self.steering_stamp = message.header.stamp

    def publish_steering(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if self.steering_stamp is not None:
                age = (now - self.steering_stamp).to_sec()
                if age < -0.05 or age > self.timeout:
                    rospy.logwarn_throttle(5.0, "Steering feedback stale: stopped publishing the moving TF edge.")
                    return
                angle = self.steering_angle
                # Preserve measurement time; never refresh stale steering as though newly measured.
                stamp = self.steering_stamp
                if stamp == self.last_published_stamp:
                    return
            elif self.assume_zero and not self.received_feedback:
                rospy.logwarn_throttle(15.0, "DEBUG TF assumes zero steering; actual steering remains unknown.")
                angle, stamp = 0.0, now
            else:
                rospy.logwarn_throttle(5.0, "No valid measured steering: moving TF edge is unavailable.")
                return
            self.last_published_stamp = stamp
        self.dynamic.sendTransform(make_transform(
            "base_link", "steering_link", (0, 0, 0), (0, 0, angle), stamp))


if __name__ == "__main__":
    rospy.init_node("vehicle_camera_tf")
    try:
        node = VehicleCameraTF()
        rospy.spin()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("Cannot initialize vehicle/camera TF: %s", error)
        raise
