#!/usr/bin/env python3
"""Publish preview commands only; never connects to a vehicle-control interface."""
import copy
import json
import math
import threading
import time

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from std_msgs.msg import Bool, Float64, String

from apriltag_servo.control import Parameters, calculate


class FollowPreview:
    def __init__(self):
        self.params = Parameters(
            wheelbase=float(rospy.get_param('~wheelbase', 0.839)),
            max_steering=math.radians(float(rospy.get_param('~max_steering_deg', 50.0))),
            max_speed=float(rospy.get_param('~max_speed', 0.15)),
            desired_distance=float(rospy.get_param('~desired_distance', 1.0)),
            distance_deadband=float(rospy.get_param('~distance_deadband', 0.05)),
            distance_gain=float(rospy.get_param('~distance_gain', 0.3)),
            acceleration=float(rospy.get_param('~max_acceleration', 0.15)),
            minimum_forward_target=float(rospy.get_param('~minimum_forward_target', 0.05)))
        self.params.validate()
        self.tag_id = int(rospy.get_param('~target_tag_id', 0))
        self.camera_frame = str(rospy.get_param('~camera_frame', 'camera_optical_frame')).lstrip('/')
        self.servo_frame = str(rospy.get_param('~servo_frame', 'camera_link')).lstrip('/')
        self.tag_frame = str(rospy.get_param('~tag_frame', 'apriltag_link')).lstrip('/')
        self.publish_tag_tf = bool(rospy.get_param('~publish_tag_tf', True))
        self.timeout = float(rospy.get_param('~target_timeout', 0.5))
        self.future_tolerance = float(rospy.get_param('~future_tolerance', 0.05))
        self.rate = float(rospy.get_param('~control_rate', 20.0))
        if (not self.camera_frame or not self.servo_frame or not self.tag_frame or self.servo_frame == self.tag_frame
                or not all(math.isfinite(v) for v in (self.timeout, self.future_tolerance,
                                                       self.rate))
                or self.timeout <= 0 or self.rate <= 0 or self.future_tolerance < 0):
            raise ValueError('invalid frame or timing configuration')
        self.lock = threading.RLock()
        self.enabled = bool(rospy.get_param('~start_enabled', False))
        self.target = None
        self.reason = 'waiting_for_target'
        self.previous_speed = 0.0
        self.last_tick = time.monotonic()
        self.cmd_pub = rospy.Publisher('preview/cmd_vel', TwistStamped, queue_size=1)
        self.steer_pub = rospy.Publisher('preview/steering_angle', Float64, queue_size=1)
        self.status_pub = rospy.Publisher('preview/status', String, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher('preview/target_pose', PoseStamped, queue_size=1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber('detections', AprilTagDetectionArray,
                                    self.on_detections, queue_size=1)
        self.enable_sub = rospy.Subscriber('/apriltag_servo/enable', Bool, self.on_enable, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('AprilTag servo PREVIEW ONLY: direct optical z -> forward x, no mounting or steering compensation.')

    def on_enable(self, message):
        with self.lock:
            self.enabled = message.data
            self.previous_speed = 0.0
            self.last_tick = time.monotonic()
            if not self.enabled:
                self.publish_stop('follow_disabled')

    def reject(self, reason):
        with self.lock:
            self.target = None
            self.previous_speed = 0.0
            self.reason = reason
            self.publish_stop(reason)

    @staticmethod
    def valid_pose(pose):
        p, q = pose.position, pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return all(math.isfinite(v) for v in values) and sum(v*v for v in values[3:]) > 1e-12

    def on_detections(self, msg):
        # Single-tag detections only: a bundle origin may differ from the tag origin.
        selected = next((d for d in msg.detections if list(d.id) == [self.tag_id]), None)
        if selected is None:
            self.reject('target_not_detected')
            return
        source = PoseStamped()
        source.header = copy.deepcopy(selected.pose.header)
        if not source.header.frame_id:
            source.header.frame_id = msg.header.frame_id
        if source.header.stamp == rospy.Time(0):
            source.header.stamp = msg.header.stamp
        source.pose = copy.deepcopy(selected.pose.pose.pose)
        now = rospy.Time.now()
        age = (now - source.header.stamp).to_sec()
        if (source.header.stamp == rospy.Time(0) or not source.header.frame_id
                or age > self.timeout or age < -self.future_tolerance):
            self.reject('invalid_or_stale_detection_stamp')
            return
        if not self.valid_pose(source.pose):
            self.reject('invalid_detection_pose')
            return
        q = source.pose.orientation
        qnorm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        q.x, q.y, q.z, q.w = q.x/qnorm, q.y/qnorm, q.z/qnorm, q.w/qnorm
        p = source.pose.position
        if source.header.frame_id.lstrip('/') != self.camera_frame:
            self.reject('unexpected_camera_frame')
            return
        # Direct observation axes only: forward=z, left=-x, up=-y.
        # This rotation changes axis convention, not the measured distance.
        rotation = TransformStamped()
        rotation.header.frame_id = self.servo_frame
        rotation.header.stamp = source.header.stamp
        rotation.transform.rotation.x = -0.5
        rotation.transform.rotation.y = 0.5
        rotation.transform.rotation.z = -0.5
        rotation.transform.rotation.w = 0.5
        target = do_transform_pose(source, rotation)
        target.pose.position.x = p.z
        target.pose.position.y = -p.x
        target.pose.position.z = -p.y
        forward_distance = p.z
        with self.lock:
            self.target = (target, forward_distance, time.monotonic())
            self.reason = 'target_available'
            self.pose_pub.publish(target)
            if self.publish_tag_tf:
                tf = TransformStamped()
                tf.header = copy.deepcopy(source.header)
                tf.child_frame_id = self.tag_frame
                tf.transform.translation.x = source.pose.position.x
                tf.transform.translation.y = source.pose.position.y
                tf.transform.translation.z = source.pose.position.z
                tf.transform.rotation = source.pose.orientation
                self.tf_broadcaster.sendTransform(tf)

    def publish(self, speed, steering, yaw_rate, reason, target=None, forward_distance=None):
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = speed
        cmd.twist.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)
        self.steer_pub.publish(Float64(data=steering))
        status = dict(mode='SOFTWARE_COMMAND', actuator_allowed=False, enabled=self.enabled, reason=reason,
                      target_tag_id=self.tag_id, coordinate_mode='optical_axes_only',
                      mounting_compensation=False, steering_compensation=False,
                      speed_mps=speed, steering_rad=steering, yaw_rate_radps=yaw_rate,
                      tag_forward_distance_m=forward_distance, desired_distance_m=self.params.desired_distance,
                      target_age_s=None if target is None else
                      (cmd.header.stamp-target.header.stamp).to_sec())
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True, allow_nan=False)))

    def publish_stop(self, reason):
        self.publish(0.0, 0.0, 0.0, reason)

    def tick(self, _event):
        wall_now = time.monotonic()
        with self.lock:
            dt = wall_now - self.last_tick
            self.last_tick = wall_now
            if not self.enabled:
                self.previous_speed = 0.0
                self.publish_stop('follow_disabled')
                return
            if self.target is None:
                self.previous_speed = 0.0
                self.publish_stop(self.reason)
                return
            target, forward_distance, received_at = self.target
            age = (rospy.Time.now() - target.header.stamp).to_sec()
            if (age > self.timeout or age < -self.future_tolerance
                    or wall_now - received_at > self.timeout):
                self.target = None
                self.previous_speed = 0.0
                self.reason = 'target_timeout_or_clock_jump'
                self.publish_stop(self.reason)
                return
            p = target.pose.position
            command = calculate(p.x, p.y, forward_distance, self.previous_speed, dt, self.params)
            self.previous_speed = command.speed
            self.publish(command.speed, command.steering, command.yaw_rate,
                         command.reason, target, forward_distance)

    def shutdown(self):
        with self.lock:
            self.target = None
            self.previous_speed = 0.0
            self.publish_stop('shutdown')


if __name__ == '__main__':
    rospy.init_node('apriltag_follow_preview')
    try:
        node = FollowPreview()
        rospy.spin()
    except (ValueError, TypeError) as exc:
        rospy.logfatal('AprilTag servo configuration error: %s', exc)
        raise
