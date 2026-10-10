#!/usr/bin/env python3
"""Differential/omni velocity bridge, with no device access or actuator output."""
import copy
import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from velocity_adapter.mapping import Parameters, Adapted, adapt


class VelocityAdapter:
    def __init__(self):
        self.params = Parameters(float(rospy.get_param('~heading_gain', .5)),
                                 bool(rospy.get_param('~allow_reverse', True)),
                                 float(rospy.get_param('~zero_tolerance', 1e-6)))
        self.params.validate()
        self.stamped = bool(rospy.get_param('~input_stamped', False))
        self.frame = str(rospy.get_param('~base_frame', 'base_link'))
        self.timeout = float(rospy.get_param('~command_timeout', .5))
        self.future_tolerance = float(rospy.get_param('~future_tolerance', .05))
        rate = float(rospy.get_param('~output_rate', 20))
        if (not self.frame or not all(math.isfinite(v) for v in (self.timeout, self.future_tolerance, rate))
                or self.timeout <= 0 or self.future_tolerance < 0 or rate <= 0):
            raise ValueError('invalid frame/timing parameters')
        self.lock = threading.RLock()
        self.latest = None
        self.pub = rospy.Publisher('adapted_cmd_vel', TwistStamped, queue_size=1)
        self.status_pub = rospy.Publisher('status', String, queue_size=1)
        self.sub = rospy.Subscriber('cmd_vel', TwistStamped if self.stamped else Twist,
                                    self.receive, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1/rate), self.tick)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('Velocity bridge: differential passthrough / holonomic bearing approximation. Software only.')

    def receive(self, message):
        with self.lock:
            self.latest = copy.deepcopy(message), time.monotonic()
            self.emit_latest()

    def emit_latest(self):
        now = rospy.Time.now()
        requested = None
        source_age = None
        if self.latest is None:
            result = Adapted()
        else:
            message, received = self.latest
            twist = message.twist if self.stamped else message
            values = (twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z)
            if all(math.isfinite(v) for v in values):
                requested = dict(vx=values[0], vy=values[1], yaw_rate=values[5])
            if self.stamped:
                source_age = (now-message.header.stamp).to_sec()
            if time.monotonic()-received > self.timeout:
                result = Adapted(reason='command_timeout')
            elif self.stamped and (message.header.stamp == rospy.Time(0) or
                    source_age > self.timeout or source_age < -self.future_tolerance):
                result = Adapted(reason='invalid_or_stale_stamp')
            elif self.stamped and message.header.frame_id != self.frame:
                result = Adapted(reason='unexpected_command_frame')
            elif not all(math.isfinite(v) for v in values):
                result = Adapted(reason='nonfinite_velocity')
            elif any(abs(v) > self.params.zero_tolerance for v in (values[2], values[3], values[4])):
                result = Adapted(reason='nonplanar_velocity_not_supported')
            else:
                result = adapt(values[0], values[1], values[5], self.params)
        self.publish(result, now, requested, source_age)

    def publish(self, result, now, requested, source_age):
        message = TwistStamped()
        message.header.stamp = now
        message.header.frame_id = self.frame
        message.twist.linear.x = result.vx
        message.twist.angular.z = result.yaw_rate
        self.pub.publish(message)
        status = dict(software_only=True, actuator_allowed=False, accepted=result.accepted,
                      approximate=result.approximate, reason=result.reason, requested=requested,
                      mapped_vx=result.vx, mapped_vy=0.0, mapped_yaw_rate=result.yaw_rate,
                      heading_error_rad=result.heading_error, source_age_s=source_age,
                      heading_gain=self.params.heading_gain,
                      note='steering/speed feasibility is enforced by tricycle_controller')
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def tick(self, event):
        with self.lock:
            self.emit_latest()

    def shutdown(self):
        with self.lock:
            self.publish(Adapted(reason='shutdown'), rospy.Time.now(), None, None)


if __name__ == '__main__':
    rospy.init_node('velocity_adapter')
    try:
        adapter = VelocityAdapter()
        rospy.spin()
    except (ValueError, TypeError) as exc:
        rospy.logfatal('Cannot initialize velocity adapter: %s', exc)
        raise
