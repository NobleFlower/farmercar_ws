#!/usr/bin/env python3
"""Software tricycle controller: no serial/device/vehicle-driver dependency."""
import copy
import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from tricycle_controller.msg import ChassisCommand
from tricycle_controller.kinematics import Command, Limits, convert


class TricycleController:
    def __init__(self):
        self.limits = Limits(
            wheelbase=float(rospy.get_param('~wheelbase', .839)),
            max_steering=math.radians(float(rospy.get_param('~max_steering_deg', 50))),
            max_forward_speed=float(rospy.get_param('~max_forward_speed', .15)),
            max_reverse_speed=float(rospy.get_param('~max_reverse_speed', .15)),
            zero_speed_tolerance=float(rospy.get_param('~zero_speed_tolerance', 1e-6)),
            lateral_tolerance=float(rospy.get_param('~lateral_tolerance', 1e-6)),
            yaw_tolerance=float(rospy.get_param('~yaw_tolerance', 1e-6)))
        self.limits.validate()
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
        self.pub = rospy.Publisher('command', ChassisCommand, queue_size=1)
        self.achievable_pub = rospy.Publisher('achievable_cmd_vel', TwistStamped, queue_size=1)
        self.status_pub = rospy.Publisher('status', String, queue_size=1)
        self.sub = rospy.Subscriber('cmd_vel', TwistStamped if self.stamped else Twist,
                                    self.receive, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.tick)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('Tricycle kinematics: vx + yaw_rate only; vy/in-place rotation rejected. Software outputs only.')

    def receive(self, message):
        with self.lock:
            self.latest = (copy.deepcopy(message), time.monotonic())
            # Do not leave a previous moving output active until the next tick
            # when an unsupported or invalid command arrives.
            self.emit_latest()

    def emit_latest(self):
        now = rospy.Time.now()
        stamp = rospy.Time(0)
        requested = None
        if self.latest is None:
            result = Command()
        else:
            message, received = self.latest
            twist = message.twist if self.stamped else message
            values = (twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z)
            if all(math.isfinite(v) for v in values):
                requested = dict(vx=values[0], vy=values[1], yaw_rate=values[5])
            if self.stamped:
                stamp = message.header.stamp
            if time.monotonic()-received > self.timeout:
                result = Command(reason='command_timeout')
            elif self.stamped and (stamp == rospy.Time(0) or
                    (now-stamp).to_sec() > self.timeout or (now-stamp).to_sec() < -self.future_tolerance):
                result = Command(reason='invalid_or_stale_stamp')
            elif self.stamped and message.header.frame_id != self.frame:
                result = Command(reason='unexpected_command_frame')
            elif not all(math.isfinite(v) for v in values):
                result = Command(reason='nonfinite_velocity')
            elif any(abs(v) > self.limits.lateral_tolerance for v in (values[2], values[3], values[4])):
                result = Command(reason='nonplanar_velocity_not_supported')
            else:
                result = convert(values[0], values[1], values[5], self.limits)
        self.publish(result, now, stamp, requested)

    def publish(self, result, now, source_stamp, requested):
        command = ChassisCommand()
        command.header.stamp = now
        command.header.frame_id = self.frame
        command.source_stamp = source_stamp
        command.speed = result.speed
        command.steering_angle = result.steering_angle
        command.yaw_rate = result.yaw_rate
        command.accepted = result.accepted
        command.saturated = result.saturated
        command.reason = result.reason
        self.pub.publish(command)
        achievable = TwistStamped()
        achievable.header = copy.deepcopy(command.header)
        achievable.twist.linear.x = result.speed
        achievable.twist.angular.z = result.yaw_rate
        self.achievable_pub.publish(achievable)
        self.status_pub.publish(String(data=json.dumps(dict(
            software_only=True, actuator_allowed=False, accepted=result.accepted,
            saturated=result.saturated, reason=result.reason, requested=requested,
            speed_mps=result.speed, steering_rad=result.steering_angle,
            steering_deg=math.degrees(result.steering_angle),
            achievable_yaw_rate=result.yaw_rate), allow_nan=False)))

    def tick(self, event):
        with self.lock:
            self.emit_latest()

    def shutdown(self):
        with self.lock:
            self.publish(Command(reason='shutdown'), rospy.Time.now(), rospy.Time(0), None)


if __name__ == '__main__':
    rospy.init_node('tricycle_controller')
    try:
        controller = TricycleController()
        rospy.spin()
    except (ValueError, TypeError) as exc:
        rospy.logfatal('Cannot initialize tricycle controller: %s', exc)
        raise
