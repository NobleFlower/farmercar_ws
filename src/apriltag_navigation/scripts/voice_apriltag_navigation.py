#!/usr/bin/env python3
"""CH340 voice commands + AprilTag lane navigation -> tricycle cmd_vel."""
import json
import math
import os
import threading
import time

import rospy
import roslib.packages
import yaml
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, String, UInt8MultiArray

from apriltag_navigation.voice_navigation import (
    Detection,
    NavigationParameters,
    VoiceFrameParser,
    VoiceNavigationStateMachine,
)
from vehicle_controller.transport import SerialTransport


class VoiceAprilTagNavigation:
    def __init__(self):
        tags_file = str(rospy.get_param(
            "~tags_file",
            os.path.join(roslib.packages.get_pkg_dir("apriltag_ros"),
                         "config", "tags.yaml")))
        self.id_to_name, self.tag_names = self._load_tags(tags_file)
        parameters = NavigationParameters(
            wheelbase=float(rospy.get_param("~wheelbase", 0.839)),
            max_steering_deg=float(rospy.get_param("~max_steering_deg", 50.0)),
            low_speed_forward_mps=float(rospy.get_param("~low_speed_forward_mps", 0.3)),
            low_speed_reverse_mps=float(rospy.get_param("~low_speed_reverse_mps", 0.3)),
            cruise_speed_mps=float(rospy.get_param("~cruise_speed_mps", 0.3)),
            approach_speed_mps=float(rospy.get_param("~approach_speed_mps", 0.3)),
            cross_turn_speed_mps=float(rospy.get_param("~cross_turn_speed_mps", 1.0)),
            cross_turn_steering_deg=float(rospy.get_param("~cross_turn_steering_deg", 45.0)),
            terminal_approach_distance_m=float(
                rospy.get_param("~terminal_approach_distance_m", 1.0)),
            cross_target_stop_distance_m=float(
                rospy.get_param("~cross_target_stop_distance_m", 0.5)),
            target_pass_distance_m=float(
                rospy.get_param("~target_pass_distance_m", 0.5)),
            max_tracking_distance_m=float(
                rospy.get_param("~max_tracking_distance_m", 12.0)),
            tag_timeout_s=float(rospy.get_param("~tag_timeout_s", 0.5)),
            turn_timeout_s=float(rospy.get_param("~turn_timeout_s", 10.0)))
        self.controller = VoiceNavigationStateMachine(parameters, self.tag_names)
        self.parser = VoiceFrameParser()
        self.rate = float(rospy.get_param("~control_rate", 20.0))
        self.detections_topic = str(rospy.get_param(
            "~detections_topic", "/tag_detections"))
        self.voice_port = str(rospy.get_param(
            "~voice_port", "/dev/ttyCH341USB0"))
        self.open_voice_port = bool(rospy.get_param("~open_voice_port", True))
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        if not math.isfinite(self.rate) or self.rate <= 0:
            raise ValueError("control_rate must be positive")
        self.lock = threading.RLock()
        self.voice_transport = None
        if self.open_voice_port:
            self.voice_transport = SerialTransport(self.voice_port, self.baudrate)
        self.last_command = None
        self.last_voice_at = 0.0
        self.cmd_pub = rospy.Publisher("cmd_vel", TwistStamped, queue_size=1)
        self.status_pub = rospy.Publisher("status", String, queue_size=1, latch=True)
        self.follow_pub = rospy.Publisher(
            "/apriltag_servo/enable", Bool, queue_size=1, latch=True)
        self.tag_sub = rospy.Subscriber(
            self.detections_topic, AprilTagDetectionArray,
            self.on_detections, queue_size=1)
        if not self.open_voice_port:
            self.inject_sub = rospy.Subscriber(
                "/voice_apriltag_navigation/rx_inject",
                UInt8MultiArray, self.inject, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
        self.required_tags = ("road14", "road21", "road12", "road23")
        self.missing_tags = sorted(set(self.required_tags) - self.tag_names)
        if self.missing_tags:
            rospy.logwarn(
                "Cross-lane tags are not configured: %s; add their real IDs to %s",
                ", ".join(self.missing_tags), tags_file)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "Voice AprilTag navigation listening on %s (%s baud); %d tags configured",
            self.voice_port if self.open_voice_port else "disabled",
            self.baudrate, len(self.id_to_name))

    @staticmethod
    def _load_tags(path):
        with open(path, "r") as stream:
            document = yaml.safe_load(stream) or {}
        id_to_name = {}
        for item in document.get("standalone_tags", []):
            name = item.get("name")
            tag_id = item.get("id")
            if name is not None and tag_id is not None:
                id_to_name[int(tag_id)] = str(name)
        return id_to_name, set(id_to_name.values())

    def on_detections(self, message):
        now = time.monotonic()
        ros_now = rospy.Time.now()
        detections = []
        for item in message.detections:
            if len(item.id) != 1:
                continue
            tag_id = int(item.id[0])
            name = self.id_to_name.get(tag_id)
            if name is None:
                continue
            stamp = item.pose.header.stamp
            if stamp == rospy.Time(0):
                stamp = message.header.stamp
            age = (ros_now - stamp).to_sec()
            if stamp == rospy.Time(0) or age > self.controller.params.tag_timeout_s or age < -0.05:
                continue
            pose = item.pose.pose.pose
            values = (pose.position.x, pose.position.y, pose.position.z)
            if not all(math.isfinite(value) for value in values):
                continue
            detections.append(Detection(tag_id, name, values[0], values[1],
                                        values[2], stamp.to_sec()))
        with self.lock:
            self.controller.on_detections(detections, now)

    def _read_voice(self):
        if self.voice_transport is None:
            return
        try:
            data = self.voice_transport.read()
        except (OSError, ValueError) as exc:
            rospy.logerr_throttle(2.0, "CH340 voice read failed: %s", exc)
            return
        self._apply_voice_bytes(data)

    def inject(self, message):
        self._apply_voice_bytes(bytes(message.data))

    def _apply_voice_bytes(self, data):
        for command in self.parser.feed(data):
            with self.lock:
                self.controller.parser_errors = self.parser.errors
                self.controller.on_voice_command(command, time.monotonic())
                self.last_command = command.kind
                self.last_voice_at = time.monotonic()
            rospy.loginfo("Voice command: %s%s", command.kind,
                          " -> lane %d" % command.target_lane
                          if command.target_lane else "")

    def tick(self, _event):
        self._read_voice()
        with self.lock:
            command = self.controller.tick(time.monotonic())
        message = TwistStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.twist.linear.x = command.speed_mps
        message.twist.angular.z = command.yaw_rate_radps
        self.cmd_pub.publish(message)
        self.follow_pub.publish(Bool(data=command.active))
        status = dict(
            mode=self.controller.mode, phase=command.phase, lane=command.lane,
            target_lane=self.controller.target_lane, target_tag=command.target_name,
            active=command.active, reason=command.reason,
            speed_mps=command.speed_mps,
            steering_deg=math.degrees(command.steering_rad),
            yaw_rate_radps=command.yaw_rate_radps,
            command=self.last_command,
            tag_timeout_s=self.controller.params.tag_timeout_s,
            missing_cross_lane_tags=self.missing_tags,
            parser_errors=self.controller.parser_errors,
            voice_port=self.voice_port if self.open_voice_port else None)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def shutdown(self):
        with self.lock:
            self.controller.on_voice_command(
                type("StopCommand", (), {"kind": "stop"})(), time.monotonic())
            command = self.controller.tick(time.monotonic())
        message = TwistStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.twist.linear.x = command.speed_mps
        message.twist.angular.z = command.yaw_rate_radps
        self.cmd_pub.publish(message)
        self.follow_pub.publish(Bool(data=False))
        if self.voice_transport:
            self.voice_transport.close()
            self.voice_transport = None


if __name__ == "__main__":
    rospy.init_node("voice_apriltag_navigation")
    try:
        node = VoiceAprilTagNavigation()
        rospy.spin()
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        rospy.logfatal("Cannot start voice AprilTag navigation: %s", exc)
        raise
