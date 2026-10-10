#!/usr/bin/env python3
"""Read-only validation of live AprilTag-to-preview axis conversion."""
import json
import threading
import time
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

rospy.init_node('verify_direct_apriltag_axes', anonymous=True)
lock = threading.Lock()
raw, converted, matches = {}, {}, []


def check(stamp):
    if stamp not in raw or stamp not in converted:
        return
    optical, axes = raw.pop(stamp), converted.pop(stamp)
    expected = (optical[2], -optical[0], -optical[1])
    error = max(abs(a-b) for a, b in zip(expected, axes))
    matches.append(dict(optical_xyz=optical, converted_xyz=axes, max_error=error))


def detection(msg):
    with lock:
        for item in msg.detections:
            if list(item.id) == [0]:
                p = item.pose.pose.pose.position
                stamp = item.pose.header.stamp.to_nsec()
                raw[stamp] = (p.x, p.y, p.z)
                check(stamp)


def pose(msg):
    with lock:
        p = msg.pose.position
        stamp = msg.header.stamp.to_nsec()
        converted[stamp] = (p.x, p.y, p.z)
        check(stamp)


sub_raw = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, detection, queue_size=20)
sub_pose = rospy.Subscriber('/apriltag_servo/preview/target_pose', PoseStamped, pose, queue_size=20)
deadline = time.monotonic() + 10
while time.monotonic() < deadline:
    with lock:
        if len(matches) >= 5:
            break
    time.sleep(.05)
with lock:
    assert len(matches) >= 5, 'Not enough matching live observations'
    assert all(item['max_error'] < 1e-12 for item in matches), matches
    print(json.dumps(dict(passed=True, samples=len(matches), examples=matches[:2]), indent=2))
