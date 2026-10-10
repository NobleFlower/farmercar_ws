#!/usr/bin/env python3
"""Tag -> manual follow gate -> adapter -> chassis -> dry-run communication."""
import json
import math
import struct
import time
import unittest
import rosnode
import rospy
import rostest
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import Bool, String, UInt8MultiArray
from tricycle_controller.msg import ChassisCommand
from vehicle_controller.wire import packet


class PipelineTests(unittest.TestCase):
    def test_pipeline_faults_manual_start_and_node_loss(self):
        self.cmd = None
        self.tx = None
        self.servo = None
        self.bridge = None
        subs = [rospy.Subscriber('/tricycle_controller/command', ChassisCommand, lambda m:setattr(self,'cmd',m)),
                rospy.Subscriber('/vehicle_controller/tx_packet',UInt8MultiArray,lambda m:setattr(self,'tx',list(m.data))),
                rospy.Subscriber('/pipeline_test/servo/status',String,lambda m:setattr(self,'servo',json.loads(m.data))),
                rospy.Subscriber('/vehicle_controller/status',String,lambda m:setattr(self,'bridge',json.loads(m.data)))]
        pub = rospy.Publisher('/pipeline_test/detections', AprilTagDetectionArray, queue_size=1)
        enable = rospy.Publisher('/apriltag_servo/enable',Bool,queue_size=1)
        arm = rospy.Publisher('/vehicle_controller/arm',Bool,queue_size=1)
        rx = rospy.Publisher('/vehicle_controller/rx_inject',UInt8MultiArray,queue_size=1)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and (pub.get_num_connections()==0 or enable.get_num_connections()<2 or arm.get_num_connections()==0 or rx.get_num_connections()==0):
            time.sleep(.03)
        self.assertGreater(pub.get_num_connections(),0)
        self.assertGreaterEqual(enable.get_num_connections(),2)
        feedback = UInt8MultiArray(data=list(packet(0xEF,2,struct.pack('<hHBBHH',0,900,2,13,0,0))))

        def message(z=3,left=.3,empty=False,tag_id=0,age=0,frame='camera_optical_frame'):
            msg=AprilTagDetectionArray()
            msg.header.frame_id=frame
            msg.header.stamp=rospy.Time.now()-rospy.Duration(age)
            if not empty:
                d=AprilTagDetection()
                d.id=[tag_id]
                d.size=[.154]
                d.pose.header=msg.header
                d.pose.pose.pose.position.x=-left
                d.pose.pose.pose.position.z=z
                d.pose.pose.pose.orientation.w=1
                msg.detections=[d]
            return msg

        def until(make,predicate,timeout=4):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                if make is not None:
                    pub.publish(make())
                rx.publish(feedback)
                time.sleep(.03)
                if self.cmd and self.tx and self.servo and self.bridge and predicate():
                    return
            self.fail(str(self.servo)+' '+str(self.bridge)+' '+str(self.cmd))

        def stopped():
            return self.cmd.speed==0 and self.cmd.yaw_rate==0 and self.bridge['output_speed_mps']==0 and self.tx==[0xEE,1,4,0,0,0x84,3,0x7A]

        until(lambda:message(),lambda:self.servo['reason']=='follow_disabled' and stopped())
        self.assertFalse(self.servo['enabled'])
        enable.publish(Bool(data=True))
        until(lambda:message(),lambda:self.cmd.speed==.15 and self.bridge['reason']=='driver_disarmed')
        self.assertEqual(self.tx,[0xEE,1,4,0,0,0x84,3,0x7A])
        arm.publish(Bool(data=True))  # Serial remains false: only exercise wire gating.
        for lateral in (.3,-.3,0):
            delta=math.atan(.839*2*lateral/(9+lateral*lateral))
            until(lambda y=lateral:message(left=y),lambda:self.cmd.speed==.15 and abs(self.cmd.steering_angle-delta)<1e-9 and self.bridge['reason']=='ok' and self.tx[3]==15)
            self.assertAlmostEqual(self.cmd.yaw_rate,.15*math.tan(delta)/.839)
            self.assertFalse(self.cmd.saturated)
            self.assertFalse(self.bridge['serial_enabled'])
            self.assertFalse(self.bridge['packet_written'])
            self.assertEqual(self.tx[-1],sum(self.tx[:-1])&255)
        scenarios=[
            (lambda:message(z=1.04,left=2),'desired_distance_reached'),
            (lambda:message(z=.5),'desired_distance_reached'),
            (lambda:message(empty=True),'target_not_detected'),
            (lambda:message(tag_id=1),'target_not_detected'),
            (lambda:message(age=2),'invalid_or_stale_detection_stamp'),
            (lambda:message(age=-2),'invalid_or_stale_detection_stamp'),
            (lambda:message(frame='wrong_camera'),'unexpected_camera_frame'),
            (lambda:message(left=float('nan')),'invalid_detection_pose'),
            (lambda:message(z=-1),'invalid_geometry'),
        ]
        for make,reason in scenarios:
            until(make,lambda r=reason:self.servo['reason']==r and stopped())
        until(lambda:message(),lambda:self.cmd.speed>0 and self.bridge['output_speed_mps']>0)
        until(None,lambda:self.servo['reason']=='target_timeout_or_clock_jump' and stopped(),timeout=2)
        until(lambda:message(),lambda:self.cmd.speed>0 and self.bridge['output_speed_mps']>0)
        enable.publish(Bool(data=False))
        until(lambda:message(),lambda:self.servo['reason']=='follow_disabled' and self.bridge['reason']=='follow_disabled' and stopped())
        enable.publish(Bool(data=True))
        until(lambda:message(),lambda:self.cmd.speed>0 and self.bridge['output_speed_mps']>0)
        rosnode.kill_nodes(['/apriltag_follow_preview'])
        until(None,lambda:stopped(),timeout=2)
        rosnode.kill_nodes(['/velocity_adapter'])
        until(None,lambda:self.cmd.reason=='command_timeout' and not self.cmd.accepted and stopped(),timeout=2)


if __name__=='__main__':
    rospy.init_node('control_pipeline_test')
    rostest.rosrun('camera_manager','control_pipeline',PipelineTests)
