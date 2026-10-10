#!/usr/bin/env python3
"""Exercise the actual ROS serial branch ONLY against a virtual PTY MCU."""
import json
import math
import os
import pty
import signal
import struct
import subprocess
import threading
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Bool, String
from tricycle_controller.msg import ChassisCommand
from vehicle_controller.msg import VehicleFeedback
from vehicle_controller.wire import packet


class SerialTests(unittest.TestCase):
    def test_virtual_mcu_roundtrip_and_feedback_loss(self):
        master, slave=pty.openpty()
        path=os.ttyname(slave)
        self.assertTrue(path.startswith('/dev/pts/'))
        os.set_blocking(master,False)
        running=threading.Event()
        running.set()
        send_feedback=threading.Event()
        send_feedback.set()
        frames=[]
        worker_errors=[]

        def mcu():
            buffer=bytearray()
            sequence=0
            while running.is_set():
                try:
                    try:
                        buffer.extend(os.read(master,1024))
                    except BlockingIOError:
                        pass
                    while buffer:
                        if buffer[0]!=0xEE:
                            del buffer[0]
                            continue
                        if len(buffer)<3 or len(buffer)<buffer[2]+4:
                            break
                        length=buffer[2]+4
                        frame=bytes(buffer[:length])
                        del buffer[:length]
                        if frame[-1]==sum(frame[:-1])&255 and frame[1]==2:
                            frames.append(frame)
                            sequence=struct.unpack('<H',frame[8:10])[0]
                    if send_feedback.is_set():
                        reply=packet(0xEF,2,struct.pack('<hHBBHH',8,1000,2,13,0,sequence))
                        os.write(master,reply)
                    time.sleep(.02)
                except OSError as exc:
                    if running.is_set():
                        worker_errors.append(str(exc))
                    break

        thread=threading.Thread(target=mcu,daemon=True)
        thread.start()
        self.status=None
        self.feedback=None
        subs=[rospy.Subscriber('/vehicle_controller/status',String,lambda m:setattr(self,'status',json.loads(m.data))),
              rospy.Subscriber('/vehicle_controller/feedback',VehicleFeedback,lambda m:setattr(self,'feedback',m))]
        cmd_pub=rospy.Publisher('/serial_test/command',ChassisCommand,queue_size=1)
        enable=rospy.Publisher('/apriltag_servo/enable',Bool,queue_size=1)
        arm=rospy.Publisher('/vehicle_controller/arm',Bool,queue_size=1)
        process=subprocess.Popen(['rosrun','vehicle_controller','vehicle_bridge_node.py',
                                  '_serial_enabled:=true','_port:='+path,'_protocol_version:=extended',
                                  '_require_ack:=true','chassis_command:=/serial_test/command'],
                                 stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            deadline=time.monotonic()+6
            while time.monotonic()<deadline and (cmd_pub.get_num_connections()==0 or enable.get_num_connections()==0 or arm.get_num_connections()==0):
                time.sleep(.03)
            self.assertIsNone(process.poll())
            self.assertGreater(cmd_pub.get_num_connections(),0)

            def wait(predicate):
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    msg=ChassisCommand()
                    msg.header.stamp=rospy.Time.now()
                    msg.header.frame_id='base_link'
                    msg.source_stamp=msg.header.stamp
                    msg.speed=.1
                    msg.steering_angle=math.radians(10)
                    msg.yaw_rate=.1*math.tan(msg.steering_angle)/.839
                    msg.accepted=True
                    cmd_pub.publish(msg)
                    time.sleep(.03)
                    if self.status and frames and predicate():
                        return
                self.fail(str(self.status)+' '+str(worker_errors))

            wait(lambda:self.status['reason']=='follow_disabled' and self.status['packet_written'])
            self.assertEqual(struct.unpack('<hH',frames[-1][3:7]),(0,900))
            enable.publish(Bool(data=True))
            arm.publish(Bool(data=True))
            wait(lambda:self.status['reason']=='ok' and self.status['packet_written'] and struct.unpack('<hH',frames[-1][3:7])==(10,1000))
            self.assertTrue(self.status['serial_enabled'])
            self.assertTrue(self.status['actuation_permitted'])
            self.assertFalse(self.feedback.simulated)
            self.assertTrue(self.feedback.fresh)
            self.assertAlmostEqual(self.feedback.speed,.08)
            self.assertAlmostEqual(self.feedback.steering_angle,math.radians(10))
            self.assertEqual(frames[-1][7],5)
            send_feedback.clear()
            wait(lambda:self.status['reason']=='feedback_timeout' and struct.unpack('<hH',frames[-1][3:7])==(0,900))
            self.assertTrue(frames[-1][7]&2)
            send_feedback.set()
            wait(lambda:self.status['reason']=='ok' and struct.unpack('<hH',frames[-1][3:7])==(10,1000))
            enable.publish(Bool(data=False))
            wait(lambda:self.status['reason']=='follow_disabled' and struct.unpack('<hH',frames[-1][3:7])==(0,900))
            self.assertEqual(worker_errors,[])
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            running.clear()
            thread.join(timeout=1)
            os.close(master)
            os.close(slave)


if __name__=='__main__':
    rospy.init_node('serial_bridge_test')
    rostest.rosrun('vehicle_controller','serial_transport',SerialTests)
