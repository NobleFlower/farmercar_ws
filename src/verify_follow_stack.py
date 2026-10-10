#!/usr/bin/env python3
"""Live acceptance check; toggles follow ONLY if physical serial is disabled.

Never arms the driver, injects feedback, or publishes chassis commands.
Restores follow disabled on exit. Run after the unified launch is ready.
"""
import json
import math
import time
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Bool, String
from tricycle_controller.msg import ChassisCommand

rospy.init_node('verify_follow_stack',anonymous=True)
assert rospy.get_param('/vehicle_controller_bridge/serial_enabled') is False, 'Only run with serial disabled'
latest={}
commands=[]
tag_count=[0]


def tag(message):
    for item in message.detections:
        if list(item.id)==[int(rospy.get_param('/apriltag_follow_preview/target_tag_id',0))]:
            tag_count[0]+=1
            latest['tag_z']=item.pose.pose.pose.position.z


def command(message):
    commands.append(message)


subs=[rospy.Subscriber('/tag_detections',AprilTagDetectionArray,tag),
      rospy.Subscriber('/tricycle_controller/command',ChassisCommand,command),
      rospy.Subscriber('/apriltag_servo/preview/status',String,lambda m:latest.__setitem__('servo',json.loads(m.data))),
      rospy.Subscriber('/vehicle_controller/status',String,lambda m:latest.__setitem__('bridge',json.loads(m.data)))]
enable=rospy.Publisher('/apriltag_servo/enable',Bool,queue_size=1)


def wait(predicate,timeout=25):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return
        time.sleep(.03)
    raise RuntimeError('Live acceptance condition missing: '+str(latest))


try:
    wait(lambda:enable.get_num_connections()>=2 and tag_count[0]>=5 and 'servo' in latest and 'bridge' in latest and bool(commands))
    assert latest['bridge']['serial_enabled'] is False
    assert latest['bridge']['armed'] is False
    enable.publish(Bool(data=False))
    wait(lambda:not latest['servo']['enabled'] and commands[-1].speed==0 and latest['bridge']['output_speed_mps']==0,5)
    enable.publish(Bool(data=True))
    start=len(commands)
    wait(lambda:latest['servo']['enabled'] and latest['servo'].get('tag_forward_distance_m') is not None and len(commands)>=start+20 and commands[-1].accepted,10)
    # Check continuous fresh, feasible, final commands rather than an upstream preview only.
    samples=commands[start:]
    for item in samples:
        assert all(math.isfinite(v) for v in (item.speed,item.steering_angle,item.yaw_rate))
        assert 0<=item.speed<=.15+1e-6
        assert abs(item.steering_angle)<=math.radians(50)+1e-6
        assert abs(item.yaw_rate-item.speed*math.tan(item.steering_angle)/.839)<1e-8
    servo_snapshot=dict(latest['servo'])
    if servo_snapshot.get('tag_forward_distance_m') is not None and servo_snapshot['tag_forward_distance_m']>servo_snapshot['desired_distance_m']+.05:
        assert any(item.speed>0 for item in samples), 'Far visible target must produce a nonzero candidate'
    assert (rospy.Time.now()-commands[-1].header.stamp).to_sec()<.5
    assert latest['bridge']['packet_written'] is False
    assert latest['bridge']['output_speed_mps']==0
    links=[('/tag_detections','/apriltag_follow_preview'),
           ('/apriltag_servo/preview/cmd_vel','/velocity_adapter'),
           ('/velocity_adapter/adapted_cmd_vel','/tricycle_controller'),
           ('/tricycle_controller/command','/vehicle_controller_bridge')]
    state=rospy.get_master().getSystemState()[2]
    subscribers=dict(state[1])
    for topic,node in links:
        assert node in subscribers.get(topic,[]), (topic,node)
    result=dict(passed=True,matching_tag_frames=tag_count[0],command_samples=len(samples),
                last_tag_optical_z=latest['tag_z'],last_speed=commands[-1].speed,
                last_steering_deg=math.degrees(commands[-1].steering_angle),
                serial_enabled=False,driver_armed=False,physical_packet_written=False,
                graph_links=links)
    enable.publish(Bool(data=False))
    wait(lambda:not latest['servo']['enabled'] and commands[-1].speed==0 and latest['bridge']['reason']=='follow_disabled',5)
    result['restored_follow_disabled']=True
    print(json.dumps(result,indent=2))
finally:
    for _ in range(3):
        enable.publish(Bool(data=False))
        time.sleep(.05)
