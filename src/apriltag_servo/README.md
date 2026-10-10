# AprilTag 最小跟随预览

直接读取 `/tag_detections`，默认目标 ID=0。将相机光学轴重排为：

```text
forward = detection.position.z
left    = -detection.position.x
up      = -detection.position.y
```

仅做轴方向转换。没有安装位置、离地高度、轴距平移或转角补偿；无需查询 TF。
前向距离直接使用 AprilTag 的 z，未转换成欧氏距离。2.9m 只是此前现场观测。

```bash
roslaunch apriltag_servo follow.launch desired_distance:=1.0 target_tag_id:=0
# 节点已启动，跟随默认禁用；手动启用/关闭：
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: true'
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: false'
```

独立停距参数 desired_distance 默认 1.0m；距离误差取 optical_z-desired_distance。
默认死区 0.05m、最大速度 0.15m/s、加速度 0.15m/s²、失去检测超时 0.5s。
仅前进，近于目标则停止。最小纯追踪使用 forward/left 计算曲率，转角限幅 ±50°；
轴距 0.839m 只用于转角和角速度关系 omega=v*tan(delta)/L，不用于补偿标签坐标。
当前采用相机与底盘回正方向一致的近似来计算候选命令。

| Topic | Type | Meaning |
|---|---|---|
| /apriltag_servo/preview/cmd_vel | TwistStamped | 候选 linear.x / angular.z，底盘命令约定 base_link |
| /apriltag_servo/preview/steering_angle | Float64 | 候选前轮角，rad，左正 |
| /apriltag_servo/preview/target_pose | PoseStamped | 仅按轴重排的观测值，frame=camera_link |
| /apriltag_servo/preview/status | String JSON | tag_forward_distance_m、desired_distance_m、停止原因 |

状态 mounting_compensation=false、steering_compensation=false、actuator_allowed=false。
本包没有硬件发送接口。目标 TF 为 camera_optical_frame -> apriltag_link，直接使用检测位姿。
原有 road11 等标签 TF 保留。

空检测、错误 ID、过期/未来时间戳、错误坐标系、非法位姿和输入中断都会发布零候选命令。
不使用控制命令模拟实测转角。统一启动已接入vehicle_controller_bridge，默认不打开串口；未来真实串口须独立启用/解锁。

```bash
python3 src/apriltag_servo/test/test_control.py
rostest apriltag_servo faults.test
```

详细通信字段与单键启动交接见工作空间DRIVER_HANDOVER.md及vehicle_controller/PROTOCOL.md。
