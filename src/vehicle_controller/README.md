# Vehicle Controller：底盘通信与上层适配

当前统一启动使用新增的 vehicle_controller_bridge，旧C++ vehicle_controller_node保留旧手动路径。
桥接输入 `/tricycle_controller/command`，已完成速度/转角组帧、串口双向收发、反馈解析与ROS反馈。
完整字段、字节位置、单位、使能/故障语义和例帧见 **PROTOCOL.md**。

```bash
cd /home/tree/farmercar_ws
source devel/setup.bash
roslaunch camera_manager system_start.launch desired_distance:=1.0
```

默认不打开车辆串口，跟随禁用、驱动未解锁。手动启用/停止跟随软件：

```bash
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: true'
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: false'
```

```bash
rostopic echo /vehicle_controller/status
rostopic echo /vehicle_controller/desired_packet
rostopic echo /vehicle_controller/tx_packet
rostopic echo /vehicle_controller/feedback
```

配置：config/bridge.yaml。驱动独立启动：launch/vehicle_bridge.launch。
serial_enabled=false时tx_packet只发布ROS，不写串口；rx_inject只在此模式订阅。
真实串口模式不允许诊断注入。手动使能、驱动arm、有效指令、反馈就绪等门控同时满足才允许运动。
任何无效、超时、急停或故障会发停车帧；故障/手动接管锁回未解锁状态。
只有有效实测转角才发布JointState，不将命令、旧设置值或默认模拟反馈当作实测。

协议legacy兼容当前8字节EE01/EF01；extended可选发送11字节EE02、反馈14字节EF02。
扩展增加使能/停止/自动请求、序号、就绪/模式/急停/测量有效/故障及ack。
上位机已支持扩展；下位机固件本次未修改，不能假定它已支持新帧。
当前V1速度反馈可能只是低速设置值，所以默认不标为实测，且旧帧运动门控需显式opt-in。

新Python传输映射波特率至B115200，非阻塞读、有限写超时，不受旧C++无限select阻塞影响。
不能同时运行新桥接与旧vehicle_core.launch占用同一串口。

```bash
python3 src/vehicle_controller/test/test_wire.py
rostest vehicle_controller bridge.test
rostest vehicle_controller serial_transport.test
rostest camera_manager control_pipeline.test
```

串口测试只针对虚拟PTY，不针对车辆设备。本次没有向实际底盘发送运动命令。
