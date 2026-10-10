# AprilTag 伺服链路与底盘通信交接（2026-10-10）

正式文档入口：[docs/README.md](docs/README.md)，合并手册：[docs/MANUAL.md](docs/MANUAL.md)。
本文件保留简明交接说明；部署、测试和协议详细内容以正式文档为准。

工作空间：tree@10.62.134.197:/home/tree/farmercar_ws。
当前二维码指已配置的AprilTag标记，默认目标ID=0。

## 完整链路

```text
usb_cam -> image_proc -> apriltag_ros -> apriltag_servo
 -> velocity_adapter -> tricycle_controller -> vehicle_controller_bridge
 -> 校验/门控/协议组帧 -> 可选真实串口
```

原始AprilTag坐标值只做x=z_optical、y=-x_optical、z=-y_optical轴重排，无安装外参补偿。
TF为base_link -> steering_link -> camera_link -> camera_optical_frame -> apriltag_link。
用户已验证方向正确。没有转角实测时按回正显示，不用期望转角代替反馈。
跟随使用原始optical z为距离，独立停距desired_distance默认1m。前进追踪，近于停距停止等待，不倒车。

## 一键启动与手动跟随

```bash
cd /home/tree/farmercar_ws
source devel/setup.bash
roslaunch camera_manager system_start.launch target_tag_id:=0 desired_distance:=1.0
```

节点统一启动，但跟随功能默认禁用，serial_enabled=false，驱动未解锁。
此时仍发布标签位姿和状态，最终运动输出为0。

```bash
# 启用/停止伺服软件计算
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: true'
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: false'
```

启用跟随不会自动打开串口或解锁驱动。查看：

```bash
rostopic echo /apriltag_servo/preview/status
rostopic echo /velocity_adapter/status
rostopic echo /tricycle_controller/command
rostopic echo /vehicle_controller/status
rostopic echo /vehicle_controller/feedback
```

上游节点的actuator_allowed=false只表明该节点不直接操作硬件。
实际发送许可看vehicle_controller/status中的serial_enabled、armed、actuation_permitted和packet_written。

## 将来连接硬件

ROS端已有串口桥接实现，不需要再次开发速度到前轮角转换。
硬件适配需确认下位机的协议支持、车体速度标度、转向90°回正/正方向及命令失联停车。
扩展V2协议的全部字段已在上位机实现，下位机固件本次未修改。

对应启动接口（本次不执行真实串口）：

```bash
roslaunch camera_manager system_start.launch \
  serial_enabled:=true vehicle_port:=/dev/ttyACM0 vehicle_baudrate:=115200 \
  vehicle_protocol:=extended desired_distance:=1.0
```

启动后仍禁止跟随、未解锁。硬件返回自动就绪无故障的有效反馈后，再由用户显式操作：

```bash
rostopic pub -1 /vehicle_controller/arm std_msgs/Bool 'data: true'
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: true'
```

停止跟随：enable=false。锁回驱动：arm=false。
主机急停：/vehicle_controller/emergency_stop=true；清false不自动重新解锁。
手动模式/下位机急停/故障会锁回未解锁，恢复后需要重新arm。
若当前反馈还未自动就绪，扩展TX先发“请求自动+停车”，待下位机认可后才允许非零运动。

串口默认V1兼容现有EE01/EF01，但该反馈缺少真实模式/故障等状态，默认不允许V1直接运动。
只有明确确认旧硬件的控制权、急停、超时和速度标度后才显式配置allow_legacy_actuation=true。
校准参数在src/vehicle_controller/config/bridge.yaml，可用vehicle_config指定外部yaml。
不要同时启动旧vehicle_core.launch占用串口。

## 给下位机开发者的字段约定

详细字节偏移、位图、校验和例帧：src/vehicle_controller/PROTOCOL.md。

发送最低需要：有符号车体期望速度、前轮期望角。
扩展还发送：驱动使能、停止请求、自动模式请求、命令序号。
反馈最低需要：速度/转角值以及是否为实测；理想扩展包含实际速度、实际前轮角、
硬件模式、就绪、急停、故障位图、最后收到的有效命令序号。
没有传感器就清测量有效位，不回显命令并假装实测。
反馈速度是实际车体速度定义，编码器轮速转换应按驱动轮位置完成。

上位机默认20Hz发送，建议下位机20Hz反馈；双方命令/反馈超时默认0.5s。
主机停止发送时，下位机自己的命令看门狗必须停车。不能依赖主机再发一个停车帧。
新传输使用非阻塞读和有限写超时，不阻塞ROS控制回调。

## 复验

```bash
bash run_checks.sh
python3 src/verify_live_axes.py
python3 src/verify_follow_stack.py
```

run_checks使用隔离测试master，串口分支只对虚拟PTY测试，不打开车辆串口。
verify_follow_stack要求serial_enabled=false、驱动未解锁，短暂启用软件跟随后恢复禁用；
不会arm、不会注入反馈、不会发布底盘指令，只检查现有真实图像链路。

验收覆盖：轴重排、左右/居中追踪、停距、错误ID/空检测/异常时间戳/非法坐标、
标签输入中断与恢复、手动启用/关闭、上游进程退出、差速/全向近似、构型限幅、
驱动解锁/急停/故障/手动优先、协议校验/半帧/坏帧、反馈丢失及虚拟串口收发。
软件验收不代表车辆运动闭环已在真实硬件上验证；实车运行需最后的协议/标度与控制响应联调。

## 本次验收结果与现场状态

远端编译通过；35项算法/协议测试及7项ROS集成测试通过。实际串口路径只对虚拟PTY验证。
真实相机链路验收通过：9条匹配标签帧、20条最终期望指令，已确认全部上层到桥接的话题连接。
本次检查时标签原始optical z约2.892m；软件候选速度/转角正常生成，没有车辆串口写入。
检查结束恢复跟随禁用。当前统一launch继续运行：serial_enabled=false、armed=false、follow_enabled=false。
启动PID记录/tmp/farmercar_system_20261010.pid，日志/tmp/farmercar_system_20261010.log。
本地validation目录保存编译、7项ROS测试及真实相机验收结果。
