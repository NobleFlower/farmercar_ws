# TF 检查与控制器对接（2026-10-10）

> 当前软件链路及驱动桥接已经完成，跟随默认禁用，通过/apriltag_servo/enable手动启用。
> 下面TF检查仍适用；旧“尚未接入串口节点”的开发建议已由新增vehicle_controller_bridge实现。
> 最新启动、双向反馈、门控和协议以DRIVER_HANDOVER.md及vehicle_controller/PROTOCOL.md为准。
## 当前任务状态

| 项目 | 当前实现 |
|---|---|
| 一键感知启动 | camera_manager/system_start.launch，包含 usb_cam、image_proc、原 AprilTag 检测 |
| 最小 TF | base_link -> steering_link -> camera_link -> camera_optical_frame -> apriltag_link |
| 坐标处理 | x=z_optical、y=-x_optical、z=-y_optical；无安装位置、距离、转角补偿 |
| 转轴 | 当前无反馈，assume_zero_steering=true；可选 JointState 用于 TF 显示 |
| 跟随控制 | apriltag_servo，直接使用光学 z 为距离，默认 desired_distance=1.0m |
| 通用速度适配 | velocity_adapter 接受vx/vy/omega，对vy非零做方向近似，再进入tricycle_controller |
| 底盘软件输出 | /tricycle_controller/command：纵向速度、前轮角、可实现偏航率 |
| 底盘约束 | wheelbase=0.839m，转角±50°，v上限0.15m/s，omega=v*tan(delta)/L |
| 协议适配 | vehicle_controller/servo_preview_adapter.py 生成 EE 协议预览，20Hz，无串口 |
| 实车接口 | 已接入新vehicle_controller_bridge，默认串口关闭、驱动未解锁 |

## 1. 准备环境与确认节点

所有命令在远端终端执行：

```bash
ssh tree@10.62.134.197
cd /home/tree/farmercar_ws
source devel/setup.bash
rosnode list
```

应包含 `/usb_cam`、`/usb_cam/image_proc`、`/apriltag_ros_continuous_node`、
`/vehicle_camera_tf`、`/apriltag_follow_preview`、`/velocity_adapter`、`/tricycle_controller`、`/servo_preview_adapter`。
当前统一launch已运行、跟随禁用，不要重复启动。

## 2. 逐段检查 TF

```bash
rosrun tf tf_echo base_link steering_link
rosrun tf tf_echo steering_link camera_link
rosrun tf tf_echo camera_link camera_optical_frame
rosrun tf tf_echo camera_link apriltag_link
```

分别执行，每个命令 Ctrl+C 退出后再执行下一个。

| 检查 | 预期 |
|---|---|
| base_link -> steering_link | 平移全零；当前回正假设下 rotation 为单位四元数 |
| steering_link -> camera_link | 平移全零，rotation 为单位四元数 |
| camera_link -> camera_optical_frame | 平移全零；RPY约[-90°,0°,-90°]，四元数约[-0.5,+0.5,-0.5,+0.5] |
| camera_link -> apriltag_link | 标签前方位置体现在 x；相机右侧标签体现在负 y |

四元数整体变号表示同一个旋转。不能要求标签本身姿态为零，它取决于标签平面朝向。
固定 TF 发布在 /tf_static，转轴与标签 TF 发布在 /tf。

当前所有辅助 frame 原点重合是简化设计，不应通过 TF 寻找轮胎位置或1.5m安装高度。
TF 检查的是轴方向与链路连接。

## 3. 用 RViz 检查方向

```bash
rviz
```

Fixed Frame 设 base_link，Add TF；展开 Frames 并打开名称/坐标轴。
红色=X、绿色=Y、蓝色=Z。回正时相机光学蓝色 Z 应与底盘红色 X 同向，
光学红色 X 指向底盘右侧(-Y)，光学绿色 Y 指向下方(-Z)。
相机和底盘原点重合是预期效果。若看到旧 rear_axle_link/车轮等缓存 frame，重开 RViz。
标签被遮挡后，TF 会保留最后一次观测，不能仅凭标签 frame 仍存在判断检测有效。

## 4. 对照同一条 AprilTag 原始观测

```bash
rostopic echo -n 1 /tag_detections
rostopic echo -n 1 /apriltag_servo/preview/target_pose
python3 src/verify_live_axes.py
```

原始 detection frame 为 camera_optical_frame，target_pose frame 为 camera_link。
应符合 `(x,y,z)_target=(z,-x,-y)_optical`。两个独立 echo 采样时间不同，允许轻微波动；
verify_live_axes.py 按同一个时间戳配对至少5条数据，成功输出 passed=true、max_error=0。
它只订阅数据，无控制发布。

不移动车辆时，可移动标签检查：正前方横向y接近0；镜头左侧转换y>0；镜头右侧y<0；
前后移动改变原始z及转换x。不要把先前2.9m当作检查标定常量。

## 5. 检查候选控制与标签丢失

```bash
rostopic echo /apriltag_servo/preview/status
rostopic echo /apriltag_servo/preview/cmd_vel
rostopic echo /vehicle_controller/status
```

默认跟随禁用，先手动发布 /apriltag_servo/enable=true检查软件候选，检查后发布false。串口默认关闭且不需要arm。
前向距离 tag_forward_distance_m 在有效跟随状态下等于 optical z；desired_distance_m 是独立停距参数。
远于停距+0.05m时有前进候选速度；小于等于停距+0.05m时停止，不倒车。
前进时标签偏左预期 steering_rad、angular.z 为正，偏右为负。
v=0时omega=0。速度变化有平滑限制，等待几帧后观察。

用遮挡标签验证丢失：候选速度与偏航速度应变零；停止原因可能为 target_not_detected，
输入中断则超过0.5s变 target_timeout_or_clock_jump。恢复检测后恢复候选命令。
驱动桥接follow_disabled/driver_disarmed状态下保持停车，即使上层存在非零候选也不执行。
标准零速度、回正90°预览帧应为 `EE 01 04 00 00 84 03 7A`。

确认上层actuator_allowed=false，驱动dry_run=true、armed=false、packet_written=false：

```bash
rostopic info /vehicle_controller/tx_packet
```

tx_packet为门控后的协议帧。当前未解锁且串口关闭，所以保持停车帧。desired_packet可查看未经门控的上层期望，不作为串口发送源。

## 6. 软件模拟转轴与自动化检查

```bash
python3 src/apriltag_servo/test/test_control.py
python3 src/test_protocol_preview.py
rostest apriltag_servo faults.test
rostest camera_manager geometry_preview.test
```

rostest 默认使用独立测试 master。geometry_preview 在测试环境模拟0°/+20°/-20°，
核对相机朝向以及反馈过期后的TF，不连接底盘。不要向正在运行的正式
/vehicle/steering_joint_states 发布测试值；这会改变正式TF的反馈状态。
当前无实测反馈，因此真实转轴朝向改变时TF不会自动跟随，这是回正假设的结果。
控制目前直接使用相机观测，不消费转轴TF来补偿。

## 7. 修改控制参数与重启

统一 launch 可暴露：target_tag_id、desired_distance、enable_follow_preview。

```bash
roslaunch camera_manager system_start.launch target_tag_id:=0 desired_distance:=1.5
```

此命令用于下一次启动，当前已运行时先停旧 launch，再执行。后台PID记录在
/tmp/farmercar_system_20261010.pid，可先确认：

```bash
ps -fp "$(cat /tmp/farmercar_system_20261010.pid)"
```

确认它是当前统一launch后，用SIGINT停止：

```bash
kill -INT "$(cat /tmp/farmercar_system_20261010.pid)"
```

其他控制增益、速度、加速度、超时在 src/apriltag_servo/config/follow.yaml。
desired_distance 的 launch 参数会覆盖 YAML 同名参数，应通过 launch 参数设置停距。
参数在节点启动时读取，rosparam set 不会即时更新运行中的计算器。
协议适配器参数在 src/vehicle_controller/launch/servo_preview.launch；两处 max_speed 应一致。

## 当前控制器对接已完成

```text
AprilTag -> apriltag_servo -> velocity_adapter -> tricycle_controller
 -> /tricycle_controller/command -> vehicle_controller_bridge -> 门控帧/双向串口
```

桥接已完成实际串口代码、组帧、解析、反馈话题、启用/解锁/超时/故障门控。
默认serial_enabled=false、armed=false，伺服默认禁用。
实时反馈/发送及协议详见DRIVER_HANDOVER.md和vehicle_controller/PROTOCOL.md。
新的桥接不使用旧C++的阻塞select循环。旧vehicle_core.launch保留兼容手动代码，不同时启动。
本次下位机固件未修改；真实串口及执行器标度仍需硬件联调。

通用差速/全向速度输入见velocity_adapter/README.md；最终硬件消费纵向速度和前轮转角，
不应直接消费适配层未限幅的中间速度。
