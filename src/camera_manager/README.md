# 最小相机与底盘方向模型

```bash
cd /home/tree/farmercar_ws
source devel/setup.bash
roslaunch camera_manager system_start.launch desired_distance:=1.0
```

统一启动 USB 相机、image_proc、原 AprilTag 检测器、最小 TF、跟随预览、tricycle_controller底盘运动学和双向通信桥接（默认无串口）。
默认不打开底盘串口。原有相机内参与 AprilTag 检测配置保留。

```text
base_link -> steering_link -> camera_link -> camera_optical_frame -> apriltag_link
```

只表示底盘、转轴、相机的轴方向关系。各原点重合，不建立轮胎、后轴、安装位置、高度等
精确几何模型。camera_link 为 x前/y左/z上，camera_optical_frame 为 x右/y下/z前。
光学轴变换为 `x=z_optical, y=-x_optical, z=-y_optical`。
转轴边绕 z 转动；无反馈时默认 assume_zero_steering=true 表示当前前轮回正。

TF 转角反馈可由 `/vehicle/steering_joint_states` 提供：JointState 的 steering_joint
position 为 rad，左正。收到反馈后，过期或非法反馈停止更新转轴 TF。
该 TF 用来表示方向，当前跟随控制不使用它来补偿转向。

控制直接使用 AprilTag 原始值：前向距离取光学 z，横向偏差取 -x，不叠加安装偏移、
轴距或高度，不计算三维距离来修正 z。相机此前约 2.9m 的观测值不写入控制目标或 TF。
伺服期望停距是独立参数 desired_distance，默认 1.0m，单位米。

```bash
rostopic echo /apriltag_servo/preview/target_pose
rostopic echo /apriltag_servo/preview/status
rosrun tf tf_echo camera_link apriltag_link
```

完整当前说明见工作空间 APRILTAG_INTEGRATION.md。

底盘软件链路：apriltag_servo/preview/cmd_vel -> velocity_adapter -> tricycle_controller ->
/tricycle_controller/command（速度+前轮角）、/tricycle_controller/achievable_cmd_vel
-> 原双向通信桥接（默认无串口）适配器。不启动底盘驱动。控制器详情见 tricycle_controller/README.md。

统一启动时跟随功能默认禁用。用 /apriltag_servo/enable（Bool）手动启用或关闭。
vehicle_controller_bridge已适配最终命令和底盘反馈，默认serial_enabled=false、armed=false。
最新完整操作与协议以工作空间DRIVER_HANDOVER.md和vehicle_controller/PROTOCOL.md为准。
