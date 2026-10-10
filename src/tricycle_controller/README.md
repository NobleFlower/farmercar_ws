# 前轮转向三轮车底盘运动学控制

独立包 `tricycle_controller`：将速度期望转换为该底盘可实现的纵向速度和前轮转角。
不连接任何驱动或串口，不依赖相机 TF，不发送电机/转向执行器命令。

## 支持的速度

输入采用 ROS 车体轴约定：x 前、y 左、z 上。yaw 输入为绕 z 角速度 rad/s，不是航向角。

| 输入 | 支持 |
|---|---|
| vx | 前进/倒车，m/s |
| vy | 只能为零；非零请求明确拒绝，不静默丢弃 |
| omega_z | 可与非零 vx 同时输入，rad/s |
| vx=0 且 omega_z!=0 | 拒绝：底盘不能原地旋转 |
| linear.z / angular.x / angular.y | 必须为零 |

微小浮点容差默认 1e-6。不能独立执行 vx、vy、omega 三自由度控制。
单独转动转向轴在机械上可能可行，但零速度的 yaw_rate 不能表达静止转向角；
若后续需要静止摆轮，需另外的 steering_angle 请求接口，而不是原地旋转速度。

## 输入与启动

默认普通 Twist 输入：

```bash
cd /home/tree/farmercar_ws
source devel/setup.bash
roslaunch tricycle_controller controller.launch
```

输入 `/tricycle_controller/cmd_vel`，类型 geometry_msgs/Twist：

```bash
rostopic pub -r 10 /tricycle_controller/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.05}}'
```

输出观测：

```bash
rostopic echo /tricycle_controller/command
rostopic echo /tricycle_controller/status
rostopic echo /tricycle_controller/achievable_cmd_vel
```

停止发送输入后0.5s输出全零。Twist没有时间戳，按接收实际经过时间判定超时。

带时间戳模式：

```bash
roslaunch tricycle_controller controller.launch input_stamped:=true \
  input_topic:=/apriltag_servo/preview/cmd_vel
```

每次只启用一个输入接口，避免两个输入源互相覆盖。TwistStamped 要求非零新鲜时间戳，
header.frame_id=base_link；旧、未来、零时间戳和错误frame均拒绝。
若重复发送一个旧时间戳，它仍会失效，不因接收刷新而通过。

以上为单独启动方式；统一 camera_manager launch 已配置同名节点，不要同时重复启动。

## 输出消息与未来驱动契约

主输出 `/tricycle_controller/command` 类型 `tricycle_controller/ChassisCommand`：

| 字段 | 定义 |
|---|---|
| header | 输出计算时刻，base_link 轴方向约定 |
| source_stamp | Stamped输入原始时间；普通Twist为0 |
| speed | 有符号车体纵向速度 m/s，前进正 |
| steering_angle | 前轮实际期望角 rad，回正0、左正，±50° |
| yaw_rate | 限幅后可实现的车体偏航角速度 rad/s |
| accepted | 输入接受；false时速度、转角、偏航率全零 |
| saturated | 是否发生限速/限转角 |
| reason | ok、limited、stop 或具体拒绝/超时原因 |

另输出 geometry_msgs/TwistStamped `/tricycle_controller/achievable_cmd_vel` 和JSON状态。
20Hz定时发布，收到输入时也立即更新；不是仅在收到输入时发布。
输出header是计算时间，并不表示底盘有反馈或正在执行该速度。

speed采用单轨运动学模型的后轴中点纵向速度定义，方向沿base_link x。
这只是速度契约，不新增后轴TF或平移补偿，也不是轮胎线速度、电机RPM或控制器原始单位。
将来驱动层根据实际驱动轮/电机标定换算。前轮驱动时轮向线速度为 speed/cos(steering)，
后轴中心驱动可用speed；实际驱动位置尚未由用户确认，本包不假定电机连接位置。

输出不是转向舵机0..180°值：90°回正偏置及硬件正负只应在驱动层做转换。
不会将 steering_angle 写入转轴TF，期望命令不能当实测转角。
当前 TF 和 AprilTag轴转换保持已验证的版本。

## 构型与运动学

```text
L = 0.839 m
delta = atan(L * requested_omega / requested_vx)
achievable_omega = output_speed * tan(output_delta) / L
```

使用有符号vx，不能取abs(vx)；否则倒车转向方向错误。
例如 vx=-0.1、omega=+0.05 时前轮角为负，车体偏航仍为正。
最大前轮角±50°，最小后轴参考转弯半径约0.704m。
后轮距0.543m、轮胎直径等不参与单个前轮角换算，不增加轮胎几何模型。
无速度/转角反馈，本包仅提供运动学解，不是轮速或转向执行器闭环。

默认前进/倒车限速均0.15m/s，仅为当前调试软件限制，并非车辆最高能力。
限速先按请求曲率保留转角，再限转角，重算实际可实现omega。不会宣称执行不可达的yaw率。
例 vx=.1、omega=1 时delta限到50°，omega输出约.142rad/s。
要求非零vy时整个请求拒绝并立即零输出，停止绕过任何限幅过程。
没有添加执行器加速/转角速率模型；AprilTag跟随上游已有速度平滑，任意外部输入仍可阶跃。

参数位于 config/controller.yaml，可通过 config_file 指定。启动时读取，运行中改参数需重启。

## AprilTag 软件链路

camera_manager/system_start.launch 现在连接：

```text
AprilTag -> apriltag_servo/preview/cmd_vel (TwistStamped)
         -> velocity_adapter（差速保持/全向近似）
         -> tricycle_controller
         -> command (车速+前轮角，未来驱动输入)
         -> achievable_cmd_vel -> vehicle_controller_bridge -> 门控后的协议帧
```

新vehicle_controller_bridge支持有符号速度。底盘运动学command本身支持倒车。
command已由vehicle_controller_bridge订阅；默认不打开串口、不解锁。旧vehicle_controller_node不启动。
集成运行时必须查看上游/底盘输出状态，不对同一个输入topic再发布手动测试数据。
要测手动输入，请单独启动本包；其输出当前没有硬件消费者。

## 测试

```bash
PYTHONPATH=src python3 -m unittest discover -s test -p test_kinematics.py -v
rostest tricycle_controller controller.test
```

10项运动学测试覆盖正反向、转向符号、限速保曲率、限角、拒绝横移/原地旋转、
非法输入与参数。ROS集成测试覆盖Twist/Stamped、超时、重复旧消息、无效frame及恢复。
测试使用独立ROS master，只发布测试话题，没有底盘驱动。

上游希望提供全向vx/vy/omega时，请启动 velocity_adapter/tricycle.launch，给
/velocity_adapter/cmd_vel。该适配层将非零vy近似为行驶中的转弯请求，再进入本包。
本包独立输入的vy=0约束保留；通用上游不必直接满足这个约束。
