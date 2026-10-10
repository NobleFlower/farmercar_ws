# vehicle_controller 通信接口与上下位机协议（2026-10-10）

本文区分当前已有兼容帧和新实现的可选扩展帧。上位机两种帧的组包、解析、ROS适配已实现；
本次没有修改下位机固件，也没有验证实际硬件的速度标度/转向方向。
扩展帧需下位机按本文实现后才能使用，不能当作当前固件已经支持的协议。

## 1. ROS 数据契约

上层最终输入 `/tricycle_controller/command`，类型 `tricycle_controller/ChassisCommand`：

| 字段 | 含义 |
|---|---|
| header.stamp / frame_id | 上层输出时间 / base_link轴方向约定 |
| speed | 车体纵向速度 m/s，前进正、倒车负 |
| steering_angle | 实际期望前轮角 rad，回正0、左正，±50° |
| yaw_rate | 构型限幅后的偏航角速度 rad/s，仅用于一致性校验，不重复发送 |
| accepted / saturated / reason | 请求接受、限幅标记、原因 |
| source_stamp | 上一级速度输入时间；用于追踪，不是下位机反馈时间 |

驱动桥接检查时间戳、frame、accepted、有限值、速度/角度限值和
`yaw_rate=speed*tan(steering_angle)/0.839`；不合格就输出停车帧。

速度是单轨模型的车体纵向速度契约，不是电机RPM、油门或任一车轮转速。
下位机/真实驱动需按驱动位置与标定换算。前轮驱动的轮向线速度为v/cos(delta)，
后轴中心速度可用v；当前没有确认实际电机驱动位置，不能将未知轮速直接标为车体速度。

反馈输出 `/vehicle_controller/feedback`，类型 `vehicle_controller/VehicleFeedback`：

| 字段 | 来源/语义 |
|---|---|
| header.stamp | 上位机收到完整有效帧的时间；不会用当前时间刷新旧反馈 |
| received / fresh | 是否收过有效帧 / 接收年龄是否<=feedback_timeout |
| simulated | 是否来自无串口模式的诊断注入 |
| protocol_version | 1兼容帧，2扩展帧 |
| speed | 由协议值标定到m/s；必须结合speed_is_measured判断是否实测 |
| steering_angle | 前轮角rad、左正；必须结合steering_is_measured判断是否实测 |
| speed_is_measured / steering_is_measured | 该字段是否为有效测量，而不是设置值/命令回显 |
| status_authoritative | 是否有扩展状态字段，可否判定硬件模式/急停/故障 |
| controller_ready / emergency_stop | 下位机就绪 / 急停状态（扩展） |
| control_mode | 0禁用，1手动，2自动（扩展） |
| fault_flags | 故障位图uint16（扩展） |
| ack_valid / ack_sequence | 是否包含命令确认 / 最后确认序号（扩展） |

兼容EF01当前文档只确认“低速设置值+转角”，因此默认两个is_measured均false，
status_authoritative=false。不伪造实测车速、转角、odom或故障状态。
只有确认旧固件某字段确为实测后才可设置legacy_*_is_measured=true。

真实串口反馈中的转角被标为有效实测、且未过期时，发布
`/vehicle/steering_joint_states`（JointState，steering_joint，rad），供最小TF更新。
默认不将模拟注入值发布到正式关节话题。速度反馈仍可用于后续速度执行器闭环，
当前AprilTag视觉跟随直接用检测位置，不依赖里程计。

## 2. 通用帧规则

```text
HEADER | TYPE | LEN | PAYLOAD[LEN] | CHECKSUM
```

- HEADER：上位机发送0xEE，下位机反馈0xEF。
- TYPE和LEN均uint8，LEN只计payload字节。
- 多字节字段全部小端。速度int16使用二进制补码。
- CHECKSUM = sum(前面全部字节) & 0xFF，包含HEADER/TYPE/LEN。
- 这是累加校验，不是多项式CRC；原代码computeCrc的名字沿用但运算是累加。
- 无额外结束符。解析器支持半帧、多帧、垃圾前缀和校验失败后的重新同步。
- 串口115200 baud、8数据位、无奇偶、1停止位、无流控。
- 建议命令及反馈均20Hz；上位机默认命令/反馈过期时间均0.5s。
- 下位机必须独立实现命令看门狗：超过0.5s没有新的有效命令则驱动速度为0。
  上位机退出或线缆断开时，不能依赖上位机仍能发送停止帧。

标定转换默认：

```text
raw_speed = round(speed_mps * protocol_speed_units_per_mps * 100)
raw_angle = round((steering_neutral_deg + steering_sign * delta_deg) * 10)
```

参数默认speed_units_per_mps=1、neutral=90°、sign=+1。速度比例及转向正负尚需硬件确认。
回正raw_angle=900；当前±50°对应raw_angle=400..1400。
速度接收换算为raw_speed/(100*speed_units_per_mps)。
转角接收为(raw_angle/10-neutral)/sign，再转为rad。

## 3. 当前兼容协议 V1（已存在）

上位机发送8字节：

```text
EE 01 04 speed_L speed_H angle_L angle_H checksum
```

下位机反馈8字节：

```text
EF 01 04 speed_L speed_H angle_L angle_H checksum
```

| 字节位置 | 类型/字段 | 含义 |
|---|---|---|
| 0 | uint8 header | EE发送、EF反馈 |
| 1 | uint8 type | 01 |
| 2 | uint8 len | 04 |
| 3..4 | int16 speed，小端 | 有符号速度值，x100定点 |
| 5..6 | uint16 angle，小端 | 协议转角，0.1°单位，默认900回正 |
| 7 | uint8 checksum | 前7字节累加低8位 |

例（默认标定）：

```text
停车、回正：          EE 01 04 00 00 84 03 7A
前进0.10m/s、左10°： EE 01 04 0A 00 E8 03 E8
```

V1没有使能、急停、硬件模式、故障和序号字段；停车用speed=0表达。
旧README对速度曾出现“无符号”的描述，与实际C++解析int16不一致，本实现统一按int16。
因为V1不能证明下位机处于自动模式/无故障，默认allow_legacy_actuation=false。
若旧硬件必须继续用V1，应明确确认模式切换及本地急停/失联停车后再在bridge配置中
显式开启allow_legacy_actuation；这不增加V1本身没有的状态字段。

## 4. 可选扩展发送 V2（上位机已实现，下位机待适配）

共11字节，payload=7：

```text
EE 02 07 speed_i16 angle_u16 control_flags_u8 sequence_u16 checksum
```

| 字节位置 | 字段 | 类型/定义 |
|---|---|---|
| 0..2 | EE 02 07 | 帧头、类型、payload长度 |
| 3..4 | target_speed | int16，小端，标度同V1 |
| 5..6 | target_angle | uint16，小端，标度同V1 |
| 7 | control_flags | uint8，见下表 |
| 8..9 | sequence | uint16，小端，每周期递增，65535后回0 |
| 10 | checksum | 前10字节累加低8位 |

control_flags：

| 位 | 名称 | 行为 |
|---|---|---|
| bit0 | drive_enable | 1允许按目标执行，0禁止运动 |
| bit1 | stop_requested | 1要求停车，优先于速度字段和bit0 |
| bit2 | auto_mode_requested | 1请求自动模式；不能覆盖下位机手动优先、急停或故障 |
| bit3..7 | reserved | 必须0 |

下位机应在允许自动接管时响应bit2，将control_mode设为2；在准备阶段先停止并回报ready。
这样上位机可以先发送“请求自动+停车”，收到自动就绪反馈后再发送允许运动。
不能要求先收到非零运动才能进入自动模式，否则双方会等待。
急停或故障时，下位机必须立即停车，不接受自动请求强行清除故障。

```text
停车/禁止运动，seq=0：      EE 02 07 00 00 84 03 02 00 00 80
请求自动但仍停车，seq=0：  EE 02 07 00 00 84 03 06 00 00 84
允许前进0.10、左10°，seq=1：EE 02 07 0A 00 E8 03 05 01 00 F2
```

## 5. 可选扩展反馈 V2

共14字节，payload=10：

```text
EF 02 0A actual_speed_i16 actual_angle_u16 mode_u8 status_flags_u8 fault_u16 ack_sequence_u16 checksum
```

| 字节位置 | 字段 | 类型/定义 |
|---|---|---|
| 0..2 | EF 02 0A | 帧头、类型、payload长度 |
| 3..4 | actual_speed | int16，小端，实际车体纵向速度x100；未实测则清测量有效位 |
| 5..6 | actual_angle | uint16，小端，实际前轮协议角0.1°；未实测则清有效位 |
| 7 | control_mode | uint8：0禁用、1手动、2自动 |
| 8 | status_flags | uint8，见下表 |
| 9..10 | fault_flags | uint16小端，0无故障，非零阻止上位机运动输出 |
| 11..12 | ack_sequence | uint16小端，最后收到且校验/字段合法的V2命令序号 |
| 13 | checksum | 前13字节累加低8位 |

status_flags：

| 位 | 名称 | 含义 |
|---|---|---|
| bit0 | controller_ready | 控制器可接收自动运动指令 |
| bit1 | emergency_stop | 下位机急停有效，必须停车 |
| bit2 | speed_measurement_valid | speed是当前有效实测车体速度 |
| bit3 | steering_measurement_valid | angle是当前有效实测前轮角 |
| bit4..7 | reserved | 必须0 |

没有编码器/转角传感器时不能将设置值或命令回显标为测量有效。
反馈是心跳并不证明传感器值是新的；下位机需在测量过期时清有效位并上报适当故障。
不需要所有字段均已实测才可做视觉伺服，但就绪、模式、急停和故障必须是真实硬件状态。

fault_flags建议统一位定义：

| 位 | 故障 |
|---|---|
| bit0 | 驱动电机/电动车控制器故障 |
| bit1 | 转向执行器故障 |
| bit2 | 下位机命令看门狗超时 |
| bit3 | 必需反馈传感器故障或无效 |
| bit4 | 控制器内部/通信故障 |
| bit5..15 | 预留扩展；上位机对任何非零故障都停车 |

例：实测0.08m/s、左10°、自动模式、就绪+两测量有效、无急停无故障、ack=1：

```text
EF 02 0A 08 00 E8 03 02 0D 00 00 01 00 FE
```

默认解析并发布ack；可配置require_ack=true检查确认滞后，max_ack_lag默认10帧。

## 6. 运行与门控

```bash
roslaunch camera_manager system_start.launch desired_distance:=1.0
```

默认serial_enabled=false，无串口打开；跟随默认禁用、驱动armed=false。

```bash
# 手动启用/停止伺服（只改变软件指令，当前不会移动车辆）
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: true'
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: false'
```

未来硬件已按协议适配且已确认标度后，启动参数serial_enabled=true、vehicle_port、
vehicle_baudrate、vehicle_protocol=extended用于选择真实串口。还需显式解锁驱动：

```bash
rostopic pub -1 /vehicle_controller/arm std_msgs/Bool 'data: true'
rostopic pub -1 /vehicle_controller/arm std_msgs/Bool 'data: false'
rostopic pub -1 /vehicle_controller/emergency_stop std_msgs/Bool 'data: true'
```

允许非零输出必须同时满足：跟随启用、驱动解锁、命令有效新鲜、反馈新鲜，
扩展协议还要求下位机自动且就绪、无急停/故障；可选再要求ack及时。
下位机手动模式、急停、故障会锁回armed=false；条件恢复不会自动解锁。
用户需先解除实际故障，再重新arm。主机急停置false只解除主机急停状态，不自动解锁。

观察话题：

| Topic | 用途 |
|---|---|
| /vehicle_controller/desired_packet | 上层有效期望的协议预览，未经过使能/反馈门控；不发送 |
| /vehicle_controller/tx_packet | 门控后的帧；serial_enabled时尝试写串口，否则只发布 |
| /vehicle_controller/feedback | 校验后的反馈及新鲜/测量属性 |
| /vehicle_controller/status | dry_run、armed、follow_enabled、packet_written、拒绝原因等 |
| /vehicle_controller/rx_inject | 仅在serial_enabled=false时存在的诊断字节注入 |

新桥接node为vehicle_controller_bridge；旧vehicle_core.launch/vehicle_controller_node
只保留旧手动路径，不在新统一launch中启动。不能两个进程同时占有同一串口。
真实串口启动时若检测到旧node已运行，新桥接拒绝启动；还有串口文件锁防重复桥接。
新传输使用非阻塞读取和有限写超时，数值115200正确映射到B115200，
不会继承旧select无限阻塞导致ROS回调/停车看门狗停滞的问题。

## 7. 已验证范围

已通过协议组包/解析、碎片与坏帧重同步、标度、门控、反馈过期、急停、故障/手动锁回测试。
真实串口代码分支用虚拟PTY下位机验证了收发和反馈断流停车；没有打开实际车辆串口。
软件协议正确不证明实际下位机固件已经支持V2，也不证明电机与转向标度已实测标定。
