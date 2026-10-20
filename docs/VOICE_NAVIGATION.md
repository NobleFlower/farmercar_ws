# 语音指令与 AprilTag 导航联调

## 1. 启动链路

统一入口：

```bash
cd /home/tree/farmercar_ws
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake -j2 -l2
source devel/setup.bash

roslaunch apriltag_navigation voice_system.launch \
  video_device:=/dev/video2 \
  voice_port:=/dev/ttyCH341USB0 \
  serial_enabled:=false \
  vehicle_port:=/dev/ttyACM0
```

数据链路：

```text
CH340语音模块 -> voice_apriltag_navigation -> /tricycle_controller/cmd_vel
                         ^                         |
                         |                         v
                    /tag_detections       tricycle_controller
                                                  |
                                                  v
                                    vehicle_controller_bridge
                                                  |
                                                  v
                                              /dev/ttyACM0
```

`serial_enabled=false` 只验证 ROS 指令和协议预览帧，不打开底盘串口。确认感知、方向和停车正常后，
才使用 `serial_enabled:=true`。桥接节点启动时仍为未解锁状态，必须显式解锁：

```bash
rostopic pub -1 /vehicle_controller/arm std_msgs/Bool 'data: true'
```

急停或结束联调：

```bash
rostopic pub -1 /vehicle_controller/emergency_stop std_msgs/Bool 'data: true'
rostopic pub -1 /vehicle_controller/arm std_msgs/Bool 'data: false'
```

新的统一语音入口不要和旧的 `vehicle_core.launch` 同时运行，后者也直接占用
`/dev/ttyCH341USB0` 和 `/dev/ttyACM0`。

## 2. 行为

| 输入 | 行为 |
|---|---|
| 前进 `AB AB AB 01` | 低速模式跟踪当前垄道最近可见标签，前进并转向对准标签中心 |
| 后退 `AB AB AB 10` | 低速模式下倒车，并利用当前垄道标签修正方向 |
| 停车 `AB AB AB 00` | 立即生成零速度、零转向指令，并关闭桥接跟随使能 |
| 低速模式 `DE DE DE 00` | 停止当前巡航/跨垄动作，回到低速模式等待前进或后退 |
| 跨垄 `AD AD AD 01/02` | 前往第 1/2 垄道 |
| 智能巡航 `DE DE DE 02` | 蛇形循环：驶向当前垄末端、左转、穿到下一垄、继续直行 |

第一垄跨到第二垄：

1. 沿第一垄前进，到 `road14` 前 1.0 m。
2. 以 45° 左转、1.0 m/s 行驶，直到检测到 `road23`。
3. 对准 `road23` 前进，到其前 0.5 m 停车。

第二垄跨到第一垄：

1. 沿第二垄前进，到 `road21` 前 1.0 m。
2. 以 45° 左转、1.0 m/s 行驶，直到检测到 `road12`。
3. 对准 `road12` 前进，到其前 0.5 m 停车。

巡航模式在每次 0.5 m 停车点后自动选择下一垄继续。停车帧、低速模式帧、标签超时、
目标丢失、跨垄转弯超过 10 s 都会停止运动。

## 3. 调参位置

主要参数在
`src/apriltag_navigation/config/voice_navigation.yaml`：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `cross_turn_speed_mps` | 1.0 | 跨垄左转线速度 m/s |
| `cross_turn_steering_deg` | 45.0 | 跨垄固定左转角 |
| `terminal_approach_distance_m` | 1.0 | 垄末端标签前开始左转的距离 |
| `cross_target_stop_distance_m` | 0.5 | 新垄目标标签前停车距离 |
| `low_speed_forward_mps` | 0.3 | 低速前进速度 |
| `low_speed_reverse_mps` | 0.3 | 低速后退速度 |
| `cruise_speed_mps` | 0.3 | 巡航直线段速度 |
| `approach_speed_mps` | 0.3 | 看到跨垄目标后的接近速度 |
| `tag_timeout_s` | 0.5 | 标签观测失联停车时间 |
| `turn_timeout_s` | 10.0 | 跨垄左转最长持续时间 |

统一语音 launch 把运动学和桥接的运行上限都设为 `max_speed:=3.0`；通用配置文件本身仍保留
0.15 m/s 的默认值，避免旧入口继承高速上限。如果以后把跨垄速度调高到 3.0 m/s 以上，需要在
`voice_system.launch` 启动命令中同步提高 `max_speed`，否则会饱和到 3.0 m/s。

## 4. 标签配置要求

`AprilTagDetection` 消息只携带数字 ID，不携带 `tags.yaml` 中的名字。新节点启动时读取
`src/apriltag_ros/config/tags.yaml`，建立“数字 ID -> road 名字”映射，名字中 `road`
后的第一个数字表示垄道。

当前仓库的该文件只配置了：

```yaml
road11 = 0
road12 = 1
road21 = 3
```

跨垄流程还要求 `road14`、`road23`，反向流程还要求 `road12`、`road21`，巡航还依赖
各垄沿途标签。必须把现场实际标签 ID 和尺寸加入 `standalone_tags`，不能仅凭名字猜 ID。
缺少某个流程所需标签时，节点会在
`/voice_apriltag_navigation/status` 的 `missing_cross_lane_tags` 中报告，并停车等待，
不会盲目前进。

示例格式如下，其中的 `REAL_ID` 必须替换成现场标签实际 ID：

```yaml
{id: REAL_ID, size: 0.154, name: 'road14'},
{id: REAL_ID, size: 0.154, name: 'road23'}
```

## 5. 查看状态

```bash
rostopic echo /voice_apriltag_navigation/status
rostopic echo /tricycle_controller/command
rostopic echo /vehicle_controller/status
rostopic echo /vehicle_controller/tx_packet
```

`/voice_apriltag_navigation/status` 会显示当前 `mode/phase/lane/target_tag/reason`、
期望速度和转角、串口读取错误计数以及缺失的跨垄标签配置。
