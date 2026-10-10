# 部署与操作说明

版本：2026-10-10。适用工作空间：`/home/tree/farmercar_ws`，ROS Noetic、Python 3、catkin。
本章命令除标明“本地”外均在远端执行。

## 1. 当前运行状态与功能边界

统一 launch 当前已运行，功能节点为：

```text
/usb_cam
/usb_cam/image_proc
/apriltag_ros_continuous_node
/vehicle_camera_tf
/apriltag_follow_preview
/velocity_adapter
/tricycle_controller
/vehicle_controller_bridge
```

`/rosout` 是 ROS 日志节点。当前跟随禁用、驱动未解锁、车辆串口关闭。
本次没有移动车辆，也没有在真实底盘上确认速度闭环或转向响应。

| 模块 | 作用 |
|---|---|
| camera_manager | 相机、去畸变、检测一键启动，最小方向 TF |
| apriltag_servo | 读取目标标签位置，生成前进跟随速度；支持 Bool 手动启用 |
| velocity_adapter | 差速速度保持；全向横移请求近似为行驶中的转向 |
| tricycle_controller | 轴距 0.839 m、前轮角 ±50°，输出车体速度和前轮角 |
| vehicle_controller_bridge | 订阅最终命令、门控、协议组帧、可选串口收发及反馈解析 |

相机和标签检测配置保留原工作空间的已验证版本。去畸变图像为 `/usb_cam/image_rect_color`。
TF 原点按重合处理，控制只做光学轴重排，无安装位置和距离补偿。

## 2. 环境与依赖检查

```bash
ssh tree@10.62.134.197
cd /home/tree/farmercar_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rospack find usb_cam
rospack find image_proc
rospack find apriltag_ros
rospack find tricycle_controller
rospack find vehicle_controller
pkg-config --modversion apriltag
python3 -c 'import rospy, tf2_ros, tf2_geometry_msgs'
command -v nosetests3
```

首次迁移到新设备时，要先准备兼容的 ROS Noetic、catkin、现有 `apriltag_ros` 源码/配置以及
原生 AprilTag、OpenCV、Eigen 等依赖，再部署增量包。现有 `apriltag_ros` CMake 使用
`pkg-config` 查找原生 AprilTag 库；仅复制本交付包不足以重建整个新设备环境。
必要时按目标设备的 ROS 环境处理 `rosdep` 依赖，不在未知设备上照搬现有 `build/` 或 `devel/`。

## 3. 增量部署到当前工作空间

### 3.1 停止原统一启动并备份

前台运行时使用 Ctrl+C。当前后台启动 PID 记录在：

```text
/tmp/farmercar_system_20261010.pid
```

在停止 master 前导出参数；确认 PID 对应统一 launch 后停止，再备份源码：

```bash
mkdir -p /home/tree/farmercar_backups
rosparam dump /home/tree/farmercar_backups/params_before_redeploy.yaml
ps -fp "$(cat /tmp/farmercar_system_20261010.pid)"
kill -INT "$(cat /tmp/farmercar_system_20261010.pid)"
tar -czf /home/tree/farmercar_backups/pre_redeploy.tar.gz \
  -C /home/tree/farmercar_ws src
```

`rosparam dump` 需要 ROS master 仍在运行，不能等 master 结束后再导出。
备份文件名应按实际部署批次调整，避免覆盖需要保留的旧备份。

### 3.2 传输与展开交付包

本地传输示例：

```bash
scp /path/to/farmercar_apriltag_20261010.tar.gz tree@10.62.134.197:/tmp/
```

远端解压到临时目录，再放入指定位置：

```bash
task_stage=$(mktemp -d /tmp/farmercar-release.XXXXXX)
tar -xzf /tmp/farmercar_apriltag_20261010.tar.gz -C "$task_stage"

for task_pkg in camera_manager apriltag_servo velocity_adapter tricycle_controller vehicle_controller; do
  cp -a "$task_stage/$task_pkg" /home/tree/farmercar_ws/src/
done

cp -a "$task_stage/docs" "$task_stage/validation" /home/tree/farmercar_ws/
cp -a "$task_stage/run_checks.sh" "$task_stage/DRIVER_HANDOVER.md" \
  /home/tree/farmercar_ws/
cp -a "$task_stage/verify_live_axes.py" "$task_stage/verify_follow_stack.py" \
  /home/tree/farmercar_ws/src/
```

不要把整个交付目录当作一个 catkin 包，也不要把工作空间根目录文档全部放进 `src/`。
该操作不会安装缺失的 `apriltag_ros` 或原生库。现有相机标定、标签大小/ID 配置应先核对，
尤其在换相机或迁移到新设备时不能无条件复用原相机内参。

### 3.3 编译与确认消息

```bash
cd /home/tree/farmercar_ws
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake -j2 -l2
source devel/setup.bash

rosmsg show tricycle_controller/ChassisCommand
rosmsg show vehicle_controller/VehicleFeedback
roslaunch --nodes camera_manager system_start.launch
```

交付包保留了文件时间戳，使用 `--force-cmake` 确保已有包的新增 Python 安装规则和消息生成
重新配置。编译后每个终端都应重新 `source devel/setup.bash`。

## 4. 一键启动与手动跟随

```bash
cd /home/tree/farmercar_ws
source devel/setup.bash
roslaunch camera_manager system_start.launch \
  target_tag_id:=0 desired_distance:=1.0 serial_enabled:=false
```

默认只运行软件与感知。跟随节点虽已启动，功能仍禁用；驱动也未解锁。
不要与旧相机 launch、旧 image_proc 或同名控制节点重复启动。

```bash
# 手动启用软件跟随
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: true'

# 关闭软件跟随
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: false'
```

启用跟随不会更改 `serial_enabled`，也不会自动设置 `/vehicle_controller/arm`。
在当前默认状态下可以观察非零软件期望，门控后的发送帧仍是停车帧且不写串口。

## 5. 主要参数及生效方式

| 参数 | 默认值 | 含义 |
|---|---|---|
| target_tag_id | 0 | 跟随标签 ID |
| desired_distance | 1.0 m | 独立停距，使用原始 optical z 判断 |
| video_device | /dev/video0 | 相机设备 |
| camera_frame | camera_optical_frame | 检测输入光学坐标名，应与 TF 约定一致 |
| assume_zero_steering | true | 无反馈时按回正显示转轴 TF |
| enable_follow_preview | true | 是否启动跟随、适配和运动学软件节点；不是手动跟随使能 |
| serial_enabled | false | 是否打开车辆串口 |
| vehicle_port | /dev/ttyACM0 | 车辆串口设备 |
| vehicle_baudrate | 115200 | 串口波特率 |
| vehicle_protocol | legacy | legacy=V1，extended=V2 |
| vehicle_config | vehicle_controller/config/bridge.yaml | 桥接和校准配置路径 |

| 配置文件 | 主要内容 |
|---|---|
| src/camera_manager/config/geometry.yaml | 最小 TF、转角话题、反馈超时 |
| src/camera_manager/config/ost.yaml | 已有相机内参 |
| src/apriltag_ros/config/tags.yaml | 现有标签 ID、尺寸与名称 |
| src/apriltag_servo/config/follow.yaml | 跟随增益、速度/加速度、死区和超时 |
| src/velocity_adapter/config/adapter.yaml | 速度方向近似增益、倒车许可及超时 |
| src/tricycle_controller/config/controller.yaml | 轴距、转角、正反向速度限制 |
| src/vehicle_controller/config/bridge.yaml | 协议标度、模式/反馈/确认门控 |

参数在节点启动时读取。运行中 `rosparam set` 不会自动重建控制器或切换串口，修改配置后重启。
`desired_distance`、`vehicle_protocol` 等 launch 参数会覆盖 YAML 的同名配置，应通过对应
launch 参数选择。上游、运动学和通信桥接的速度/转角限制应保持一致，当前均为 0.15 m/s、±50°。

前进跟随在 optical z <= desired_distance+0.05 m 时停止，不倒车。
2.9 m 是之前的观测值，不写入标定或停距配置。
全向速度经过中间包做方向近似，不能精确侧移；纯原地旋转不自动变成前进动作。

## 6. 状态与数据检查

```bash
rosnode list
rostopic echo /apriltag_servo/preview/status
rostopic echo /velocity_adapter/status
rostopic echo /tricycle_controller/command
rostopic echo /vehicle_controller/status
rostopic echo /vehicle_controller/feedback
```

| 状态/字段 | 当前默认状态或判断方法 |
|---|---|
| servo.enabled | false；手动 enable=true 后才计算跟随速度 |
| driver.serial_enabled / dry_run | false / true |
| driver.armed | false |
| driver.packet_written | false |
| driver.reason | follow_disabled；启用跟随后通常为 driver_disarmed |
| feedback.received / fresh | 无底盘连接时 false，不应将默认零值解释为实测 |
| command.accepted | 只表示运动学请求有效，不表示允许硬件执行 |
| 上游 actuator_allowed=false | 上游模块不直接操作硬件；实际许可看驱动 actuation_permitted |

`/vehicle_controller/desired_packet` 是未经过权限/反馈门控的协议预览，不是串口发送源。
`/vehicle_controller/tx_packet` 是门控后的帧；serial_enabled=false 时仍只发布 ROS 数据。

## 7. 未来真实下位机接入

本节是硬件已适配后的操作说明，本次没有执行真实车辆串口步骤。
先确认速度标度、回正角/正负、控制权和下位机独立命令看门狗。
V2 上位机已经实现，下位机必须按 [协议说明](PROTOCOL.md) 实现相应字段。

```bash
roslaunch camera_manager system_start.launch \
  serial_enabled:=true vehicle_port:=/dev/ttyACM0 vehicle_baudrate:=115200 \
  vehicle_protocol:=extended desired_distance:=1.0
```

启动后仍未解锁、跟随禁用。由用户显式操作：

```bash
rostopic pub -1 /vehicle_controller/arm std_msgs/Bool 'data: true'
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: true'
```

若尚未自动就绪，V2 先发送“请求自动+停车”，收到自动就绪无故障反馈后才允许非零运动。
命令或反馈超过 0.5 s 失效，急停/故障/手动模式会停车并锁回驱动；恢复后需重新 arm。

```bash
# 停跟随、锁回驱动
rostopic pub -1 /apriltag_servo/enable std_msgs/Bool 'data: false'
rostopic pub -1 /vehicle_controller/arm std_msgs/Bool 'data: false'

# 主机急停
rostopic pub -1 /vehicle_controller/emergency_stop std_msgs/Bool 'data: true'
```

急停置 false 只解除主机急停状态，不自动重新解锁。
V1 无模式、急停、故障字段，默认 `allow_legacy_actuation=false`。继续用 V1 时需明确确认
旧硬件条件，再通过桥接 YAML 显式配置该选项；这不会给 V1 增加不存在的状态能力。
真实转角未知时保持测量有效位为假，不将指令回显当反馈。
不要同时启动 `vehicle_core.launch`；旧 C++ 手动节点与新桥接不能并发占用同一串口。

## 8. 日志、故障定位与回退

当前后台 launch 日志：`/tmp/farmercar_system_20261010.log`。ROS 日志位于 `~/.ros/log/`。
最终验收日志位于工作空间 `validation/`，详见 [测试文档](TESTING.md)。

| 现象 | 检查方向 |
|---|---|
| 相机打不开 | 设备编号、权限、是否被旧节点占用 |
| 没有目标 | 标签 ID/尺寸、图像清晰度、去畸变话题和检测消息 |
| 有观测但候选速度为零 | 跟随是否启用、是否达到停距、检测是否过期或坐标非法 |
| 有候选但 tx_packet 为停车帧 | 是否未解锁、反馈失效、下位机未就绪，查看 driver.reason |
| legacy_feedback_requires_explicit_opt_in | 旧帧无法提供自动状态，按旧协议接入说明确认条件 |
| serial_io_fault | 串口断开或写失败；故障保持，修复后重启桥接 |
| 模式/故障恢复后仍停 | 驱动已锁回，需先确认条件恢复再重新 arm |
| 新消息或 Python 模块找不到 | 强制重新配置编译，重新 source 工作空间 |

既有备份位于 `/home/tree/farmercar_backups/`，包括首次集成、最小 TF、底盘控制、
速度适配、驱动集成前的备份。回退前先停止新 launch，恢复所选备份覆盖的源码文件，
再 `catkin_make --force-cmake`。新增包不属于旧备份时须单独移出 `src/`，不能假定解压会删除它们。
回退后从跟随禁用、串口关闭的状态重新检查。文档整理没有执行回退或修改运行代码。
