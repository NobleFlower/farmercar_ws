# 农业三轮车 AprilTag 伺服：部署、测试与协议完整手册

版本：2026-10-10。远端工作空间：`/home/tree/farmercar_ws`。

本文件合并正式的部署、测试和协议说明。按需查阅也可使用 [文档目录](README.md)。
默认运行状态为跟随禁用、驱动未解锁、车辆串口关闭。软件与虚拟串口验收通过，真实下位机
V2 固件支持、执行器标度和车辆运动闭环仍需硬件联调。本次文档整理未改变运行代码和使能状态。


## 一、部署与操作

版本：2026-10-10。适用工作空间：`/home/tree/farmercar_ws`，ROS Noetic、Python 3、catkin。
本章命令除标明“本地”外均在远端执行。

### 1. 当前运行状态与功能边界

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

### 2. 环境与依赖检查

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

### 3. 增量部署到当前工作空间

#### 3.1 停止原统一启动并备份

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

#### 3.2 传输与展开交付包

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

#### 3.3 编译与确认消息

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

### 4. 一键启动与手动跟随

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

### 5. 主要参数及生效方式

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

### 6. 状态与数据检查

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

### 7. 未来真实下位机接入

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

### 8. 日志、故障定位与回退

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

## 二、测试与验收

版本：2026-10-10。本文整理已完成的最终验收记录，并给出复验方法。
本次整理文档没有重跑测试或切换现场使能。

### 1. 验收结论与边界

远端编译通过；35 项算法/协议单元测试及 7 项 ROS 集成测试通过。
真实相机数据链路和同时间戳轴转换通过。串口代码的实际分支已对虚拟 PTY 下位机验证。
测试没有打开实际车辆串口，也没有验证实际车辆的运动闭环、转向响应或 V2 固件支持。

| 验证类别 | 范围 | 结果 |
|---|---|---|
| 编译 | 新消息、Python 包及旧 C++ 包共存 | 通过 |
| 算法/协议 | 8+10+10+7 项 | 35 项通过 |
| ROS 集成 | 7 个测试场景 | 全部 RESULT: SUCCESS |
| 真实相机 | 感知到驱动桥接的完整软件话题链 | passed=true |
| 轴重排 | 5 个相同检测时间戳样本 | max_error=0 |
| 串口代码 | 非阻塞收发、确认序号、反馈断流停车，虚拟 PTY | 通过 |
| 实际底盘 | 物理标度、真实执行器及车辆轨迹 | 尚未实车验收 |

### 2. 35 项算法/协议测试

| 测试文件 | 数量 | 核心覆盖 |
|---|---:|---|
| apriltag_servo/test/test_control.py | 8 | 前进控制、转向符号、限角、速度变化限制、停距、非法输入/参数 |
| tricycle_controller/test/test_kinematics.py | 10 | 正反向运动学、速度/转角限幅、保曲率、横移/原地旋转拒绝、数值边界 |
| velocity_adapter/test/test_mapping.py | 10 | 差速保持、斜向/纯横向近似、倒车朝向、增益/前馈、纯旋转拒绝 |
| vehicle_controller/test/test_wire.py | 7 | 已知例帧、V2 标志/序号、反馈语义、标度、碎片/坏帧、虚拟串口往返 |

`test_protocol_preview.py` 是此前旧协议预览器的测试，不计入当前 35 项主链路统计。

### 3. 7 项 ROS 集成测试与最终证据

下表每个日志都来自最终版本，通过标志为 `* RESULT: SUCCESS`。
旧开发阶段日志另有保存，不应将它们作为最终版本的验收依据。

| 命令 | 验证内容 | 最终日志 |
|---|---|---|
| rostest apriltag_servo faults.test | 空帧、过期/未来检测、错误坐标系、非法位姿、近距离、输入超时及恢复 | [日志](../validation/farmercar_final_apriltag_servo.faults.test.log) |
| rostest tricycle_controller controller.test | Twist/Stamped 输入、正反向、限幅、非法请求、旧时间戳及恢复 | [日志](../validation/farmercar_final_tricycle_controller.controller.test.log) |
| rostest velocity_adapter adapter.test | 差速/全向输入到最终三轮车命令、近似标记、限角、超时和恢复 | [日志](../validation/farmercar_final_velocity_adapter.adapter.test.log) |
| rostest camera_manager geometry_preview.test | 0°/+20°/−20° 最小 TF 朝向、反馈过期和旧协议预览看门狗 | [日志](../validation/farmercar_final_camera_manager.geometry_preview.test.log) |
| rostest vehicle_controller bridge.test | 解锁、跟随开关、反馈过期、急停、故障/手动锁回、旧反馈语义 | [日志](../validation/farmercar_final_vehicle_controller.bridge.test.log) |
| rostest vehicle_controller serial_transport.test | 真正串口代码分支对虚拟 PTY 收发、ACK、反馈断流和停止 | [日志](../validation/farmercar_final_vehicle_controller.serial_transport.test.log) |
| rostest camera_manager control_pipeline.test | 手动启动、左右/居中、停距、错 ID、丢失、异常输入、进程退出、最终协议停车 | [日志](../validation/farmercar_final_camera_manager.control_pipeline.test.log) |

这些 `rostest` 使用独立测试 master。进程退出测试只终止隔离 master 中的测试节点。
`serial_transport.test` 在代码中创建 `/dev/pts/` 虚拟终端，并断言路径前缀，不使用车辆设备。
`geometry_preview.test` 保留了旧预览器作为独立测试对象，现场默认链路使用新的通信桥接。

### 4. 软件复验步骤

```bash
cd /home/tree/farmercar_ws
source devel/setup.bash
bash run_checks.sh
```

脚本先执行 35 项算法/协议测试，再执行上述 7 项 ROS 场景；任一步失败返回非零状态。
单元测试结果输出到终端。ROS 日志写入：

```text
/tmp/farmercar_final_<package>.<testfile>.log
```

若需要保存完整复验输出：

```bash
bash run_checks.sh > /tmp/farmercar_retest_output.log 2>&1
echo $?
```

退出码应为 0；日志应显示单元测试 `OK`、每项 ROS `RESULT: SUCCESS`。
复验会更新 `/tmp/` 测试日志，不会自动覆盖文档附带的历史验收证据。

### 5. TF 与真实检测检查

启动统一软件链后，分别检查：

```bash
rosrun tf tf_echo base_link steering_link
rosrun tf tf_echo steering_link camera_link
rosrun tf tf_echo camera_link camera_optical_frame
rosrun tf tf_echo camera_link apriltag_link
```

| TF | 预期 |
|---|---|
| base_link→steering_link | 当前无测量时按回正，平移为零 |
| steering_link→camera_link | 平移零、旋转零 |
| camera_link→camera_optical_frame | 平移零，RPY约 −90°/0°/−90° |
| camera_link→apriltag_link | 原始 z 体现在前向 x，原始右侧 x 体现在负 y |

RViz 中 Fixed Frame 设为 `base_link`，添加 TF；红=X、绿=Y、蓝=Z。
回正时相机光学蓝色 Z 应与底盘红色 X 同向；光学 X 指向车体右侧，光学 Y 指向下方。
标签被遮挡后 TF 缓存仍可保留旧位姿，判断有效性要结合时间戳和检测状态。

严格按同一时间戳检查轴重排：

```bash
python3 src/verify_live_axes.py
```

该脚本只读订阅原始检测和转换位姿，配对至少 5 个样本。
检查 `(x,y,z)_converted=(z,-x,-y)_optical`，正确输出 `passed=true`、`max_error=0`。
既有结果见 [同时间戳轴转换结果](../validation/farmercar_final_live_axes_20261010.json)。

### 6. 真实相机完整软件链验收

前提：统一 launch 已启动、目标 ID 可见、串口关闭、驱动未解锁。

```bash
python3 src/verify_follow_stack.py
```

这不是纯只读脚本：它短暂切换跟随软件使能，并在结束/异常退出时恢复禁用。
它会先确认 `serial_enabled=false`、驱动未解锁，不执行 arm、不注入反馈、不直接发布底盘指令。
不要在真实串口已启用或车辆正在执行任务时运行。

检查内容：

- 默认禁用时最终速度为零。
- 手动启用后，对可见远目标生成有效候选；近目标则允许停车。
- 最终速度有限且 <=0.15 m/s，前轮角 <=±50°，yaw 满足单轨运动学关系。
- 检测→伺服→适配→运动学→通信桥接的话题订阅连接完整。
- 驱动不写车辆串口，门控后的速度为零。
- 完成后恢复跟随禁用。

既有结果见 [真实相机完整链结果](../validation/farmercar_live_follow_acceptance_20261010.json)：

| 字段 | 既有验收值 |
|---|---|
| passed | true |
| matching_tag_frames | 9 |
| command_samples | 20 |
| last_tag_optical_z | 约 2.8922 m |
| last_speed | 约 0.06178 m/s，软件期望，不是实测车速 |
| last_steering_deg | 约 +8.2738°，软件期望，不是实测前轮角 |
| serial_enabled / driver_armed | false / false |
| physical_packet_written | false |
| restored_follow_disabled | true |

以上位置数值是验收时的一次观测，不是后续测试必须达到的固定值，尤其不能将其作为距离补偿常数。

### 7. 可人工确认的跟随行为

保持车辆不动、串口关闭，可以移动标签检查：

1. 软件跟随禁用时，位置仍发布，速度为零。
2. 启用后，远于停距+死区产生前进候选；标签在左侧时正转角，在右侧时负转角。
3. 原始 optical z <= desired_distance+0.05 m 时立即停车，不倒车。
4. 遮挡目标或换成错误 ID 时速度、yaw 为零。
5. 停止输入超过 0.5 s 后保持零输出，恢复有效观测可重新跟随。
6. enable=false 后保持零输出；无需解锁驱动完成这些软件检查。

不要把人工测试值发布到正在运行的正式转角反馈话题来冒充实测；转轴模拟已在隔离 ROS 测试中完成。

### 8. 失败定位与验收证据清单

| 失败位置 | 优先查看 |
|---|---|
| 算法测试 | 参数范围、单位、Python source 环境 |
| ROS 消息导入 | 强制重新配置编译及 `source devel/setup.bash` |
| 真实检测不出现 | 相机设备、标签 ID/尺寸、去畸变图像、光照 |
| 最终命令为零 | servo.reason、适配状态、是否达到停距 |
| TX 仍停车但候选非零 | driver.reason，未解锁或反馈未就绪是默认行为 |
| PTY 测试失败 | 测试日志、虚拟设备创建、ACK/反馈状态，不改动实际车辆串口 |

最终证据包括 [编译日志](../validation/farmercar_driver_build_20261010.log)、上表 7 份 ROS 日志、
两份真实相机 JSON 和 [验收摘要](../validation/RESULTS.md)。
文档整理时只读状态见 [现场状态快照](evidence/running_state.json)，可与历史验收结果分开理解。

## 三、上下位机协议

本文区分当前已有兼容帧和新实现的可选扩展帧。上位机两种帧的组包、解析、ROS适配已实现；
本次没有修改下位机固件，也没有验证实际硬件的速度标度/转向方向。
扩展帧需下位机按本文实现后才能使用，不能当作当前固件已经支持的协议。

### 1. ROS 数据契约

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

### 2. 通用帧规则

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

### 3. 当前兼容协议 V1（已存在）

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

### 4. 可选扩展发送 V2（上位机已实现，下位机待适配）

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

### 5. 可选扩展反馈 V2

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

### 6. 运行与门控

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

### 7. 已验证范围

已通过协议组包/解析、碎片与坏帧重同步、标度、门控、反馈过期、急停、故障/手动锁回测试。
真实串口代码分支用虚拟PTY下位机验证了收发和反馈断流停车；没有打开实际车辆串口。
软件协议正确不证明实际下位机固件已经支持V2，也不证明电机与转向标度已实测标定。
