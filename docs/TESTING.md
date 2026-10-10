# 测试与验收说明

版本：2026-10-10。本文整理已完成的最终验收记录，并给出复验方法。
本次整理文档没有重跑测试或切换现场使能。

## 1. 验收结论与边界

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

## 2. 35 项算法/协议测试

| 测试文件 | 数量 | 核心覆盖 |
|---|---:|---|
| apriltag_servo/test/test_control.py | 8 | 前进控制、转向符号、限角、速度变化限制、停距、非法输入/参数 |
| tricycle_controller/test/test_kinematics.py | 10 | 正反向运动学、速度/转角限幅、保曲率、横移/原地旋转拒绝、数值边界 |
| velocity_adapter/test/test_mapping.py | 10 | 差速保持、斜向/纯横向近似、倒车朝向、增益/前馈、纯旋转拒绝 |
| vehicle_controller/test/test_wire.py | 7 | 已知例帧、V2 标志/序号、反馈语义、标度、碎片/坏帧、虚拟串口往返 |

`test_protocol_preview.py` 是此前旧协议预览器的测试，不计入当前 35 项主链路统计。

## 3. 7 项 ROS 集成测试与最终证据

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

## 4. 软件复验步骤

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

## 5. TF 与真实检测检查

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

## 6. 真实相机完整软件链验收

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

## 7. 可人工确认的跟随行为

保持车辆不动、串口关闭，可以移动标签检查：

1. 软件跟随禁用时，位置仍发布，速度为零。
2. 启用后，远于停距+死区产生前进候选；标签在左侧时正转角，在右侧时负转角。
3. 原始 optical z <= desired_distance+0.05 m 时立即停车，不倒车。
4. 遮挡目标或换成错误 ID 时速度、yaw 为零。
5. 停止输入超过 0.5 s 后保持零输出，恢复有效观测可重新跟随。
6. enable=false 后保持零输出；无需解锁驱动完成这些软件检查。

不要把人工测试值发布到正在运行的正式转角反馈话题来冒充实测；转轴模拟已在隔离 ROS 测试中完成。

## 8. 失败定位与验收证据清单

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
