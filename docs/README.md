# 农业三轮车 AprilTag 伺服跟随文档

版本：2026-10-10。目标工作空间：`tree@10.62.134.197:/home/tree/farmercar_ws`。

本套文档描述已完成的相机感知、最小 TF、视觉伺服、通用速度适配、三轮车运动学及
`vehicle_controller` 双向通信桥接。默认运行不打开车辆串口、不解锁驱动、不自动启用跟随。

## 文档入口

| 文档 | 使用对象与内容 |
|---|---|
| [完整手册](MANUAL.md) | 部署、测试、协议的合并版，适合一次性阅读或打印 |
| [部署与操作](DEPLOYMENT.md) | 环境与依赖、增量部署、编译、一键启动、手动跟随、参数、状态、停止与回退 |
| [测试与验收](TESTING.md) | 35 项算法/协议测试、7 项 ROS 测试、真实相机验收、复验步骤及最终日志 |
| [上下位机协议](PROTOCOL.md) | ROS 字段、V1/V2 字节协议、单位、小端、校验、位图、例帧和下位机行为 |

协议正文与代码包内 `vehicle_controller/PROTOCOL.md` 内容一致；远端对应文件为
`src/vehicle_controller/PROTOCOL.md`。扩展 V2 的上位机实现已完成，下位机固件本次未修改。

## 当前系统

```text
usb_cam → image_proc → apriltag_ros → apriltag_servo
 → velocity_adapter → tricycle_controller → vehicle_controller_bridge
 → 指令校验/权限与超时门控 → 协议帧 → 可选车辆串口

底盘反馈 → 帧校验/解析 → VehicleFeedback
                     → 有效实测转角 JointState → 最小 TF
```

最小 TF 为 `base_link → steering_link → camera_link → camera_optical_frame → apriltag_link`。
各原点按重合处理，只体现方向；控制使用 `x=z_optical、y=-x_optical、z=-y_optical`，
不叠加相机安装位置、离地高度、轴距平移或转角补偿。

默认目标 ID=0、停距参数 `desired_distance=1.0 m`、最大跟随速度 `0.15 m/s`。
停距判断直接使用 AprilTag 原始 optical z。此前约 2.9 m 是观测值，不是标定常数或停距目标。
当前视觉伺服采用前进追踪，近于停距停止等待，不倒车追离。

## 已验证与尚待硬件验证

| 范围 | 结果 |
|---|---|
| 远端 catkin 编译 | 通过 |
| 算法/协议单元测试 | 35 项通过 |
| ROS 集成测试 | 7 项通过 |
| 真实相机链路 | 验收通过；记录 9 条目标观测、20 条最终期望指令 |
| 同时间戳轴转换 | 5 条观测通过，误差为 0 |
| 串口实际代码分支 | 已使用虚拟 PTY 下位机验证，未打开车辆串口 |
| 实际下位机 V2 支持、速度/转向标度、车辆运动闭环 | 未实车验证，需硬件最后联调 |

本次文档整理只核对已有验收记录、代码和现场运行参数，没有重新执行测试或改变使能状态。
核对时现场统一 launch 正在运行，跟随禁用、驱动未解锁、串口关闭。
状态快照见 [运行状态](evidence/running_state.json)，代码指纹见 [源码 SHA256](evidence/source_sha256.json)。

## 交付结构

```text
docs/                   本套正式文档及只读状态/源码证据
validation/             最终编译、ROS 验收日志和真实相机结果
camera_manager/         一键启动、相机与最小 TF
apriltag_servo/          AprilTag 前进跟随和手动使能
velocity_adapter/       差速/全向速度适配
tricycle_controller/    三轮车运动学及最终期望消息
vehicle_controller/     双向串口桥接、协议与反馈消息
run_checks.sh           隔离软件测试入口
verify_live_axes.py     真实观测轴重排检查
verify_follow_stack.py  真实相机链路检查，要求串口关闭且驱动未解锁
```

源码交付包是面向当前工作空间的增量包，不包含现有 `apriltag_ros`、原生 AprilTag 库、
完整 ROS 安装或下位机固件。远端源码部署到 `src/`，文档及验收记录部署到工作空间根目录。

既有 `DRIVER_HANDOVER.md` 与 `TF_AND_CONTROLLER_CHECKLIST.md` 可作为简明入口；
部署、测试、协议的正式说明以本目录为准。
