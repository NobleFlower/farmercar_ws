# AprilTag伺服与底盘通信集成（2026-10-10）

已完成完整软件链路与双向驱动桥接，默认一键启动保持跟随禁用、串口关闭、驱动未解锁。
最新完整交接、启动/启用命令、验收和硬件接口见 **DRIVER_HANDOVER.md**。
具体字节协议见 **src/vehicle_controller/PROTOCOL.md**。
TF最小模型已由用户确认正确；软件控制直接使用AprilTag观测轴重排，无安装/距离补偿。
扩展下位机协议尚需真实固件适配；本次没有移动实际车辆。

正式文档与合并手册：[docs/README.md](docs/README.md)、[docs/MANUAL.md](docs/MANUAL.md)。
