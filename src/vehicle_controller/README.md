# Vehicle Controller

## 1. 概述

这个包实现了一个 ROS 节点，用于连接外部控制命令、CH340 串口设备以及虚拟低速串口协议。

当前节点主要负责：

- 解析 CH340 固定帧命令
- 跟踪低速模式状态
- 解析虚拟串口中速度/转向数据包
- 生成外发控制指令包
- 根据 launch 参数配置串口

主代码文件为：

- `src/vehicle_controller_node.cpp`

---

## 2. 串口通信协议

### 2.1 CH340 4 字节命令帧

CH340 设备发送的是 4 字节固定帧，用于上位机发出动作和模式控制命令。

结构：

```text
[byte0][byte1][byte2][byte3]
```

常见命令组：

- 运动控制：`AB AB AB xx`
  - `01` -> 前进
  - `10` -> 后退
  - `00` -> 停车

- 卸货/复位：`CD CD CD xx`
  - `01` -> 卸货
  - `10` -> 复位

- 模式控制：`DE DE DE xx`
  - `00` -> 低速模式
  - `01` -> 间歇模式
  - `02` -> 智能巡航

- 过道导航：`AD AD AD xx`
  - `01` ~ `09` -> 前往第 N 过道

这些命令最终会被处理成 `ParsedCommand`，并通过 `CommandType` 语义字段输出为清晰的动作说明。

### 2.2 下位机 -> 上位机：虚拟串口状态包

下位机通过虚拟串口发送给上位机的协议，当前统一为 `0xEF` 帧头格式。

这也是上位机在代码中实际解析的包类型：`parseVirtualSpeedPacket()`。

数据包结构：

```text
[0xEF][type][len][speed_u16_le][steering_u16_le][crc]
```

字段说明：

- `0xEF`：起始帧头，表示虚拟串口状态/控制数据包
- `type`：类型字段，当前固定为 `0x01`
- `len`：数据长度，当前固定为 `0x04`
- `speed_u16_le`：下位机标定后的速度，无符号 16 位定点值，低字节在前，范围为 `0..100`（对应 `0.00..1.00`）
- `steering_u16_le`：下位机当前转向角，低字节在前，单位为 `0.1°`
- `crc`：前面所有字节累加后取低 8 位

#### 速度编码

速度采用带符号定点格式，保留 2 位小数：

```text
speed_value = raw_speed / 100.0
```

示例：

- `563` -> `5.63`
- `-120` -> `-1.20`
- `0` -> `0.00`

#### 转向角编码

转向角使用 `0.1°` 为单位编码：

```text
angle_deg = raw_angle / 10.0
```

示例：

- `900` -> `90.0°`
- `450` -> `45.0°`

这个包代表的是“下位机当前的低速模式速度设置值 + 当前转向角”，也是上位机用来更新 `VehicleState.currentLowSpeedValue` 和 `VehicleState.currentSteeringAngle` 的来源。

### 2.3 上位机 -> 下位机：外发运动控制包

节点向底层控制器发送的 motion 包使用 `0xEE` 帧头。

包结构：

```text
[0xEE][type][len][data...][crc]
```

当前运动包格式：

```text
[0xEE][0x01][0x04][speed_i16_le][steering_u16_le][crc]
```

其中：

- `speed`：带符号定点值，可表示小数
- `steering`：转向角，单位为 `0.1°`
- 默认转向角为 `900`，表示 `90.0°`

#### 当前真实运行语义

当前代码中的低速模式不是在算法层直接重计算速度，而是先保持“接收什么、回发什么”的统一协议：

- 先解析来自虚拟串口的 `0xEF` 包
- 读取其中的 `speed` 与 `steering`
- 更新 `VehicleState.currentLowSpeedValue` 和 `VehicleState.currentSteeringAngle`
- 在低速模式下，收到 `前进 / 停止 / 后退` 命令时，按当前状态生成 `0xEE` 包回发

也就是说，当前阶段的目标是：先保证协议一致、值一致、回传一致；之后再继续调试控制逻辑和参数。

---

## 3. 数据模型

### 3.1 CommandType

代码中不再依赖原始字符串或 `cmd.name` 之类的魔法值，而是使用语义明确的枚举：

```cpp
enum class CommandType
{
    Unknown,
    MoveForward,
    MoveReverse,
    MoveStop,
    Unload,
    Reset,
    LowSpeedMode,
    IntermittentMode,
    IntelligentCruise,
    GoAisle
};
```

这样日志、分发逻辑和后续维护都更清晰。

### 3.2 ParsedCommand

```cpp
struct ParsedCommand
{
    CommandType type = CommandType::Unknown;
    std::string detail = "无法识别的帧";
    bool requiresQrNavigation = false;
};
```

它保存解析后的命令，包括：

- 语义类型
- 可读说明
- 是否需要二维码导航

### 3.3 VehicleState

```cpp
struct VehicleState
{
    bool lowSpeedMode = true;
    bool qrNavigationRequired = false;
    float currentLowSpeedValue = 0.0f;
    float currentSteeringAngle = 90.0f;
    std::string currentMotion = "停车";
    std::string currentMode = "低速模式";
    int targetAisle = 0;
};
```

这个结构统一保存运行时状态，便于命令处理和状态判断。

---

## 4. 代码结构

### 4.1 解析与解码辅助函数

当前节点中主要的辅助函数包括：

- `bytesToHexString()`
- `decodeSignedFixedPointValue()`
- `decodeSteeringAngleFromRaw()`
- `encodeSignedFixedPointValue()`
- `encodeSteeringAngleToRaw()`
- `parseVirtualSpeedPacket()`

这些函数负责把底层字节转换成真实的速度、角度和动作状态。

### 4.2 包组装与校验

关键辅助函数：

- `computeCrc()`
- `buildMotionPacket()`
- `sendPacket()`

CRC 为所有字节累加后取低 8 位。

### 4.3 串口配置

```cpp
bool configureSerialPort(int fd, int baudrate)
```

用于设置：

- 波特率
- 8N1 格式
- 无校验位
- 无硬件流控
- 非阻塞读取和超时

### 4.4 帧解析器

```cpp
ParsedCommand parseFrame(const std::array<uint8_t, 4>& frame)
```

它用于识别 CH340 固定帧命令，并映射成语义动作。

### 4.5 主循环

`main()` 中的流程为：

1. 初始化 ROS 节点
2. 读取 `port` 和 `baudrate` 参数
3. 打开并配置串口
4. 循环读取串口数据
5. 解析：
   - CH340 4 字节固定命令
   - `0xEF` 虚拟串口低速包
6. 更新 `VehicleState`
7. 在需要时发送 `0xEE` 运动控制包

---

## 5. 运行参数

当前 launch 文件中实际暴露的参数为：

- `port`：CH340 命令串口，默认值 `/dev/ttyUSB0`
- `virtual_port`：速度/角度虚拟串口，默认值 `/dev/ttyACM0`，负责状态接收和运动数据发送
- `baudrate`：默认值 `115200`
- `log_level`：默认值 `info`

示例：

```bash
roslaunch vehicle_controller vehicle_core.launch port:=/dev/ttyUSB0 baudrate:=115200 log_level:=info
```

---

## 6. 注意事项

- 协议设计故意避免直接写死 `ABABAB` 这类魔法值，改为语义枚举，便于调试和维护。
- 新版 `0xEF` 虚拟包是当前正式解析格式，包含 `speed` 与 `steering` 两个数据。
- 发送时仍然使用 `0xEE` 头，但速度字段采用带符号定点值，可以表示小数。
- 当前低速模式是“先回传最近接收值”，也就是先保证收发协议一致，再后续升级控制算法。

---

## 7. 示例换算

### 示例 A：速度 = 5.63

```text
raw_speed = 563
```

其字节表示为低字节在前：

```text
0x37 0x02
```

对应包中会出现：

```text
[0xEF][0x01][0x04][0x37][0x02][angle_low][angle_high][crc]
```

### 示例 B：转向角 = 90.0°

```text
raw_angle = 900
```

其字节表示为：

```text
0x84 0x03
```

---

## 8. 当前开发阶段说明

当前节点已经完成的状态是：

- CH340 4 字节命令帧解析已实现
- `0xEF` 虚拟串口状态包解析已实现
- `speed` 与 `steering` 统一按相同格式编码/解码
- 低速模式下先按最近一次接收到的值回传 `0xEE` 运动包
- 代码已编译通过，当前阶段重点是协议稳定性和真实设备联调

后续扩展建议：

- 明确的模式状态机
- 更清晰的解析器和执行器分离
- CRC 与编码/解码的单元测试
- 更详细的原始字节、解码结果和校验日志
- 在硬件上验证低速回传后的实际响应，再决定是否加入更复杂控制策略
