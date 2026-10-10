# 差速/全向速度到前轮转向三轮车的中间包

上游继续输出容易得到的 body-frame vx、vy、omega_z。velocity_adapter负责近似转换，
tricycle_controller负责底盘运动学和可实现范围。这两个包都没有实际驱动接口。

```text
差速/全向 Twist(vx,vy,w)
 -> velocity_adapter -> TwistStamped(v,0,w_adapted)
 -> tricycle_controller -> ChassisCommand(速度,前轮角,可实现yaw率)
```

## 单独测试/作为控制链路使用

```bash
cd /home/tree/farmercar_ws
source devel/setup.bash
roslaunch velocity_adapter tricycle.launch
```

在另一个已source环境的终端给全向斜向请求：

```bash
rostopic pub -r 10 /velocity_adapter/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

查看中间与最终结果：

```bash
rostopic echo /velocity_adapter/status
rostopic echo /velocity_adapter/adapted_cmd_vel
rostopic echo /tricycle_controller/command
```

默认Twist输入可映射到任意上游话题：

```bash
roslaunch velocity_adapter tricycle.launch input_topic:=/planner/cmd_vel
```

Stamped输入：

```bash
roslaunch velocity_adapter tricycle.launch input_stamped:=true \
  input_topic:=/planner/cmd_vel_stamped
```

Stamped要求header.frame_id=base_link，且时间戳非零、新鲜。Twist默认即body-frame；
本包不把map/odom世界坐标速度自动当作车体速度。世界坐标输入需由上游先旋转到车体系。
差速与全向输入使用相同接口并自动识别，不是接收左右轮速度或全向轮转速。
每次只选择一个上游来源，不要让多个发布者混用同一输入。
仅运行适配器可用 adapter.launch；运行完整软件转换链用 tricycle.launch。
camera_manager统一启动也包含这两个节点，不要同时重复启动。

## 转换策略：方向跟随近似

差速式vy=0且有平移速度时，vx、w直接传到后面的底盘包。底盘包仍会限速/限角；
所以差速底盘可以做到的原地转动、大曲率动作不能保证在三轮车上同样做到。

全向式vy非零时，默认选最近的前进/倒车朝向：

```text
s = -1 if vx < -epsilon else +1   # 纯横向请求默认选前进
v = s * hypot(vx, vy)
alpha = atan2(s*vy, s*vx)
w_adapted = w_requested + heading_gain * alpha
vy_output = 0
```

heading_gain默认0.5/s，方向偏差alpha单位rad，w单位rad/s。
这是带原始yaw率前馈的速度方向近似：前方偏左的平移请求转成前进左转，偏右转成前进右转。
倒车时方向偏差相对车体后向计算，底盘包用有符号v计算转角。
allow_reverse=false会拒绝负vx请求。零容差默认1e-6。

这不是数学上等价的全向速度实现：车辆瞬时vy仍是0，原始横向需求由行驶中转向代替。
固定不变的body-frame纯横向请求会持续转弯，不会平移到侧方并自动停止/回正。
若需要跟踪世界坐标轨迹，上游必须随车体朝向更新body-frame速度请求；本包没有里程计反馈，
也不提供世界轨迹闭环或终点停靠。平移方向与独立yaw请求有冲突时按上述叠加取舍，
不会承诺二者均精确执行。heading_gain是调节这个取舍的参数，不是相机外参。

纯旋转vx=vy=0而w!=0：输出零，accepted=false、reason=pure_rotation_unreachable。
不会偷偷给车辆加前进速度。全零输入正常停止。

后级三轮车使用L=0.839m，delta=atan(L*w_adapted/v)，delta限±50°，再重算实际yaw率。
默认前进/倒车限速0.15m/s。这一层保留原来的构型指令输出和严格vy=0契约。

## 默认参数下的示例

| 上游vx,vy,w | 中间输出 | 最终底盘输出 |
|---|---|---|
| .1,0,.05 | v=.1,w=.05 | speed=.1, steering约22.76°, yaw=.05 |
| .1,.1,0 | v约.1414,w约.3927 | speed约.1414, steering限到50°, yaw约.2009 |
| 0,.1,0 | v=.1,w约.7854 | speed=.1, steering限到50°, yaw约.1420 |
| 0,-.1,0 | v=.1,w约-.7854 | speed=.1, steering限到-50°, yaw约-.1420 |
| -.1,.1,0 | v约-.1414,w约-.3927 | 倒车、前轮正转角、车体负yaw率 |
| 0,0,.1 | 零输出，拒绝 | 停止 |

中间w可能超出底盘能力，必须继续接入tricycle_controller，不应直接把中间输出发到驱动。

## 接口、状态与失效处理

| 话题 | 类型 | 用途 |
|---|---|---|
| /velocity_adapter/cmd_vel | Twist或选定的TwistStamped | 上游通用期望速度 |
| /velocity_adapter/adapted_cmd_vel | TwistStamped | 转成vx+omega，linear.y=0 |
| /velocity_adapter/status | String JSON | 原始输入、转换值、approximate、accepted、reason |
| /tricycle_controller/command | ChassisCommand | 最终可实现速度、前轮角、yaw率、saturated |
| /tricycle_controller/achievable_cmd_vel | TwistStamped | 最终可实现速度，vy=0 |

approximate=false表示差速请求没有经过全向方向近似，并不保证下级无需限幅。
approximate=true表示对非零vy做了近似，最终限幅看底盘输出saturated。
实际执行速度未知；所有话题均为期望命令，不是测量。

输出20Hz，输入到达时也更新。无输入超过0.5s、输入过期/非法/错误frame时输出零。
停止时下级接受的是正常零速度，请看adapter/status了解上游拒绝/超时原因。
不设置非零超时爬行、不改变TF、不用指令值假装转角反馈。

配置位于config/adapter.yaml，可通过adapter_config传入自定义配置文件，启动时读取。
下级参数通过controller_config指定。所有参数修改后需重启。

## 与现有AprilTag链路结合

统一启动已改为：AprilTag候选速度 -> velocity_adapter -> tricycle_controller
-> vehicle_controller_bridge（默认无串口）。当前AprilTag本来输出vy=0，因此这一步保持原始候选速度。
手动或规划器提供vy非零时，本包才做方向近似。
旧vehicle_controller_node未启动；新vehicle_controller_bridge默认不打开串口、不解锁。

## 验证

```bash
PYTHONPATH=src python3 -m unittest discover -s test -p test_mapping.py -v
rostest velocity_adapter adapter.test
rostest camera_manager control_pipeline.test
```

测试只启动软件节点与测试输入。覆盖差速、斜向/纯横向、倒车方向、近似标记、最终限角，
纯旋转拒绝、两种输入格式的超时与恢复、重复旧消息和错误frame。
