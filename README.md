# mt_ws

KW Drive 的 ROS 2 Jazzy 工作空间。当前只保留已经实机跑通的串口适配路径：
U2CANFD 在 WSL 中映射为 `/dev/ttyACM0`，ROS 2 C++ 节点通过 30 字节串口封装帧发送 CAN
控制，并解析 16 字节反馈帧。

历史 SDK 路线已经清理，不再作为工程路径使用。

## 当前功能

- 使能电机：发送 `0xFC`
- 失能电机：发送 `0xFD`
- MIT 控制：`kp / kd / q / dq / iq`
- 状态反馈：错误标志、位置、速度、q 轴电流
- 单电机段落感 smart knob demo：虚拟档位吸附、MIT `kp/kd` 阻尼

CAN ID 固定规则：

```text
使能/失能：can_id
MIT 控制：can_id | 0x70
状态反馈：can_id | 0x10
```

## 编译和运行

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
scripts/fix_kw_u2canfd_permissions.sh
ros2 launch kw_drive motor_test.launch.py
```

运行前先改配置：

```text
src/kw_drive/config/motor_test.yaml
```

实机测试记录见 `LEARNING.md`。

## 段落感 Smart Knob

`kw_smart_knob` 是单电机力反馈旋钮 demo，复用同一条 KW 串口/CAN 通信链路。
节点读取电机位置和速度，根据 `basic.mode` 计算当前手感目标，然后通过 MIT 控制报文发送目标位置和 `kp/kd`：

```text
kp / kd / q_ref=target_q / dq_ref=0 / iq_ff
```

驱动器内部 MIT 误差已经做 wrap；ROS 侧 wrap 只用于选择最近虚拟档位，并把 `q_ref`
归一化到 `[-pos_max, +pos_max]` 反馈范围。

运行前先改配置：

```text
src/kw_drive/config/smart_knob.yaml
```

`motor.enable_on_start` 控制启动时是否发送使能命令。第一次试新参数时建议先用 `false`，
确认串口、CAN ID 和反馈后再改成 `true` 试手感。

```bash
ros2 launch kw_drive smart_knob.launch.py
```

常用调参项：

```text
basic.mode：手感模式，当前支持 off / detent / limit / spring
detent.spacing：档位间隔，越小档位越密
detent.per_revolution：每圈档位数，0 表示直接使用 detent.spacing
detent.kp：MIT 位置比例增益，越大段落吸附越明显，先从小值开始
detent.kd：MIT 速度阻尼，越大越不容易抖
detent.iq_ff：MIT q 轴前馈电流，默认 0
limit.min / limit.max：limit 模式的虚拟左右限位，相对启动中心点
limit.kp / limit.kd：limit 模式越界后拉回边界的 MIT 增益
spring.kp / spring.kd：spring 模式回中用的 MIT 增益
spring.iq_ff：spring 模式 q 轴前馈电流，默认 0
mit.kp_max / mit.kd_max：MIT 报文字段映射上限，要和驱动器协议配置一致
angle.wrap：按一圈绝对角度反馈做环形误差
angle.period：0 表示自动使用 2 * motor.pos_max，适配 [-pos_max, +pos_max] 反馈
```

`smart_knob.yaml` 按功能分块组织：

```yaml
basic:
  mode: "limit"
  period_ms: 5.0
  feedback_timeout_ms: 100.0

motor:
  can_id: 1
  enable_on_start: false
  pos_max: 3.14

detent:
  spacing: 0.1265
  kp: 1.5
  kd: 0.05

limit:
  min: -1.57
  max: 1.57
  kp: 1.5
  kd: 0.05

spring:
  kp: 0.5
  kd: 0.01
  iq_ff: 0.0
```

模式行为：

```text
off：始终发送零 MIT 命令
detent：吸附到最近虚拟档位
limit：范围内发送零 MIT 命令，越过 limit.min/limit.max 后吸回边界
spring：始终吸回启动中心点
```

节点支持运行时调参，不需要重启 launch：

```bash
ros2 param set /kw_smart_knob basic.mode detent
ros2 param set /kw_smart_knob basic.mode limit
ros2 param set /kw_smart_knob basic.mode spring
ros2 param set /kw_smart_knob detent.kp 0.8
ros2 param set /kw_smart_knob spring.kp 0.3
```

运行时调参使用 ROS 2 parameter service，节点内部通过 `add_on_set_parameters_callback`
校验并更新参数。串口、CAN ID、`motor.pos_max`、`mit.kp_max/kd_max`、`angle.period`
这类硬件/协议参数不支持运行中修改，改 YAML 后重启节点。

后续要扩展 `damper`、`lock` 等手感时，在 `kw_smart_knob_node.cpp` 里新增一个
`KnobMode` 和对应的 `compute_*_command()` 即可。

如果一使能后电机发散，立刻 `Ctrl-C` 停止节点，先把 `detent.kp/detent.kd/detent.iq_ff`
降到 0，确认反馈方向和 `mit.kp_max/mit.kd_max` 后再从小值加起。

## 示波器工具

`kw_motor_test` 和 `kw_smart_knob` 都会发布 `/kw_motor/state`，消息类型为
`sensor_msgs/JointState`：

```text
position[0]：位置
velocity[0]：转速
effort[0]：q 轴电流
```

独立工具包：`src/kw_drive_tools`

```bash
ros2 run kw_drive_tools kw_scope
```

窗口布局为 3 行 1 列：位置、转速、电流。

如果图形界面卡顿，可以降低示波器刷新率：

```bash
ros2 run kw_drive_tools kw_scope --ros-args -p refresh_hz:=20.0
```
