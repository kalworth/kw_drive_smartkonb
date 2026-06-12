# kw_drive ROS 2 包

这是 KW Drive 自制 FOC 驱动器的 ROS 2 Jazzy 控制包。当前工程只保留已经跑通的串口适配路径，不再依赖旧 SDK 静态库。

## 功能范围

- 电机使能：`0xFC`
- 电机失能：`0xFD`
- MIT 控制：`kp / kd / q / dq / iq`
- 状态反馈：错误标志、位置、速度、q 轴电流
- 单电机段落感 smart knob demo：虚拟档位吸附、MIT `kp/kd` 阻尼

不支持寄存器读写、模式切换、零点设置等扩展功能。

## CAN ID 规则

```text
使能/失能 ID：can_id
MIT 控制 ID：can_id | 0x70
状态反馈 ID：can_id | 0x10
```

例如 `can_id: 1`：

```text
使能/失能：0x01
MIT 控制：0x71
状态反馈：0x11
```

## 编译

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 配置

修改：

```text
src/kw_drive/config/motor_test.yaml
```

重点参数：

```yaml
serial_port: "/dev/ttyACM0"
serial_baud: 921600
can_id: 1
enable_on_start: true
period_ms: 1.0
print_every: 100
pos_max: 12.5
vel_max: 45.0
iq_max: 2.0
kp_max: 500.0
kd_max: 5.0
kp: 0.0
kd: 0.0
q: 0.0
dq: 0.0
iq: 0.0
```

`enable_on_start: true` 表示节点启动后会使能电机；不想启动即使能就改成 `false`。

## 运行

确认电机固定安全、CAN ID 正确、电源正常后运行：

```bash
ros2 launch kw_drive motor_test.launch.py
```

如果 WSL 里没有权限访问 `/dev/ttyACM0`，先运行：

```bash
cd ~/mt_ws
scripts/fix_kw_u2canfd_permissions.sh
```

## 段落感 Smart Knob Demo

`kw_smart_knob` 是单电机力反馈旋钮 demo。它读取电机位置 `q` 和速度 `dq`，根据 `basic.mode`
计算当前手感目标，然后通过 MIT 控制报文发送目标位置和 `kp/kd`，由驱动器执行位置环：

```text
kp / kd / q_ref=target_q / dq_ref=0 / iq_ff
```

驱动器内部 MIT 误差已经做 wrap；ROS 侧 wrap 只用于选择最近虚拟档位，并把 `q_ref`
归一化到 `[-pos_max, +pos_max]` 反馈范围。

修改配置：

```text
src/kw_drive/config/smart_knob.yaml
```

`motor.enable_on_start` 控制启动时是否发送使能命令。第一次试新参数时建议先用 `false`，
确认串口、CAN ID 和反馈后再改成 `true` 试手感。

```bash
ros2 launch kw_drive smart_knob.launch.py
```

重点参数：

```text
basic.mode：手感模式，当前支持 off / detent / limit / spring
basic.feedback_timeout_ms：反馈超时保护，超时后发送零 MIT 命令
basic.zero_on_start：启动收到第一次反馈时，把当前位置作为中心点
detent.spacing：档位间隔，单位 rad，越小档位越密
detent.per_revolution：每圈档位数，0 表示直接使用 detent.spacing
detent.kp：MIT 位置比例增益，越大段落感越明显，先从小值开始
detent.kd：MIT 速度阻尼，越大越不容易抖
detent.iq_ff：MIT q 轴前馈电流，默认 0
limit.min / limit.max：limit 模式的虚拟左右限位，相对启动中心点
limit.kp / limit.kd：limit 模式越界后拉回边界的 MIT 增益
spring.kp / spring.kd：spring 模式回中用的 MIT 增益
spring.iq_ff：spring 模式 q 轴前馈电流，默认 0
mit.kp_max / mit.kd_max：MIT 报文字段映射上限，要和驱动器协议配置一致
angle.wrap：按一圈绝对角度反馈做环形最短误差
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

## 反馈 topic

`kw_motor_test` 和 `kw_smart_knob` 都会发布：

```text
/kw_motor/state
```

类型为 `sensor_msgs/JointState`：

```text
position[0]：位置
velocity[0]：转速
effort[0]：q 轴电流
```

示波器工具在独立包 `kw_drive_tools` 中，不放进驱动包目录。
