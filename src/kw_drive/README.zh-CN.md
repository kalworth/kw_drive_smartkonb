# kw_drive ROS 2 包

这是 KW Drive 自制 FOC 驱动器的 ROS 2 Jazzy 控制包。当前工程只保留已经跑通的串口适配路径，不再依赖旧 SDK 静态库。

## 功能范围

- 电机使能：`0xFC`
- 电机失能：`0xFD`
- MIT 控制：`kp / kd / q / dq / iq`
- 状态反馈：错误标志、位置、速度、q 轴电流
- 单电机段落感 smart knob demo：虚拟档位吸附、阻尼、q 轴电流限幅

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

`kw_smart_knob` 是单电机力反馈旋钮 demo。它读取电机位置 `q` 和速度 `dq`，计算最近的虚拟档位，
然后只通过动态 `iq` 输出吸附力矩：

```text
iq = detent_strength * (target_q - q) - damping * dq
```

修改配置：

```text
src/kw_drive/config/smart_knob.yaml
```

默认配置里 `enable_on_start: false`，第一次启动用于确认串口、CAN ID 和反馈链路；确认电机固定安全后，
再改成 `true` 试手感。

```bash
ros2 launch kw_drive smart_knob.launch.py
```

重点参数：

```text
detent_spacing：档位间隔，单位 rad，越小档位越密
detents_per_revolution：每圈档位数，0 表示直接使用 detent_spacing
detent_strength：档位吸附力度，越大段落感越明显
damping：速度阻尼，越大越不容易抖
iq_limit：输出 q 轴电流上限，先从小值开始
torque_sign：力矩方向，吸附方向反了就改成 -1.0
wrap_angle：按一圈绝对角度反馈做环形最短误差
wrap_period：0 表示自动使用 2 * pos_max，适配 [-pos_max, +pos_max] 反馈
feedback_timeout_ms：反馈超时保护，超时后发送 0 iq
zero_on_start：启动收到第一次反馈时，把当前位置作为中心点
```

如果一使能后电机远离档位或发散，立刻 `Ctrl-C` 停止节点，把 `torque_sign` 在 `1.0` 和 `-1.0`
之间切换后再试。

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
