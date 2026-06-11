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
- 单电机段落感 smart knob demo：虚拟档位吸附、阻尼、限流

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
节点读取电机位置和速度，计算最近的虚拟档位，然后只通过 `iq` 输出吸附力矩：

```text
iq = detent_strength * (target_q - q) - damping * dq
```

运行前先改配置：

```text
src/kw_drive/config/smart_knob.yaml
```

默认 `enable_on_start: false`，第一次启动只用于确认串口、CAN ID 和反馈。确认安全后再改成
`true` 试手感。

```bash
ros2 launch kw_drive smart_knob.launch.py
```

常用调参项：

```text
detent_spacing：档位间隔，越小档位越密
detents_per_revolution：每圈档位数，0 表示直接使用 detent_spacing
detent_strength：吸附力度，越大段落感越明显
damping：阻尼，越大越不容易抖
iq_limit：最大输出 q 轴电流，先从小值开始
torque_sign：力矩方向，吸附方向反了就改成 -1.0
wrap_angle：按一圈绝对角度反馈做环形误差
wrap_period：0 表示自动使用 2 * pos_max，适配 [-pos_max, +pos_max] 反馈
```

如果一使能后电机远离档位或发散，立刻 `Ctrl-C` 停止节点，把 `torque_sign` 从 `1.0`
改为 `-1.0` 后再试。

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
