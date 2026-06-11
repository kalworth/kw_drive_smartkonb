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

## 示波器工具

`kw_motor_test` 会发布 `/kw_motor/state`，消息类型为 `sensor_msgs/JointState`：

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
