# kw_drive_tools

这是 KW Drive 的独立 ROS 2 工具包，不放在驱动包 `kw_drive` 目录里，避免污染核心驱动代码。

## 示波器

运行前先让 `kw_motor_test` 发布反馈 topic：

```text
/kw_motor/state
```

消息类型为 `sensor_msgs/JointState`：

```text
position[0]：位置
velocity[0]：转速
effort[0]：q 轴电流
```

启动示波器：

```bash
source ~/mt_ws/install/setup.bash
ros2 run kw_drive_tools kw_scope
```

示波器使用 PyQt5 自绘曲线，比之前的 Matplotlib 更适合实时刷新。

窗口布局为 3 行 1 列：

```text
第 1 行：位置
第 2 行：转速
第 3 行：q 轴电流
```

常用参数：

```bash
ros2 run kw_drive_tools kw_scope --ros-args \
  -p window_sec:=5.0 \
  -p refresh_hz:=30.0
```

反馈来一条就进 buffer，不做限频丢点。`refresh_hz` 只控制界面每秒重画几次。
如果 WSL 图形界面仍然卡，就降低 `refresh_hz` 或缩短 `window_sec`。
