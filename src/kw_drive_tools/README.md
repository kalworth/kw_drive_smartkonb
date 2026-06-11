# kw_drive_tools

中文说明见 [README.zh-CN.md](README.zh-CN.md).

Independent ROS 2 tools for KW Drive.

## Scope

Run after `kw_motor_test` is publishing `/kw_motor/state`:

```bash
ros2 run kw_drive_tools kw_scope
```

The scope subscribes to `sensor_msgs/JointState` and displays a 3x1 layout:
position, velocity, and q-axis current. It uses a lightweight PyQt5 renderer for
live display.

Useful parameters:

```bash
ros2 run kw_drive_tools kw_scope --ros-args \
  -p window_sec:=5.0 \
  -p refresh_hz:=30.0
```

Every feedback sample is stored in the visible time window. `refresh_hz` only
controls how often the GUI repaints. Lower `refresh_hz` or `window_sec` if the
WSL GUI feels slow.
