# kw_drive ROS 2 package

中文说明见 [README.zh-CN.md](README.zh-CN.md).

`kw_drive` is the ROS 2 Jazzy package for the KW Drive custom FOC controller.
The package keeps only the verified serial transport path: `/dev/ttyACM0` at
`921600`, KW Drive CAN ID rules, MIT control, and status feedback.

## Supported Runtime Path

- enable motor: `0xFC`
- disable motor: `0xFD`
- MIT control: `kp / kd / q / dq / iq`
- feedback: error flag, position, velocity, q-axis current
- single-motor smart knob demo: virtual detents, damping, and q-axis current limiting

CAN ID mapping:

```text
enable/disable: can_id
MIT command:    can_id | 0x70
feedback:       can_id | 0x10
```

Legacy SDK files and alternate runtime utilities are intentionally not part of
this package.

## Build

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Edit `src/kw_drive/config/motor_test.yaml`, then run:

```bash
ros2 launch kw_drive motor_test.launch.py
```

`enable_on_start: true` enables the motor when the node starts. Keep command
values at zero until the motor is safely mounted and the CAN ID is confirmed.

The node publishes `/kw_motor/state` as `sensor_msgs/JointState`:
`position[0]` is position, `velocity[0]` is velocity, and `effort[0]` is q-axis
current. The plotting tool is kept in the separate `kw_drive_tools` package.

## Smart Knob Demo

`kw_smart_knob` is a single-motor force-feedback knob demo. It reads the motor
position and velocity, snaps the current angle to the nearest virtual detent,
and sends only a dynamic `iq` command:

```text
iq = detent_strength * (target_q - q) - damping * dq
```

Edit `src/kw_drive/config/smart_knob.yaml`, then run:

```bash
ros2 launch kw_drive smart_knob.launch.py
```

The default config keeps `enable_on_start: false` so the first run can verify
the serial port, CAN ID, and feedback path before the motor is enabled. Change
it to `true` only after the motor is mounted safely.

Key parameters:

```text
detent_spacing: distance between virtual detents, in radians
detents_per_revolution: detent count per absolute-angle turn; 0 uses detent_spacing directly
detent_strength: attraction gain toward the nearest detent
damping: velocity damping term
iq_limit: hard limit for commanded q-axis current
torque_sign: set to -1.0 if the knob pushes away from detents
wrap_angle: use circular shortest-angle error for absolute-angle feedback
wrap_period: 0 uses 2 * pos_max, matching [-pos_max, +pos_max] feedback
```

If the knob pushes away from a detent or diverges after enabling, stop the node
immediately and flip `torque_sign` between `1.0` and `-1.0`.
