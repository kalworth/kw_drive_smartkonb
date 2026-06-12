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
- single-motor smart knob demo: virtual detents with MIT `kp/kd` damping

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
position and velocity, computes the active feel target from `basic.mode`, and
sends the target through the MIT command fields:

```text
kp / kd / q_ref=target_q / dq_ref=0 / iq_ff
```

The drive's MIT controller already wraps the position error internally. The ROS
side wrap is only used to select the nearest virtual detent and normalize
`q_ref` into the `[-pos_max, +pos_max]` feedback range.

Edit `src/kw_drive/config/smart_knob.yaml`, then run:

```bash
ros2 launch kw_drive smart_knob.launch.py
```

`motor.enable_on_start` controls whether the node sends the enable command at
startup. For a first run with new gains, keep it `false` until the serial port,
CAN ID, and feedback path are confirmed.

Key parameters:

```text
basic.mode: feel mode; currently supports off / detent / limit / spring
detent.spacing: distance between virtual detents, in radians
detent.per_revolution: detent count per absolute-angle turn; 0 uses detent.spacing directly
detent.kp: MIT position gain; start small and increase for stronger detents
detent.kd: MIT velocity damping term
detent.iq_ff: MIT q-axis feed-forward current, default 0
limit.min / limit.max: virtual limit range relative to the startup center
limit.kp / limit.kd: MIT gains used only after the knob crosses a virtual limit
spring.kp / spring.kd: MIT gains used to return to the startup center
spring.iq_ff: MIT q-axis feed-forward current for spring mode, default 0
mit.kp_max / mit.kd_max: MIT field scaling limits; keep them matched to the drive protocol
angle.wrap: use circular shortest-angle error for absolute-angle feedback
angle.period: 0 uses 2 * motor.pos_max, matching [-pos_max, +pos_max] feedback
```

`smart_knob.yaml` is grouped by responsibility:

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

Mode behavior:

```text
off: always sends a zero MIT command
detent: attracts the knob to the nearest virtual detent
limit: sends zero MIT command inside the range and pulls back to limit.min/limit.max outside it
spring: always attracts the knob back to the startup center
```

Runtime tuning is supported without restarting the launch:

```bash
ros2 param set /kw_smart_knob basic.mode detent
ros2 param set /kw_smart_knob basic.mode limit
ros2 param set /kw_smart_knob basic.mode spring
ros2 param set /kw_smart_knob detent.kp 0.8
ros2 param set /kw_smart_knob spring.kp 0.3
```

Runtime tuning uses ROS 2 parameter services. The node validates and applies
changes through `add_on_set_parameters_callback`. Hardware/protocol parameters
such as `serial.*`, `motor.can_id`, `motor.pos_max`, `mit.kp_max/kd_max`, and
`angle.period` are not runtime-editable; edit YAML and restart the node.

Future feel modes such as `damper` or `lock` should be added by extending
`KnobMode` and implementing a matching `compute_*_command()` helper in
`kw_smart_knob_node.cpp`.

If the knob diverges after enabling, stop the node immediately, set
`detent.kp/detent.kd/detent.iq_ff` back to 0, verify feedback direction and
`mit.kp_max/mit.kd_max`, then increase gains from small values.
