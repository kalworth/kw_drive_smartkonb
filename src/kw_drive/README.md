# 🔌 kw_drive ROS 2 Package

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)
[![Build](https://img.shields.io/badge/Build-ament__cmake-6A5ACD.svg)](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html)
[![Runtime](https://img.shields.io/badge/Runtime-KW%20Serial%20U2CANFD-0E7C7B.svg)](../../README.md)

## ✨ New

- 🔌 **[2026-06-12][v0.1.0]** 按固定 README 模板重写驱动包说明，补齐节点、配置、验证和安全边界。
- 🕹️ **[2026-06-11][v0.1.0]** `kw_smart_knob` 支持 `free / detent / endstop / spring` 模式，并通过参数回调支持运行时调参。

<details>
<summary>历史更新</summary>

- 🧪 **[2026-06-10][v0.1.0]** `kw_motor_test` 统一发布 `/kw_motor/state`，供外部工具订阅。
- 🔌 **[2026-06-10][v0.1.0]** 驱动包清理为 KW 串口传输路径，固定使用 `/dev/ttyACM0` 和当前 CAN ID 规则。

</details>

`kw_drive` 是 KW Drive 自制 FOC 驱动器的 ROS 2 Jazzy C++ 驱动包。它负责串口传输、CAN ID 映射、MIT 控制报文编码、反馈解析，以及两个可运行节点：`kw_motor_test` 和 `kw_smart_knob`。

本包只保留当前支持的 KW 串口/U2CANFD 路线，不再维护旧 SDK 静态库、备用 SN 工具或其他运行路径。

## 🧩 架构

包内结构分成一层协议库和两类运行节点：协议库 `kw_motor` 封装串口和电机对象，节点负责参数读取、控制循环和状态发布。

| 模块 | 文件 | 作用 |
| :--- | :--- | :--- |
| 协议库 | `src/protocol/kw_motor.cpp` | 电机对象、MIT 控制、反馈更新 |
| 串口传输 | `src/protocol/kw_serial_transport.cpp` | KW 30 字节串口封装帧收发 |
| 基础测试节点 | `src/kw_motor_test_node.cpp` | 周期发送固定 MIT 指令 |
| Smart Knob 节点 | `src/kw_smart_knob_node.cpp` | 根据手感模式计算 MIT 目标 |
| 运行配置 | `config/*.yaml` | 串口、CAN ID、限制值和控制参数 |
| 启动入口 | `launch/*.launch.py` | 加载对应 YAML 并启动节点 |

## 🧭 核心流程

| 输入 | 说明 |
| :--- | :--- |
| `serial_port` / `serial.port` | U2CANFD 串口设备，当前配置为 `/dev/ttyACM0` |
| `serial_baud` / `serial.baud` | 串口波特率，当前配置为 `921600` |
| `can_id` / `motor.can_id` | 基础电机 CAN ID，当前配置为 `1` |
| `kp / kd / q / dq / iq` | MIT 控制字段，`iq` 表示 q 轴电流 |
| `basic.mode` | Smart Knob 手感模式 |

| 输出 | 说明 |
| :--- | :--- |
| KW 串口帧 | 30 字节封装帧，内部承载 CAN 控制报文 |
| `/kw_motor/state` | `sensor_msgs/JointState` 状态 topic |
| 节点日志 | 打印 CAN ID、反馈、错误位和当前控制状态 |

CAN ID 规则：

```text
使能/失能：can_id
MIT 控制：can_id | 0x70
状态反馈：can_id | 0x10
```

## 🚀 快速开始

### 1. 🧰 环境

- ROS 2 Jazzy
- C++17
- `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- U2CANFD 在 WSL 中可见为 `/dev/ttyACM0`

从工作空间根目录进入：

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
```

### 2. 🔧 配置

基础电机测试配置：

```text
src/kw_drive/config/motor_test.yaml
```

当前关键默认值：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `serial_port` | `/dev/ttyACM0` | 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |
| `can_id` | `1` | 基础 CAN ID |
| `enable_on_start` | `true` | 节点启动后发送使能 |
| `state_topic` | `/kw_motor/state` | 状态发布 topic |
| `period_ms` | `10.0` | 控制周期 |
| `pos_max` / `vel_max` / `iq_max` | `3.14` / `45.0` / `2.0` | MIT 字段物理上限 |
| `kp_max` / `kd_max` | `500.0` / `5.0` | MIT 增益字段映射上限 |
| `kp / kd / q / dq / iq` | `0.0` | 默认零输出指令 |

Smart Knob 配置：

```text
src/kw_drive/config/smart_knob.yaml
```

当前关键默认值：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `basic.mode` | `"detent"` | 默认棘轮档位模式 |
| `basic.period_ms` | `5.0` | 控制周期 |
| `basic.feedback_timeout_ms` | `100.0` | 反馈超时保护 |
| `basic.zero_on_start` | `true` | 第一次反馈作为中心点 |
| `motor.enable_on_start` | `true` | 节点启动后发送使能 |
| `mit.kp_max` / `mit.kd_max` | `10.0` / `1.0` | Smart Knob 使用的 MIT 增益映射上限 |
| `detent.spacing` | `0.02` | 虚拟档位间隔 |
| `endstop.min` / `endstop.max` | `-1.57` / `1.57` | 虚拟 endstop 左右止挡 |

### 3. ▶️ 编译和基础运行

编译本包：

```bash
colcon build --symlink-install --packages-select kw_drive
source install/setup.bash
```

启动基础测试节点：

```bash
ros2 launch kw_drive motor_test.launch.py
```

常用检查：

| 命令 | 说明 |
| :--- | :--- |
| `ros2 pkg executables kw_drive` | 确认 `kw_motor_test` 和 `kw_smart_knob` 已安装 |
| `ros2 topic echo /kw_motor/state` | 查看状态发布 |
| `test -e /dev/ttyACM0` | 确认串口设备存在 |

### 4. 🕹️ Smart Knob 运行

`kw_smart_knob` 读取电机反馈，根据 `basic.mode` 计算目标，再发送 MIT 命令。命名对齐开源 SmartKnob 的 `virtual detents` 和 `endstops`：`detent` 是棘轮/虚拟档位，`endstop` 是软件止挡。

```text
kp / kd / q_ref=target_q / dq_ref=0 / iq_ff
```

启动：

```bash
ros2 launch kw_drive smart_knob.launch.py
```

模式行为：

| `basic.mode` | 行为 |
| :--- | :--- |
| `"free"` | 始终发送零 MIT 命令 |
| `"detent"` | 吸附到最近棘轮档位 |
| `"endstop"` | 范围内零输出，越过 `endstop.min/max` 后吸回边界 |
| `"spring"` | 始终吸回启动中心点 |

兼容说明：旧模式名 `off` 会映射到 `free`，旧模式名 `limit` 会映射到 `endstop`。

运行时可修改参数：

| 参数 | 说明 |
| :--- | :--- |
| `basic.mode` | 切换 `free / detent / endstop / spring` |
| `basic.print_every` | 日志打印间隔 |
| `basic.feedback_timeout_ms` | 反馈超时阈值 |
| `detent.per_revolution` / `detent.spacing` | 虚拟档位密度 |
| `detent.kp` / `detent.kd` / `detent.iq_ff` | 棘轮档位 MIT 参数 |
| `endstop.min` / `endstop.max` / `endstop.kp` / `endstop.kd` | 虚拟 endstop 参数 |
| `spring.kp` / `spring.kd` / `spring.iq_ff` | 回中模式参数 |

旧参数组 `limit.*` 仍可在线修改并映射到 `endstop.*`；新配置统一使用 `endstop.*`。

示例：

```bash
ros2 param set /kw_smart_knob basic.mode "endstop"
ros2 param set /kw_smart_knob detent.kp 0.8
ros2 param set /kw_smart_knob spring.kp 0.3
```

不支持运行时修改的硬件/协议参数：

| 参数 | 原因 |
| :--- | :--- |
| `serial.*` | 串口对象已创建 |
| `motor.can_id` | 电机对象和 CAN 路由已注册 |
| `motor.pos_max` / `motor.vel_max` / `motor.iq_max` | MIT 映射限制已绑定 |
| `mit.kp_max` / `mit.kd_max` | MIT 增益映射限制已绑定 |
| `angle.period` | 环形角度周期影响内部归一化 |

### 5. 🧰 工具

| 工具 | 包 | 用途 |
| :--- | :--- | :--- |
| `kw_scope` | `kw_drive_tools` | 查看 `/kw_motor/state` 三路实时曲线 |
| `scripts/fix_kw_u2canfd_permissions.sh` | 工作空间脚本 | 修复 `/dev/ttyACM0` 权限 |

## 📊 验证

最低软件验证：

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kw_drive
source install/setup.bash
ros2 pkg executables kw_drive
```

硬件运行前条件：

```text
/dev/ttyACM0 存在
CAN ID 与驱动器配置一致
电机固定安全
第一次测试使用零输出或关闭 enable_on_start
```

状态 topic 合格条件：

```text
/kw_motor/state 持续发布
position[0] = 位置
velocity[0] = 转速
effort[0] = q 轴电流
```

## 🗂️ 目录结构

| 路径 | 说明 |
| :--- | :--- |
| `CMakeLists.txt` | `ament_cmake` 构建入口 |
| `package.xml` | ROS 2 包元数据 |
| `include/kw_drive/protocol` | 协议公共头文件 |
| `src/protocol` | 串口传输和电机协议实现 |
| `src/kw_motor_test_node.cpp` | 基础 MIT 控制测试节点 |
| `src/kw_smart_knob_node.cpp` | 力反馈旋钮节点 |
| `launch/motor_test.launch.py` | 基础测试 launch |
| `launch/smart_knob.launch.py` | Smart Knob launch |
| `config/motor_test.yaml` | 基础测试运行参数 |
| `config/smart_knob.yaml` | Smart Knob 运行参数 |

## 🧵 运行分工

```text
kw_motor library -> 串口帧、CAN ID、MIT 打包、反馈解析
kw_motor_test    -> 固定 MIT 指令测试
kw_smart_knob    -> 手感模式计算和在线调参
```

| 任务 | 执行位置 | 输出 |
| :--- | :--- | :--- |
| 串口收发 | `kw::MotorControl` | 更新 `kw::Motor` 反馈 |
| 周期控制 | ROS 2 wall timer | 发送 MIT 命令 |
| 状态发布 | `sensor_msgs/JointState` | `/kw_motor/state` |
| 在线调参 | `add_on_set_parameters_callback()` | 更新 Smart Knob 手感参数 |

## ⚠️ 注意

- `motor_test.launch.py` 和 `smart_knob.launch.py` 都可能在启动时使能电机，运行前必须检查 YAML。
- 第一次试新参数，先把 `kp=0`、`kd=0`、`q=0`、`dq=0`、`iq=0`，或把 `enable_on_start` 改成 `false`。
- 如果电机一使能就发散，立刻 `Ctrl-C` 停止节点，再检查反馈方向、CAN ID、`mit.kp_max/kd_max` 和增益。
- 不要重新引入旧 SDK 静态库或备用运行路径。
- 改协议、launch、YAML 或用户可见行为时，同步更新根目录 README 和本包 README。

## 📈 扩展方向

| 方向 | 入口 | 说明 |
| :--- | :--- | :--- |
| 新手感模式 | `src/kw_smart_knob_node.cpp` | 扩展 `KnobMode`、`parse_mode()` 和 `compute_*_command()` |
| MIT 协议调整 | `src/protocol/kw_motor.cpp` | 改字段映射前先核对驱动器侧协议 |
| 新状态字段 | `publish_state()` | 保持 `/kw_motor/state` 合约清晰，必要时新增 topic |
