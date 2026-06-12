# 🧭 KW Drive ROS 2 Workspace

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3-3776AB.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2024.04%20WSL-E95420.svg)](https://ubuntu.com/)

## ✨ New

- 🧭 **[2026-06-12][v0.1.0]** 按固定 README 模板重写工作空间总览，补齐运行路径、参数、验证和安全注意。
- 🕹️ **[2026-06-11][v0.1.0]** `kw_smart_knob` 支持 `off / detent / limit / spring` 手感模式，并支持 ROS 2 参数服务在线调参。

<details>
<summary>历史更新</summary>

- 📈 **[2026-06-10][v0.1.0]** 新增独立工具包 `kw_drive_tools`，通过 `kw_scope` 查看 `/kw_motor/state` 实时曲线。
- 🔌 **[2026-06-10][v0.1.0]** 清理旧 SDK 路线，工作空间统一到 KW 串口传输、`/dev/ttyACM0` 和当前 CAN ID 规则。

</details>

这是 KW Drive 自制 FOC 驱动器的 ROS 2 Jazzy 工作空间。当前只保留已经实机跑通的串口适配路径：U2CANFD 在 WSL 中映射为 `/dev/ttyACM0`，ROS 2 C++ 节点通过 30 字节串口封装帧发送 CAN 控制，并解析 16 字节反馈帧。

工程目标是把底层驱动通信、单电机 MIT 控制、力反馈旋钮 demo 和实时示波器工具组织成清晰的 ROS 2 工作空间。历史 SDK 路线已经清理，不再作为工程路径使用。

## 🧩 架构

`kw_drive` 负责串口/CAN/MIT 控制和状态发布，`kw_drive_tools` 只放独立可视化工具，两个包通过 `/kw_motor/state` 这个 ROS 2 topic 解耦。

| 模块 | 位置 | 作用 |
| :--- | :--- | :--- |
| 驱动包 | `src/kw_drive` | KW 串口传输、CAN 封装、MIT 控制、launch 和运行 YAML |
| 工具包 | `src/kw_drive_tools` | PyQt5 示波器 `kw_scope` |
| 配置 | `src/kw_drive/config` | `motor_test.yaml`、`smart_knob.yaml` |
| 权限脚本 | `scripts/fix_kw_u2canfd_permissions.sh` | 修复 WSL 下 `/dev/ttyACM0` 访问权限 |

## 🧭 核心流程

| 输入 | 说明 |
| :--- | :--- |
| `/dev/ttyACM0` | WSL 中的 U2CANFD 串口设备 |
| `serial_baud` / `serial.baud` | 默认 `921600` |
| `can_id` / `motor.can_id` | 电机基础 CAN ID，默认 `1` |
| `motor_test.yaml` | 单电机零输出或固定 MIT 指令测试配置 |
| `smart_knob.yaml` | 力反馈旋钮模式、增益、限位和周期配置 |

| 输出 | 说明 |
| :--- | :--- |
| CAN 控制帧 | 通过 KW 串口封装发送到 U2CANFD |
| `/kw_motor/state` | `sensor_msgs/JointState`，包含位置、速度、q 轴电流 |
| `kw_scope` 窗口 | 位置、转速、电流三路实时曲线 |

CAN ID 固定规则：

```text
使能/失能：can_id
MIT 控制：can_id | 0x70
状态反馈：can_id | 0x10
```

当前支持功能：

| 功能 | 报文/接口 | 说明 |
| :--- | :--- | :--- |
| 使能电机 | `0xFC` | 可由 YAML 控制是否启动即发送 |
| 失能电机 | `0xFD` | 节点退出时用于关闭输出 |
| MIT 控制 | `kp / kd / q / dq / iq` | `iq` 表示 q 轴电流 |
| 状态反馈 | `/kw_motor/state` | 错误标志在节点日志中打印，曲线 topic 发布运动量 |
| Smart Knob | `kw_smart_knob` | 单电机虚拟档位、限位、回中手感 demo |

## 🚀 快速开始

### 1. 🧰 环境

- Ubuntu 24.04 WSL
- ROS 2 Jazzy
- `colcon`
- `python3-pyqt5`
- U2CANFD 已在 WSL 中显示为 `/dev/ttyACM0`

进入工作空间：

```bash
cd ~/mt_ws
```

### 2. 🔧 配置

单电机基础测试先改：

```text
src/kw_drive/config/motor_test.yaml
```

建议第一次硬件测试使用零输出值：

```yaml
kp: 0.0
kd: 0.0
q: 0.0
dq: 0.0
iq: 0.0
```

关键参数：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `serial_port` | `/dev/ttyACM0` | U2CANFD 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |
| `can_id` | `1` | 基础 CAN ID |
| `enable_on_start` | `true` | 启动后是否发送使能 |
| `state_topic` | `/kw_motor/state` | 反馈发布 topic |
| `period_ms` | `10.0` | 控制周期 |
| `kp_max` / `kd_max` | `500.0` / `5.0` | MIT 报文字段映射上限 |

### 3. ▶️ 编译和基础运行

编译整个工作空间：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如需修复串口权限：

```bash
scripts/fix_kw_u2canfd_permissions.sh
```

启动基础电机测试：

```bash
ros2 launch kw_drive motor_test.launch.py
```

### 4. 🕹️ Smart Knob

`kw_smart_knob` 读取电机位置和速度，根据 `basic.mode` 计算当前手感目标，再通过 MIT 控制报文发送：

```text
kp / kd / q_ref=target_q / dq_ref=0 / iq_ff
```

运行前修改：

```text
src/kw_drive/config/smart_knob.yaml
```

常用模式：

| `basic.mode` | 行为 |
| :--- | :--- |
| `"off"` | 始终发送零 MIT 命令 |
| `"detent"` | 吸附到最近虚拟档位 |
| `"limit"` | 范围内零输出，越界后吸回边界 |
| `"spring"` | 始终吸回启动中心点 |

常用参数：

| 参数 | 说明 |
| :--- | :--- |
| `basic.feedback_timeout_ms` | 反馈超时后发送零 MIT 命令 |
| `basic.zero_on_start` | 第一次收到反馈时把当前位置作为中心 |
| `detent.spacing` | 档位间隔，单位 rad |
| `detent.per_revolution` | 每圈档位数，`0` 表示直接使用 `detent.spacing` |
| `detent.kp` / `detent.kd` | 段落吸附 MIT 增益 |
| `limit.min` / `limit.max` | 相对启动中心点的虚拟左右限位 |
| `spring.kp` / `spring.kd` | 回中手感 MIT 增益 |
| `mit.kp_max` / `mit.kd_max` | 要和驱动器协议配置一致 |
| `angle.wrap` / `angle.period` | 绝对角度反馈的环形误差设置 |

启动：

```bash
ros2 launch kw_drive smart_knob.launch.py
```

运行时调参不需要重启 launch：

```bash
ros2 param set /kw_smart_knob basic.mode "detent"
ros2 param set /kw_smart_knob basic.mode "limit"
ros2 param set /kw_smart_knob basic.mode "spring"
ros2 param set /kw_smart_knob detent.kp 0.8
ros2 param set /kw_smart_knob spring.kp 0.3
```

硬件/协议参数不支持运行中修改，例如 `serial.*`、`motor.can_id`、`motor.pos_max`、`mit.kp_max/kd_max`、`angle.period`。这些参数改 YAML 后重启节点。

### 5. 📈 示波器工具

`kw_motor_test` 和 `kw_smart_knob` 都会发布：

```text
/kw_motor/state
```

字段约定：

| 字段 | 含义 |
| :--- | :--- |
| `position[0]` | 位置 |
| `velocity[0]` | 转速 |
| `effort[0]` | q 轴电流 |

启动 PyQt5 示波器：

```bash
ros2 run kw_drive_tools kw_scope
```

降低界面刷新率：

```bash
ros2 run kw_drive_tools kw_scope --ros-args -p refresh_hz:=20.0
```

## 📊 验证

软件侧最低验证：

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables kw_drive
ros2 pkg executables kw_drive_tools
```

硬件侧运行前检查：

```bash
test -e /dev/ttyACM0
```

通过条件：

```text
构建通过
/dev/ttyACM0 存在
CAN ID 与电机配置一致
第一次测试 kp=0, kd=0, q=0, dq=0, iq=0
/kw_motor/state 能稳定发布 position / velocity / effort
```

## 🗂️ 目录结构

| 路径 | 说明 |
| :--- | :--- |
| `src/kw_drive` | 核心 ROS 2 C++ 驱动包 |
| `src/kw_drive/src` | 节点和协议实现 |
| `src/kw_drive/include` | 公共头文件 |
| `src/kw_drive/launch` | `motor_test.launch.py`、`smart_knob.launch.py` |
| `src/kw_drive/config` | 运行 YAML |
| `src/kw_drive_tools` | 独立 Python 工具包 |
| `src/kw_drive_tools/kw_drive_tools/kw_scope.py` | PyQt5 实时示波器 |
| `scripts` | 工作空间辅助脚本 |
| `mit.md` | MIT 协议笔记 |

## 🧵 节点分工

```text
kw_motor_test   -> 串口/CAN 基础测试，发布 /kw_motor/state
kw_smart_knob   -> 单电机力反馈旋钮 demo，发布 /kw_motor/state
kw_scope        -> 订阅 /kw_motor/state，只做显示，不参与控制
```

| 节点 | 包 | 主要职责 |
| :--- | :--- | :--- |
| `kw_motor_test` | `kw_drive` | 周期发送 MIT 命令，打印反馈，发布状态 |
| `kw_smart_knob` | `kw_drive` | 根据手感模式计算目标，支持运行时调参 |
| `kw_scope` | `kw_drive_tools` | 缓存反馈数据，按 `refresh_hz` 刷新曲线 |

## ⚠️ 注意

- `motor_test.launch.py` 和 `smart_knob.launch.py` 都可能使能电机，运行前先看对应 YAML。
- 新参数第一次上电建议保持 `kp=0`、`kd=0`、`q=0`、`dq=0`、`iq=0` 或把 `enable_on_start` 设为 `false`。
- 如果一使能后电机发散，立刻 `Ctrl-C` 停止节点，再检查反馈方向、CAN ID、`mit.kp_max/kd_max` 和增益。
- 不要重新引入旧 SDK 静态库或备用运行路径；当前支持路线是 KW 串口传输、`/dev/ttyACM0` 和现有 CAN ID 规则。
- 修改代码、launch、运行 YAML 或用户可见行为时，同步更新相关 README。

## 📈 扩展方向

| 方向 | 入口 | 说明 |
| :--- | :--- | :--- |
| 新手感模式 | `src/kw_drive/src/kw_smart_knob_node.cpp` | 扩展 `KnobMode` 和对应 `compute_*_command()` |
| 新曲线显示 | `src/kw_drive_tools/kw_drive_tools/kw_scope.py` | 继续订阅 `/kw_motor/state` 或新增只读 topic |
| 协议核对 | `mit.md`、`src/kw_drive/src/protocol/kw_motor.cpp` | 改 MIT 字段前先和驱动器侧实现对齐 |
