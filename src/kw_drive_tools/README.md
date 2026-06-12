# 📈 kw_drive_tools

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3-3776AB.svg)](https://www.python.org/)
[![Build](https://img.shields.io/badge/Build-ament__python-6A5ACD.svg)](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Python-Documentation.html)
[![GUI](https://img.shields.io/badge/GUI-PyQt5-41CD52.svg)](https://riverbankcomputing.com/software/pyqt/)

## ✨ New

- 📈 **[2026-06-12][v0.1.0]** 按固定 README 模板重写工具包说明，补齐输入、参数、验证和运行分工。
- 🧵 **[2026-06-10][v0.1.0]** `kw_scope` 使用 PyQt5 自绘曲线，反馈样本全部进入时间窗口 buffer，`refresh_hz` 只控制界面刷新。

<details>
<summary>历史更新</summary>

- 📦 **[2026-06-10][v0.1.0]** `kw_drive_tools` 从驱动包中独立出来，避免可视化工具污染核心驱动代码。

</details>

`kw_drive_tools` 是 KW Drive 的独立 ROS 2 Python 工具包。当前提供 `kw_scope`，用于订阅 `/kw_motor/state` 并实时显示位置、转速和 q 轴电流。

工具包只做只读可视化，不发送电机命令，不修改驱动参数，也不参与硬件使能。

## 🧩 架构

`kw_scope` 由一个 ROS 2 订阅节点、一个时间窗口 buffer 和一个 PyQt5 主窗口组成；ROS spin 在线程中运行，Qt 主线程按 `refresh_hz` 重绘曲线。

| 模块 | 文件 | 作用 |
| :--- | :--- | :--- |
| ROS 节点 | `kw_drive_tools/kw_scope.py` | 订阅 `sensor_msgs/JointState` |
| Buffer | `ScopeBuffer` | 保留最近 `window_sec` 秒的全部反馈样本 |
| 曲线控件 | `TraceWidget` | 绘制单路曲线 |
| 主窗口 | `ScopeWindow` | 3 行 1 列显示位置、转速、`iq` |
| Launch | `launch/kw_scope.launch.py` | 使用默认 topic 和刷新参数启动 |

## 🧭 核心流程

| 输入 | 说明 |
| :--- | :--- |
| `/kw_motor/state` | 默认订阅 topic |
| `sensor_msgs/JointState.position[0]` | 位置曲线 |
| `sensor_msgs/JointState.velocity[0]` | 转速曲线 |
| `sensor_msgs/JointState.effort[0]` | q 轴电流曲线 |
| `window_sec` | 保留的可视时间窗口 |
| `refresh_hz` | GUI 每秒重绘次数 |

| 输出 | 说明 |
| :--- | :--- |
| `KW Drive Scope` 窗口 | 3 行 1 列实时曲线 |
| ROS 日志 | 当前订阅 topic、时间窗口、刷新率 |

## 🚀 快速开始

### 1. 🧰 环境

- ROS 2 Jazzy
- Python 3
- `ament_python`
- `rclpy`
- `sensor_msgs`
- `python3-pyqt5`
- 已有 `kw_motor_test` 或 `kw_smart_knob` 发布 `/kw_motor/state`

从工作空间根目录进入：

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
```

### 2. 🔧 配置

默认 launch 参数：

```yaml
topic: "/kw_motor/state"
window_sec: 5.0
refresh_hz: 30.0
```

参数说明：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `topic` | `/kw_motor/state` | 订阅的 `JointState` topic |
| `window_sec` | `5.0` | 曲线保留最近多少秒 |
| `refresh_hz` | `30.0` | GUI 每秒刷新次数 |

### 3. ▶️ 运行

先启动状态发布节点，例如：

```bash
ros2 launch kw_drive motor_test.launch.py
```

再启动示波器：

```bash
ros2 run kw_drive_tools kw_scope
```

如果 WSL 图形界面卡顿，可以降低刷新率：

```bash
ros2 run kw_drive_tools kw_scope --ros-args -p refresh_hz:=20.0
```

### 4. 🧪 Launch 方式

也可以通过 launch 使用默认参数启动：

```bash
ros2 launch kw_drive_tools kw_scope.launch.py
```

常用覆盖参数：

| 目的 | 参数 | 示例 |
| :--- | :--- | :--- |
| 换 topic | `topic` | `-p topic:=/kw_motor/state` |
| 缩短窗口 | `window_sec` | `-p window_sec:=3.0` |
| 降低刷新 | `refresh_hz` | `-p refresh_hz:=15.0` |

### 5. 🧰 工具

| 工具 | 用途 |
| :--- | :--- |
| `kw_scope` | 实时查看位置、速度、q 轴电流 |
| `ros2 topic echo /kw_motor/state` | 检查原始反馈消息 |
| `ros2 topic hz /kw_motor/state` | 检查反馈发布频率 |

## 📊 验证

软件侧最低验证：

```bash
cd ~/mt_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kw_drive_tools
source install/setup.bash
ros2 pkg executables kw_drive_tools
```

运行前输入条件：

```text
/kw_motor/state 存在
消息类型为 sensor_msgs/JointState
position、velocity、effort 至少各有 1 个元素
```

显示通过条件：

```text
窗口标题为 KW Drive Scope
三行曲线分别为 position、velocity、iq
收到反馈后 waiting for /kw_motor/state 提示消失
```

## 🗂️ 目录结构

| 路径 | 说明 |
| :--- | :--- |
| `setup.py` | `ament_python` 包入口和 console script |
| `package.xml` | ROS 2 包元数据 |
| `kw_drive_tools/kw_scope.py` | PyQt5 示波器实现 |
| `launch/kw_scope.launch.py` | launch 启动入口 |
| `resource/kw_drive_tools` | ament resource index 标记 |

## 🧵 运行分工

```text
rclpy spin thread -> 接收 /kw_motor/state 并写入 ScopeBuffer
Qt main thread    -> 按 refresh_hz 读取快照并重绘窗口
```

| 任务 | 执行位置 | 说明 |
| :--- | :--- | :--- |
| 消息接收 | `KwScopeNode._on_state()` | 每条反馈都进入 buffer |
| 时间裁剪 | `ScopeBuffer.append()` | 只删除超出 `window_sec` 的旧样本 |
| 曲线刷新 | `ScopeWindow._refresh()` | 按 `refresh_hz` 从 buffer 取快照 |
| 绘制 | `TraceWidget.paintEvent()` | 根据当前窗口自适应纵轴范围 |

## ⚠️ 注意

- `kw_scope` 不会使能电机，但它依赖其他节点发布 `/kw_motor/state`。
- `refresh_hz` 只控制界面重绘频率，不会降低反馈采样写入频率。
- 如果 WSL GUI 卡顿，优先降低 `refresh_hz` 或缩短 `window_sec`。
- 不要把硬件控制逻辑放进 `kw_drive_tools`；控制逻辑属于 `kw_drive`。
- 修改工具入口、参数或显示行为时，同步更新根目录 README 和本包 README。

## 📈 扩展方向

| 方向 | 入口 | 说明 |
| :--- | :--- | :--- |
| 增加曲线 | `ScopeWindow` / `TraceWidget` | 保持只读显示，不参与控制 |
| 增加统计量 | `ScopeBuffer.snapshot()` | 可基于窗口内样本计算 |
| 支持新 topic | `topic` 参数 | 仍建议使用明确的消息字段约定 |
