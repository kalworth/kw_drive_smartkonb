from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import sys
import threading
import time

from PyQt5.QtCore import QPointF, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class ScopeBuffer:
    window_sec: float
    start_time: float | None = None
    t: deque[float] = field(default_factory=deque)
    position: deque[float] = field(default_factory=deque)
    velocity: deque[float] = field(default_factory=deque)
    current: deque[float] = field(default_factory=deque)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def append(self, stamp: float, position: float, velocity: float, current: float) -> None:
        with self.lock:
            if self.start_time is None:
                self.start_time = stamp
            relative_t = stamp - self.start_time
            self.t.append(relative_t)
            self.position.append(position)
            self.velocity.append(velocity)
            self.current.append(current)

            cutoff = relative_t - self.window_sec
            while self.t and self.t[0] < cutoff:
                self.t.popleft()
                self.position.popleft()
                self.velocity.popleft()
                self.current.popleft()

    def snapshot(self) -> tuple[list[float], list[float], list[float], list[float]]:
        with self.lock:
            return (
                list(self.t),
                list(self.position),
                list(self.velocity),
                list(self.current),
            )


class KwScopeNode(Node):
    def __init__(self) -> None:
        super().__init__("kw_scope")
        self.declare_parameter("topic", "/kw_motor/state")
        self.declare_parameter("window_sec", 5.0)
        self.declare_parameter("refresh_hz", 30.0)

        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.window_sec = self._positive_double("window_sec", 5.0)
        self.refresh_hz = self._positive_double("refresh_hz", 30.0)
        self.buffer = ScopeBuffer(window_sec=self.window_sec)

        self.create_subscription(JointState, self.topic, self._on_state, 20)
        self.get_logger().info(
            f"Listening on {self.topic}, window={self.window_sec:.2f}s, "
            f"refresh_hz={self.refresh_hz:.1f}; every feedback sample is buffered"
        )

    def _positive_double(self, name: str, fallback: float) -> float:
        value = self.get_parameter(name).get_parameter_value().double_value
        if not math.isfinite(value) or value <= 0.0:
            self.get_logger().warn(f"{name} must be > 0, using {fallback}")
            return fallback
        return value

    def _on_state(self, msg: JointState) -> None:
        if not msg.position or not msg.velocity or not msg.effort:
            return

        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            stamp = time.monotonic()

        self.buffer.append(stamp, msg.position[0], msg.velocity[0], msg.effort[0])


class TraceWidget(QWidget):
    def __init__(self, label: str, color: QColor, window_sec: float, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._label = label
        self._color = color
        self._window_sec = window_sec
        self._t: list[float] = []
        self._y: list[float] = []
        self.setMinimumHeight(150)

    def set_series(self, t: list[float], y: list[float]) -> None:
        self._t = t
        self._y = y
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(18, 22, 28))
        painter.setRenderHint(QPainter.Antialiasing, False)

        left = 62
        right = 10
        top = 10
        bottom = 24
        width = max(1, self.width() - left - right)
        height = max(1, self.height() - top - bottom)

        grid_pen = QPen(QColor(58, 65, 75), 1)
        painter.setPen(grid_pen)
        for i in range(6):
            x = left + width * i / 5
            painter.drawLine(int(x), top, int(x), top + height)
        for i in range(4):
            y = top + height * i / 3
            painter.drawLine(left, int(y), left + width, int(y))

        painter.setPen(QPen(QColor(210, 215, 220), 1))
        painter.drawText(8, 22, self._label)

        if len(self._t) < 2:
            painter.drawText(left + 12, top + 28, "waiting for /kw_motor/state")
            return

        x_max = self._t[-1]
        x_min = max(0.0, x_max - self._window_sec)
        y_min = min(self._y)
        y_max = max(self._y)
        if abs(y_max - y_min) < 1e-9:
            y_min -= 1e-3
            y_max += 1e-3
        else:
            pad = (y_max - y_min) * 0.12
            y_min -= pad
            y_max += pad

        painter.drawText(8, top + 42, f"{y_max:.3f}")
        painter.drawText(8, top + height, f"{y_min:.3f}")

        points = QPolygonF()
        inv_x = width / max(self._window_sec, 1e-9)
        inv_y = height / max(y_max - y_min, 1e-9)
        for stamp, value in zip(self._t, self._y):
            if stamp < x_min:
                continue
            x = left + (stamp - x_min) * inv_x
            y = top + height - (value - y_min) * inv_y
            points.append(QPointF(x, y))

        painter.setPen(QPen(self._color, 2))
        painter.drawPolyline(points)


class ScopeWindow(QMainWindow):
    def __init__(self, node: KwScopeNode) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle("KW Drive Scope")

        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self._widgets = [
            TraceWidget("position", QColor(70, 150, 255), node.window_sec),
            TraceWidget("velocity", QColor(70, 210, 130), node.window_sec),
            TraceWidget("iq", QColor(255, 95, 95), node.window_sec),
        ]
        for widget in self._widgets:
            layout.addWidget(widget)
        self.setCentralWidget(root)
        self.resize(980, 620)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(max(10, int(1000.0 / node.refresh_hz)))

    def _refresh(self) -> None:
        t, position, velocity, current = self._node.buffer.snapshot()
        self._widgets[0].set_series(t, position)
        self._widgets[1].set_series(t, velocity)
        self._widgets[2].set_series(t, current)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = None
    app = None
    try:
        node = KwScopeNode()
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        app = QApplication(sys.argv[:1])
        window = ScopeWindow(node)
        window.show()
        app.exec_()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
