"""Recording Timeline Panel — visual timeline with event markers."""

from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QPen, QColor
from PySide6.QtWidgets import (
    QGroupBox, QPushButton, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS


class TimelineWidget(QWidget):
    """Custom painted timeline showing recording duration and event markers."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(60)
        self.setMaximumHeight(80)
        self._duration = 0.0
        self._events = []
        self._is_recording = False

    def set_duration(self, seconds: float):
        self._duration = seconds
        self.update()

    def set_recording(self, recording: bool):
        self._is_recording = recording
        self.update()

    def add_event(self, timestamp: float, label: str = "", color: str = None):
        self._events.append({"time": timestamp, "label": label, "color": color})
        self.update()

    def clear_events(self):
        self._events.clear()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        bar_y = h // 2 - 5
        bar_h = 10

        painter.setPen(QPen(QColor(COLORS["border"]), 1))
        painter.setBrush(QColor(COLORS["bg_dark"]))
        painter.drawRoundedRect(0, bar_y, w, bar_h, 4, 4)

        if self._is_recording:
            painter.setBrush(QColor(COLORS["red"]))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRoundedRect(0, bar_y, w, bar_h, 4, 4)

        if self._duration > 0:
            for ev in self._events:
                x = int(ev["time"] / self._duration * w) if self._duration > 0 else 0
                x = max(0, min(x, w - 2))

                c = QColor(ev.get("color") or COLORS["yellow"])
                painter.setPen(QPen(c, 2))
                painter.drawLine(x, bar_y - 8, x, bar_y + bar_h + 8)

                if ev["label"]:
                    painter.setPen(QPen(c, 1))
                    painter.drawText(x + 3, bar_y - 10, ev["label"])

        painter.setPen(QPen(QColor(COLORS["text_dim"]), 1))
        painter.drawText(4, h - 4, f"{self._duration:.0f}s")
        if self._events:
            painter.drawText(w - 60, h - 4, f"{len(self._events)} events")

        painter.end()


class TimelinePanel(QWidget):
    """Recording timeline with event marking controls."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("Recording Timeline")
        g = QVBoxLayout(group)

        self.timeline = TimelineWidget()
        g.addWidget(self.timeline)

        btn_row = QVBoxLayout()
        self.mark_btn = QPushButton("Mark Event")
        self.mark_btn.clicked.connect(self._on_mark)
        btn_row.addWidget(self.mark_btn)
        g.addLayout(btn_row)

        layout.addWidget(group)

        self._events = []
        self._recording = False
        self._start_time = 0

    def set_recording(self, recording: bool, start_time: float = 0):
        self._recording = recording
        self._start_time = start_time
        self.timeline.set_recording(recording)

    def update_duration(self, seconds: float):
        self.timeline.set_duration(seconds)

    def _on_mark(self):
        import time
        ts = time.time() - self._start_time if self._start_time else 0
        self._events.append({"time": ts, "label": f"E{len(self._events)+1}"})
        self.timeline.add_event(ts, f"E{len(self._events)}")

    def add_event(self, timestamp: float, label: str = ""):
        self._events.append({"time": timestamp, "label": label})
        self.timeline.add_event(timestamp, label)

    def clear(self):
        self._events.clear()
        self.timeline.clear_events()
