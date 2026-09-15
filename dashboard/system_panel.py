"""System Panel — sensor health status indicators."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS, status_dot


class StatusLight(QWidget):
    """Single sensor status indicator with label and dot."""

    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)

        self.dot = QLabel()
        self.dot.setFixedWidth(20)
        self.dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.dot)

        self.name_label = QLabel(name)
        self.name_label.setStyleSheet(f"color: {COLORS['text']};")
        layout.addWidget(self.name_label)

        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px;")
        layout.addStretch()
        layout.addWidget(self.status_label)

        self.set_offline()

    def set_online(self, text: str = "Online"):
        self.dot.setText(status_dot(COLORS["green"]))
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {COLORS['green']}; font-size: 11px;")

    def set_offline(self, text: str = "Disconnected"):
        self.dot.setText(status_dot(COLORS["red"]))
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {COLORS['red']}; font-size: 11px;")

    def set_warning(self, text: str = "Warning"):
        self.dot.setText(status_dot(COLORS["yellow"]))
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {COLORS['yellow']}; font-size: 11px;")


class SystemPanel(QWidget):
    """Panel showing health of all sensors."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("System Health")
        group_layout = QVBoxLayout(group)

        self.zed = StatusLight("ZED 2 Camera")
        self.imu = StatusLight("MTi-8 IMU")
        self.gnss = StatusLight("GNSS/RTK")
        self.radar = StatusLight("AWR1843 Radar")
        self.lidar = StatusLight("Ouster OS-0")
        self.recorder = StatusLight("Recorder")
        self.ssd = StatusLight("SSD")
        self.sync = StatusLight("Synchronizer")

        for light in [self.zed, self.imu, self.gnss, self.radar, self.lidar,
                      self.recorder, self.ssd, self.sync]:
            group_layout.addWidget(light)

        group_layout.addStretch()
        layout.addWidget(group)
