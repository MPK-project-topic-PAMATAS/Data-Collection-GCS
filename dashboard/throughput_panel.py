"""Throughput & System Diagnostics Panel — sensor rates, disk, CPU/RAM/GPU."""

import shutil
import subprocess
import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox, QGridLayout, QLabel, QProgressBar, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


class StatRow(QWidget):
    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 1, 0, 1)
        self.name = QLabel(name)
        self.name.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px;")
        self.value = QLabel("—")
        self.value.setStyleSheet(f"color: {COLORS['text_bright']}; font-weight: bold;")
        self.value.setAlignment(Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self.name, 0, 0)
        layout.addWidget(self.value, 0, 1)

    def set(self, text: str, color: str = None):
        self.value.setText(text)
        if color:
            self.value.setStyleSheet(f"color: {color}; font-weight: bold;")


def _get_gpu_usage() -> tuple:
    """Try nvidia-smi for GPU util/memory/temp. Returns (util%, mem_mb, temp°C)."""
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,temperature.gpu",
             "--format=csv,noheader,nounits"],
            timeout=1, stderr=subprocess.DEVNULL,
        ).decode().strip().split(", ")
        return float(out[0]), float(out[1]), float(out[2])
    except Exception:
        return None


def _color_pct(val: float) -> str:
    if val > 85:
        return COLORS["red"]
    elif val > 60:
        return COLORS["yellow"]
    return COLORS["green"]


class ThroughputPanel(QWidget):
    """Recording throughput and system diagnostics."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rec_start = 0
        self._bytes_written = 0
        self._last_bytes = 0
        self._last_time = 0
        self._write_rate = 0.0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("Throughput & System")
        g = QVBoxLayout(group)

        grid = QGridLayout()
        grid.setSpacing(4)

        # Sensor rates
        self.cam_fps = StatRow("Camera FPS")
        self.mti_fps = StatRow("MTi FPS")
        self.zed_imu_fps = StatRow("ZED IMU FPS")

        # Disk
        self.disk_mb = StatRow("Disk MB/s")
        self.write_lat = StatRow("Avg write time")
        self.dropped = StatRow("Dropped frames")

        # System
        self.cpu = StatRow("CPU")
        self.ram = StatRow("RAM")
        self.gpu_util = StatRow("GPU util")
        self.gpu_mem = StatRow("GPU memory")
        self.temp = StatRow("Temperature")
        self.disk_pct = StatRow("Disk usage")

        rows = [self.cam_fps, self.mti_fps, self.zed_imu_fps,
                self.disk_mb, self.write_lat, self.dropped,
                self.cpu, self.ram, self.gpu_util, self.gpu_mem,
                self.temp, self.disk_pct]
        for i, row in enumerate(rows):
            grid.addWidget(row, i, 0)

        g.addLayout(grid)

        self.disk_bar = QProgressBar()
        self.disk_bar.setMaximumHeight(12)
        self.disk_bar.setRange(0, 100)
        self.disk_bar.setValue(0)
        self.disk_bar.setFormat("%v% used")
        g.addWidget(self.disk_bar)

        layout.addWidget(group)

        self._prev_sys_time = 0
        self._sys_interval = 1.0

    def start_recording(self, path: str = "/"):
        self._rec_start = time.time()
        self._bytes_written = 0
        self._last_bytes = 0
        self._last_time = time.time()
        self._disk_path = path

    def update(self, frames: int = 0, bytes_written: int = 0,
               queue_size: int = 0, queue_peak: int = 0,
               imu_rate: float = 0.0, camera_fps: float = 0.0,
               zed_imu_rate: float = 0.0, dropped: int = 0,
               avg_write_ms: float = 0.0):
        now = time.time()
        dt = now - self._last_time if self._last_time else 1.0

        if dt > 0.5:
            self._write_rate = (bytes_written - self._last_bytes) / dt / (1024 * 1024)
            self._last_bytes = bytes_written
            self._last_time = now

        self.cam_fps.set(f"{camera_fps:.1f}", _color_pct(camera_fps / 30 * 100))
        self.mti_fps.set(f"{imu_rate:.0f}", _color_pct(imu_rate / 200 * 100))
        self.zed_imu_fps.set(f"{zed_imu_rate:.0f}")
        self.disk_mb.set(f"{self._write_rate:.1f} MB/s")
        self.write_lat.set(f"{avg_write_ms:.1f} ms")
        drop_c = COLORS["green"] if dropped == 0 else COLORS["red"]
        self.dropped.set(str(dropped), drop_c)

        # System stats (throttled to every 1s)
        if now - self._prev_sys_time >= self._sys_interval:
            self._prev_sys_time = now
            self._update_system_stats()

    def _update_system_stats(self):
        if HAS_PSUTIL:
            cpu = psutil.cpu_percent(interval=0)
            self.cpu.set(f"{cpu:.0f}%", _color_pct(cpu))

            mem = psutil.virtual_memory()
            self.ram.set(f"{mem.percent:.0f}% ({mem.used / 1024**3:.1f} GB)",
                         _color_pct(mem.percent))
        else:
            self.cpu.set("psutil N/A")
            self.ram.set("psutil N/A")

        gpu = _get_gpu_usage()
        if gpu:
            util, mem_mb, temp = gpu
            self.gpu_util.set(f"{util:.0f}%", _color_pct(util))
            self.gpu_mem.set(f"{mem_mb:.0f} MB")
            self.temp.set(f"{temp:.0f}°C", _color_pct(temp / 90 * 100))
        else:
            self.gpu_util.set("N/A")
            self.gpu_mem.set("N/A")
            self.temp.set("N/A")

        try:
            disk_path = getattr(self, "_disk_path", "/")
            usage = shutil.disk_usage(disk_path)
            used_pct = int((usage.used / usage.total) * 100)
            self.disk_pct.set(f"{used_pct}%", _color_pct(used_pct))
            self.disk_bar.setValue(used_pct)
        except Exception:
            pass
