"""Recorder Panel — recording stats, controls, and dataset info."""

from __future__ import annotations
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QSpacerItem, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS


class StatLabel(QWidget):
    """Label with a name and value on the same row."""

    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.name_label = QLabel(name)
        self.name_label.setStyleSheet(f"color: {COLORS['text_dim']};")
        self.value_label = QLabel("0")
        self.value_label.setStyleSheet(f"color: {COLORS['text_bright']}; font-weight: bold;")
        layout.addWidget(self.name_label)
        layout.addStretch()
        layout.addWidget(self.value_label)

    def set_value(self, value: str, color: str = None):
        self.value_label.setText(value)
        if color:
            self.value_label.setStyleSheet(f"color: {color}; font-weight: bold;")


class RecorderPanel(QWidget):
    """Recording controls and statistics display."""

    sig_start = Signal()
    sig_stop = Signal()
    sig_snapshot = Signal()
    sig_mark = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("Recording")
        group_layout = QVBoxLayout(group)

        btn_row = QHBoxLayout()
        self.btn_record = QPushButton("REC")
        self.btn_record.setMinimumHeight(40)
        self.btn_record.clicked.connect(self._on_record)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setMinimumHeight(40)
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self.sig_stop.emit)
        self.btn_snapshot = QPushButton("SNAP")
        self.btn_snapshot.setMinimumHeight(40)
        self.btn_snapshot.clicked.connect(self.sig_snapshot.emit)
        self.btn_mark = QPushButton("MARK")
        self.btn_mark.setMinimumHeight(40)
        self.btn_mark.clicked.connect(self.sig_mark.emit)

        btn_row.addWidget(self.btn_record)
        btn_row.addWidget(self.btn_stop)
        btn_row.addWidget(self.btn_snapshot)
        btn_row.addWidget(self.btn_mark)
        group_layout.addLayout(btn_row)

        self.recording_indicator = QLabel("IDLE")
        self.recording_indicator.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.recording_indicator.setStyleSheet(
            f"color: {COLORS['text_dim']}; font-size: 16px; font-weight: bold; padding: 4px;"
        )
        group_layout.addWidget(self.recording_indicator)

        stats = QGroupBox("Statistics")
        stats_layout = QVBoxLayout(stats)
        self.stat_frame_count = StatLabel("Frames (stereo pairs)")
        self.stat_imu_count = StatLabel("MTi samples (HDF5)")
        self.stat_sync_err = StatLabel("Sync Error")
        self.stat_dropped = StatLabel("Dropped Frames")
        self.stat_mti_drops = StatLabel("MTi drops")
        self.stat_write_latency = StatLabel("Image Queue")
        self.stat_queue = StatLabel("HDF5 Queue")
        self.stat_disk = StatLabel("Disk Free")
        self.stat_duration = StatLabel("Duration")
        self.stat_size = StatLabel("Dataset Size")
        self.stat_session = StatLabel("Session ID")
        self.stat_root = StatLabel("Dataset Root")

        for s in [self.stat_frame_count, self.stat_imu_count, self.stat_sync_err,
                  self.stat_dropped, self.stat_mti_drops,
                  self.stat_write_latency,
                  self.stat_queue, self.stat_disk, self.stat_duration, self.stat_size,
                  self.stat_session, self.stat_root]:
            stats_layout.addWidget(s)

        group_layout.addWidget(stats)
        layout.addWidget(group)
        layout.addSpacerItem(QSpacerItem(0, 0, QSizePolicy.Policy.Minimum,
                                         QSizePolicy.Policy.Expanding))

        self._is_recording = False

    def _on_record(self):
        if self._is_recording:
            self.sig_stop.emit()
        else:
            self.sig_start.emit()

    def set_recording(self, recording: bool):
        self._is_recording = recording
        if recording:
            self.btn_record.setText("PAUSE")
            self.btn_record.setProperty("recording", True)
            self.btn_record.style().unpolish(self.btn_record)
            self.btn_record.style().polish(self.btn_record)
            self.btn_stop.setEnabled(True)
            self.recording_indicator.setText("● RECORDING")
            self.recording_indicator.setStyleSheet(
                f"color: {COLORS['red']}; font-size: 16px; font-weight: bold; padding: 4px;"
            )
        else:
            self.btn_record.setText("REC")
            self.btn_record.setProperty("recording", False)
            self.btn_record.style().unpolish(self.btn_record)
            self.btn_record.style().polish(self.btn_record)
            self.btn_stop.setEnabled(False)
            self.recording_indicator.setText("IDLE")
            self.recording_indicator.setStyleSheet(
                f"color: {COLORS['text_dim']}; font-size: 16px; font-weight: bold; padding: 4px;"
            )

    def update_stats(self, frames: int = 0, sync_err_ms: float = 0.0,
                     queue_size: int = 0, disk_free_gb: float = 0.0,
                     duration_s: float = 0.0, size_mb: float = 0.0,
                     imu_packets: int = 0, dropped: int = 0,
                     avg_write_ms: float = 0.0,
                     hdf5_queue: int = 0, mti_drops: int = 0,
                     session_id: str = "", dataset_root: str = ""):
        self.stat_frame_count.set_value(str(frames))
        self.stat_imu_count.set_value(str(imu_packets))
        self.stat_sync_err.set_value(f"{sync_err_ms:.2f} ms")
        drop_color = COLORS["green"] if dropped == 0 else COLORS["red"]
        self.stat_dropped.set_value(str(dropped), drop_color)
        mti_drop_color = COLORS["green"] if mti_drops == 0 else COLORS["red"]
        self.stat_mti_drops.set_value(str(mti_drops), mti_drop_color)
        self.stat_write_latency.set_value(str(queue_size))
        self.stat_queue.set_value(str(hdf5_queue))
        self.stat_disk.set_value(f"{disk_free_gb:.1f} GB")
        m, s = divmod(int(duration_s), 60)
        self.stat_duration.set_value(f"{m:02d}:{s:02d}")
        self.stat_size.set_value(f"{size_mb:.1f} MB")
        self.stat_session.set_value(session_id or "none")
        self.stat_root.set_value(dataset_root or "none")
