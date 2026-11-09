"""Main Window v4 — strict 3-tier grid layout with dark-mode dashboard.

Tier 1 (top):    Stereo + Depth feeds — 4 equal columns
Tier 2 (middle): IMU panels + 3D viewport — [25% | 50% | 25%]
Tier 3 (bottom): System health, quality, recording, log — 4 columns
"""

from __future__ import annotations
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication, QHBoxLayout, QMainWindow,
    QPushButton, QVBoxLayout, QWidget,
)

from dashboard.theme import STYLESHEET, COLORS
from dashboard.camera_panel import CameraPanel
from dashboard.imu_panel import IMUPanel
from dashboard.recorder_panel import RecorderPanel
from dashboard.system_panel import SystemPanel
from dashboard.log_panel import LogPanel
from dashboard.quality_panel import QualityPanel
from dashboard.radar_panel import RadarPanel
from dashboard.map_panel import MapPanel, HDOP_TO_METERS
from dashboard.heading import GNSSYawCorrector
from dashboard.acquisition import SharedFrame
from config.settings import (
    MTI_YAW_GNSS_CORRECTION_ENABLED,
    MTI_YAW_GNSS_MIN_SPEED_MPS,
    MTI_YAW_CORRECTION_GAIN,
)


class MainWindow(QMainWindow):
    """Ground Control Station main window — 3-tier grid layout."""

    def __init__(self, shared: SharedFrame):
        super().__init__()
        self.shared = shared

        self.setWindowTitle("Stereo+MTi-8 Ground Control Station")
        self.setMinimumSize(1600, 1000)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        self.camera_panel = CameraPanel()
        self.imu_panel = IMUPanel()
        self.system_panel = SystemPanel()
        self.recorder_panel = RecorderPanel()
        self.quality_panel = QualityPanel()
        self.radar_panel = RadarPanel()
        self.log_panel = LogPanel()

        # OSM minimap in the bottom-right corner (floating overlay)
        self.map_panel = MapPanel(self)
        self.map_panel.raise_()

        self.recorder_panel.sig_start.connect(self._on_start)
        self.recorder_panel.sig_stop.connect(self._on_stop)
        self.recorder_panel.sig_snapshot.connect(self._on_snapshot)
        self.recorder_panel.sig_mark.connect(self._on_mark)

        # Depth / confidence toggle for bottom-right camera slot
        self._depth_view_mode = "disp"
        self._depth_view_btn = QPushButton("Confidence")
        self._depth_view_btn.setFixedHeight(24)
        self._depth_view_btn.setStyleSheet(
            f"background-color: {COLORS['bg_elevated']}; color: {COLORS['zed']};"
            f"font-size: 10px; font-weight: 600; padding: 2px 12px;"
            f"border: 1px solid {COLORS['border']}; border-radius: 6px;"
        )
        self._depth_view_btn.clicked.connect(self._toggle_depth_view)

        # ── Tier 1: Stereo + Depth feeds ────────────────────────────
        tier1_btn_row = QHBoxLayout()
        tier1_btn_row.addStretch(1)
        tier1_btn_row.addWidget(self._depth_view_btn)
        main_layout.addLayout(tier1_btn_row)
        main_layout.addWidget(self.camera_panel, stretch=4)

        # ── Tier 2: IMU left | Radar (large) right ─────────────────
        tier2 = QHBoxLayout()
        tier2.setSpacing(4)
        tier2.addWidget(self.imu_panel, stretch=1)
        tier2.addWidget(self.radar_panel, stretch=4)
        main_layout.addLayout(tier2, stretch=5)

        # ── Tier 3: Health | Quality | Recording | Log ──────────────
        tier3 = QHBoxLayout()
        tier3.setSpacing(4)
        tier3.addWidget(self.system_panel, stretch=1)
        tier3.addWidget(self.quality_panel, stretch=1)
        tier3.addWidget(self.recorder_panel, stretch=1)
        tier3.addWidget(self.log_panel, stretch=2)
        main_layout.addLayout(tier3, stretch=1)

        self.setStyleSheet(STYLESHEET)

        # Rate-limited timers (ms interval → Hz)
        self._camera_timer = QTimer()
        self._camera_timer.timeout.connect(self._update_camera)
        self._camera_timer.start(66)   # ~15 Hz

        self._imu_timer = QTimer()
        self._imu_timer.timeout.connect(self._update_imu)
        self._imu_timer.start(66)      # ~15 Hz

        self._stats_timer = QTimer()
        self._stats_timer.timeout.connect(self._update_stats)
        self._stats_timer.start(500)   # 2 Hz

        self._quality_timer = QTimer()
        self._quality_timer.timeout.connect(self._update_quality)
        self._quality_timer.start(200) # 5 Hz

        self._radar_timer = QTimer()
        self._radar_timer.timeout.connect(self._update_radar)
        self._radar_timer.start(100)   # 10 Hz — radar runs ~20Hz

        self._mti_att_roll = 0.0
        self._mti_att_pitch = 0.0
        self._mti_att_yaw = 0.0
        self._mti_last_time = None
        self._radar_last_frame = None

        self._yaw_corr_enabled = MTI_YAW_GNSS_CORRECTION_ENABLED
        self._yaw_corr = GNSSYawCorrector(
            gain=MTI_YAW_CORRECTION_GAIN,
            min_speed=MTI_YAW_GNSS_MIN_SPEED_MPS,
        )

        self.log_panel.log("GCS v4 initialized", "success")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        m = 8
        self.map_panel.move(self.width() - self.map_panel.width() - m,
                            self.height() - self.map_panel.height() - m)
        self.map_panel.raise_()

    def _update_camera(self):
        s = self.shared
        left = s.left_rgb
        right = s.right_rgb
        depth = s.depth
        fps = s.camera_fps
        self.camera_panel.update_frames(
            left_rgb=left, right_rgb=right,
            depth=depth, left_fps=fps,
            view_depth=s.view_depth,
            view_confidence=s.view_confidence,
            disparity=s.disparity,
            valid_pct=s.valid_depth_pct,
            invalid_pct=s.invalid_depth_pct,
            mean_conf=s.mean_confidence,
            median_d=s.median_depth,
        )
        self.camera_panel.update_lidar(
            points=s.lidar_points,
            num_points=s.lidar_num_points,
            rate=s.lidar_rate,
            streaming=s.lidar_streaming,
            frame_id=s.lidar_frame_id,
            matched=s.lidar_matched,
            sync_delta_ms=s.lidar_sync_delta_ms,
        )

    def _update_imu(self):
        s = self.shared
        quat = list(s.quat)
        accel = list(s.accel)
        gyro = list(s.gyro)
        stale = getattr(s, 'mti_stale', False)

        r, p, y = self._mti_euler(quat)
        self._mti_att_roll, self._mti_att_pitch, self._mti_att_yaw = r, p, y
        self.imu_panel.update_data(
            accel=accel, gyro=gyro,
            roll=r, pitch=p, yaw=y, quat=quat, stale=stale,
        )
        # ZED IMU displayed for diagnostic comparison only
        zed_quat = list(s.zed_quat)
        zed_accel = list(s.zed_accel)
        zed_gyro = list(s.zed_gyro)
        zr, zp, zy = _zed_quat_to_euler(zed_quat)
        self.imu_panel.update_zed_data(
            accel=zed_accel, gyro=zed_gyro,
            roll=zr, pitch=zp, yaw=zy, quat=zed_quat,
            stale=getattr(s, 'zed_imu_stale', False),
        )

    def _update_stats(self):
        s = self.shared
        zed_quat = list(s.zed_quat)
        quat = list(s.quat)
        frame_count = s.frame_count
        camera_fps = s.camera_fps
        zr, zp, zy = _zed_quat_to_euler(zed_quat)

        mti_has_quat = quat != [0, 0, 0, 1]
        if mti_has_quat:
            r, p, y = self._mti_euler(quat)
        else:
            r, p, y = self._mti_update(s.accel, s.gyro)
            y = self._correct_yaw(y)
        self._mti_att_roll, self._mti_att_pitch, self._mti_att_yaw = r, p, y

        self.recorder_panel.set_recording(s.recording)
        self.recorder_panel.update_stats(
            frames=frame_count,
            sync_err_ms=s.sync_error_ms,
            queue_size=s.write_queue_size,
            disk_free_gb=s.disk_free_gb,
            duration_s=s.recording_duration_s,
            size_mb=s.dataset_size_mb,
            imu_packets=s.mti_samples_written,
            dropped=s.dropped_frames,
            hdf5_queue=s.hdf5_queue_depth,
            mti_drops=s.mti_drops,
            session_id=s.session_id,
            dataset_root=s.dataset_root,
        )

        sp = self.system_panel
        if frame_count > 0:
            sp.zed.set_online(f"{camera_fps:.0f} FPS")
        else:
            sp.zed.set_offline()
        sp.imu.set_online("200 Hz") if s.imu_packets > 0 else sp.imu.set_offline()
        sp.gnss.set_online("Connected") if frame_count > 0 else sp.gnss.set_offline()
        if s.radar_streaming:
            if s.radar_frame_id > 0 and s.radar_num_objects > 0:
                sp.radar.set_online(f"{s.radar_num_objects} det")
            else:
                sp.radar.set_online(f"{s.radar_rate:.1f} Hz")
        else:
            sp.radar.set_offline()

        if s.lidar_streaming:
            sp.lidar.set_online(f"{s.lidar_rate:.1f} Hz")
        else:
            sp.lidar.set_offline()

        if s.gnss_fix >= 1 and (s.lat != 0.0 or s.lon != 0.0):
            if s.rtk_fixed:
                label = f"RTK FIXED · {s.gnss_satellites} sat"
                color = COLORS["green"]
            elif s.rtk_float:
                label = f"RTK FLOAT · {s.gnss_satellites} sat"
                color = COLORS["yellow"]
            else:
                label = f"GPS FIX · {s.gnss_satellites} sat"
                color = COLORS["zed"]
            self.map_panel.set_position(
                s.lat, s.lon, label, color,
                accuracy_m=s.gnss_hdop * HDOP_TO_METERS,
            )
        else:
            self.map_panel.set_position(None, None, "No GNSS fix", COLORS["red"])
        if s.recording:
            sp.recorder.set_online("Recording")
        else:
            sp.recorder.set_offline()
        if s.disk_free_gb > 10:
            sp.ssd.set_online("OK")
        else:
            sp.ssd.set_warning("Low Space")
        if s.sync_error_ms < 50:
            sp.sync.set_online(f"{s.sync_error_ms:.1f} ms")
        else:
            sp.sync.set_offline()

    def _update_quality(self):
        s = self.shared
        left = s.left_rgb
        dropped = s.dropped_frames
        self.quality_panel.update_from_frame(left, dropped=dropped)

    def _update_radar(self):
        s = self.shared
        # Radar produces ~6.6 Hz but this timer ticks at 10 Hz.  Skip
        # re-rendering until a new radar frame actually arrives so the panel
        # isn't re-drawn with identical data ~1/3 of the time.
        frame_id = s.radar_frame_id
        if frame_id == getattr(self, "_radar_last_frame", None):
            return
        self._radar_last_frame = frame_id
        self.radar_panel.update(
            points=s.radar_points,
            range_azimuth=s.radar_range_azimuth,
            radar_ts_ns=s.radar_ts_ns,
            radar_frame_id=s.radar_frame_id,
            num_objects=s.radar_num_objects,
            camera_ts_ns=s.timestamp_ns,
            camera_rgb=s.left_rgb,
            lidar_points=s.lidar_points,
        )

    def _mti_euler(self, quat):
        """Euler from MTi quaternion with GNSS-course yaw correction."""
        r, p, y = _mti_quat_to_euler(quat)
        return r, p, self._correct_yaw(y)

    def _correct_yaw(self, yaw):
        s = self.shared
        if self._yaw_corr_enabled and s.gnss_fix >= 1:
            return self._yaw_corr.correct(
                yaw, s.gnss_course, s.gnss_speed, time.monotonic()
            )
        return yaw

    def _mti_update(self, accel, gyro):
        if not accel or not gyro:
            return self._mti_att_roll, self._mti_att_pitch, self._mti_att_yaw
        ax, ay, az = accel
        gx, gy, gz = gyro
        now = time.monotonic()
        dt = now - self._mti_last_time if self._mti_last_time else 0.01
        self._mti_last_time = now
        dt = min(dt, 0.05)
        alpha = 0.98
        ar = math.degrees(math.atan2(ay, az))
        ap = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        self._mti_att_roll = alpha * (self._mti_att_roll + gx * dt) + (1 - alpha) * ar
        self._mti_att_pitch = alpha * (self._mti_att_pitch + gy * dt) + (1 - alpha) * ap
        self._mti_att_yaw += gz * dt
        return self._mti_att_roll, self._mti_att_pitch, self._mti_att_yaw

    def _on_start(self):
        self.shared.recording = True
        self.shared.recording_start = time.time()
        self.log_panel.log("Recording started", "success")

    def _on_stop(self):
        self.shared.recording = False
        self.log_panel.log("Recording stopped", "warning")

    def _on_snapshot(self):
        self.log_panel.log("Snapshot saved", "info")

    def _on_mark(self):
        ts = time.time()
        self.log_panel.log(f"Event marked at {ts:.1f}", "info")

    def _toggle_depth_view(self):
        if self._depth_view_mode == "disp":
            self._depth_view_mode = "confidence"
            self._depth_view_btn.setText("Disparity")
        else:
            self._depth_view_mode = "disp"
            self._depth_view_btn.setText("Confidence")
        self.camera_panel.set_bottom_mode(self._depth_view_mode)


def _mti_quat_to_euler(q):
    """MTi-8 XSens quaternion (w,x,y,z) → (roll, pitch, yaw) degrees."""
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def _zed_quat_to_euler(q):
    """ZED SDK quaternion (x,y,z,w) → (roll, pitch, yaw) degrees."""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def run_demo():
    """Run GCS with simulated data for testing."""
    import cv2
    app = QApplication.instance() or QApplication(sys.argv)

    shared = SharedFrame()
    shared.frame_count = 0

    window = MainWindow(shared)
    window.show()

    t0 = time.time()
    _frame_count = [0]

    def simulate():
        t = time.time() - t0
        _frame_count[0] += 1

        left = numpy_random_stereo(t)
        right = numpy_random_stereo(t + 0.01)
        depth = numpy_random_depth(t)

        shared.left_rgb = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
        shared.right_rgb = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)
        shared.depth = depth
        shared.frame_id = _frame_count[0]
        shared.frame_count = _frame_count[0]
        shared.camera_fps = 29.5 + 0.5 * math.sin(t)

        # Simulated OS-0 point cloud (ring of returns around the vehicle)
        shared.lidar_streaming = True
        shared.lidar_rate = 10.0
        shared.lidar_frame_id = _frame_count[0] // 3
        shared.lidar_num_points = 20000
        az = np.linspace(0, 2 * math.pi, 20000)
        rr = 20.0 + 3.0 * np.sin(3 * az + t)
        shared.lidar_points = np.stack(
            [rr * np.cos(az), rr * np.sin(az),
             np.clip(np.sin(az * 4) * 1.5, -2, 3)],
            axis=1,
        ).astype(np.float32)

        cr = math.radians(5.0 * math.sin(0.1 * t))
        cp = math.radians(3.0 * math.cos(0.15 * t))
        cy = math.radians(30.0 * math.sin(0.05 * t))
        sc, cc = math.sin(cr/2), math.cos(cr/2)
        sp_, cp_ = math.sin(cp/2), math.cos(cp/2)
        sy, cy_ = math.sin(cy/2), math.cos(cy/2)
        shared.quat = [
            sc*cp_*cy_ - cc*sp_*sy,
            cc*sp_*cy_ + sc*cp_*sy,
            cc*cp_*sy - sc*sp_*cy_,
            cc*cp_*cy_ + sc*sp_*sy,
        ]
        shared.zed_quat = shared.quat
        shared.accel = [
            0.1 * math.sin(0.5 * t),
            0.1 * math.cos(0.3 * t),
            9.81 + 0.05 * math.sin(0.7 * t),
        ]
        shared.gyro = [
            0.02 * math.sin(0.2 * t),
            0.03 * math.cos(0.4 * t),
            0.01 * math.sin(0.6 * t),
        ]
        shared.sync_error_ms = 1.5 + 0.5 * math.sin(0.3 * t)
        shared.imu_rate = 100.0 + 5 * math.sin(t)
        shared.imu_packets = _frame_count[0] * 3
        shared.bytes_written = _frame_count[0] * 100 * 1024
        shared.disk_free_gb = 238.4

        # Simulated GNSS: plain GPS fix until t=5, then RTK fixed
        shared.lat = 48.2082 + 0.00004 * math.sin(0.1 * t)
        shared.lon = 16.3738 + 0.00004 * math.cos(0.1 * t)
        shared.alt = 200.0
        shared.gnss_satellites = 22
        shared.gnss_hdop = 0.8
        shared.ntrip_connected = t > 3
        shared.rtk_fixed = t > 5
        shared.rtk_float = (t > 3) and not (t > 5)
        shared.gnss_fix = 4 if t > 5 else (5 if t > 3 else (1 if t > 1 else 0))
        shared.gnss_speed = 5.0
        shared.gnss_course = 30.0 * math.sin(0.05 * t)

    _rng = np.random.default_rng(42)

    def numpy_random_stereo(t):
        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        for y in range(0, 360, 20):
            v = int(128 + 60 * math.sin(y * 0.05 + t))
            frame[y:y+10, :] = v
        return frame

    def numpy_random_depth(t):
        d = np.full((360, 640), 5.0, dtype=np.float32)
        for x in range(0, 640, 64):
            d[:, x:x+32] = 2.0 + 3.0 * math.sin(x * 0.01 + t * 0.5)
        return d

    timer = QTimer()
    timer.timeout.connect(simulate)
    timer.start(33)

    app.exec()


if __name__ == "__main__":
    run_demo()
