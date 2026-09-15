"""IMU Panel — MTi-8 primary + ZED IMU diagnostic, with delta for monitoring."""

import math

import numpy as np
from scipy.spatial.transform import Rotation
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QGridLayout, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS


def _delta_color(delta: float) -> str:
    """Color for delta: green <3, yellow 3-8, red >8 degrees."""
    d = abs(delta)
    if d < 3:
        return COLORS["green"]
    elif d < 8:
        return COLORS["yellow"]
    return COLORS["red"]


def _orientation_error_deg(mti_quat, zed_quat):
    """True angular error between MTi-8 (w,x,y,z) and ZED (x,y,z,w) quats."""
    R1 = Rotation.from_quat(
        [mti_quat[1], mti_quat[2], mti_quat[3], mti_quat[0]]
    ).as_matrix()
    R2 = Rotation.from_quat(
        [zed_quat[0], zed_quat[1], zed_quat[2], zed_quat[3]]
    ).as_matrix()
    R_err = R1 @ R2.T
    trace = np.clip((np.trace(R_err) - 1) / 2, -1, 1)
    return math.degrees(math.acos(trace))


def _make_label(font_size=12, bold=False):
    lbl = QLabel("—")
    fw = "bold" if bold else "normal"
    lbl.setStyleSheet(
        f"color: {COLORS['text_bright']}; font-size: {font_size}px; font-weight: {fw};"
    )
    return lbl


class SingleIMUWidget(QWidget):
    """Compact IMU display for one sensor: numeric accel/gyro/euler."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        self._title = title
        group = QGroupBox(title)
        g = QGridLayout(group)
        g.setSpacing(1)

        # Euler
        self.roll_lbl = _make_label(13, True)
        self.pitch_lbl = _make_label(13, True)
        self.yaw_lbl = _make_label(13, True)
        g.addWidget(self.roll_lbl, 0, 0)
        g.addWidget(self.pitch_lbl, 0, 1)
        g.addWidget(self.yaw_lbl, 0, 2)

        # Accel
        self.ax_lbl = _make_label(11)
        self.ay_lbl = _make_label(11)
        self.az_lbl = _make_label(11)
        g.addWidget(self.ax_lbl, 1, 0)
        g.addWidget(self.ay_lbl, 1, 1)
        g.addWidget(self.az_lbl, 1, 2)

        # Gyro
        self.gx_lbl = _make_label(11)
        self.gy_lbl = _make_label(11)
        self.gz_lbl = _make_label(11)
        g.addWidget(self.gx_lbl, 2, 0)
        g.addWidget(self.gy_lbl, 2, 1)
        g.addWidget(self.gz_lbl, 2, 2)

        # Header labels
        hdr = QLabel("R         P         Y")
        hdr.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 10px;")
        g.addWidget(hdr, 3, 0, 1, 3)

        self._group = group
        self._stale = False
        layout.addWidget(group)

    def update_data(self, accel, gyro, roll, pitch, yaw, stale=False):
        self._stale = stale
        if stale:
            self._group.setTitle(f"{self._title} — STALE")
            self._group.setStyleSheet(
                f"QGroupBox {{ color: {COLORS['yellow']}; font-weight: bold; }}"
            )
        else:
            self._group.setTitle(self._title)
            self._group.setStyleSheet("")

        self.roll_lbl.setText(f"R  {roll:+.1f}\u00b0")
        self.pitch_lbl.setText(f"P  {pitch:+.1f}\u00b0")
        self.yaw_lbl.setText(f"Y  {yaw:+.1f}\u00b0")

        if accel and len(accel) >= 3:
            self.ax_lbl.setText(f"ax {accel[0]:+.2f}")
            self.ay_lbl.setText(f"ay {accel[1]:+.2f}")
            self.az_lbl.setText(f"az {accel[2]:+.2f}")
        if gyro and len(gyro) >= 3:
            self.gx_lbl.setText(f"gx {gyro[0]:+.3f}")
            self.gy_lbl.setText(f"gy {gyro[1]:+.3f}")
            self.gz_lbl.setText(f"gz {gyro[2]:+.3f}")


class DeltaWidget(QWidget):
    """Delta between MTi-8 and ZED: Euler deltas + quaternion error."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        group = QGroupBox("IMU Delta (MTi \u2212 ZED) — diagnostic")
        g = QGridLayout(group)
        g.setSpacing(4)

        header = QLabel("\u0394 Roll      \u0394 Pitch     \u0394 Yaw")
        header.setStyleSheet(
            f"color: {COLORS['text_dim']}; font-size: 11px;"
        )
        g.addWidget(header, 0, 0, 1, 3)

        self.delta_roll = QLabel("0.0\u00b0")
        self.delta_pitch = QLabel("0.0\u00b0")
        self.delta_yaw = QLabel("0.0\u00b0")
        for lbl in (self.delta_roll, self.delta_pitch, self.delta_yaw):
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 16px;"
                f" font-weight: bold; background-color: {COLORS['bg_dark']};"
                f" padding: 4px; border: 1px solid {COLORS['border']};"
            )
        g.addWidget(self.delta_roll, 1, 0)
        g.addWidget(self.delta_pitch, 1, 1)
        g.addWidget(self.delta_yaw, 1, 2)

        self.error_label = QLabel("\u2014")
        self.error_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.error_label.setStyleSheet(
            f"color: {COLORS['green']}; font-size: 14px;"
            f" font-weight: bold; background-color: {COLORS['bg_dark']};"
            f" padding: 4px; border: 1px solid {COLORS['border']};"
        )
        g.addWidget(self.error_label, 2, 0, 1, 3)

        err_hdr = QLabel("Quaternion error (R_ZED \u00d7 R_MTi^T)")
        err_hdr.setStyleSheet(
            f"color: {COLORS['text_dim']}; font-size: 10px;"
        )
        g.addWidget(err_hdr, 3, 0, 1, 3)

        layout.addWidget(group)

    def update(self, mti_roll, mti_pitch, mti_yaw,
               zed_roll, zed_pitch, zed_yaw,
               mti_quat=None, zed_quat=None):
        dr = mti_roll - zed_roll
        dp = mti_pitch - zed_pitch
        dy = mti_yaw - zed_yaw
        dy = (dy + 180) % 360 - 180

        self.delta_roll.setText(f"{dr:+.1f}\u00b0")
        self.delta_pitch.setText(f"{dp:+.1f}\u00b0")
        self.delta_yaw.setText(f"{dy:+.1f}\u00b0")

        self.delta_roll.setStyleSheet(
            f"color: {_delta_color(dr)}; font-size: 16px;"
            f" font-weight: bold; background-color: {COLORS['bg_dark']};"
            f" padding: 4px; border: 1px solid {COLORS['border']};"
        )
        self.delta_pitch.setStyleSheet(
            f"color: {_delta_color(dp)}; font-size: 16px;"
            f" font-weight: bold; background-color: {COLORS['bg_dark']};"
            f" padding: 4px; border: 1px solid {COLORS['border']};"
        )
        self.delta_yaw.setStyleSheet(
            f"color: {_delta_color(dy)}; font-size: 16px;"
            f" font-weight: bold; background-color: {COLORS['bg_dark']};"
            f" padding: 4px; border: 1px solid {COLORS['border']};"
        )

        if mti_quat is not None and zed_quat is not None:
            err = _orientation_error_deg(mti_quat, zed_quat)
            c = _delta_color(err)
            self.error_label.setText(f"{err:.1f}\u00b0")
            self.error_label.setStyleSheet(
                f"color: {c}; font-size: 14px; font-weight: bold;"
                f" background-color: {COLORS['bg_dark']}; padding: 4px;"
                f" border: 1px solid {COLORS['border']};"
            )


class IMUPanel(QWidget):
    """Panel showing MTi-8 and ZED IMU side by side, with delta."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        imus = QHBoxLayout()
        self.mti = SingleIMUWidget("MTi-8 (nav source)")
        self.zed = SingleIMUWidget("ZED (diagnostic)")
        imus.addWidget(self.mti, stretch=1)
        imus.addWidget(self.zed, stretch=1)
        layout.addLayout(imus, stretch=1)

        self.delta = DeltaWidget()
        layout.addWidget(self.delta)

        self._mti_rpy = (0.0, 0.0, 0.0)
        self._zed_rpy = (0.0, 0.0, 0.0)
        self._mti_quat = None
        self._zed_quat = None

    def update_data(self, accel=None, gyro=None, roll=0, pitch=0, yaw=0,
                    quat=None, stale=False):
        """Update MTi-8 IMU display."""
        self.mti.update_data(accel, gyro, roll, pitch, yaw, stale=stale)
        self._mti_rpy = (roll, pitch, yaw)
        if quat is not None:
            self._mti_quat = quat
        self.delta.update(
            *self._mti_rpy, *self._zed_rpy,
            mti_quat=self._mti_quat, zed_quat=self._zed_quat,
        )

    def update_zed_data(self, accel=None, gyro=None, roll=0, pitch=0,
                        yaw=0, quat=None, stale=False):
        """Update ZED IMU display.

        stale=True (no ZED IMU sample timestamp change for >1s — typically
        a USB HID re-enumeration glitch on camera open, see
        dashboard/acquisition.py) shows "STALE" instead of silently
        displaying a frozen reading. This is diagnostic-only: it never
        affects recording — raw ZED IMU lives inside the SVO2 regardless.
        """
        self.zed.update_data(accel, gyro, roll, pitch, yaw, stale=stale)
        self._zed_rpy = (roll, pitch, yaw)
        if quat is not None:
            self._zed_quat = quat
        self.delta.update(
            *self._mti_rpy, *self._zed_rpy,
            mti_quat=self._mti_quat, zed_quat=self._zed_quat,
        )
