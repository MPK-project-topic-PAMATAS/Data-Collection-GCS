"""Radar Panel — AWR1843 point cloud + camera overlay.

Two plots, side by side:
  Left:  top-down point cloud (lateral X vs longitudinal Y), points coloured
         by radial velocity.
  Right: Radar points projected onto the live camera image (see radar_camera_demo.py
         for the projection math and limitations).

Radar config (radar/receiver.py's _SENSOR_CONFIG_CMDS) is 3TX (TX0+TX1+TX2,
elevation-capable), 256 ADC samples, heatmap output off, so point z varies.
"""

from __future__ import annotations

import cv2
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSlider,
    QVBoxLayout, QWidget,
)

from dashboard.camera_panel import CameraImage
from dashboard.theme import COLORS

pg.setConfigOptions(antialias=True, background=COLORS["bg_dark"], foreground=COLORS["text"])

_MAX_POINTS = 256
_MAX_X = 8.0    # lateral half-extent of the heatmap view (m)
_MAX_Y = 10.0   # forward range shown (m)


def _color_by_velocity(vel_mps: float, max_vel: float = 5.0) -> tuple:
    """Map radial velocity to (R,G,B,A) — approaching (blue), receding (red)."""
    t = np.clip(abs(vel_mps) / max_vel, 0.0, 1.0)
    if vel_mps < 0:  # approaching (negative Doppler) -> cyan/blue
        r, g, b = int(32), int(64 + 120 * t), int(200 + 55 * t)
    elif vel_mps > 0:  # receding -> orange/red
        r, g, b = int(150 + 105 * t), int(60), int(40)
    else:  # static -> muted green
        r, g, b = 60, 185, 60
    return (r, g, b, 220)


_FAN_HALF_DEG = 70.0     # half field of view drawn for the guide lines


class RadarPointView(QWidget):
    """Top-down radar point cloud (lateral X vs longitudinal Y), points
    coloured by radial velocity, with range rings and angle guides."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("Radar Point Cloud (top-down)")
        gl = QVBoxLayout(group)

        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setMinimumHeight(160)
        self.plot = self.glw.addPlot()
        self.plot.setLabel("left", "Longitudinal distance (m)")
        self.plot.setLabel("bottom", "Lateral distance (m)")
        self.plot.setAspectLocked(True)
        self.plot.setMouseEnabled(False, False)
        self.plot.setXRange(-_MAX_X, _MAX_X, padding=0)
        self.plot.setYRange(0, _MAX_Y, padding=0)
        self.plot.disableAutoRange()

        ring_pen = pg.mkPen(255, 255, 255, 60, width=1)
        for r in range(2, int(_MAX_Y) + 1, 2):
            t = np.radians(np.linspace(-_FAN_HALF_DEG, _FAN_HALF_DEG, 90))
            self.plot.addItem(pg.PlotCurveItem(r * np.sin(t), r * np.cos(t), pen=ring_pen))
        for deg in (-60, -30, 0, 30, 60):
            t = np.radians(deg)
            self.plot.addItem(pg.PlotCurveItem(
                [0, _MAX_Y * np.sin(t)], [0, _MAX_Y * np.cos(t)], pen=ring_pen))

        self.points = pg.ScatterPlotItem(pen=pg.mkPen(0, 0, 0, 160), size=9)
        self.plot.addItem(self.points)
        gl.addWidget(self.glw)
        layout.addWidget(group)

    def update_points(self, points):
        pts = (points or [])[:_MAX_POINTS]
        self.points.setData(
            [p.x for p in pts], [p.y for p in pts],
            brush=[pg.mkBrush(*_color_by_velocity(p.velocity_mps)) for p in pts],
        )


class RadarCameraOverlay(QWidget):
    """Radar points projected onto the live camera image.

    Projection: the AWR1843's detected-object TLV reports Cartesian points
    as x=lateral(+right), y=forward/range, z=up (radar/parser.py's TLV
    docstring; also how the heatmap's point overlay already plots them). Relabeled
    for the pinhole model (camera x=right, y=down, z=forward):
        Xc, Yc, Zc = p.x, -p.z, p.y
        u = cx + fx * Xc / Zc
        v = cy + fy * Yc / Zc
    using the camera's own intrinsics (set_calib()) — same math as
    radar_camera_demo.py, via radar/projection.py. Applies
    config.settings.RADAR_TO_CAMERA_OFFSET_M (radar->camera lever arm),
    which defaults to (0, 0, 0) (co-located approximation) until measured
    — see docs/RADAR_CAMERA_OVERLAY.md. The active radar config is
    azimuth-only (no elevation antenna), so z is currently always 0 —
    every point lands on one row until/unless that's revisited (see the
    module docstring above).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("Lidar + Radar on Camera")
        gl = QVBoxLayout(group)

        self._calib: dict | None = None
        self._ext = None
        self._last = None          # (camera_rgb, points, lidar_points) for redraw

        self.image = CameraImage()
        gl.addWidget(self.image)
        gl.addWidget(self._build_align_controls())
        layout.addWidget(group)

    def _extrinsics(self):
        if self._ext is None:
            import warnings
            from fusion.transforms import load_extrinsics
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")   # placeholder poses are expected
                self._ext = load_extrinsics()
        return self._ext

    # (key, label, slider scale, min, max) — translation in mm, angles in 0.1 deg
    _SLIDERS = (
        ("x", "x fwd (m)", 1000.0, -1500, 1500),
        ("y", "y left (m)", 1000.0, -1500, 1500),
        ("z", "z up (m)", 1000.0, -1500, 1500),
        ("roll", "roll (°)", 10.0, -450, 450),
        ("pitch", "pitch (°)", 10.0, -450, 450),
        ("yaw", "yaw (°)", 10.0, -1800, 1800),
    )

    def _build_align_controls(self) -> QGroupBox:
        """Collapsible sliders that move the lidar's pose (vehicle frame, IMU
        origin; camera is the reference, so this is the lidar-vs-camera
        alignment) and redraw the overlay live. Save writes extrinsics.yaml."""
        box = QGroupBox("Align lidar → camera (vehicle axes: x fwd, y left, z up)")
        box.setCheckable(True)
        box.setChecked(False)
        inner = QWidget()
        grid = QGridLayout(inner)
        grid.setContentsMargins(0, 0, 0, 0)
        self._sliders, self._value_lbls = {}, {}
        for row, (key, label, scale, lo, hi) in enumerate(self._SLIDERS):
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(lo, hi)
            lbl = QLabel("0.000")
            lbl.setMinimumWidth(48)
            sl.valueChanged.connect(self._on_slider)
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(sl, row, 1)
            grid.addWidget(lbl, row, 2)
            self._sliders[key], self._value_lbls[key] = sl, lbl
        btns = QHBoxLayout()
        self._save_btn = QPushButton("Save to extrinsics.yaml")
        self._reset_btn = QPushButton("Reset")
        self._save_btn.clicked.connect(self._save_pose)
        self._reset_btn.clicked.connect(self._reset_pose)
        self._align_status = QLabel("")
        btns.addWidget(self._save_btn)
        btns.addWidget(self._reset_btn)
        btns.addWidget(self._align_status, 1)
        grid.addLayout(btns, len(self._SLIDERS), 0, 1, 3)
        v = QVBoxLayout(box)
        v.setContentsMargins(6, 2, 6, 2)
        v.addWidget(inner)
        inner.setVisible(False)
        box.toggled.connect(inner.setVisible)
        self._load_sliders_from_ext()
        return box

    def _pose_from_sliders(self):
        v = {k: self._sliders[k].value() / sc for k, _, sc, _, _ in self._SLIDERS}
        return [v["x"], v["y"], v["z"]], [v["roll"], v["pitch"], v["yaw"]]

    def _load_sliders_from_ext(self):
        prm = self._extrinsics().params["lidar"]
        vals = list(prm["translation_m"]) + list(prm["rpy_deg"])
        for (key, _, scale, lo, hi), val in zip(self._SLIDERS, vals):
            sl = self._sliders[key]
            sl.blockSignals(True)
            sl.setValue(int(round(max(lo, min(hi, val * scale)))))
            sl.blockSignals(False)
            self._value_lbls[key].setText(f"{sl.value() / scale:.3f}" if scale == 1000.0
                                          else f"{sl.value() / scale:.1f}")

    def _on_slider(self, _=None):
        t, rpy = self._pose_from_sliders()
        for key, _, scale, _, _ in self._SLIDERS:
            val = self._sliders[key].value() / scale
            self._value_lbls[key].setText(f"{val:.3f}" if scale == 1000.0 else f"{val:.1f}")
        self._extrinsics().set_pose("lidar", t, rpy)
        self._align_status.setText("unsaved")
        if self._last is not None:
            self.update_overlay(*self._last)

    def _save_pose(self):
        from fusion.transforms import save_sensor_pose
        t, rpy = self._pose_from_sliders()
        try:
            save_sensor_pose("lidar", t, rpy)
            self._align_status.setText("saved")
        except Exception as e:  # noqa: BLE001
            self._align_status.setText(f"save failed: {e}")

    def _reset_pose(self):
        import warnings
        from fusion.transforms import load_extrinsics
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            fresh = load_extrinsics().params["lidar"]
        self._extrinsics().set_pose("lidar", fresh["translation_m"], fresh["rpy_deg"])
        self._load_sliders_from_ext()
        self._align_status.setText("reset to saved")
        if self._last is not None:
            self.update_overlay(*self._last)

    def set_calib(self, calib: dict | None):
        """Camera intrinsics dict with fx/fy/cx/cy (camera.calib after
        ZEDCamera.open()). Overlay is skipped until this is set."""
        self._calib = calib

    def update_overlay(self, camera_rgb: np.ndarray | None, points,
                       lidar_points=None):
        if camera_rgb is None:
            return
        self._last = (camera_rgb, points, lidar_points)
        have_lidar = lidar_points is not None and len(lidar_points) > 0
        if self._calib is None or (not points and not have_lidar):
            self.image.set_frame(camera_rgb)
            return

        from config.settings import RADAR_TO_CAMERA_OFFSET_M
        from radar.projection import project_point

        img = camera_rgb.copy()
        h, w = img.shape[:2]
        fx, fy = self._calib["fx"], self._calib["fy"]
        cx, cy = self._calib["cx"], self._calib["cy"]

        if have_lidar:
            from lidar.projection import draw_depth_dots, project_lidar_to_camera
            u, v, d = project_lidar_to_camera(
                lidar_points, self._extrinsics(), fx, fy, cx, cy, w, h)
            draw_depth_dots(img, u, v, d)

        for p in points or []:
            px = project_point(p, fx, fy, cx, cy, RADAR_TO_CAMERA_OFFSET_M)
            if px is None:
                continue
            u, v = px
            if not (0 <= u < w and 0 <= v < h):
                continue
            color = _color_by_velocity(p.velocity_mps)[:3]
            cv2.circle(img, (u, v), 14, color, -1)
            cv2.circle(img, (u, v), 14, (255, 255, 255), 2)

        self.image.set_frame(img)


class RadarPanel(QWidget):
    """Combined radar panel: top-down point cloud + camera overlay side by side."""

    def __init__(self, parent=None):
        super().__init__(parent)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        plots = QHBoxLayout()
        plots.setSpacing(4)
        self.heatmap = RadarPointView()
        self.overlay = RadarCameraOverlay()
        plots.addWidget(self.heatmap, stretch=1)
        plots.addWidget(self.overlay, stretch=1)
        outer.addLayout(plots, stretch=1)

        stats = QHBoxLayout()
        self.frame_lbl = QLabel("Frame: —")
        self.frame_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 10px;")
        self.obj_lbl = QLabel("Objects: —")
        self.obj_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 10px;")
        self.ts_lbl = QLabel("Δt: —")
        self.ts_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 10px;")
        self.status_lbl = QLabel("No data")
        self.status_lbl.setStyleSheet(
            f"color: {COLORS['red']}; font-size: 10px; font-weight: bold;"
        )
        stats.addWidget(self.status_lbl)
        stats.addWidget(self.frame_lbl)
        stats.addWidget(self.obj_lbl)
        stats.addWidget(self.ts_lbl)
        stats.addStretch()
        outer.addLayout(stats)

    def set_camera_calib(self, calib: dict | None):
        self.overlay.set_calib(calib)

    def update(self, points, radar_ts_ns, radar_frame_id,
               num_objects, camera_ts_ns=0, camera_rgb=None,
               range_azimuth=None, lidar_points=None):
        self.heatmap.update_points(points)
        self.overlay.update_overlay(camera_rgb, points, lidar_points)
        self.frame_lbl.setText(f"Frame: {radar_frame_id}")
        self.obj_lbl.setText(f"Objects: {num_objects}")
        if points:
            self.status_lbl.setText("Detections")
            self.status_lbl.setStyleSheet(
                f"color: {COLORS['green']}; font-size: 10px; font-weight: bold;"
            )
        elif radar_frame_id > 0:
            self.status_lbl.setText("No detections")
            self.status_lbl.setStyleSheet(
                f"color: {COLORS['yellow']}; font-size: 10px; font-weight: bold;"
            )
        else:
            self.status_lbl.setText("No data")
            self.status_lbl.setStyleSheet(
                f"color: {COLORS['red']}; font-size: 10px; font-weight: bold;"
            )
        if camera_ts_ns > 0 and radar_ts_ns > 0:
            dt_ms = abs(radar_ts_ns - camera_ts_ns) / 1e6
            self.ts_lbl.setText(f"Δt: {dt_ms:.1f} ms")
        else:
            self.ts_lbl.setText("Δt: —")