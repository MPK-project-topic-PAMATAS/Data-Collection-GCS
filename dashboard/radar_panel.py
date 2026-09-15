"""Radar Panel — AWR1843 point cloud XY scatter + camera overlay.

Two plots:
  Top:   XY scatter (range vs azimuth) — bird's-eye view, points colored by Doppler velocity
  Bottom: Radar points projected onto the live camera image (see radar_camera_demo.py
          for the standalone version and the projection math/limitations —
          no measured camera<->radar extrinsic, assumes co-located sensors)

The panel used to show a range-Doppler heatmap here instead. The radar's
active sensor config (radar/receiver.py's _SENSOR_CONFIG_CMDS) is azimuth-only
(TX0+TX2 MIMO, no elevation antenna) with the heatmap TLV enabled; a 3TX
elevation-capable profile was tried and reverted after live testing showed
noisier detections (untuned CFAR thresholds) plus this firmware couldn't
combine 3TX with the heatmap output anyway — see git history on this file
around that experiment. Camera overlay was chosen over the heatmap panel
regardless of that revert.
"""

from __future__ import annotations

import cv2
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget,
)

from dashboard.camera_panel import CameraImage
from dashboard.theme import COLORS

pg.setConfigOptions(antialias=True, background=COLORS["bg_dark"], foreground=COLORS["text"])

_MAX_POINTS = 256
_MAX_X = 12.0   # lateral extent shown in the XY scatter (m)
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


class RadarXYScatter(QWidget):
    """Bird's-eye view: X (lateral) vs Y (forward/range) scatter."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("Radar XY (bird's-eye)")
        gl = QVBoxLayout(group)

        self.plot = pg.PlotWidget()
        self.plot.setLabel("left", "Forward (m)")
        self.plot.setLabel("bottom", "Lateral X (m)")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setAspectLocked(True)
        self.plot.setMinimumHeight(160)
        self.plot.setXRange(-_MAX_X, _MAX_X)
        self.plot.setYRange(0, _MAX_Y)

        ego = pg.ScatterPlotItem([0], [0], pen=None,
                                 brush=pg.mkBrush(COLORS["zed"]),
                                 size=10, symbol="t1")
        self.plot.addItem(ego)

        self.scatter = pg.ScatterPlotItem()
        self.plot.addItem(self.scatter)

        # Circle overlay for moving objects
        self.circles_scatter = None

        gl.addWidget(self.plot)
        layout.addWidget(group)

    def update_points(self, points):
        # Clear old circle overlay
        if self.circles_scatter is not None:
            self.plot.removeItem(self.circles_scatter)
            self.circles_scatter = None

        if not points:
            self.scatter.setData([])
            return

        n = min(len(points), _MAX_POINTS)
        x, y, vel, circle_x, circle_y = [], [], [], [], []

        for p in points[:n]:
            x.append(p.x)
            y.append(p.y)
            vel.append(p.velocity_mps)

            # Track moving objects for circle visualization
            if abs(p.velocity_mps) > 0.3:  # threshold: 0.3 m/s
                circle_x.append(p.x)
                circle_y.append(p.y)

        # Draw hollow circles for moving objects using scatter with ring symbol
        if circle_x:
            circles = pg.ScatterPlotItem()
            circles.setData(circle_x, circle_y,
                          pen=pg.mkPen(color=pg.mkColor(255, 255, 100), width=2),
                          brush=pg.mkBrush(color=pg.mkColor(100, 100, 100, 80)),
                          size=12, symbol='o')
            self.plot.addItem(circles)
            self.circles_scatter = circles

        brushes = [pg.mkBrush(*_color_by_velocity(v)) for v in vel]
        self.scatter.setData(x, y, pen=None, brush=brushes, size=7)


class RadarCameraOverlay(QWidget):
    """Radar points projected onto the live camera image.

    Projection: the AWR1843's detected-object TLV reports Cartesian points
    as x=lateral(+right), y=forward/range, z=up (radar/parser.py's TLV
    docstring; also how RadarXYScatter above already plots them). Relabeled
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

        group = QGroupBox("Radar on Camera")
        gl = QVBoxLayout(group)

        self.image = CameraImage()
        gl.addWidget(self.image)
        layout.addWidget(group)

        self._calib: dict | None = None

    def set_calib(self, calib: dict | None):
        """Camera intrinsics dict with fx/fy/cx/cy (camera.calib after
        ZEDCamera.open()). Overlay is skipped until this is set."""
        self._calib = calib

    def update_overlay(self, camera_rgb: np.ndarray | None, points):
        if camera_rgb is None:
            return
        if self._calib is None or not points:
            self.image.set_frame(camera_rgb)
            return

        from config.settings import RADAR_TO_CAMERA_OFFSET_M
        from radar.projection import project_point

        img = camera_rgb.copy()
        h, w = img.shape[:2]
        fx, fy = self._calib["fx"], self._calib["fy"]
        cx, cy = self._calib["cx"], self._calib["cy"]

        for p in points:
            px = project_point(p, fx, fy, cx, cy, RADAR_TO_CAMERA_OFFSET_M)
            if px is None:
                continue
            u, v = px
            if not (0 <= u < w and 0 <= v < h):
                continue
            color = _color_by_velocity(p.velocity_mps)[:3]
            cv2.circle(img, (u, v), 8, color, 2)
            cv2.circle(img, (u, v), 2, color, -1)

        self.image.set_frame(img)


class RadarPanel(QWidget):
    """Combined radar panel: XY scatter + range-Doppler side by side."""

    def __init__(self, parent=None):
        super().__init__(parent)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        plots = QHBoxLayout()
        plots.setSpacing(4)
        self.xy = RadarXYScatter()
        self.overlay = RadarCameraOverlay()
        plots.addWidget(self.xy, stretch=1)
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
               num_objects, camera_ts_ns=0, camera_rgb=None):
        self.xy.update_points(points)
        self.overlay.update_overlay(camera_rgb, points)
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