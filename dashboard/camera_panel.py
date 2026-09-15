"""Camera Panel — ZED stereo feeds + lidar top-view, resizable via splitter.

Left pane: 2x2 grid (Left | Right | Depth | Disparity).
Right pane: Lidar top-view, separated by a draggable QSplitter handle.
"""

import cv2
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QGridLayout, QLabel,
    QSizePolicy, QSplitter, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS


class CameraImage(QLabel):
    """QLabel that displays a CV2 image. Reuses QPixmap buffer."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(280, 200)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self.setStyleSheet(
            f"background-color: {COLORS['bg_dark']};"
            f"border: 1px solid {COLORS['border']};"
        )
        self.setText("No Signal")
        self._pixmap = QPixmap()
        self._ref = None

    def set_frame(self, frame: np.ndarray):
        if frame is None:
            return
        self._ref = frame
        frame = np.ascontiguousarray(frame)
        if frame.ndim == 2:
            h, w = frame.shape
            bpl = w
            fmt = QImage.Format.Format_Grayscale8
        else:
            h, w, ch = frame.shape
            bpl = ch * w
            fmt = QImage.Format.Format_RGB888
        qimg = QImage(frame.data, w, h, bpl, fmt)
        self._pixmap.convertFromImage(qimg)
        scaled = self._pixmap.scaled(
            self.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.setPixmap(scaled)


class _FpsBadge(QLabel):
    """Floating FPS overlay badge."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet(
            f"background-color: rgba(13,14,18,180);"
            f"color: {COLORS['green']};"
            f"font-size: 11px; font-weight: bold;"
            f"padding: 2px 6px; border-radius: 4px;"
            f"border: 1px solid {COLORS['border']};"
        )
        self.setText("0 FPS")


class DepthMetricsBadge(QLabel):
    """Overlay showing depth quality metrics."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.setStyleSheet(
            f"background-color: rgba(13,14,18,180);"
            f"color: {COLORS['zed']};"
            f"font-size: 10px;"
            f"padding: 3px 6px; border-radius: 4px;"
            f"border: 1px solid {COLORS['border']};"
        )
        self.setText("")

    def update_metrics(self, valid_pct: float, invalid_pct: float,
                       mean_conf: float, median_d: float):
        self.setText(
            f"Valid: {valid_pct:.1f}%  |  "
            f"Invalid: {invalid_pct:.1f}%  |  "
            f"Conf: {mean_conf:.0f}  |  "
            f"Median: {median_d:.2f}m"
        )


def compute_pointcloud_bgr(depth: np.ndarray, fx: float = 521.92,
                           fy: float = 521.92, cx: float = 320.0,
                           cy: float = 180.0, step: int = 4) -> np.ndarray:
    """Downsampled XYZ → RGB point cloud as a BGR image for display."""
    if depth is None or depth.size == 0:
        return None
    valid = np.isfinite(depth) & (depth > 0.1)
    if not valid.any():
        return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

    h, w = depth.shape
    yy, xx = np.mgrid[0:h:step, 0:w:step]
    d = depth[yy, xx]
    v = np.isfinite(d) & (d > 0.1)

    xs = (xx[v] - cx) * d[v] / fx
    ys = (yy[v] - cy) * d[v] / fy
    zs = d[v]

    # Color by height (z)
    zmin, zmax = np.percentile(zs, [2, 98])
    if zmax - zmin < 0.01:
        zmax = zmin + 1.0
    z_norm = np.clip((zs - zmin) / (zmax - zmin), 0, 1)

    # Project XY onto image: normalize to image coords
    xmin, xmax = xs.min(), xs.max()
    ymin, ymax = ys.min(), ys.max()
    xr = (xs - xmin) / max(xmax - xmin, 1e-6)
    yr = (ys - ymin) / max(ymax - ymin, 1e-6)

    out_h, out_w = h // step, w // step
    out = np.zeros((out_h, out_w, 3), dtype=np.uint8)

    ix = np.clip((xr * (out_w - 1)).astype(int), 0, out_w - 1)
    iy = np.clip((yr * (out_h - 1)).astype(int), 0, out_h - 1)

    # Jet colormap on z_norm
    cmap_in = (z_norm * 255).astype(np.uint8)
    colored = cv2.applyColorMap(cmap_in[:, None], cv2.COLORMAP_JET)
    out[iy, ix] = colored[:, 0]

    return out


class CameraPanel(QWidget):
    """3x2 grid panel: Left, Right, Depth, Disparity/Confidence + lidar."""

    def __init__(self, parent=None):
        super().__init__(parent)

        grid = QGridLayout(self)
        grid.setSpacing(4)
        grid.setContentsMargins(4, 4, 4, 4)

        def _make_slot(label_text: str) -> tuple:
            container = QWidget()
            vl = QVBoxLayout(container)
            vl.setContentsMargins(0, 0, 0, 0)
            vl.setSpacing(2)

            title = QLabel(label_text)
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title.setStyleSheet(
                f"color: {COLORS['zed']}; font-weight: bold; font-size: 12px;"
            )

            image = CameraImage()
            fps_badge = _FpsBadge(image)

            vl.addWidget(title)
            vl.addWidget(image, stretch=1)

            return container, image, fps_badge, title

        self.left_container, self.left_image, self.left_fps, self.left_title = _make_slot("Left Camera")
        self.right_container, self.right_image, self.right_fps, self.right_title = _make_slot("Right Camera")
        self.depth_container, self.depth_image, self.depth_fps, self.depth_title = _make_slot("Depth (ZED VIEW)")
        self.bottom_container, self.bottom_image, self.bottom_fps, self.bottom_title = _make_slot("Disparity")

        # Depth metrics badge overlay on depth slot
        self.depth_metrics = DepthMetricsBadge(self.depth_image)

        # Lidar top-view panel in a resizable splitter (right pane)
        from dashboard.lidar_panel import LidarPanel
        self.lidar_panel = LidarPanel()

        grid.addWidget(self.left_container, 0, 0)
        grid.addWidget(self.right_container, 0, 1)
        grid.addWidget(self.depth_container, 1, 0)
        grid.addWidget(self.bottom_container, 1, 1)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        grid_widget = QWidget()
        grid_widget.setLayout(grid)
        splitter.addWidget(grid_widget)
        splitter.addWidget(self.lidar_panel)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)
        grid_widget.setMinimumWidth(400)
        self.lidar_panel.setMinimumWidth(280)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(splitter)

        # Bottom-right mode: "disp" or "confidence"
        self._bottom_mode = "disp"

    def update_lidar(self, points, num_points: int, rate: float,
                     streaming: bool, frame_id: int, matched: bool = False,
                     sync_delta_ms: float | None = None):
        self.lidar_panel.update_data(
            points, num_points, rate, streaming, frame_id,
            matched, sync_delta_ms,
        )

    def update_frames(self, left_rgb=None, right_rgb=None, depth=None,
                      left_fps: float = 0.0, disparity=None,
                      view_depth=None, view_confidence=None,
                      valid_pct=0.0, invalid_pct=0.0,
                      mean_conf=0.0, median_d=0.0):
        """Update all camera feeds.

        depth: raw MEASURE.DEPTH (float32) — for demo fallback only
        view_depth: VIEW.DEPTH (uint8 BGRA) — direct from ZED, no filtering
        view_confidence: VIEW.CONFIDENCE (uint8 grayscale) — direct from ZED
        disparity: raw MEASURE.DISPARITY (float32) — normalized for display
        """
        if left_rgb is not None:
            self.left_image.set_frame(left_rgb)
        if right_rgb is not None:
            self.right_image.set_frame(right_rgb)

        # Depth: use VIEW.DEPTH directly (no manual normalization)
        if view_depth is not None:
            rgb = cv2.cvtColor(view_depth, cv2.COLOR_BGRA2RGB)
            self.depth_image.set_frame(rgb)
        elif depth is not None:
            # Fallback: manual colormap from raw depth (demo mode)
            colored = _apply_colormap_fallback(depth)
            if colored is not None:
                self.depth_image.set_frame(cv2.cvtColor(colored, cv2.COLOR_BGR2RGB))

        # Depth metrics overlay
        self.depth_metrics.update_metrics(valid_pct, invalid_pct, mean_conf, median_d)

        # Bottom-right: disparity (raw MEASURE.DISPARITY + colormap) or confidence
        if self._bottom_mode == "confidence" and view_confidence is not None:
            self.bottom_title.setText("Confidence (ZED VIEW)")
            gray = view_confidence[:, :, 0] if view_confidence.ndim == 3 else view_confidence
            self.bottom_image.set_frame(gray)
        elif disparity is not None:
            self.bottom_title.setText("Disparity (MEASURE)")
            disp_vis = _apply_disparity_colormap(disparity)
            if disp_vis is not None:
                self.bottom_image.set_frame(cv2.cvtColor(disp_vis, cv2.COLOR_BGR2RGB))
        elif depth is not None:
            self.bottom_title.setText("Disparity (computed)")
            disp = _compute_disparity(depth)
            if disp is not None:
                disp_vis = _apply_disparity_colormap(disp)
                if disp_vis is not None:
                    self.bottom_image.set_frame(cv2.cvtColor(disp_vis, cv2.COLOR_BGR2RGB))

        fps_text = f"{left_fps:.1f} FPS"
        self.left_fps.setText(fps_text)
        self.right_fps.setText(fps_text)

    def set_bottom_mode(self, mode: str):
        """Switch bottom-right slot: 'disp' or 'confidence'."""
        self._bottom_mode = mode


# ── Fallback display helpers (used in demo mode when no ZED VIEW data) ──

def _apply_colormap_fallback(depth: np.ndarray) -> np.ndarray:
    """Depth → Inferno colormap. Fallback for demo mode."""
    if depth is None or depth.size == 0:
        return None
    d = depth.copy()
    valid = np.isfinite(d) & (d > 0)
    if not valid.any():
        return np.zeros((*d.shape, 3), dtype=np.uint8)
    d[~valid] = 0
    p2, p98 = np.percentile(d[valid], [2, 98])
    if p98 - p2 < 1e-6:
        p98 = p2 + 1.0
    d = np.clip(d, p2, p98)
    d_norm = np.zeros(d.shape, dtype=np.uint8)
    d_norm[valid] = ((d[valid] - p2) / (p98 - p2) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(d_norm, cv2.COLORMAP_INFERNO)
    colored[~valid] = [0, 0, 0]
    return colored


def _apply_disparity_colormap(disp: np.ndarray) -> np.ndarray:
    """Disparity → Plasma colormap. Display only.

    Accepts negative disparity (ZED SDK convention). Invalid / zero pixels
    are rendered as black.
    """
    if disp is None or disp.size == 0:
        return None
    d = disp.copy()
    valid = np.isfinite(d) & (d != 0)
    if not valid.any():
        return np.zeros((*d.shape, 3), dtype=np.uint8)
    vals = np.abs(d[valid])
    p2, p98 = np.percentile(vals, [2, 98])
    if p98 - p2 < 1e-6:
        p98 = p2 + 1.0
    d_norm = np.zeros(d.shape, dtype=np.uint8)
    clipped = np.clip(vals, p2, p98)
    d_norm[valid] = ((clipped - p2) / (p98 - p2) * 255).astype(np.uint8)
    result = cv2.applyColorMap(d_norm, cv2.COLORMAP_PLASMA)
    result[~valid] = [0, 0, 0]
    return result


def _compute_disparity(depth: np.ndarray, baseline: float = 0.1196,
                       fx: float = 521.92) -> np.ndarray:
    """Compute disparity from depth map."""
    if depth is None or depth.size == 0:
        return None
    disp = np.zeros_like(depth, dtype=np.float32)
    valid = depth > 0
    disp[valid] = (fx * baseline) / depth[valid]
    return disp
