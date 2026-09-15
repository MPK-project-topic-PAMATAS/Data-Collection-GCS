"""Quality Panel — expanded image quality metrics."""

import cv2
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox, QGridLayout, QLabel, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS


class QualityIndicator(QWidget):
    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 1, 0, 1)
        self.name_lbl = QLabel(name)
        self.name_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 10px;")
        self.val_lbl = QLabel("—")
        self.val_lbl.setStyleSheet(f"color: {COLORS['text_bright']}; font-weight: bold;")
        self.val_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.bar = QLabel()
        self.bar.setFixedHeight(3)
        self.bar.setStyleSheet(f"background-color: {COLORS['green']};")
        layout.addWidget(self.name_lbl, 0, 0)
        layout.addWidget(self.val_lbl, 0, 1)
        layout.addWidget(self.bar, 1, 0, 1, 2)

    def set(self, text: str, quality: float = 1.0):
        self.val_lbl.setText(text)
        color = COLORS["green"] if quality > 0.6 else (
            COLORS["yellow"] if quality > 0.3 else COLORS["red"]
        )
        self.bar.setStyleSheet(f"background-color: {color};")


class QualityPanel(QWidget):
    """Expanded image quality metrics from live camera feed."""

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("Image Quality")
        g = QVBoxLayout(group)

        self.brightness = QualityIndicator("Luminance")
        self.blur_score = QualityIndicator("Sharpness")
        self.motion_blur = QualityIndicator("Motion blur")
        self.contrast = QualityIndicator("Contrast")
        self.dynamic_range = QualityIndicator("Dynamic range")
        self.overexposed = QualityIndicator("Overexposed")
        self.underexposed = QualityIndicator("Underexposed")
        self.dropped = QualityIndicator("Dropped frames")

        for w in [self.brightness, self.blur_score, self.motion_blur,
                  self.contrast, self.dynamic_range,
                  self.overexposed, self.underexposed, self.dropped]:
            g.addWidget(w)

        layout.addWidget(group)

    def update_from_frame(self, frame_bgr, dropped: int = 0):
        if frame_bgr is None:
            return

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) if frame_bgr.ndim == 3 else frame_bgr

        # Luminance
        mean_brightness = np.mean(gray)
        brightness_q = 1.0 - abs(mean_brightness - 128) / 128
        self.brightness.set(f"{mean_brightness:.0f}/255", brightness_q)

        # Sharpness (Laplacian variance)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        blur_val = laplacian.var()
        blur_q = min(blur_val / 500, 1.0)
        self.blur_score.set(f"{blur_val:.0f}", blur_q)

        # Motion blur (horizontal gradient variance — low = motion blur)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        motion_val = np.std(sobelx)
        motion_q = min(motion_val / 40, 1.0)
        self.motion_blur.set(f"{motion_val:.0f}", motion_q)

        # Contrast (std dev)
        contrast_val = np.std(gray)
        contrast_q = min(contrast_val / 60, 1.0)
        self.contrast.set(f"{contrast_val:.0f}", contrast_q)

        # Dynamic range (p99 - p1 of histogram)
        p1, p99 = np.percentile(gray, [1, 99])
        dr = p99 - p1
        dr_q = min(dr / 200, 1.0)
        self.dynamic_range.set(f"{dr:.0f}", dr_q)

        # Overexposed / underexposed
        over_pct = np.sum(gray > 240) / gray.size * 100
        under_pct = np.sum(gray < 15) / gray.size * 100
        over_q = 1.0 - over_pct / 100
        under_q = 1.0 - under_pct / 100
        self.overexposed.set(f"{over_pct:.1f}%", over_q)
        self.underexposed.set(f"{under_pct:.1f}%", under_q)

        # Dropped
        self.dropped.set(str(dropped), 1.0 if dropped == 0 else 0.0)
