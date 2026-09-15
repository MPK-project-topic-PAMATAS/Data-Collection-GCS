"""Sync Diagnostics Panel — percentiles, worst, rejected + scrolling graph."""

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox, QGridLayout, QLabel, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS


class SyncDiagnosticsPanel(QWidget):
    """Synchronization error display with statistics, histogram, and graph."""

    REJECT_THRESHOLD_MS = 20.0

    def __init__(self, max_points: int = 600, parent=None):
        super().__init__(parent)
        self.max_points = max_points
        self.errors = np.zeros(max_points)
        self.count = 0
        self.total = 0.0
        self.sum_sq = 0.0
        self.min_err = float("inf")
        self.max_err = 0.0
        self.rejected_count = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("Synchronization")
        group_layout = QVBoxLayout(group)

        # Current / Average / Min / Max
        stats = QGridLayout()
        stats.setSpacing(6)

        def make_stat(label_text, color):
            lbl = QLabel(label_text)
            lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px;")
            val = QLabel("0.00 ms")
            val.setStyleSheet(f"color: {color}; font-size: 15px; font-weight: bold;")
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            return lbl, val

        self.lbl_cur, self.val_cur = make_stat("Current", COLORS["text_bright"])
        self.lbl_avg, self.val_avg = make_stat("Average", COLORS["green"])
        self.lbl_min, self.val_min = make_stat("Min", COLORS["zed"])
        self.lbl_max, self.val_max = make_stat("Max", COLORS["red"])

        stats.addWidget(self.lbl_cur, 0, 0)
        stats.addWidget(self.val_cur, 0, 1)
        stats.addWidget(self.lbl_avg, 0, 2)
        stats.addWidget(self.val_avg, 0, 3)
        stats.addWidget(self.lbl_min, 1, 0)
        stats.addWidget(self.val_min, 1, 1)
        stats.addWidget(self.lbl_max, 1, 2)
        stats.addWidget(self.val_max, 1, 3)
        group_layout.addLayout(stats)

        # Percentiles / worst / rejected
        pct = QGridLayout()
        pct.setSpacing(6)

        def make_pct(label_text, color=COLORS["text_bright"]):
            lbl = QLabel(label_text)
            lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 10px;")
            val = QLabel("—")
            val.setStyleSheet(f"color: {color}; font-size: 12px; font-weight: bold;")
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            return lbl, val

        self.lbl_p50, self.val_p50 = make_pct("Median (p50)")
        self.lbl_p95, self.val_p95 = make_pct("p95", COLORS["yellow"])
        self.lbl_p99, self.val_p99 = make_pct("p99", COLORS["red"])
        self.lbl_worst, self.val_worst = make_pct("Worst", COLORS["red"])
        self.lbl_rej, self.val_rej = make_pct("Rejected %", COLORS["red"])

        pct.addWidget(self.lbl_p50, 0, 0)
        pct.addWidget(self.val_p50, 0, 1)
        pct.addWidget(self.lbl_p95, 0, 2)
        pct.addWidget(self.val_p95, 0, 3)
        pct.addWidget(self.lbl_p99, 1, 0)
        pct.addWidget(self.val_p99, 1, 1)
        pct.addWidget(self.lbl_worst, 1, 2)
        pct.addWidget(self.val_worst, 1, 3)
        pct.addWidget(self.lbl_rej, 2, 0)
        pct.addWidget(self.val_rej, 2, 1)
        group_layout.addLayout(pct)

        # Scrolling time-series plot
        self.plot = pg.PlotWidget()
        self.plot.setLabel("left", "ms")
        self.plot.setLabel("bottom", "Samples")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setYRange(0, 25)
        self.plot.setMinimumHeight(80)

        pen = pg.mkPen(color=COLORS["green"], width=2)
        self.curve = self.plot.plot(pen=pen)

        self.warn_line = pg.InfiniteLine(pos=15, angle=0, movable=False,
                                         pen=pg.mkPen(color=COLORS["yellow"],
                                                       style=Qt.PenStyle.DashLine))
        self.plot.addItem(self.warn_line)

        self.danger_line = pg.InfiniteLine(pos=self.REJECT_THRESHOLD_MS, angle=0,
                                           movable=False,
                                           pen=pg.mkPen(color=COLORS["red"],
                                                         style=Qt.PenStyle.DashLine))
        self.plot.addItem(self.danger_line)

        group_layout.addWidget(self.plot)

        # Histogram
        self.hist_plot = pg.PlotWidget()
        self.hist_plot.setLabel("left", "Count")
        self.hist_plot.setLabel("bottom", "ms")
        self.hist_plot.showGrid(x=True, y=True, alpha=0.3)
        self.hist_plot.setMinimumHeight(70)
        group_layout.addWidget(self.hist_plot)

        layout.addWidget(group)

        self._all_errors = []

    def push(self, error_ms: float):
        self.errors = np.roll(self.errors, -1)
        self.errors[-1] = error_ms
        self.count += 1
        self.total += error_ms
        self.sum_sq += error_ms * error_ms
        self.min_err = min(self.min_err, error_ms)
        self.max_err = max(self.max_err, error_ms)

        self._all_errors.append(error_ms)
        if len(self._all_errors) > 5000:
            self._all_errors = self._all_errors[-3000:]

        if error_ms > self.REJECT_THRESHOLD_MS:
            self.rejected_count += 1

        avg = self.total / self.count

        self.val_cur.setText(f"{error_ms:.2f} ms")
        self.val_avg.setText(f"{avg:.2f} ms")
        self.val_min.setText(f"{self.min_err:.2f} ms")
        self.val_max.setText(f"{self.max_err:.2f} ms")

        color = COLORS["green"]
        if error_ms > self.REJECT_THRESHOLD_MS:
            color = COLORS["red"]
        elif error_ms > 15:
            color = COLORS["yellow"]
        self.val_cur.setStyleSheet(f"color: {color}; font-size: 15px; font-weight: bold;")

        self.curve.setData(self.errors)

        # Percentiles and rejected (every 30 frames to avoid overhead)
        if self.count % 30 == 0 and len(self._all_errors) >= 10:
            arr = np.array(self._all_errors)
            p50 = np.percentile(arr, 50)
            p95 = np.percentile(arr, 95)
            p99 = np.percentile(arr, 99)
            self.val_p50.setText(f"{p50:.2f} ms")
            self.val_p95.setText(f"{p95:.2f} ms")
            self.val_p99.setText(f"{p99:.2f} ms")
            self.val_worst.setText(f"{np.max(arr):.2f} ms")
            rej_pct = self.rejected_count / self.count * 100
            rej_color = COLORS["green"] if rej_pct < 1 else (
                COLORS["yellow"] if rej_pct < 5 else COLORS["red"]
            )
            self.val_rej.setText(f"{rej_pct:.1f}%")
            self.val_rej.setStyleSheet(f"color: {rej_color}; font-size: 12px; font-weight: bold;")

            # Update histogram
            valid = arr[arr >= 0]
            if len(valid) > 0:
                bins = np.linspace(
                    0, min(np.percentile(valid, 99) * 1.2, 30), 30
                )
                counts, edges = np.histogram(valid, bins=bins)
                centers = (edges[:-1] + edges[1:]) / 2
                self.hist_plot.clear()
                bg = pg.BarGraphItem(
                    x=centers, width=edges[1] - edges[0],
                    height=counts,
                    brush=pg.mkBrush(COLORS["zed"]),
                )
                self.hist_plot.addItem(bg)
