"""Reusable high-performance scrolling plot widgets using PyQtGraph."""

from __future__ import annotations
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QWidget

from dashboard.theme import COLORS

pg.setConfigOptions(antialias=True, background=COLORS["bg_dark"], foreground=COLORS["text"])


class ScrollingPlot(QWidget):
    """Real-time scrolling plot for N channels."""

    def __init__(self, title: str, channels: list[str], max_points: int = 300,
                 y_label: str = "", legend: bool = True, parent=None):
        super().__init__(parent)
        self.max_points = max_points
        self.channels = channels
        self.data = {ch: np.zeros(max_points) for ch in channels}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox(title)
        group_layout = QVBoxLayout(group)

        self.plot = pg.PlotWidget()
        self.plot.setLabel("left", y_label)
        self.plot.setLabel("bottom", "Samples")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setMinimumHeight(150)

        colors = [COLORS["red"], COLORS["green"], COLORS["zed"]]
        self.curves = {}
        for i, ch in enumerate(channels):
            c = colors[i % len(colors)]
            pen = pg.mkPen(color=c, width=2)
            self.curves[ch] = self.plot.plot(pen=pen, name=ch)

        if legend and len(channels) > 1:
            legend_item = self.plot.addLegend(offset=(10, 10))
            legend_item.setLabelTextColor(COLORS["text_dim"])

        group_layout.addWidget(self.plot)
        layout.addWidget(group)

    def update_data(self, values: dict[str, float]):
        for ch, val in values.items():
            if ch in self.data:
                self.data[ch] = np.roll(self.data[ch], -1)
                self.data[ch][-1] = val
                self.curves[ch].setData(self.data[ch])

    def clear(self):
        for ch in self.channels:
            self.data[ch][:] = 0
            self.curves[ch].setData(self.data[ch])
