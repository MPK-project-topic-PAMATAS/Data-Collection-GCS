"""Log Panel — scrolling event log console."""

import datetime

from PySide6.QtWidgets import (
    QGroupBox, QTextBrowser, QVBoxLayout, QWidget,
)

from dashboard.theme import COLORS


class LogPanel(QWidget):
    """Scrolling console showing system events."""

    MAX_LINES = 500

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("Event Log")
        group_layout = QVBoxLayout(group)

        self.console = QTextBrowser()
        self.console.setOpenExternalLinks(False)
        group_layout.addWidget(self.console)

        layout.addWidget(group)

    def log(self, message: str, level: str = "info"):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        color_map = {
            "info": COLORS["text"],
            "success": COLORS["green"],
            "warning": COLORS["yellow"],
            "error": COLORS["red"],
        }
        color = color_map.get(level, COLORS["text"])
        self.console.append(
            f'<span style="color:{COLORS["text_dim"]};">[{ts}]</span> '
            f'<span style="color:{color};">{message}</span>'
        )

    def clear(self):
        self.console.clear()
