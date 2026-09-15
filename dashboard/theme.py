"""Dark theme and styling constants for the Ground Control Station.

Apple-inspired: true-black canvas, layered "card" surfaces, a restrained
system-color palette (blue / teal / green / yellow / red), SF-style
typography stack, and generous corner radii.
"""

# fmt: off
COLORS = {
    # Surfaces — layered from canvas (darkest) to elevated controls.
    "bg_dark":       "#000000",
    "bg_panel":      "#161618",
    "bg_widget":     "#1E1E21",
    "bg_elevated":   "#2A2A2D",
    "border":        "#2E2E32",
    "border_soft":   "#232326",

    # Accent / brand.
    "accent":        "#0A84FF",
    "zed":           "#64D2FF",
    "mti":           "#FF9F0A",

    # Semantic status.
    "green":         "#30D158",
    "yellow":        "#FFD60A",
    "red":           "#FF453A",

    # Text.
    "text":          "#D1D1D6",
    "text_dim":      "#8E8E93",
    "text_bright":   "#F5F5F7",
}
# fmt: on

FONT_MONO = '"SF Mono", "JetBrains Mono", Menlo, Consolas, monospace'
FONT_SANS = (
    '-apple-system, BlinkMacSystemFont, "SF Pro Text", "Segoe UI", '
    '"Helvetica Neue", Arial, sans-serif'
)

RADIUS = 10
RADIUS_SM = 6

STYLESHEET = f"""
* {{
    outline: none;
}}
QMainWindow, QWidget {{
    background-color: {COLORS['bg_dark']};
    color: {COLORS['text']};
    font-family: {FONT_SANS};
    font-size: 12px;
    selection-background-color: {COLORS['accent']};
    selection-color: {COLORS['text_bright']};
}}
QToolTip {{
    background-color: {COLORS['bg_elevated']};
    color: {COLORS['text_bright']};
    border: 1px solid {COLORS['border']};
    border-radius: {RADIUS_SM}px;
    padding: 4px 8px;
}}
QGroupBox {{
    background-color: {COLORS['bg_panel']};
    border: 1px solid {COLORS['border_soft']};
    border-radius: {RADIUS}px;
    margin-top: 8px;
    padding-top: 14px;
    font-weight: 600;
    font-size: 11px;
    color: {COLORS['text_dim']};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    top: 2px;
    padding: 0 4px;
    color: {COLORS['text_dim']};
}}
QLabel {{
    color: {COLORS['text']};
    background: transparent;
}}
QLabel[dim] {{
    color: {COLORS['text_dim']};
    font-size: 11px;
}}
QLabel[title] {{
    color: {COLORS['text_bright']};
    font-size: 14px;
    font-weight: 600;
}}
QPushButton {{
    background-color: {COLORS['bg_elevated']};
    color: {COLORS['text_bright']};
    border: 1px solid {COLORS['border']};
    border-radius: {RADIUS_SM}px;
    padding: 6px 16px;
    font-weight: 600;
}}
QPushButton:hover {{
    background-color: #35353A;
    border-color: #414146;
}}
QPushButton:pressed {{
    background-color: #232326;
}}
QPushButton:disabled {{
    background-color: {COLORS['bg_panel']};
    color: {COLORS['text_dim']};
    border-color: {COLORS['border_soft']};
}}
QPushButton[accent] {{
    background-color: {COLORS['accent']};
    color: #FFFFFF;
    border: 1px solid {COLORS['accent']};
}}
QPushButton[accent]:hover {{
    background-color: #2B93FF;
}}
QPushButton[accent]:pressed {{
    background-color: #0870D6;
}}
QPushButton[recording] {{
    background-color: {COLORS['red']};
    color: #FFFFFF;
    border: 1px solid {COLORS['red']};
}}
QPushButton[recording]:hover {{
    background-color: #FF6259;
}}
QLineEdit, QTextEdit, QPlainTextEdit {{
    background-color: {COLORS['bg_dark']};
    color: {COLORS['green']};
    border: 1px solid {COLORS['border']};
    border-radius: {RADIUS_SM}px;
    font-family: {FONT_MONO};
    font-size: 11px;
    padding: 4px;
}}
QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus {{
    border: 1px solid {COLORS['accent']};
}}
QComboBox {{
    background-color: {COLORS['bg_elevated']};
    color: {COLORS['text_bright']};
    border: 1px solid {COLORS['border']};
    border-radius: {RADIUS_SM}px;
    padding: 4px 10px;
}}
QComboBox:hover {{
    border-color: #414146;
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox QAbstractItemView {{
    background-color: {COLORS['bg_elevated']};
    color: {COLORS['text_bright']};
    border: 1px solid {COLORS['border']};
    selection-background-color: {COLORS['accent']};
    outline: none;
}}
QCheckBox {{
    color: {COLORS['text']};
    spacing: 8px;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border-radius: 4px;
    border: 1px solid {COLORS['border']};
    background-color: {COLORS['bg_widget']};
}}
QCheckBox::indicator:checked {{
    background-color: {COLORS['accent']};
    border-color: {COLORS['accent']};
}}
QSlider::groove:horizontal {{
    height: 4px;
    background: {COLORS['border']};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {COLORS['text_bright']};
    width: 14px;
    height: 14px;
    margin: -5px 0;
    border-radius: 7px;
}}
QSlider::sub-page:horizontal {{
    background: {COLORS['accent']};
    border-radius: 2px;
}}
QSplitter::handle {{
    background-color: {COLORS['bg_dark']};
}}
QSplitter::handle:hover {{
    background-color: {COLORS['border']};
}}
QScrollBar:vertical {{
    background: transparent;
    width: 10px;
    margin: 2px;
}}
QScrollBar::handle:vertical {{
    background: {COLORS['bg_elevated']};
    border-radius: 4px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{
    background: #414146;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0px;
}}
QScrollBar:horizontal {{
    background: transparent;
    height: 10px;
    margin: 2px;
}}
QScrollBar::handle:horizontal {{
    background: {COLORS['bg_elevated']};
    border-radius: 4px;
    min-width: 24px;
}}
QScrollBar::handle:horizontal:hover {{
    background: #414146;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0px;
}}
QTabWidget::pane {{
    border: 1px solid {COLORS['border_soft']};
    border-radius: {RADIUS}px;
    background-color: {COLORS['bg_panel']};
}}
QTabBar::tab {{
    background: transparent;
    color: {COLORS['text_dim']};
    padding: 6px 14px;
    border-bottom: 2px solid transparent;
    font-weight: 600;
}}
QTabBar::tab:selected {{
    color: {COLORS['text_bright']};
    border-bottom: 2px solid {COLORS['accent']};
}}
QTabBar::tab:hover {{
    color: {COLORS['text_bright']};
}}
QProgressBar {{
    border: 1px solid {COLORS['border_soft']};
    border-radius: {RADIUS_SM}px;
    text-align: center;
    color: {COLORS['text_bright']};
    background-color: {COLORS['bg_widget']};
}}
QProgressBar::chunk {{
    background-color: {COLORS['accent']};
    border-radius: {RADIUS_SM - 1}px;
}}
QHeaderView::section {{
    background-color: {COLORS['bg_panel']};
    color: {COLORS['text_dim']};
    border: none;
    border-bottom: 1px solid {COLORS['border_soft']};
    padding: 4px 8px;
    font-weight: 600;
}}
QTableView, QTreeView, QListView {{
    background-color: {COLORS['bg_panel']};
    alternate-background-color: {COLORS['bg_widget']};
    border: 1px solid {COLORS['border_soft']};
    border-radius: {RADIUS_SM}px;
    gridline-color: {COLORS['border_soft']};
}}
QMenu {{
    background-color: {COLORS['bg_elevated']};
    color: {COLORS['text_bright']};
    border: 1px solid {COLORS['border']};
    border-radius: {RADIUS_SM}px;
    padding: 4px;
}}
QMenu::item {{
    padding: 6px 20px;
    border-radius: 4px;
}}
QMenu::item:selected {{
    background-color: {COLORS['accent']};
}}
"""


def status_dot(color: str) -> str:
    return f'<span style="color:{color};font-size:18px;">&#9679;</span>'
