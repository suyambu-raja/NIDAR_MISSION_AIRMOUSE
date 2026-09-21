"""
Styles and Theme Definitions for NIDAR AirMouse GCS.
Provides a modern, high-contrast, dark tactical aeronautical theme.
"""

DARK_THEME_QSS = """
/* Global Window & Font Settings */
QWidget {
    background-color: #121418;
    color: #e0e6ed;
    font-family: "Segoe UI", "Roboto", "Helvetica Neue", sans-serif;
    font-size: 12px;
}

/* Group Boxes & Panels */
QGroupBox {
    background-color: #181b22;
    border: 1px solid #282f3a;
    border-radius: 6px;
    margin-top: 22px;
    font-weight: bold;
    font-size: 12px;
    color: #00bcd4;
    padding: 10px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 2px 8px;
    background-color: #1f242d;
    border: 1px solid #2d3748;
    border-radius: 4px;
    color: #00e5ff;
}

/* Push Buttons */
QPushButton {
    background-color: #212733;
    color: #ffffff;
    border: 1px solid #364153;
    border-radius: 4px;
    padding: 6px 14px;
    font-weight: 600;
    min-height: 22px;
}

QPushButton:hover {
    background-color: #2d3748;
    border-color: #00bcd4;
    color: #00e5ff;
}

QPushButton:pressed {
    background-color: #1a202c;
}

QPushButton:disabled {
    background-color: #15181f;
    color: #4a5568;
    border-color: #222731;
}

/* Primary Action Buttons */
QPushButton#btn_start {
    background-color: #00875a;
    border: 1px solid #00b875;
    color: #ffffff;
    font-size: 13px;
    font-weight: bold;
    padding: 8px 18px;
}

QPushButton#btn_start:hover {
    background-color: #00a36c;
    border-color: #00e676;
}

QPushButton#btn_start:disabled {
    background-color: #1b382b;
    border-color: #244b3a;
    color: #5c8370;
}

/* Emergency Abort Button */
QPushButton#btn_abort {
    background-color: #d32f2f;
    border: 2px solid #ff5252;
    color: #ffffff;
    font-size: 14px;
    font-weight: bold;
    letter-spacing: 1px;
    padding: 8px 22px;
}

QPushButton#btn_abort:hover {
    background-color: #f44336;
    border-color: #ff8a80;
}

/* Connect / Disconnect Buttons */
QPushButton#btn_connect {
    background-color: #0288d1;
    border: 1px solid #29b6f6;
    color: #ffffff;
}

QPushButton#btn_connect:hover {
    background-color: #039be5;
}

/* Combo Boxes & Inputs */
QComboBox {
    background-color: #1e242f;
    border: 1px solid #364153;
    border-radius: 4px;
    padding: 4px 8px;
    color: #e2e8f0;
    min-width: 100px;
}

QComboBox::drop-down {
    border: none;
    width: 20px;
}

QComboBox QAbstractItemView {
    background-color: #1e242f;
    border: 1px solid #364153;
    selection-background-color: #00bcd4;
    selection-color: #000000;
}

/* Progress Bars */
QProgressBar {
    background-color: #1a1e26;
    border: 1px solid #2e3846;
    border-radius: 4px;
    text-align: center;
    color: #ffffff;
    font-weight: bold;
}

QProgressBar::chunk {
    background-color: #00bcd4;
    border-radius: 3px;
}

/* Tables & Tree Views */
QTableWidget, QTableView {
    background-color: #161920;
    border: 1px solid #28303d;
    gridline-color: #222936;
    color: #e0e6ed;
    selection-background-color: #1e3a4a;
    selection-color: #00e5ff;
}

QHeaderView::section {
    background-color: #1e242e;
    color: #8fa0b5;
    border: 1px solid #28303d;
    padding: 4px 6px;
    font-weight: bold;
}

/* Scrollbars */
QScrollBar:vertical {
    background: #14171d;
    width: 10px;
    margin: 0px;
}

QScrollBar::handle:vertical {
    background: #2a3342;
    min-height: 20px;
    border-radius: 4px;
}

QScrollBar::handle:vertical:hover {
    background: #3b485d;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* Labels */
QLabel {
    color: #c5d1de;
}

QLabel#telemetry_value {
    font-family: "Consolas", "Courier New", monospace;
    font-size: 13px;
    font-weight: bold;
    color: #ffffff;
}

QLabel#telemetry_unit {
    font-size: 11px;
    color: #718096;
}

/* Status Badges */
QLabel#badge_ok {
    background-color: #143825;
    color: #00e676;
    border: 1px solid #00b875;
    border-radius: 4px;
    padding: 2px 8px;
    font-weight: bold;
}

QLabel#badge_warn {
    background-color: #3d3112;
    color: #ffca28;
    border: 1px solid #ffb300;
    border-radius: 4px;
    padding: 2px 8px;
    font-weight: bold;
}

QLabel#badge_err {
    background-color: #3d1416;
    color: #ff5252;
    border: 1px solid #d32f2f;
    border-radius: 4px;
    padding: 2px 8px;
    font-weight: bold;
}
"""

COLOR_PALETTE = {
    "bg_main": "#121418",
    "bg_panel": "#181b22",
    "border": "#282f3a",
    "accent_cyan": "#00e5ff",
    "accent_blue": "#0288d1",
    "status_green": "#00e676",
    "status_yellow": "#ffca28",
    "status_red": "#ff5252",
    "map_unexplored": "#161920",
    "map_free": "#2a3442",
    "map_occupied": "#ffffff",
    "map_grid_lines": "#222a36",
    "drone_marker": "#ffea00",
    "survivor_marker": "#00e676",
}
