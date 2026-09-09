"""
Mission Log Viewer & Exporter Dialog for NIDAR AirMouse GCS.
Enables offline review of SQLite logs and export to CSV/JSON formats.
"""
from pathlib import Path
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QPushButton,
    QLabel,
    QFileDialog,
    QMessageBox,
    QHeaderView,
)
from logging_system.mission_logger import MissionLogger
from logging_system.log_exporter import LogExporter


class LogDialog(QDialog):
    """
    Log viewer dialog to inspect events and export SQLite records.
    """

    def __init__(self, logger: MissionLogger, parent=None):
        super().__init__(parent)
        self.logger = logger
        self.setWindowTitle("NIDAR AirMouse - Mission Log Viewer")
        self.resize(750, 480)
        self._init_ui()
        self._load_data()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # Header Info
        hdr = QHBoxLayout()
        lbl_db = QLabel(f"Database: {self.logger.db_path.name}")
        lbl_db.setStyleSheet("color: #00e5ff; font-weight: bold;")
        
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.setFixedWidth(80)
        self.btn_refresh.clicked.connect(self._load_data)

        hdr.addWidget(lbl_db)
        hdr.addStretch()
        hdr.addWidget(self.btn_refresh)
        layout.addLayout(hdr)

        # Events Table
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Time", "Level", "Category", "Message"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        layout.addWidget(self.table, 1)

        # Export & Close Actions
        action_box = QHBoxLayout()
        action_box.setSpacing(8)

        self.btn_exp_events = QPushButton("Export Events (CSV)")
        self.btn_exp_events.clicked.connect(self._export_events)

        self.btn_exp_telem = QPushButton("Export Telemetry (CSV)")
        self.btn_exp_telem.clicked.connect(self._export_telemetry)

        self.btn_exp_surv = QPushButton("Export Survivors (JSON)")
        self.btn_exp_surv.clicked.connect(self._export_survivors)

        self.btn_close = QPushButton("Close")
        self.btn_close.setFixedWidth(80)
        self.btn_close.clicked.connect(self.accept)

        action_box.addWidget(self.btn_exp_events)
        action_box.addWidget(self.btn_exp_telem)
        action_box.addWidget(self.btn_exp_surv)
        action_box.addStretch()
        action_box.addWidget(self.btn_close)

        layout.addLayout(action_box)

    def _load_data(self):
        """Populates the table with recent events."""
        events = self.logger.get_recent_events(limit=200)
        self.table.setRowCount(len(events))

        for row, evt in enumerate(events):
            item_time = QTableWidgetItem(evt.get("datetime_str", ""))
            item_level = QTableWidgetItem(evt.get("level", ""))
            item_cat = QTableWidgetItem(evt.get("category", ""))
            item_msg = QTableWidgetItem(evt.get("message", ""))

            # Color styling based on level
            level = evt.get("level", "")
            if level == "CRITICAL" or level == "ERROR":
                item_level.setForeground(Qt.red)
            elif level == "WARNING":
                item_level.setForeground(Qt.yellow)
            elif level == "SUCCESS":
                item_level.setForeground(Qt.green)
            else:
                item_level.setForeground(Qt.cyan)

            self.table.setItem(row, 0, item_time)
            self.table.setItem(row, 1, item_level)
            self.table.setItem(row, 2, item_cat)
            self.table.setItem(row, 3, item_msg)

    def _export_events(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Events CSV", "mission_events.csv", "CSV Files (*.csv)")
        if path:
            ok = LogExporter.export_events_to_csv(self.logger.db_path, Path(path))
            if ok:
                QMessageBox.information(self, "Success", f"Events exported to {path}")

    def _export_telemetry(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Telemetry CSV", "mission_telemetry.csv", "CSV Files (*.csv)")
        if path:
            ok = LogExporter.export_telemetry_to_csv(self.logger.db_path, Path(path))
            if ok:
                QMessageBox.information(self, "Success", f"Telemetry exported to {path}")

    def _export_survivors(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Survivors JSON", "mission_survivors.json", "JSON Files (*.json)")
        if path:
            ok = LogExporter.export_survivors_to_json(self.logger.db_path, Path(path))
            if ok:
                QMessageBox.information(self, "Success", f"Survivors exported to {path}")
