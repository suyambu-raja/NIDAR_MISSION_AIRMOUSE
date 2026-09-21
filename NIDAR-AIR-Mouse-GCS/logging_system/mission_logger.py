"""
Mission Logger for NIDAR AirMouse GCS.
Provides offline, structured SQLite persistence for telemetry, events, survivor detections, and flight metrics.
"""
import os
import sqlite3
import time
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any, Optional
from PySide6.QtCore import QObject, Slot
from communication.message_models import TelemetryData, SurvivorDetection, MissionEvent


class MissionLogger(QObject):
    """
    Handles SQLite database creation and asynchronous/synchronous inserts for all mission data.
    Ensures zero cloud dependency and 100% local persistence.
    """

    def __init__(self, log_dir: Optional[str] = None, parent=None):
        super().__init__(parent)
        self.log_dir = Path(log_dir) if log_dir else Path(__file__).resolve().parent.parent / "logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)

        session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.db_path = self.log_dir / f"mission_{session_timestamp}.db"
        self._init_db()

    def _init_db(self):
        """Creates database schema if it doesn't exist."""
        with sqlite3.connect(self.db_path) as conn:
            cursor = conn.cursor()
            
            # Events table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    datetime_str TEXT NOT NULL,
                    level TEXT NOT NULL,
                    category TEXT NOT NULL,
                    message TEXT NOT NULL
                )
            """)

            # Telemetry records
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS telemetry (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    flight_mode TEXT,
                    armed INTEGER,
                    x_m REAL,
                    y_m REAL,
                    z_m REAL,
                    altitude_m REAL,
                    speed_mps REAL,
                    heading_deg REAL,
                    roll_deg REAL,
                    pitch_deg REAL,
                    yaw_deg REAL,
                    battery_v REAL,
                    battery_pct REAL
                )
            """)

            # Survivor Detections
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS survivors (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    survivor_id TEXT NOT NULL,
                    grid_cell TEXT NOT NULL,
                    x_m REAL,
                    y_m REAL,
                    confidence REAL,
                    status TEXT,
                    source TEXT
                )
            """)

            # Mission metadata
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT
                )
            """)
            conn.commit()

        self.log_event(MissionEvent(
            timestamp=time.time(),
            level="INFO",
            category="SYSTEM",
            message=f"Mission database initialized at {self.db_path.name}"
        ))

    @Slot(object)
    def log_event(self, event: MissionEvent):
        """Logs a MissionEvent instance into SQLite."""
        try:
            dt_str = datetime.fromtimestamp(event.timestamp).strftime("%H:%M:%S.%f")[:-3]
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute(
                    "INSERT INTO events (timestamp, datetime_str, level, category, message) VALUES (?, ?, ?, ?, ?)",
                    (event.timestamp, dt_str, event.level, event.category, event.message)
                )
                conn.commit()
        except Exception as e:
            print(f"[MissionLogger] Error logging event: {e}")

    @Slot(object)
    def log_telemetry(self, telem: TelemetryData):
        """Logs a TelemetryData snapshot into SQLite."""
        try:
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT INTO telemetry (
                        timestamp, flight_mode, armed, x_m, y_m, z_m,
                        altitude_m, speed_mps, heading_deg, roll_deg,
                        pitch_deg, yaw_deg, battery_v, battery_pct
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    telem.timestamp,
                    telem.flight_mode,
                    1 if telem.armed else 0,
                    telem.x_m,
                    telem.y_m,
                    telem.z_m,
                    telem.altitude_m,
                    telem.ground_speed_mps,
                    telem.heading_deg,
                    telem.roll_deg,
                    telem.pitch_deg,
                    telem.yaw_deg,
                    telem.battery_voltage_v,
                    telem.battery_percentage,
                ))
                conn.commit()
        except Exception as e:
            print(f"[MissionLogger] Error logging telemetry: {e}")

    @Slot(object)
    def log_survivor(self, detection: SurvivorDetection):
        """Logs a SurvivorDetection into SQLite."""
        try:
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT INTO survivors (
                        timestamp, survivor_id, grid_cell, x_m, y_m, confidence, status, source
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    detection.timestamp,
                    detection.survivor_id,
                    detection.grid_cell,
                    detection.x,
                    detection.y,
                    detection.confidence,
                    detection.status,
                    detection.source,
                ))
                conn.commit()
        except Exception as e:
            print(f"[MissionLogger] Error logging survivor: {e}")

    def get_recent_events(self, limit: int = 100) -> List[Dict[str, Any]]:
        """Retrieves recent event logs for UI inspection."""
        try:
            with sqlite3.connect(self.db_path) as conn:
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()
                cursor.execute(
                    "SELECT timestamp, datetime_str, level, category, message FROM events ORDER BY id DESC LIMIT ?",
                    (limit,)
                )
                rows = cursor.fetchall()
                return [dict(row) for row in reversed(rows)]
        except Exception as e:
            print(f"[MissionLogger] Error reading events: {e}")
            return []
