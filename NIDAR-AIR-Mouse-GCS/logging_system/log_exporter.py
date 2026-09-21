"""
Log Exporter for NIDAR AirMouse GCS.
Exports SQLite mission logs to CSV or JSON formats for competition submission and post-mission debriefs.
"""
import csv
import json
import sqlite3
from pathlib import Path
from typing import Optional, List, Dict, Any


class LogExporter:
    """Utility to export mission events, telemetry traces, and survivor detections."""

    @staticmethod
    def export_events_to_csv(db_path: Path, output_file: Path) -> bool:
        """Exports the events table to CSV."""
        try:
            with sqlite3.connect(db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT id, timestamp, datetime_str, level, category, message FROM events ORDER BY id ASC")
                rows = cursor.fetchall()

            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["ID", "Timestamp", "DateTime", "Level", "Category", "Message"])
                writer.writerows(rows)
            return True
        except Exception as e:
            print(f"[LogExporter] Failed to export events CSV: {e}")
            return False

    @staticmethod
    def export_survivors_to_json(db_path: Path, output_file: Path) -> bool:
        """Exports detected survivors to formatted JSON for competition scorekeeping."""
        try:
            with sqlite3.connect(db_path) as conn:
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()
                cursor.execute("SELECT survivor_id, grid_cell, x_m, y_m, confidence, status, source, timestamp FROM survivors ORDER BY id ASC")
                rows = cursor.fetchall()

            data = [dict(row) for row in rows]
            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, "w", encoding="utf-8") as f:
                json.dump({"survivor_detections": data}, f, indent=2)
            return True
        except Exception as e:
            print(f"[LogExporter] Failed to export survivors JSON: {e}")
            return False

    @staticmethod
    def export_telemetry_to_csv(db_path: Path, output_file: Path) -> bool:
        """Exports high-frequency flight telemetry to CSV."""
        try:
            with sqlite3.connect(db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT id, timestamp, flight_mode, armed, x_m, y_m, z_m,
                           altitude_m, speed_mps, heading_deg, roll_deg,
                           pitch_deg, yaw_deg, battery_v, battery_pct
                    FROM telemetry ORDER BY id ASC
                """)
                rows = cursor.fetchall()

            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "ID", "Timestamp", "FlightMode", "Armed", "X_m", "Y_m", "Z_m",
                    "Altitude_m", "Speed_mps", "Heading_deg", "Roll_deg",
                    "Pitch_deg", "Yaw_deg", "Battery_V", "Battery_Pct"
                ])
                writer.writerows(rows)
            return True
        except Exception as e:
            print(f"[LogExporter] Failed to export telemetry CSV: {e}")
            return False
