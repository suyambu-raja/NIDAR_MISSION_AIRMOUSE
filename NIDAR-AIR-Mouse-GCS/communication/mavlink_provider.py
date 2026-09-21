"""
MAVLink Telemetry Provider for NIDAR AirMouse GCS.
Interfaces with Pixhawk 6C flight controller via pymavlink over Serial (RFD900x) or UDP.
"""
import math
import time
from typing import Optional
from PySide6.QtCore import QObject, Signal, QThread
from communication.message_models import TelemetryData

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


class MavlinkProvider(QObject):
    """
    Manages MAVLink connection to the Pixhawk 6C flight controller.
    Runs reader loop in a dedicated QThread.
    """
    telemetry_received = Signal(object)  # Emits TelemetryData
    connection_changed = Signal(bool)
    status_message = Signal(str)

    def __init__(self, connection_string: str = "udp:127.0.0.1:14550", baud: int = 57600, parent=None):
        super().__init__(parent)
        self.connection_string = connection_string
        self.baud = baud
        self._worker: Optional['_MavlinkWorker'] = None

    def start(self):
        if self._worker is not None:
            return

        self._worker = _MavlinkWorker(self.connection_string, self.baud)
        self._worker.telemetry_ready.connect(self.telemetry_received.emit)
        self._worker.connected_status.connect(self.connection_changed.emit)
        self._worker.log_message.connect(self.status_message.emit)
        self._worker.start()

    def stop(self):
        if self._worker:
            self._worker.stop()
            self._worker.wait(1000)
            self._worker = None
        self.connection_changed.emit(False)

    def send_arm(self, arm: bool = True):
        """Sends MAV_CMD_COMPONENT_ARM_DISARM command to Pixhawk."""
        if self._worker:
            self._worker.send_arm_command(arm)

    def send_mode(self, custom_mode: str):
        """Sets flight mode (e.g. 'GUIDED', 'AUTO', 'LAND', 'BRAKE')."""
        if self._worker:
            self._worker.send_flight_mode(custom_mode)

    def send_emergency_abort(self):
        """Sends emergency land or brake command."""
        self.send_mode("LAND")


class _MavlinkWorker(QThread):
    telemetry_ready = Signal(object)
    connected_status = Signal(bool)
    log_message = Signal(str)

    def __init__(self, connection_string: str, baud: int):
        super().__init__()
        self.connection_string = connection_string
        self.baud = baud
        self._running = True
        self._mav = None

        # Internal state buffer
        self._latest_telem = TelemetryData()
        self._is_connected = False
        self._last_heartbeat = 0.0

    def run(self):
        if mavutil is None:
            self.log_message.emit("Error: pymavlink library is not installed.")
            self.connected_status.emit(False)
            return

        self.log_message.emit(f"Connecting to MAVLink endpoint: {self.connection_string} (Baud: {self.baud})...")
        try:
            self._mav = mavutil.mavlink_connection(self.connection_string, baud=self.baud)
        except Exception as e:
            self.log_message.emit(f"MAVLink connection failed: {e}")
            self.connected_status.emit(False)
            return

        while self._running:
            try:
                # Non-blocking message read with 0.05s timeout
                msg = self._mav.recv_match(blocking=False)
                now = time.time()

                if msg is not None:
                    msg_type = msg.get_type()

                    if msg_type == "HEARTBEAT":
                        self._last_heartbeat = now
                        if not self._is_connected:
                            self._is_connected = True
                            self.connected_status.emit(True)
                            self.log_message.emit(f"Pixhawk Heartbeat acquired (SysID: {msg.get_srcSystem()})")
                            # Request data streams
                            self._request_data_streams()

                        self._latest_telem.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        self._latest_telem.flight_mode = mavutil.mode_string_v10(msg)

                    elif msg_type == "ATTITUDE":
                        self._latest_telem.roll_deg = math.degrees(msg.roll)
                        self._latest_telem.pitch_deg = math.degrees(msg.pitch)
                        self._latest_telem.yaw_deg = (math.degrees(msg.yaw) + 360.0) % 360.0
                        self._latest_telem.heading_deg = self._latest_telem.yaw_deg

                    elif msg_type == "LOCAL_POSITION_NED":
                        self._latest_telem.x_m = msg.x
                        self._latest_telem.y_m = msg.y
                        self._latest_telem.z_m = -msg.z
                        self._latest_telem.altitude_m = -msg.z
                        self._latest_telem.ground_speed_mps = math.hypot(msg.vx, msg.vy)

                    elif msg_type == "BATTERY_STATUS":
                        if len(msg.voltages) > 0 and msg.voltages[0] != 65535:
                            self._latest_telem.battery_voltage_v = msg.voltages[0] / 1000.0
                        if msg.current_battery != -1:
                            self._latest_telem.battery_current_a = msg.current_battery / 100.0
                        if msg.battery_remaining != -1:
                            self._latest_telem.battery_percentage = float(msg.battery_remaining)

                    elif msg_type == "SYS_STATUS":
                        # Estimator / EKF flags
                        sensors_ok = (msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_AHRS) != 0
                        self._latest_telem.ekf_healthy = sensors_ok
                        if msg.voltage_battery != 65535:
                            self._latest_telem.battery_voltage_v = msg.voltage_battery / 1000.0

                    self._latest_telem.timestamp = now
                    self._latest_telem.mavlink_connected = True
                    self.telemetry_ready.emit(self._latest_telem)

                # Check heartbeat timeout (2.5s)
                if self._is_connected and (now - self._last_heartbeat > 2.5):
                    self._is_connected = False
                    self._latest_telem.mavlink_connected = False
                    self.connected_status.emit(False)
                    self.log_message.emit("MAVLink Heartbeat lost!")

                time.sleep(0.01)

            except Exception as e:
                self.log_message.emit(f"MAVLink parse error: {e}")
                time.sleep(0.1)

        if self._mav:
            self._mav.close()

    def _request_data_streams(self):
        """Requests high-frequency data streams from Pixhawk."""
        if not self._mav:
            return
        # Request all streams at 10 Hz
        self._mav.mav.request_data_stream_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )

    def send_arm_command(self, arm: bool):
        if not self._mav:
            return
        arm_val = 1 if arm else 0
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_val, 0, 0, 0, 0, 0, 0
        )

    def send_flight_mode(self, mode_str: str):
        if not self._mav:
            return
        mode_id = self._mav.mode_mapping().get(mode_str.upper())
        if mode_id is not None:
            self._mav.set_mode(mode_id)

    def stop(self):
        self._running = False
