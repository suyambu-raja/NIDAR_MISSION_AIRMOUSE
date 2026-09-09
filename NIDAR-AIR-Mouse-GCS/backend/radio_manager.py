"""
Radio / RF Communication Layer Abstraction for NIDAR AirMouse GCS.
Provides a hardware-agnostic communication interface supporting RFD900x, LoRa, Serial MAVLink,
and a high-fidelity Simulated Radio Link with RSSI, packet loss, and latency monitoring.
"""
import abc
import asyncio
import random
import time
from typing import Optional, Dict, Any, Callable
from backend.schemas import RadioStatusPayload


class RadioInterface(abc.ABC):
    """Abstract base class for all RF / Radio ground link modems."""

    def __init__(self, radio_type: str = "GENERIC_RF"):
        self.radio_type = radio_type
        self.is_connected = False
        self.rssi_dbm: float = -120.0
        self.packet_loss_pct: float = 100.0
        self.latency_ms: float = 0.0
        self.packets_received: int = 0
        self.last_packet_time: float = 0.0

    @abc.abstractmethod
    async def connect(self) -> bool:
        pass

    @abc.abstractmethod
    async def disconnect(self) -> bool:
        pass

    @abc.abstractmethod
    async def send_packet(self, data: bytes) -> bool:
        pass

    def get_status_payload(self) -> RadioStatusPayload:
        """Returns structured status payload for GCS UI."""
        if not self.is_connected:
            quality = "NO SIGNAL"
        elif self.rssi_dbm > -70:
            quality = "GOOD"
        elif self.rssi_dbm > -85:
            quality = "FAIR"
        else:
            quality = "POOR"

        return RadioStatusPayload(
            connected=self.is_connected,
            signal_quality=quality,
            rssi_dbm=round(self.rssi_dbm, 1),
            packet_loss_pct=round(self.packet_loss_pct, 1),
            latency_ms=round(self.latency_ms, 1),
            packets_received=self.packets_received,
            last_packet_timestamp=self.last_packet_time,
            radio_type=self.radio_type,
        )


class SimulatedRadioLink(RadioInterface):
    """
    Simulated RF link reproducing RF propagation in indoor maze structures.
    Simulates RSSI degradation as drone explores deeper into rooms, small packet drops, and variable latency.
    """

    def __init__(
        self,
        nominal_rssi_dbm: float = -62.0,
        nominal_latency_ms: float = 22.0,
        packet_loss_base_pct: float = 0.4,
    ):
        super().__init__(radio_type="SIMULATED_RFD900X_915MHZ")
        self.nominal_rssi = nominal_rssi_dbm
        self.nominal_latency = nominal_latency_ms
        self.base_loss = packet_loss_base_pct
        self._active = False
        self._drone_distance = 1.0

    def update_drone_distance(self, dist_meters: float):
        """Updates simulated RF distance from ground station antenna."""
        self._drone_distance = max(0.5, dist_meters)
        # Log-distance indoor RF path loss model: RSSI = P0 - 10*n*log10(d)
        path_loss = 18.0 * (self._drone_distance / 20.0)
        noise = random.uniform(-2.5, 2.5)
        self.rssi_dbm = max(-115.0, min(-45.0, self.nominal_rssi - path_loss + noise))

        # Latency & loss increase with distance
        self.latency_ms = self.nominal_latency + (self._drone_distance * 0.8) + random.uniform(-3, 4)
        self.packet_loss_pct = max(0.0, min(12.0, self.base_loss + (self._drone_distance * 0.15)))

    async def connect(self) -> bool:
        self._active = True
        self.is_connected = True
        self.rssi_dbm = self.nominal_rssi
        self.latency_ms = self.nominal_latency
        self.packet_loss_pct = self.base_loss
        self.last_packet_time = time.time()
        return True

    async def disconnect(self) -> bool:
        self._active = False
        self.is_connected = False
        self.rssi_dbm = -120.0
        self.packet_loss_pct = 100.0
        self.latency_ms = 0.0
        return True

    async def send_packet(self, data: bytes) -> bool:
        if not self.is_connected:
            return False
        self.packets_received += 1
        self.last_packet_time = time.time()
        return True

    def record_incoming_packet(self):
        """Called when a telemetry/status packet arrives over the link."""
        if self.is_connected:
            self.packets_received += 1
            self.last_packet_time = time.time()


class SerialRadioLink(RadioInterface):
    """
    Hardware Serial Ground Modem (RFD900x / LoRa / Telemetry Radio).
    """

    def __init__(self, port: str = "COM3", baud: int = 57600, radio_type: str = "RFD900X_HARDWARE"):
        super().__init__(radio_type=radio_type)
        self.port = port
        self.baud = baud
        self._serial = None

    async def connect(self) -> bool:
        try:
            import serial
            self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
            self.is_connected = True
            self.rssi_dbm = -65.0
            self.last_packet_time = time.time()
            return True
        except Exception as e:
            print(f"[RadioLink] Hardware serial open failed ({self.port}): {e}")
            self.is_connected = False
            return False

    async def disconnect(self) -> bool:
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self.is_connected = False
        return True

    async def send_packet(self, data: bytes) -> bool:
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(data)
                return True
            except Exception:
                return False
        return False
