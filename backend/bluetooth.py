"""
PySerial HC-05 Bluetooth Hardware Interface Manager
Provides robust serial connection to the TETRIX PRIZM robot controller
with non-blocking reception and graceful simulation fallback.
"""

import sys
import time
import threading
from typing import Optional, List, Callable, Dict, Any
from .config import BLUETOOTH_PORT, BLUETOOTH_BAUD, SERIAL_TIMEOUT
from .protocol import parse_telemetry_line

try:
    import serial
    import serial.tools.list_ports
    PYSERIAL_AVAILABLE = True
except ImportError:
    PYSERIAL_AVAILABLE = False

class BluetoothManager:
    """Manages serial communication over HC-05 Bluetooth transceiver."""

    def __init__(self, port: str = BLUETOOTH_PORT, baud: int = BLUETOOTH_BAUD):
        self.port = port
        self.baud = baud
        self.serial_conn: Optional[Any] = None
        self.is_connected = False
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self.telemetry_callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self.last_error: Optional[str] = None

    def list_available_ports(self) -> List[Dict[str, str]]:
        """Scans host system for available COM / tty ports."""
        if not PYSERIAL_AVAILABLE:
            return []
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append({
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid
            })
        return ports

    def connect(self, port: Optional[str] = None, baud: Optional[int] = None) -> bool:
        """Attempts connection to designated Bluetooth serial COM port."""
        if not PYSERIAL_AVAILABLE:
            self.last_error = "PySerial library not installed"
            return False

        if port:
            self.port = port
        if baud:
            self.baud = baud

        self.disconnect()

        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT
            )
            self.is_connected = True
            self.last_error = None
            self._running = True

            self._read_thread = threading.Thread(target=self._read_worker, daemon=True)
            self._read_thread.start()
            return True
        except Exception as e:
            self.is_connected = False
            self.last_error = str(e)
            return False

    def disconnect(self):
        """Disconnects serial port and cleans up reader thread."""
        self._running = False
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        self.serial_conn = None
        self.is_connected = False

    def send_line(self, line: str) -> bool:
        """Sends an ASCII line to TETRIX PRIZM controller."""
        if not self.is_connected or not self.serial_conn:
            return False
        try:
            if not line.endswith("\n"):
                line += "\n"
            self.serial_conn.write(line.encode("ascii"))
            self.serial_conn.flush()
            return True
        except Exception as e:
            self.last_error = str(e)
            return False

    def register_callback(self, cb: Callable[[Dict[str, Any]], None]):
        self.telemetry_callbacks.append(cb)

    def _read_worker(self):
        """Continuous background thread reading telemetry lines from PRIZM."""
        while self._running and self.serial_conn and self.serial_conn.is_open:
            try:
                raw_line = self.serial_conn.readline()
                if raw_line:
                    text = raw_line.decode("ascii", errors="ignore").strip()
                    parsed = parse_telemetry_line(text)
                    if parsed:
                        for cb in self.telemetry_callbacks:
                            try:
                                cb(parsed)
                            except Exception:
                                pass
            except Exception as e:
                self.last_error = str(e)
                time.sleep(0.5)

bt_manager = BluetoothManager()
