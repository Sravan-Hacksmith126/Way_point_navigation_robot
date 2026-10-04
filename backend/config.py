import os

# Base Directories
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRONTEND_DIR = os.path.join(BASE_DIR, "frontend")
DATA_DIR = os.path.join(BASE_DIR, "data", "missions")

# Ensure data directory exists
os.makedirs(DATA_DIR, exist_ok=True)

# Hardware Configuration (HC-05 via PySerial)
BLUETOOTH_PORT = os.environ.get("ROBOT_COM_PORT", "COM3")
BLUETOOTH_BAUD = int(os.environ.get("ROBOT_BAUD_RATE", 9600))
SERIAL_TIMEOUT = 1.0

# Simulation Configuration
SIMULATION_MODE_DEFAULT = True
SIMULATION_SPEED_MPS = 1.5  # Approximate physical robot speed in meters/second
TELEMETRY_INTERVAL_SEC = 0.5

# Web Server
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 5000
