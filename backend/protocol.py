"""
PRIZM Communication Protocol Parser & Serializer
Handles conversion between JSON mission packets and lightweight ASCII streams
for the TETRIX PRIZM microcontroller via HC-05 Bluetooth.
"""

from typing import List, Dict, Any, Optional

def serialize_mission(waypoints: List[Dict[str, Any]]) -> str:
    """
    Serializes a list of waypoints into PRIZM ASCII protocol format.
    Format:
      MISSION,<count>\n
      WP,<id>,<lat>,<lon>\n
      ...
      END\n
    """
    lines = [f"MISSION,{len(waypoints)}"]
    for idx, wp in enumerate(waypoints, start=1):
        wp_id = wp.get("id", idx)
        lat = f"{float(wp['lat']):.6f}"
        lon = f"{float(wp['lon']):.6f}"
        lines.append(f"WP,{wp_id},{lat},{lon}")
    lines.append("END")
    return "\n".join(lines) + "\n"

def serialize_command(command: str) -> str:
    """
    Serializes standard control commands:
    START, PAUSE, RESUME, STOP, CLEAR
    """
    cmd = command.strip().upper()
    valid_commands = {"START", "PAUSE", "RESUME", "STOP", "CLEAR"}
    if cmd not in valid_commands:
        raise ValueError(f"Unknown robot command: {command}")
    return f"{cmd}\n"

def parse_telemetry_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Parses incoming telemetry line from PRIZM controller.
    Examples:
      GPS,17.780421,83.374920
      HEAD,82.4
      BEAR,88.7
      DIST,12.4
      WP,2
      STATUS,RUNNING
    """
    line = line.strip()
    if not line:
        return None

    parts = line.split(",")
    header = parts[0].upper()

    try:
        if header == "GPS" and len(parts) >= 3:
            return {"type": "gps", "lat": float(parts[1]), "lon": float(parts[2])}
        elif header == "HEAD" and len(parts) >= 2:
            return {"type": "heading", "heading": float(parts[1])}
        elif header == "BEAR" and len(parts) >= 2:
            return {"type": "bearing", "bearing": float(parts[1])}
        elif header == "DIST" and len(parts) >= 2:
            return {"type": "distance", "distance": float(parts[1])}
        elif header == "WP" and len(parts) >= 2:
            return {"type": "current_wp", "wp_id": int(parts[1])}
        elif header == "STATUS" and len(parts) >= 2:
            return {"type": "status", "status": parts[1].upper()}
    except (ValueError, IndexError):
        pass

    return {"type": "raw", "data": line}
