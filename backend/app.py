"""
GPS Waypoint Navigation Robot — Flask Ground Station Backend
Provides REST API endpoints for mission planning, hardware telemetry,
PySerial Bluetooth link, and simulation kinematics.
"""

import os
import json
import time
from flask import Flask, request, jsonify, send_from_directory
from backend.config import FRONTEND_DIR, DATA_DIR, SIMULATION_MODE_DEFAULT, SERVER_HOST, SERVER_PORT
from backend.protocol import serialize_mission, serialize_command
from backend.simulation import sim_rover, haversine_distance
from backend.bluetooth import bt_manager

app = Flask(__name__, static_folder=FRONTEND_DIR, static_url_path="")

# Runtime State
system_state = {
    "simulation_mode": SIMULATION_MODE_DEFAULT,
    "last_mission": None,
    "physical_telemetry": {
        "lat": None,
        "lon": None,
        "heading": 0.0,
        "bearing": 0.0,
        "distance": 0.0,
        "current_wp": None,
        "status": "DISCONNECTED"
    }
}

def on_hardware_telemetry(parsed_data):
    """Callback for PRIZM incoming telemetry lines."""
    t_type = parsed_data.get("type")
    pt = system_state["physical_telemetry"]
    if t_type == "gps":
        pt["lat"] = parsed_data["lat"]
        pt["lon"] = parsed_data["lon"]
    elif t_type == "heading":
        pt["heading"] = parsed_data["heading"]
    elif t_type == "bearing":
        pt["bearing"] = parsed_data["bearing"]
    elif t_type == "distance":
        pt["distance"] = parsed_data["distance"]
    elif t_type == "current_wp":
        pt["current_wp"] = f"W{parsed_data['wp_id']}"
    elif t_type == "status":
        pt["status"] = parsed_data["status"]

bt_manager.register_callback(on_hardware_telemetry)

# ----------------- Static Frontend Routes -----------------

@app.route("/")
def index():
    return send_from_directory(FRONTEND_DIR, "index.html")

@app.route("/<path:path>")
def static_files(path):
    return send_from_directory(FRONTEND_DIR, path)

# ----------------- REST API Endpoints -----------------

@app.route("/api/status", methods=["GET"])
def get_system_status():
    """Returns ground station state and hardware connection status."""
    ports = bt_manager.list_available_ports()
    return jsonify({
        "status": "ok",
        "simulation_mode": system_state["simulation_mode"],
        "bluetooth": {
            "connected": bt_manager.is_connected,
            "port": bt_manager.port,
            "baud": bt_manager.baud,
            "error": bt_manager.last_error,
            "available_ports": ports
        },
        "has_active_mission": system_state["last_mission"] is not None
    })

@app.route("/api/mode", methods=["POST"])
def set_operational_mode():
    """Toggle or set simulation mode."""
    data = request.get_json() or {}
    sim_mode = data.get("simulation", True)
    system_state["simulation_mode"] = bool(sim_mode)
    return jsonify({
        "status": "ok",
        "simulation_mode": system_state["simulation_mode"]
    })

@app.route("/api/mission", methods=["POST"])
def upload_mission():
    """
    Receives planned waypoint mission from frontend.
    Validates 1-5 waypoints with latitude and longitude.
    """
    data = request.get_json() or {}
    waypoints = data.get("waypoints", [])

    if not isinstance(waypoints, list) or len(waypoints) < 1:
        return jsonify({"error": "Mission must contain between 1 and 5 waypoints"}), 400
    if len(waypoints) > 5:
        return jsonify({"error": "Maximum 5 waypoints allowed"}), 400

    clean_waypoints = []
    for idx, wp in enumerate(waypoints, start=1):
        try:
            lat = float(wp["lat"])
            lon = float(wp["lon"])
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                return jsonify({"error": f"Waypoint W{idx} coordinates out of bounds"}), 400
            clean_waypoints.append({
                "id": idx,
                "lat": round(lat, 6),
                "lon": round(lon, 6)
            })
        except (KeyError, ValueError, TypeError):
            return jsonify({"error": f"Invalid coordinate format at waypoint {idx}"}), 400

    # Calculate total planned distance
    total_dist = 0.0
    for i in range(len(clean_waypoints) - 1):
        total_dist += haversine_distance(
            clean_waypoints[i]["lat"], clean_waypoints[i]["lon"],
            clean_waypoints[i+1]["lat"], clean_waypoints[i+1]["lon"]
        )

    mission_payload = {
        "command": "MISSION",
        "timestamp": int(time.time()),
        "waypoint_count": len(clean_waypoints),
        "total_distance_m": round(total_dist, 1),
        "waypoints": clean_waypoints
    }

    system_state["last_mission"] = mission_payload

    # Persist mission profile to JSON
    try:
        mission_filepath = os.path.join(DATA_DIR, "active_mission.json")
        with open(mission_filepath, "w") as f:
            json.dump(mission_payload, f, indent=2)
    except Exception as e:
        app.logger.warning(f"Failed to persist mission JSON: {e}")

    # Load into simulation engine
    sim_rover.load_mission(clean_waypoints)

    # If physical Bluetooth is connected and not in simulation, transmit ASCII frames
    if not system_state["simulation_mode"] and bt_manager.is_connected:
        ascii_protocol = serialize_mission(clean_waypoints)
        bt_manager.send_line(ascii_protocol)

    return jsonify({
        "status": "ok",
        "message": f"Mission with {len(clean_waypoints)} waypoints armed",
        "mission": mission_payload
    })

@app.route("/api/command", methods=["POST"])
def execute_command():
    """
    Handles control commands: START, PAUSE, RESUME, STOP, CLEAR
    """
    data = request.get_json() or {}
    cmd = data.get("command", "").upper()

    if cmd not in {"START", "PAUSE", "RESUME", "STOP", "CLEAR"}:
        return jsonify({"error": f"Invalid command '{cmd}'"}), 400

    # Route command to simulation or hardware
    if system_state["simulation_mode"] or not bt_manager.is_connected:
        if cmd == "START":
            sim_rover.start_mission()
        elif cmd == "PAUSE":
            sim_rover.pause_mission()
        elif cmd == "RESUME":
            sim_rover.resume_mission()
        elif cmd == "STOP":
            sim_rover.stop_mission()
        elif cmd == "CLEAR":
            sim_rover.clear_mission()
            system_state["last_mission"] = None
    else:
        # Physical robot
        try:
            ascii_cmd = serialize_command(cmd)
            bt_manager.send_line(ascii_cmd)
        except Exception as e:
            return jsonify({"error": f"Failed to send command to robot: {e}"}), 500

    return jsonify({"status": "ok", "command": cmd})

@app.route("/api/telemetry", methods=["GET"])
def get_telemetry():
    """Returns current robot telemetry, position, heading, and status."""
    if system_state["simulation_mode"] or not bt_manager.is_connected:
        data = sim_rover.get_telemetry()
        return jsonify(data)
    else:
        pt = system_state["physical_telemetry"]
        return jsonify({
            "is_simulation": False,
            "connection": "CONNECTED" if bt_manager.is_connected else "DISCONNECTED",
            "gps_fix": "3D FIX" if pt["lat"] is not None else "NO FIX",
            "status": pt["status"],
            "lat": pt["lat"],
            "lon": pt["lon"],
            "heading": pt["heading"],
            "bearing": pt["bearing"],
            "distance_to_target_m": pt["distance"],
            "current_waypoint": pt["current_wp"]
        })

@app.route("/api/bluetooth/connect", methods=["POST"])
def connect_bluetooth():
    """Attempts connection to designated COM port."""
    data = request.get_json() or {}
    port = data.get("port")
    baud = data.get("baud", 9600)
    success = bt_manager.connect(port=port, baud=int(baud))
    if success:
        system_state["simulation_mode"] = False
        return jsonify({"status": "connected", "port": bt_manager.port, "baud": bt_manager.baud})
    else:
        return jsonify({"status": "failed", "error": bt_manager.last_error}), 400

@app.route("/api/bluetooth/disconnect", methods=["POST"])
def disconnect_bluetooth():
    bt_manager.disconnect()
    return jsonify({"status": "disconnected"})

@app.route("/api/bluetooth/ports", methods=["GET"])
def list_ports():
    ports = bt_manager.list_available_ports()
    return jsonify({"ports": ports})

if __name__ == "__main__":
    app.run(host=SERVER_HOST, port=SERVER_PORT, debug=True)
