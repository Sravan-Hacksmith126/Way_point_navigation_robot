"""
Autonomous GPS Waypoint Navigation Simulation Engine
Calculates geodesic trajectories using Haversine formula and forward azimuth,
and simulates rover kinematics and telemetry without physical hardware.
"""

import math
import time
import threading
from typing import List, Dict, Any, Optional

EARTH_RADIUS_M = 6371000.0

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculates geodesic distance between two coordinates in meters."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c

def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculates initial forward bearing/azimuth from point 1 to point 2 in degrees [0, 360)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)

    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360.0) % 360.0

class SimulationRover:
    """Simulates physical TETRIX PRIZM rover moving along GPS waypoints."""

    def __init__(self, speed_mps: float = 1.5):
        self.speed_mps = speed_mps
        self.waypoints: List[Dict[str, Any]] = []
        self.current_wp_index = 0
        self.current_lat: Optional[float] = None
        self.current_lon: Optional[float] = None
        self.heading: float = 0.0
        self.bearing: float = 0.0
        self.distance_to_target: float = 0.0
        self.total_planned_distance: float = 0.0

        self.status = "READY"  # READY, RUNNING, PAUSED, STOPPED, COMPLETED
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def load_mission(self, waypoints: List[Dict[str, Any]]) -> Dict[str, Any]:
        with self._lock:
            self.waypoints = [
                {
                    "id": wp.get("id", i + 1),
                    "lat": float(wp["lat"]),
                    "lon": float(wp["lon"]),
                    "status": "PENDING"
                }
                for i, wp in enumerate(waypoints)
            ]
            self.current_wp_index = 0
            if self.waypoints:
                self.current_lat = self.waypoints[0]["lat"]
                self.current_lon = self.waypoints[0]["lon"]
                if len(self.waypoints) > 1:
                    self.bearing = calculate_bearing(
                        self.current_lat, self.current_lon,
                        self.waypoints[1]["lat"], self.waypoints[1]["lon"]
                    )
                    self.heading = self.bearing
                    self.distance_to_target = haversine_distance(
                        self.current_lat, self.current_lon,
                        self.waypoints[1]["lat"], self.waypoints[1]["lon"]
                    )
                else:
                    self.bearing = 0.0
                    self.heading = 0.0
                    self.distance_to_target = 0.0

            # Calculate total route distance
            total_dist = 0.0
            for i in range(len(self.waypoints) - 1):
                total_dist += haversine_distance(
                    self.waypoints[i]["lat"], self.waypoints[i]["lon"],
                    self.waypoints[i+1]["lat"], self.waypoints[i+1]["lon"]
                )
            self.total_planned_distance = total_dist
            self.status = "READY"

            return {
                "waypoint_count": len(self.waypoints),
                "total_distance_m": round(self.total_planned_distance, 1),
                "status": self.status
            }

    def start_mission(self):
        with self._lock:
            if not self.waypoints:
                return False
            if self.status == "RUNNING":
                return True
            self.status = "RUNNING"
            self._running = True

            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._run_loop, daemon=True)
                self._thread.start()
            return True

    def pause_mission(self):
        with self._lock:
            if self.status == "RUNNING":
                self.status = "PAUSED"
                return True
            return False

    def resume_mission(self):
        with self._lock:
            if self.status == "PAUSED":
                self.status = "RUNNING"
                return True
            return False

    def stop_mission(self):
        with self._lock:
            self.status = "STOPPED"
            self._running = False
            return True

    def clear_mission(self):
        with self._lock:
            self.status = "READY"
            self._running = False
            self.waypoints = []
            self.current_wp_index = 0
            self.distance_to_target = 0.0
            self.total_planned_distance = 0.0
            return True

    def get_telemetry(self) -> Dict[str, Any]:
        with self._lock:
            curr_wp_label = None
            if self.waypoints and self.current_wp_index < len(self.waypoints):
                curr_wp_label = f"W{self.waypoints[self.current_wp_index]['id']}"

            return {
                "is_simulation": True,
                "connection": "CONNECTED",
                "gps_fix": "3D DGPS FIX",
                "status": self.status,
                "lat": round(self.current_lat, 6) if self.current_lat is not None else None,
                "lon": round(self.current_lon, 6) if self.current_lon is not None else None,
                "heading": round(self.heading, 1),
                "bearing": round(self.bearing, 1),
                "distance_to_target_m": round(self.distance_to_target, 1),
                "current_waypoint": curr_wp_label,
                "current_waypoint_index": self.current_wp_index,
                "total_waypoints": len(self.waypoints),
                "planned_distance_m": round(self.total_planned_distance, 1),
                "waypoints": [
                    {
                        "id": wp["id"],
                        "lat": round(wp["lat"], 6),
                        "lon": round(wp["lon"], 6),
                        "status": wp["status"]
                    }
                    for wp in self.waypoints
                ]
            }

    def _run_loop(self):
        dt = 0.25  # Simulation step 250ms
        while self._running:
            with self._lock:
                if self.status != "RUNNING":
                    time.sleep(dt)
                    continue

                if self.current_wp_index >= len(self.waypoints):
                    self.status = "COMPLETED"
                    self._running = False
                    break

                target = self.waypoints[self.current_wp_index]
                target_lat, target_lon = target["lat"], target["lon"]

                dist = haversine_distance(self.current_lat, self.current_lon, target_lat, target_lon)
                self.distance_to_target = dist
                bearing = calculate_bearing(self.current_lat, self.current_lon, target_lat, target_lon)
                self.bearing = bearing

                # Gradually align heading toward target bearing
                angle_diff = (bearing - self.heading + 180.0) % 360.0 - 180.0
                turn_rate = 45.0 * dt  # 45 deg/sec turn rate
                if abs(angle_diff) < turn_rate:
                    self.heading = bearing
                else:
                    self.heading = (self.heading + math.copysign(turn_rate, angle_diff) + 360.0) % 360.0

                # Waypoint arrival threshold: 0.8 meters
                step_dist = self.speed_mps * dt
                if dist <= max(0.8, step_dist):
                    self.current_lat = target_lat
                    self.current_lon = target_lon
                    self.waypoints[self.current_wp_index]["status"] = "COMPLETED"
                    self.current_wp_index += 1

                    if self.current_wp_index >= len(self.waypoints):
                        self.status = "COMPLETED"
                        self.distance_to_target = 0.0
                        self._running = False
                        break
                    else:
                        next_target = self.waypoints[self.current_wp_index]
                        next_target["status"] = "ACTIVE"
                        self.bearing = calculate_bearing(
                            self.current_lat, self.current_lon,
                            next_target["lat"], next_target["lon"]
                        )
                else:
                    # Move towards target
                    fraction = min(1.0, step_dist / dist)
                    self.current_lat += (target_lat - self.current_lat) * fraction
                    self.current_lon += (target_lon - self.current_lon) * fraction
                    self.waypoints[self.current_wp_index]["status"] = "ACTIVE"

            time.sleep(dt)

# Global simulation rover instance
sim_rover = SimulationRover()
