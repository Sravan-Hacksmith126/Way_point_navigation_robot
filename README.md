# GPS Waypoint Navigation Robot — Ground Control Station

Professional GIS Ground Control Station & Mission Planner for an autonomous dual-motor differential drive robot powered by **TETRIX PRIZM**, **Beitian BN-880 GPS**, **QMC5883P digital compass**, and **HC-05 Bluetooth transceiver**.

---

## 1. System Overview

- **Real Satellite Map Experience**: Powered by Leaflet.js with legitimate high-resolution **Esri World Imagery** tiles, hybrid labels, and OpenStreetMap street layers.
- **Deep Zoom Inspection**: Supports high-resolution zoom (up to zoom level 21) for inspecting pathways, campus grounds, obstacles, and waypoints.
- **1–5 Waypoint Mission Planner**: Interactive map placement with strict validation, automatic coordinate extraction at 6-decimal WGS84 precision (±0.11 m resolution).
- **Geodesic Routing & Metrics**: Real-time Haversine distance calculation and estimated mission duration.
- **Simulation Engine**: Built-in 2D kinematics simulation engine for testing full missions without physical hardware.
- **Microcontroller Protocol**: ASCII streaming protocol for TETRIX PRIZM over PySerial / HC-05 Bluetooth.

---

## 2. Directory Structure

```
Way_point_navigation_robot/
├── backend/
│   ├── app.py              # Flask ground station API & static server
│   ├── config.py           # COM port, baud rate & simulation configurations
│   ├── protocol.py         # TETRIX PRIZM ASCII protocol parser & serializer
│   ├── simulation.py       # Geodesic kinematics & rover trajectory simulation
│   └── bluetooth.py        # PySerial HC-05 transceiver manager
├── frontend/
│   ├── index.html          # Semantic engineering layout
│   ├── style.css           # Technical slate design system (responsive)
│   └── app.js              # Leaflet GIS engine, waypoint controller & telemetry HUD
├── data/
│   └── missions/           # Active and saved mission JSON profiles
├── requirements.txt        # Flask & PySerial dependencies
└── README.md               # Documentation & PRIZM Arduino firmware reference
```

---

## 3. Quick Start & Execution

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Start the Ground Station Backend
```bash
python -m backend.app
```
By default, the server launches on `http://127.0.0.1:5000`.

### 3. Open in Browser
Open `http://127.0.0.1:5000` on your desktop, laptop, or mobile browser.

---

## 4. TETRIX PRIZM Firmware Protocol Reference

The Ground Station transmits ASCII command frames over Bluetooth Serial (9600 baud default):

### Mission Payload Frame:
```
MISSION,3
WP,1,17.780300,83.374800
WP,2,17.780512,83.375045
WP,3,17.780820,83.374710
END
```

### Action Commands:
- `START\n` — Initiates motor closed-loop navigation to W1.
- `PAUSE\n` — Pauses DC motors in place.
- `RESUME\n` — Continues navigation to active waypoint.
- `STOP\n` — Immediate emergency stop (cuts motor power).
- `CLEAR\n` — Resets active waypoint buffer.

### Telemetry Stream from PRIZM to Ground Station:
```
GPS,17.780421,83.374920
HEAD,82.4
BEAR,88.7
DIST,12.4
WP,2
STATUS,RUNNING
```

---

## 5. Microcontroller Firmware Example (TETRIX PRIZM / Arduino)

```cpp
#include <PRIZM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

PRIZM prizm;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(10, 11); // RX, TX to BN-880 GPS

struct Waypoint {
  double lat;
  double lon;
};

Waypoint waypoints[5];
int totalWaypoints = 0;
int currentWpIndex = 0;
bool missionActive = false;

void setup() {
  prizm.PrizmBegin();
  Serial.begin(9600);    // Hardware Serial to HC-05 Bluetooth
  gpsSerial.begin(9600); // Software Serial to BN-880 GPS
}

void loop() {
  // 1. Process Bluetooth Commands from Ground Station
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("MISSION,")) {
      totalWaypoints = line.substring(8).toInt();
      currentWpIndex = 0;
    } else if (line.startsWith("WP,")) {
      int comma1 = line.indexOf(',');
      int comma2 = line.indexOf(',', comma1 + 1);
      int comma3 = line.indexOf(',', comma2 + 1);
      int id = line.substring(comma1 + 1, comma2).toInt();
      double lat = line.substring(comma2 + 1, comma3).toDouble();
      double lon = line.substring(comma3 + 1).toDouble();
      if (id >= 1 && id <= 5) {
        waypoints[id - 1] = {lat, lon};
      }
    } else if (line == "START") {
      missionActive = true;
    } else if (line == "PAUSE" || line == "STOP") {
      missionActive = false;
      prizm.setMotorPowers(125, 125); // Coast stop
    }
  }

  // 2. Navigation & Steering Logic
  if (missionActive && currentWpIndex < totalWaypoints) {
    // Read GPS & QMC5883P compass heading
    // Compute cross-track steering error and drive PRIZM motors
    // prizm.setMotorPowers(pwrLeft, pwrRight);
  }
}
```
