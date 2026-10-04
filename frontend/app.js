/**
 * GPS WAYPOINT NAVIGATION ROBOT — GROUND CONTROL STATION ENGINE
 * Technical GIS Mission Planner & Telemetry Client
 */

(function () {
  "use strict";

  // --- Configuration & Constants ---
  const DEFAULT_LAT = 17.780300;
  const DEFAULT_LON = 83.374800;
  const DEFAULT_ZOOM = 18;
  const MAX_ALLOWED_WAYPOINTS = 5;
  const ROVER_CRUISE_SPEED_MPS = 1.5;

  // --- Geodesic Earth Calculation (Haversine & Azimuth) ---
  const EARTH_RADIUS_M = 6371000.0;

  function toRad(deg) {
    return (deg * Math.PI) / 180.0;
  }

  function toDeg(rad) {
    return (rad * 180.0) / Math.PI;
  }

  function calculateDistance(lat1, lon1, lat2, lon2) {
    const dLat = toRad(lat2 - lat1);
    const dLon = toRad(lon2 - lon1);
    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    return EARTH_RADIUS_M * c;
  }

  function calculateBearing(lat1, lon1, lat2, lon2) {
    const y = Math.sin(toRad(lon2 - lon1)) * Math.cos(toRad(lat2));
    const x =
      Math.cos(toRad(lat1)) * Math.sin(toRad(lat2)) -
      Math.sin(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.cos(toRad(lon2 - lon1));
    const brng = toDeg(Math.atan2(y, x));
    return (brng + 360.0) % 360.0;
  }

  // --- Application State ---
  const state = {
    targetCount: 3,
    waypoints: [], // { id, lat, lon, marker, status: 'PENDING'|'ACTIVE'|'COMPLETED' }
    routePolyline: null,
    searchMarker: null,
    userLocationMarker: null,
    robotMarker: null,
    missionStatus: "READY", // READY, RUNNING, PAUSED, STOPPED, COMPLETED
    isSimulationMode: true,
    telemetryTimer: null,
    backendConnected: false,
    robotState: {
      lat: DEFAULT_LAT,
      lon: DEFAULT_LON,
      heading: 0.0,
      bearing: 0.0,
      currentWpIndex: 0,
      distToWp: 0.0
    }
  };

  // --- DOM Elements Cache ---
  const dom = {
    map: document.getElementById("map"),
    guidanceText: document.getElementById("guidanceText"),
    cursorCoordsText: document.getElementById("cursorCoordsText"),
    wpCountBtns: document.querySelectorAll(".btn-wp-count"),
    wpProgressBadge: document.getElementById("wpProgressBadge"),
    plannedDistDisplay: document.getElementById("plannedDistanceDisplay"),
    estimatedTimeDisplay: document.getElementById("estimatedTimeDisplay"),
    missionStatusBadge: document.getElementById("missionStatusBadge"),
    btnStart: document.getElementById("btnStartMission"),
    btnPause: document.getElementById("btnPauseMission"),
    btnResume: document.getElementById("btnResumeMission"),
    btnStop: document.getElementById("btnStopMission"),
    btnClear: document.getElementById("btnClearMission"),
    telemState: document.getElementById("telemState"),
    telemCurrentWP: document.getElementById("telemCurrentWP"),
    telemDistWP: document.getElementById("telemDistWP"),
    telemHeading: document.getElementById("telemHeading"),
    telemBearing: document.getElementById("telemBearing"),
    telemGpsFix: document.getElementById("telemGpsFix"),
    dialDegreeVal: document.getElementById("dialDegreeVal"),
    headingTapeTrack: document.getElementById("headingTapeTrack"),
    waypointsTableBody: document.getElementById("waypointsTableBody"),
    emptyWpRow: document.getElementById("emptyWpRow"),
    btnToggleJson: document.getElementById("btnToggleJsonDrawer"),
    jsonDrawer: document.getElementById("missionJsonDrawer"),
    btnCopyJson: document.getElementById("btnCopyJson"),
    jsonOutputContent: document.getElementById("jsonOutputContent"),
    btnLocateMe: document.getElementById("btnLocateMe"),
    btnFitRoute: document.getElementById("btnFitRoute"),
    btnToggleFullscreen: document.getElementById("btnToggleFullscreen"),
    searchInput: document.getElementById("locationSearchInput"),
    btnSearchExec: document.getElementById("execSearchBtn"),
    btnClearSearch: document.getElementById("clearSearchBtn"),
    searchResultsDropdown: document.getElementById("searchResultsDropdown"),
    toastContainer: document.getElementById("toastContainer"),
    // Hardware Modal
    toggleHardwareBtn: document.getElementById("toggleHardwareModalBtn"),
    hwModal: document.getElementById("hardwareModal"),
    btnCloseModal: document.getElementById("btnCloseModal"),
    comPortSelect: document.getElementById("comPortSelect"),
    baudRateSelect: document.getElementById("baudRateSelect"),
    btnRefreshPorts: document.getElementById("btnRefreshPorts"),
    btnModeSim: document.getElementById("btnModeSim"),
    btnModeHardware: document.getElementById("btnModeHardware"),
    btnConnectBt: document.getElementById("btnConnectBt"),
    btnDisconnectBt: document.getElementById("btnDisconnectBt"),
    hwModalNotice: document.getElementById("hwModalNotice"),
    simBadgeText: document.getElementById("simBadgeText"),
    hwBadgeText: document.getElementById("hwBadgeText"),
    hardwareBadge: document.getElementById("hardwareBadge")
  };

  // ==========================================================================
  // Leaflet Map Initialization & Real Satellite Tile Providers
  // ==========================================================================
  
  let map;

  function initMap() {
    // 1. Esri World Imagery (Legitimate, High-Resolution Satellite Tiles)
    const esriSatellite = L.tileLayer(
      "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      {
        attribution: "Tiles &copy; Esri &mdash; Source: Esri, Maxar, Earthstar Geographics, CNES/Airbus DS, and GIS User Community",
        maxNativeZoom: 19,
        maxZoom: 21,
        subdomains: ["server", "services"]
      }
    );

    // 2. Hybrid Overlay: Boundaries, Highways & Place Names
    const esriLabels = L.tileLayer(
      "https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}",
      {
        attribution: "Reference &copy; Esri",
        maxNativeZoom: 19,
        maxZoom: 21
      }
    );

    const esriHybrid = L.layerGroup([esriSatellite, esriLabels]);

    // 3. Cartographic Street Map (OpenStreetMap)
    const osmStreet = L.tileLayer(
      "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
      {
        attribution: "&copy; <a href='https://www.openstreetmap.org/copyright'>OpenStreetMap</a> contributors",
        maxNativeZoom: 19,
        maxZoom: 21
      }
    );

    // Initialize Map with Satellite Default
    map = L.map("map", {
      center: [DEFAULT_LAT, DEFAULT_LON],
      zoom: DEFAULT_ZOOM,
      minZoom: 3,
      maxZoom: 21,
      layers: [esriSatellite], // Default is Satellite
      zoomControl: false // Custom placement
    });

    // Clean compact zoom control on top-left
    L.control.zoom({ position: "topleft" }).addTo(map);

    // Base Layers Controller
    const baseMaps = {
      "Satellite": esriSatellite,
      "Hybrid (Satellite + Labels)": esriHybrid,
      "Street Map": osmStreet
    };

    L.control.layers(baseMaps, null, { position: "topright", collapsed: true }).addTo(map);

    // Track Cursor Coordinates
    map.on("mousemove", function (e) {
      dom.cursorCoordsText.textContent = `${e.latlng.lat.toFixed(6)}, ${e.latlng.lng.toFixed(6)}`;
    });

    // Map Click -> Drop Waypoint
    map.on("click", function (e) {
      handleMapClick(e.latlng.lat, e.latlng.lng);
    });

    // Update North Indicator needle when map is rotated (if rotated)
    updateGuidance();
  }

  // ==========================================================================
  // Custom Leaflet Icons (Aviation/Robotics Grade)
  // ==========================================================================

  function createWpIcon(id, status = "pending") {
    let statusClass = "pending";
    if (status === "ACTIVE") statusClass = "active";
    if (status === "COMPLETED") statusClass = "completed";

    return L.divIcon({
      className: "custom-wp-div-icon",
      html: `<div class="wp-leaflet-marker ${statusClass}" style="width:26px; height:26px;">W${id}</div>`,
      iconSize: [26, 26],
      iconAnchor: [13, 13],
      popupAnchor: [0, -14]
    });
  }

  function createRobotIcon(heading = 0) {
    return L.divIcon({
      className: "robot-leaflet-marker-wrap",
      html: `
        <div class="robot-leaflet-marker" style="transform: rotate(${heading}deg);">
          <svg class="robot-marker-svg" viewBox="0 0 32 32">
            <circle cx="16" cy="16" r="14" fill="rgba(11, 16, 26, 0.85)" stroke="#38bdf8" stroke-width="2"/>
            <polygon points="16,4 23,24 16,19 9,24" fill="#38bdf8"/>
            <circle cx="16" cy="16" r="2.5" fill="#f8fafc"/>
          </svg>
        </div>
      `,
      iconSize: [32, 32],
      iconAnchor: [16, 16]
    });
  }

  // ==========================================================================
  // Waypoint Management System (1–5 Limit)
  // ==========================================================================

  function handleMapClick(lat, lon) {
    if (state.missionStatus === "RUNNING") {
      showToast("Cannot modify waypoints while mission is running. Stop first.", "warn");
      return;
    }

    if (state.waypoints.length >= state.targetCount) {
      if (state.targetCount >= MAX_ALLOWED_WAYPOINTS) {
        showToast("Maximum 5 waypoints reached. Delete an existing waypoint or clear route.", "warn");
      } else {
        showToast(`Target count (${state.targetCount}) reached. Select a higher waypoint count (up to 5) to add more.`, "info");
      }
      return;
    }

    if (state.waypoints.length >= MAX_ALLOWED_WAYPOINTS) {
      showToast("Strict limit: Maximum 5 waypoints allowed.", "error");
      return;
    }

    const nextId = state.waypoints.length + 1;
    const roundedLat = Number(lat.toFixed(6));
    const roundedLon = Number(lon.toFixed(6));

    // Create Leaflet Marker
    const marker = L.marker([roundedLat, roundedLon], {
      icon: createWpIcon(nextId, "pending"),
      draggable: true
    }).addTo(map);

    // Bind Popup
    const popupContent = generatePopupContent(nextId, roundedLat, roundedLon, "Pending");
    marker.bindPopup(popupContent);

    // Marker Drag Event
    marker.on("dragend", function (e) {
      const pos = e.target.getLatLng();
      updateWaypointPosition(nextId, pos.lat, pos.lng);
    });

    marker.on("click", function () {
      highlightTableRow(nextId);
    });

    state.waypoints.push({
      id: nextId,
      lat: roundedLat,
      lon: roundedLon,
      marker: marker,
      status: "PENDING"
    });

    recalculateRoute();
    updateGuidance();
    showToast(`Waypoint W${nextId} placed (${roundedLat.toFixed(6)}, ${roundedLon.toFixed(6)})`, "info");
  }

  function updateWaypointPosition(id, newLat, newLon) {
    const wp = state.waypoints.find(w => w.id === id);
    if (!wp) return;

    wp.lat = Number(newLat.toFixed(6));
    wp.lon = Number(newLon.toFixed(6));
    wp.marker.setPopupContent(generatePopupContent(id, wp.lat, wp.lon, wp.status));

    recalculateRoute();
    showToast(`Waypoint W${id} repositioned`, "info");
  }

  function removeWaypoint(id) {
    if (state.missionStatus === "RUNNING") {
      showToast("Cannot remove waypoint while mission is running", "warn");
      return;
    }

    const index = state.waypoints.findIndex(w => w.id === id);
    if (index === -1) return;

    // Remove marker from map
    map.removeLayer(state.waypoints[index].marker);
    state.waypoints.splice(index, 1);

    // Re-index remaining waypoints
    state.waypoints.forEach((wp, idx) => {
      wp.id = idx + 1;
      wp.marker.setIcon(createWpIcon(wp.id, wp.status));
      wp.marker.setPopupContent(generatePopupContent(wp.id, wp.lat, wp.lon, wp.status));
    });

    recalculateRoute();
    updateGuidance();
    showToast(`Waypoint removed. Remaining: ${state.waypoints.length}`, "info");
  }

  function clearAllWaypoints() {
    if (state.missionStatus === "RUNNING") {
      stopMission();
    }

    state.waypoints.forEach(wp => {
      map.removeLayer(wp.marker);
    });
    state.waypoints = [];

    if (state.routePolyline) {
      map.removeLayer(state.routePolyline);
      state.routePolyline = null;
    }

    if (state.robotMarker) {
      map.removeLayer(state.robotMarker);
      state.robotMarker = null;
    }

    state.missionStatus = "READY";
    dom.missionStatusBadge.textContent = "READY";
    dom.missionStatusBadge.className = "badge-status";
    dom.btnStart.disabled = true;
    dom.btnPause.disabled = true;
    dom.btnResume.style.display = "none";
    dom.btnPause.style.display = "inline-flex";
    dom.btnStop.disabled = true;

    // Reset backend simulation
    fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: "CLEAR" })
    }).catch(() => {});

    recalculateRoute();
    updateGuidance();
    showToast("Route cleared", "info");
  }

  function generatePopupContent(id, lat, lon, status) {
    return `
      <div class="custom-popup-box">
        <div class="popup-title">WAYPOINT W${id}</div>
        <div class="popup-row">
          <span class="popup-label">Latitude:</span>
          <span class="popup-val">${Number(lat).toFixed(6)}</span>
        </div>
        <div class="popup-row">
          <span class="popup-label">Longitude:</span>
          <span class="popup-val">${Number(lon).toFixed(6)}</span>
        </div>
        <div class="popup-row">
          <span class="popup-label">Status:</span>
          <span class="popup-val">${status}</span>
        </div>
      </div>
    `;
  }

  // ==========================================================================
  // Route Calculation, Polylines & Metrics
  // ==========================================================================

  function recalculateRoute() {
    const latlngs = state.waypoints.map(w => [w.lat, w.lon]);

    // Update or create Polyline
    if (state.routePolyline) {
      state.routePolyline.setLatLngs(latlngs);
    } else if (latlngs.length > 1) {
      state.routePolyline = L.polyline(latlngs, {
        color: "#38bdf8",
        weight: 3,
        opacity: 0.85,
        dashArray: "6, 8"
      }).addTo(map);
    }

    // Calculate Geodesic Distance
    let totalDist = 0.0;
    const legDistances = [0.0];

    for (let i = 0; i < state.waypoints.length - 1; i++) {
      const p1 = state.waypoints[i];
      const p2 = state.waypoints[i + 1];
      const d = calculateDistance(p1.lat, p1.lon, p2.lat, p2.lon);
      totalDist += d;
      legDistances.push(d);
    }

    // Update Metrics HUD
    dom.plannedDistDisplay.textContent = `${totalDist.toFixed(1)} m`;
    const estTimeSec = Math.round(totalDist / ROVER_CRUISE_SPEED_MPS);
    dom.estimatedTimeDisplay.textContent = estTimeSec < 60 ? `${estTimeSec}s` : `${Math.floor(estTimeSec / 60)}m ${estTimeSec % 60}s`;

    // Render Table
    renderWaypointsTable(legDistances);

    // Update Progress Badge
    dom.wpProgressBadge.textContent = `${state.waypoints.length} / ${state.targetCount} Set`;
    if (state.waypoints.length === state.targetCount) {
      dom.wpProgressBadge.style.color = "#10b981";
      dom.wpProgressBadge.style.borderColor = "rgba(16, 185, 129, 0.3)";
      dom.wpProgressBadge.style.background = "rgba(16, 185, 129, 0.12)";
    } else {
      dom.wpProgressBadge.style.color = "#38bdf8";
      dom.wpProgressBadge.style.borderColor = "rgba(56, 189, 248, 0.3)";
      dom.wpProgressBadge.style.background = "rgba(56, 189, 248, 0.12)";
    }

    // START Button State: strictly enabled only when target count is fulfilled
    const canStart = state.waypoints.length === state.targetCount && state.missionStatus !== "RUNNING";
    dom.btnStart.disabled = !canStart;

    // Update Developer JSON Output
    updateMissionJson(totalDist);
  }

  function renderWaypointsTable(legDistances) {
    if (state.waypoints.length === 0) {
      dom.emptyWpRow.style.display = "";
      dom.waypointsTableBody.innerHTML = "";
      dom.waypointsTableBody.appendChild(dom.emptyWpRow);
      return;
    }

    dom.emptyWpRow.style.display = "none";
    dom.waypointsTableBody.innerHTML = "";

    state.waypoints.forEach((wp, idx) => {
      const tr = document.createElement("tr");
      tr.id = `wp-row-${wp.id}`;

      let statusBadgeClass = "pending";
      if (wp.status === "ACTIVE") statusBadgeClass = "active";
      if (wp.status === "COMPLETED") statusBadgeClass = "completed";

      tr.innerHTML = `
        <td class="col-wp">W${wp.id}</td>
        <td class="col-lat">${wp.lat.toFixed(6)}°N</td>
        <td class="col-lon">${wp.lon.toFixed(6)}°E</td>
        <td class="col-leg">${(legDistances[idx] || 0).toFixed(1)} m</td>
        <td class="col-status">
          <span class="status-tag ${statusBadgeClass}">${wp.status}</span>
        </td>
        <td class="col-action">
          <button type="button" class="btn-table-action btn-table-focus" data-id="${wp.id}">Focus</button>
          <button type="button" class="btn-table-action btn-table-del" data-id="${wp.id}">Remove</button>
        </td>
      `;

      // Row Click -> Focus
      tr.addEventListener("click", function (e) {
        if (!e.target.classList.contains("btn-table-action")) {
          focusWaypoint(wp.id);
        }
      });

      dom.waypointsTableBody.appendChild(tr);
    });

    // Attach Action Buttons
    dom.waypointsTableBody.querySelectorAll(".btn-table-focus").forEach(b => {
      b.addEventListener("click", e => {
        e.stopPropagation();
        focusWaypoint(Number(b.dataset.id));
      });
    });

    dom.waypointsTableBody.querySelectorAll(".btn-table-del").forEach(b => {
      b.addEventListener("click", e => {
        e.stopPropagation();
        removeWaypoint(Number(b.dataset.id));
      });
    });
  }

  function focusWaypoint(id) {
    const wp = state.waypoints.find(w => w.id === id);
    if (!wp) return;

    map.flyTo([wp.lat, wp.lon], Math.max(map.getZoom(), 19), { duration: 0.8 });
    wp.marker.openPopup();
    highlightTableRow(id);
  }

  function highlightTableRow(id) {
    document.querySelectorAll(".waypoints-table tbody tr").forEach(tr => {
      tr.classList.remove("row-active");
    });
    const targetRow = document.getElementById(`wp-row-${id}`);
    if (targetRow) {
      targetRow.classList.add("row-active");
      targetRow.scrollIntoView({ behavior: "smooth", block: "nearest" });
    }
  }

  function updateGuidance() {
    const count = state.waypoints.length;
    const target = state.targetCount;

    if (state.missionStatus === "RUNNING") {
      dom.guidanceText.textContent = `Mission active: Rover navigating toward waypoint target`;
    } else if (count === 0) {
      dom.guidanceText.textContent = `Select ${target} location${target > 1 ? "s" : ""} on the satellite map`;
    } else if (count < target) {
      const remaining = target - count;
      dom.guidanceText.textContent = `Select ${remaining} more location${remaining > 1 ? "s" : ""} (W${count + 1})`;
    } else {
      dom.guidanceText.textContent = `All ${target} waypoints designated. Ready to transmit & start mission.`;
    }
  }

  function updateMissionJson(totalDist) {
    const payload = {
      command: "MISSION",
      waypoint_count: state.waypoints.length,
      total_planned_distance_m: Number(totalDist.toFixed(1)),
      waypoints: state.waypoints.map(w => ({
        id: w.id,
        lat: w.lat,
        lon: w.lon
      }))
    };
    dom.jsonOutputContent.textContent = JSON.stringify(payload, null, 2);
  }

  // ==========================================================================
  // Location Search (OpenStreetMap Nominatim Geocoding)
  // ==========================================================================

  let searchDebounce = null;

  function executeSearch(query) {
    const q = query.trim();
    if (!q) return;

    dom.btnSearchExec.textContent = "...";

    fetch(`https://nominatim.openstreetmap.org/search?format=json&q=${encodeURIComponent(q)}&limit=5`)
      .then(res => res.json())
      .then(results => {
        dom.btnSearchExec.textContent = "Search";
        renderSearchResults(results);
      })
      .catch(err => {
        dom.btnSearchExec.textContent = "Search";
        showToast("Search failed. Check network connection.", "error");
      });
  }

  function renderSearchResults(results) {
    dom.searchResultsDropdown.innerHTML = "";
    if (!results || results.length === 0) {
      dom.searchResultsDropdown.innerHTML = `<div class="search-result-item" style="color:var(--text-muted); font-style:italic;">No locations found.</div>`;
      dom.searchResultsDropdown.style.display = "block";
      return;
    }

    results.forEach(item => {
      const div = document.createElement("div");
      div.className = "search-result-item";
      div.textContent = item.display_name;
      div.addEventListener("click", () => {
        selectSearchResult(item);
      });
      dom.searchResultsDropdown.appendChild(div);
    });

    dom.searchResultsDropdown.style.display = "block";
  }

  function selectSearchResult(item) {
    const lat = parseFloat(item.lat);
    const lon = parseFloat(item.lon);

    dom.searchResultsDropdown.style.display = "none";
    dom.searchInput.value = item.display_name.split(",")[0];
    dom.btnClearSearch.style.display = "inline-block";

    // Move Map
    map.flyTo([lat, lon], 18, { duration: 1.2 });

    // Place Temporary Search Pin
    if (state.searchMarker) map.removeLayer(state.searchMarker);
    state.searchMarker = L.circleMarker([lat, lon], {
      radius: 8,
      fillColor: "#38bdf8",
      color: "#ffffff",
      weight: 2,
      opacity: 1,
      fillOpacity: 0.8
    }).addTo(map);

    state.searchMarker.bindPopup(`
      <div class="custom-popup-box">
        <div class="popup-title">LOCATION FOUND</div>
        <div style="font-size:11px; margin-bottom:6px; color:#cbd5e1;">${item.display_name}</div>
        <button type="button" class="btn-search-exec" id="btnAddWpFromSearch" style="margin:0; width:100%;">Add as Waypoint</button>
      </div>
    `).openPopup();

    setTimeout(() => {
      const btn = document.getElementById("btnAddWpFromSearch");
      if (btn) {
        btn.onclick = () => {
          map.closePopup();
          handleMapClick(lat, lon);
        };
      }
    }, 100);
  }

  // ==========================================================================
  // Locate Me (Browser Geolocation API)
  // ==========================================================================

  function handleLocateMe() {
    if (!navigator.geolocation) {
      showToast("Geolocation is not supported by your browser", "error");
      return;
    }

    showToast("Requesting browser GPS position...", "info");

    navigator.geolocation.getCurrentPosition(
      pos => {
        const lat = pos.coords.latitude;
        const lon = pos.coords.longitude;
        const accuracy = pos.coords.accuracy;

        map.flyTo([lat, lon], 19, { duration: 1.0 });

        if (state.userLocationMarker) map.removeLayer(state.userLocationMarker);

        state.userLocationMarker = L.circleMarker([lat, lon], {
          radius: 9,
          fillColor: "#10b981",
          color: "#ffffff",
          weight: 2,
          opacity: 1,
          fillOpacity: 0.9
        }).addTo(map);

        state.userLocationMarker.bindPopup(`
          <div class="custom-popup-box">
            <div class="popup-title" style="color:#10b981;">YOUR LOCATION</div>
            <div class="popup-row">
              <span class="popup-label">Latitude:</span>
              <span class="popup-val">${lat.toFixed(6)}</span>
            </div>
            <div class="popup-row">
              <span class="popup-label">Longitude:</span>
              <span class="popup-val">${lon.toFixed(6)}</span>
            </div>
            <div class="popup-row">
              <span class="popup-label">Accuracy:</span>
              <span class="popup-val">&plusmn;${accuracy.toFixed(1)} m</span>
            </div>
            <button type="button" class="btn-search-exec" id="btnAddWpFromUserLoc" style="margin-top:6px; width:100%; border-color:#10b981; color:#10b981;">Add as Waypoint W${state.waypoints.length + 1}</button>
          </div>
        `).openPopup();

        setTimeout(() => {
          const btn = document.getElementById("btnAddWpFromUserLoc");
          if (btn) {
            btn.onclick = () => {
              map.closePopup();
              handleMapClick(lat, lon);
            };
          }
        }, 100);

        showToast(`Located: ${lat.toFixed(6)}, ${lon.toFixed(6)} (±${accuracy.toFixed(1)}m)`, "success");
      },
      err => {
        let msg = "Location request failed";
        if (err.code === err.PERMISSION_DENIED) {
          msg = "Location permission denied. You can search or pan directly to your area.";
        } else if (err.code === err.POSITION_UNAVAILABLE) {
          msg = "GPS position unavailable. Check device settings.";
        } else if (err.code === err.TIMEOUT) {
          msg = "Location request timed out.";
        }
        showToast(msg, "error");
      },
      {
        enableHighAccuracy: true,
        timeout: 10000,
        maximumAge: 0
      }
    );
  }

  // ==========================================================================
  // Mission Control & Simulation / Hardware Execution
  // ==========================================================================

  function startMission() {
    if (state.waypoints.length !== state.targetCount) {
      showToast(`Please select all ${state.targetCount} waypoints before starting.`, "warn");
      return;
    }

    // Arm Mission in Backend
    const missionPayload = {
      waypoints: state.waypoints.map(w => ({
        id: w.id,
        lat: w.lat,
        lon: w.lon
      }))
    };

    fetch("/api/mission", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(missionPayload)
    })
      .then(res => res.json())
      .then(data => {
        // Send START command
        return fetch("/api/command", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ command: "START" })
        });
      })
      .then(() => {
        state.missionStatus = "RUNNING";
        dom.missionStatusBadge.textContent = "RUNNING";
        dom.missionStatusBadge.className = "badge-status running";

        dom.btnStart.disabled = true;
        dom.btnPause.disabled = false;
        dom.btnResume.style.display = "none";
        dom.btnPause.style.display = "inline-flex";
        dom.btnStop.disabled = false;

        // Initialize Robot Marker at W1
        const firstWp = state.waypoints[0];
        if (!state.robotMarker) {
          state.robotMarker = L.marker([firstWp.lat, firstWp.lon], {
            icon: createRobotIcon(0),
            zIndexOffset: 1000
          }).addTo(map);
        } else {
          state.robotMarker.setLatLng([firstWp.lat, firstWp.lon]);
        }

        updateGuidance();
        startTelemetryLoop();
        showToast("Mission started. Telemetry stream active.", "success");
      })
      .catch(err => {
        showToast("Failed to arm mission with backend.", "error");
      });
  }

  function pauseMission() {
    fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: "PAUSE" })
    })
      .then(() => {
        state.missionStatus = "PAUSED";
        dom.missionStatusBadge.textContent = "PAUSED";
        dom.missionStatusBadge.className = "badge-status paused";
        dom.btnPause.style.display = "none";
        dom.btnResume.style.display = "inline-flex";
        dom.btnResume.disabled = false;
        showToast("Mission paused", "info");
      })
      .catch(() => showToast("Command failed", "error"));
  }

  function resumeMission() {
    fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: "RESUME" })
    })
      .then(() => {
        state.missionStatus = "RUNNING";
        dom.missionStatusBadge.textContent = "RUNNING";
        dom.missionStatusBadge.className = "badge-status running";
        dom.btnResume.style.display = "none";
        dom.btnPause.style.display = "inline-flex";
        dom.btnPause.disabled = false;
        showToast("Mission resumed", "info");
      })
      .catch(() => showToast("Command failed", "error"));
  }

  function stopMission() {
    fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: "STOP" })
    })
      .then(() => {
        state.missionStatus = "STOPPED";
        dom.missionStatusBadge.textContent = "STOPPED";
        dom.missionStatusBadge.className = "badge-status stopped";
        dom.btnStart.disabled = false;
        dom.btnPause.disabled = true;
        dom.btnResume.style.display = "none";
        dom.btnPause.style.display = "inline-flex";
        dom.btnStop.disabled = true;
        showToast("Mission stopped", "warn");
      })
      .catch(() => showToast("Command failed", "error"));
  }

  // ==========================================================================
  // Telemetry Loop & Compass Tape Synchronization
  // ==========================================================================

  function startTelemetryLoop() {
    if (state.telemetryTimer) clearInterval(state.telemetryTimer);

    state.telemetryTimer = setInterval(() => {
      fetch("/api/telemetry")
        .then(res => res.json())
        .then(data => {
          updateTelemetryHUD(data);
        })
        .catch(() => {
          // If backend connection fails temporarily, don't crash
        });
    }, 300);
  }

  function updateTelemetryHUD(data) {
    if (!data) return;

    dom.telemState.textContent = data.status || "--";
    dom.telemCurrentWP.textContent = data.current_waypoint || "--";
    dom.telemDistWP.textContent = `${(data.distance_to_target_m || 0).toFixed(1)} m`;
    dom.telemHeading.textContent = `${(data.heading || 0).toFixed(1)}°`;
    dom.telemBearing.textContent = `${(data.bearing || 0).toFixed(1)}°`;
    dom.telemGpsFix.textContent = data.gps_fix || "3D FIX";

    // Update Robot Marker Position & Heading on Map
    if (data.lat !== null && data.lon !== null && state.robotMarker) {
      state.robotMarker.setLatLng([data.lat, data.lon]);
      state.robotMarker.setIcon(createRobotIcon(data.heading || 0));
    }

    // Update Compass Tape
    updateCompassTape(data.heading || 0);

    // Update Waypoint Completion States
    if (data.waypoints && data.waypoints.length > 0) {
      data.waypoints.forEach(wpTelem => {
        const wp = state.waypoints.find(w => w.id === wpTelem.id);
        if (wp && wp.status !== wpTelem.status) {
          wp.status = wpTelem.status;
          wp.marker.setIcon(createWpIcon(wp.id, wp.status));
          wp.marker.setPopupContent(generatePopupContent(wp.id, wp.lat, wp.lon, wp.status));
        }
      });
      // Refresh table status tags
      state.waypoints.forEach(wp => {
        const row = document.getElementById(`wp-row-${wp.id}`);
        if (row) {
          const tag = row.querySelector(".status-tag");
          if (tag) {
            tag.className = `status-tag ${wp.status.toLowerCase()}`;
            tag.textContent = wp.status;
          }
        }
      });
    }

    // Handle Mission Completion
    if (data.status === "COMPLETED" && state.missionStatus !== "COMPLETED") {
      state.missionStatus = "COMPLETED";
      dom.missionStatusBadge.textContent = "COMPLETED";
      dom.missionStatusBadge.className = "badge-status running";
      dom.btnStart.disabled = false;
      dom.btnPause.disabled = true;
      dom.btnStop.disabled = true;
      showToast("Mission completed successfully! All waypoints reached.", "success");
    }
  }

  function updateCompassTape(heading) {
    const deg = Math.round((heading + 360) % 360);
    dom.dialDegreeVal.textContent = `HDG ${String(deg).padStart(3, "0")}°`;

    // Tape track has 360 degrees spread across width
    // Every 30 degrees ~ 35px shift
    const shiftPx = -(deg / 360.0) * 280;
    dom.headingTapeTrack.style.transform = `translateY(-50%) translateX(${shiftPx}px)`;
  }

  // ==========================================================================
  // Hardware Link & Bluetooth Modal Handling
  // ==========================================================================

  function refreshComPorts() {
    dom.comPortSelect.innerHTML = "<option>Scanning...</option>";
    fetch("/api/bluetooth/ports")
      .then(res => res.json())
      .then(data => {
        dom.comPortSelect.innerHTML = "";
        const ports = data.ports || [];
        if (ports.length === 0) {
          dom.comPortSelect.innerHTML = `<option value="COM3">COM3 (Default)</option>`;
        } else {
          ports.forEach(p => {
            const opt = document.createElement("option");
            opt.value = p.device;
            opt.textContent = `${p.device} - ${p.description || "Serial Device"}`;
            dom.comPortSelect.appendChild(opt);
          });
        }
      })
      .catch(() => {
        dom.comPortSelect.innerHTML = `<option value="COM3">COM3 (Fallback)</option>`;
      });
  }

  function connectBluetooth() {
    const port = dom.comPortSelect.value;
    const baud = dom.baudRateSelect.value;

    dom.hwModalNotice.textContent = `Attempting connection to ${port} @ ${baud} baud...`;

    fetch("/api/bluetooth/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port, baud })
    })
      .then(res => res.json())
      .then(data => {
        if (data.status === "connected") {
          state.isSimulationMode = false;
          dom.hwModalNotice.textContent = `Connected to ${data.port} @ ${data.baud} baud`;
          dom.simBadgeText.textContent = "PHYSICAL HARDWARE";
          dom.hwBadgeText.textContent = `LINK: ${data.port}`;
          dom.hardwareBadge.classList.add("connected");
          showToast(`Connected to TETRIX PRIZM on ${port}`, "success");
        } else {
          dom.hwModalNotice.textContent = `Connection failed: ${data.error || "Port unavailable"}`;
          showToast(`Hardware connection failed: ${data.error || "Device not responding"}`, "error");
        }
      })
      .catch(err => {
        dom.hwModalNotice.textContent = `Connection error: ${err.message}`;
        showToast("Backend link error", "error");
      });
  }

  function disconnectBluetooth() {
    fetch("/api/bluetooth/disconnect", { method: "POST" })
      .then(() => {
        state.isSimulationMode = true;
        dom.hwModalNotice.textContent = "Disconnected. Running in Simulation Mode.";
        dom.simBadgeText.textContent = "SIMULATION MODE";
        dom.hwBadgeText.textContent = "LINK: SIMULATED";
        dom.hardwareBadge.classList.remove("connected");
        showToast("Disconnected hardware. Simulation mode active.", "info");
      })
      .catch(() => {});
  }

  // ==========================================================================
  // UI Helpers (Toast, Fullscreen, Copy)
  // ==========================================================================

  function showToast(msg, type = "info") {
    const toast = document.createElement("div");
    toast.className = `toast toast-${type}`;
    toast.textContent = msg;

    dom.toastContainer.appendChild(toast);

    setTimeout(() => {
      toast.style.transition = "opacity 0.3s ease, transform 0.3s ease";
      toast.style.opacity = "0";
      toast.style.transform = "translateY(8px)";
      setTimeout(() => toast.remove(), 300);
    }, 4000);
  }

  // ==========================================================================
  // Event Listeners Binding
  // ==========================================================================

  function setupEventListeners() {
    // 1. Waypoint Count Selector (1 to 5)
    dom.wpCountBtns.forEach(btn => {
      btn.addEventListener("click", () => {
        if (state.missionStatus === "RUNNING") {
          showToast("Cannot alter waypoint target while mission is running", "warn");
          return;
        }
        dom.wpCountBtns.forEach(b => b.classList.remove("active"));
        btn.classList.add("active");
        state.targetCount = parseInt(btn.dataset.count, 10);
        recalculateRoute();
        updateGuidance();
        showToast(`Target set to ${state.targetCount} waypoint${state.targetCount > 1 ? "s" : ""}`, "info");
      });
    });

    // 2. Control Buttons
    dom.btnStart.addEventListener("click", startMission);
    dom.btnPause.addEventListener("click", pauseMission);
    dom.btnResume.addEventListener("click", resumeMission);
    dom.btnStop.addEventListener("click", stopMission);
    dom.btnClear.addEventListener("click", clearAllWaypoints);

    // 3. Search Bar Interaction
    dom.btnSearchExec.addEventListener("click", () => {
      executeSearch(dom.searchInput.value);
    });

    dom.searchInput.addEventListener("keydown", e => {
      if (e.key === "Enter") {
        executeSearch(dom.searchInput.value);
      }
    });

    dom.searchInput.addEventListener("input", e => {
      if (e.target.value.length > 0) {
        dom.btnClearSearch.style.display = "inline-block";
      } else {
        dom.btnClearSearch.style.display = "none";
        dom.searchResultsDropdown.style.display = "none";
      }
    });

    dom.btnClearSearch.addEventListener("click", () => {
      dom.searchInput.value = "";
      dom.btnClearSearch.style.display = "none";
      dom.searchResultsDropdown.style.display = "none";
    });

    // Close dropdown on outside click
    document.addEventListener("click", e => {
      if (!dom.searchResultsDropdown.contains(e.target) && e.target !== dom.searchInput) {
        dom.searchResultsDropdown.style.display = "none";
      }
    });

    // 4. Locate Me
    dom.btnLocateMe.addEventListener("click", handleLocateMe);

    // 5. Fit Route
    dom.btnFitRoute.addEventListener("click", () => {
      if (state.waypoints.length > 0) {
        const group = L.featureGroup(state.waypoints.map(w => w.marker));
        map.fitBounds(group.getBounds().pad(0.2));
      } else {
        showToast("No waypoints placed to fit.", "info");
      }
    });

    // 6. Fullscreen Map
    dom.btnToggleFullscreen.addEventListener("click", () => {
      const elem = document.getElementById("mapSection");
      if (!document.fullscreenElement) {
        elem.requestFullscreen().catch(() => {});
      } else {
        document.exitFullscreen().catch(() => {});
      }
    });

    // 7. Developer Mission JSON Drawer
    dom.btnToggleJson.addEventListener("click", () => {
      const isVisible = dom.jsonDrawer.style.display !== "none";
      dom.jsonDrawer.style.display = isVisible ? "none" : "flex";
      dom.btnToggleJson.classList.toggle("active", !isVisible);
    });

    dom.btnCopyJson.addEventListener("click", () => {
      navigator.clipboard.writeText(dom.jsonOutputContent.textContent)
        .then(() => showToast("Mission JSON copied to clipboard", "success"))
        .catch(() => showToast("Failed to copy JSON", "error"));
    });

    // 8. Hardware Modal
    dom.toggleHardwareBtn.addEventListener("click", () => {
      dom.hwModal.style.display = "flex";
      refreshComPorts();
    });

    dom.btnCloseModal.addEventListener("click", () => {
      dom.hwModal.style.display = "none";
    });

    dom.btnRefreshPorts.addEventListener("click", refreshComPorts);
    dom.btnConnectBt.addEventListener("click", connectBluetooth);
    dom.btnDisconnectBt.addEventListener("click", disconnectBluetooth);

    dom.btnModeSim.addEventListener("click", () => {
      dom.btnModeSim.classList.add("active");
      dom.btnModeHardware.classList.remove("active");
      disconnectBluetooth();
    });

    dom.btnModeHardware.addEventListener("click", () => {
      dom.btnModeHardware.classList.add("active");
      dom.btnModeSim.classList.remove("active");
    });
  }

  // --- Initial Launch ---
  window.addEventListener("DOMContentLoaded", () => {
    initMap();
    setupEventListeners();
    recalculateRoute();
    startTelemetryLoop();
  });

})();
