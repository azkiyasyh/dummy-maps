/**
 * ZooSMate - Sistem Navigasi Cerdas, GPS Realtime, & Petunjuk Arah
 * Algoritma A* & Dijkstra pada Jaringan Jalan Setapak Asli (OpenStreetMap)
 * Dilengkapi Realtime GPS Tracker, Proximity Alert Geofencing, & Custom Coordinate Manager
 */

// -------------------------------------------------------------
// 1. DATA GRAPH JALAN SETAPAK (OSM FOOTPATHS) & POI
// -------------------------------------------------------------

const NETWORK_DATA = window.WALKABLE_NETWORK || { ways: [], nodes: {}, pois: [] };

const OSM_NODES = NETWORK_DATA.nodes;
const OSM_WAYS = NETWORK_DATA.ways;
const PRESET_POIS = NETWORK_DATA.pois || [];

// Koleksi seluruh POI (Bawaan + Kustom dari Pengguna)
let ALL_POIS = [...PRESET_POIS];
const POI_MAP = {};

// Bangun Adjacency List: nodeId -> Array of { node, weight, wayType, wayName }
const GRAPH_ADJ = {};
for (const id in OSM_NODES) {
  GRAPH_ADJ[id] = [];
}

OSM_WAYS.forEach(way => {
  const wayName = way.name || (way.highway === 'path' ? 'Jalan Setapak Hutan' : (way.highway === 'footway' ? 'Jalur Pejalan Kaki' : 'Jalur Kampus'));
  const wayType = way.highway || 'footway';

  for (let i = 0; i < way.nodes.length - 1; i++) {
    const u = way.nodes[i];
    const v = way.nodes[i + 1];
    if (!OSM_NODES[u] || !OSM_NODES[v]) continue;

    const dist = haversineDistance(OSM_NODES[u], OSM_NODES[v]);
    GRAPH_ADJ[u].push({ node: v, weight: dist, wayType, wayName });
    GRAPH_ADJ[v].push({ node: u, weight: dist, wayType, wayName });
  }
});

// -------------------------------------------------------------
// 2. UTILITAS SPASIAL (Haversine, Bearing, Snapping, & Turns)
// -------------------------------------------------------------

function haversineDistance(p1, p2) {
  const R = 6371000;
  const toRad = deg => (deg * Math.PI) / 180;
  const lat1 = p1.lat, lon1 = p1.lon !== undefined ? p1.lon : p1.lng;
  const lat2 = p2.lat, lon2 = p2.lon !== undefined ? p2.lon : p2.lng;

  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) *
    Math.sin(dLon / 2) * Math.sin(dLon / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  return Math.round(R * c);
}

function findNearestPathNode(lat, lon, maxDistance = 400) {
  let nearestId = null;
  let minDist = Infinity;

  for (const id in GRAPH_ADJ) {
    if (GRAPH_ADJ[id].length === 0) continue;
    const node = OSM_NODES[id];
    const d = haversineDistance({ lat, lon }, node);
    if (d < minDist) {
      minDist = d;
      nearestId = id;
    }
  }

  if (minDist <= maxDistance) {
    return { id: nearestId, dist: minDist, node: OSM_NODES[nearestId] };
  }
  return null;
}

function calculateBearing(p1, p2) {
  const toRad = deg => (deg * Math.PI) / 180;
  const toDeg = rad => (rad * 180) / Math.PI;

  const lat1 = p1.lat, lon1 = p1.lon !== undefined ? p1.lon : p1.lng;
  const lat2 = p2.lat, lon2 = p2.lon !== undefined ? p2.lon : p2.lng;

  const phi1 = toRad(lat1);
  const phi2 = toRad(lat2);
  const deltaLon = toRad(lon2 - lon1);

  const y = Math.sin(deltaLon) * Math.cos(phi2);
  const x =
    Math.cos(phi1) * Math.sin(phi2) -
    Math.sin(phi1) * Math.cos(phi2) * Math.cos(deltaLon);

  return (toDeg(Math.atan2(y, x)) + 360) % 360;
}

function getCompassDirection(bearing) {
  const directions = [
    { name: "Utara", min: 337.5, max: 360 },
    { name: "Utara", min: 0, max: 22.5 },
    { name: "Timur Laut", min: 22.5, max: 67.5 },
    { name: "Timur", min: 67.5, max: 112.5 },
    { name: "Tenggara", min: 112.5, max: 157.5 },
    { name: "Selatan", min: 157.5, max: 202.5 },
    { name: "Barat Daya", min: 202.5, max: 247.5 },
    { name: "Barat", min: 247.5, max: 292.5 },
    { name: "Barat Laut", min: 292.5, max: 337.5 }
  ];
  for (const d of directions) {
    if (bearing >= d.min && bearing < d.max) return d.name;
  }
  return "Utara";
}

function getTurnInstruction(diff) {
  if (diff > 180) diff -= 360;

  if (Math.abs(diff) <= 25) {
    return { text: "Lurus terus", icon: "⬆️", type: "straight" };
  } else if (diff > 25 && diff <= 65) {
    return { text: "Belok serong kanan", icon: "↗️", type: "slight-right" };
  } else if (diff > 65 && diff <= 120) {
    return { text: "Belok kanan", icon: "➡️", type: "turn-right" };
  } else if (diff > 120) {
    return { text: "Belok tajam ke kanan", icon: "↪️", type: "sharp-right" };
  } else if (diff < -25 && diff >= -65) {
    return { text: "Belok serong kiri", icon: "↖️", type: "slight-left" };
  } else if (diff < -65 && diff >= -120) {
    return { text: "Belok kiri", icon: "⬅️", type: "turn-left" };
  } else {
    return { text: "Belok tajam ke kiri", icon: "↩️", type: "sharp-left" };
  }
}

// -------------------------------------------------------------
// 3. ALGORITMA PENCARIAN RUTE (A* & DIJKSTRA)
// -------------------------------------------------------------

function runAStar(startNodeId, endNodeId) {
  const startTime = performance.now();
  const gScore = {};
  const fScore = {};
  const previous = {};
  const visited = new Set();
  const visitedOrder = [];

  for (const k in GRAPH_ADJ) {
    gScore[k] = Infinity;
    fScore[k] = Infinity;
  }

  gScore[startNodeId] = 0;
  const initialH = haversineDistance(OSM_NODES[startNodeId], OSM_NODES[endNodeId]);
  fScore[startNodeId] = initialH;

  const openSet = [{ id: startNodeId, f: fScore[startNodeId] }];

  while (openSet.length > 0) {
    openSet.sort((a, b) => a.f - b.f);
    const { id: u } = openSet.shift();

    if (visited.has(u)) continue;
    visited.add(u);
    visitedOrder.push(u);

    if (u === endNodeId) break;

    for (const edge of GRAPH_ADJ[u]) {
      const v = edge.node;
      const tentG = gScore[u] + edge.weight;

      if (tentG < gScore[v]) {
        previous[v] = u;
        gScore[v] = tentG;
        const h = haversineDistance(OSM_NODES[v], OSM_NODES[endNodeId]);
        fScore[v] = tentG + h;
        openSet.push({ id: v, f: fScore[v] });
      }
    }
  }

  const path = [];
  let curr = endNodeId;
  while (curr) {
    path.unshift(curr);
    curr = previous[curr];
  }

  const executionTime = (performance.now() - startTime).toFixed(2);
  const isFound = path.length > 0 && path[0] === startNodeId;

  return {
    algorithm: "A*",
    path: isFound ? path : [],
    totalDistance: isFound ? gScore[endNodeId] : 0,
    visitedNodesCount: visited.size,
    visitedOrder,
    executionTimeMs: executionTime
  };
}

function runDijkstra(startNodeId, endNodeId) {
  const startTime = performance.now();
  const distances = {};
  const previous = {};
  const visited = new Set();
  const queue = [{ id: startNodeId, dist: 0 }];
  const visitedOrder = [];

  for (const k in GRAPH_ADJ) {
    distances[k] = Infinity;
  }
  distances[startNodeId] = 0;

  while (queue.length > 0) {
    queue.sort((a, b) => a.dist - b.dist);
    const { id: u, dist } = queue.shift();

    if (visited.has(u)) continue;
    visited.add(u);
    visitedOrder.push(u);

    if (u === endNodeId) break;

    for (const edge of GRAPH_ADJ[u]) {
      const v = edge.node;
      const alt = dist + edge.weight;
      if (alt < distances[v]) {
        distances[v] = alt;
        previous[v] = u;
        queue.push({ id: v, dist: alt });
      }
    }
  }

  const path = [];
  let curr = endNodeId;
  while (curr) {
    path.unshift(curr);
    curr = previous[curr];
  }

  const executionTime = (performance.now() - startTime).toFixed(2);
  const isFound = path.length > 0 && path[0] === startNodeId;

  return {
    algorithm: "Dijkstra",
    path: isFound ? path : [],
    totalDistance: isFound ? distances[endNodeId] : 0,
    visitedNodesCount: visited.size,
    visitedOrder,
    executionTimeMs: executionTime
  };
}

// -------------------------------------------------------------
// 4. GENERATOR PETUNJUK ARAH CERDAS
// -------------------------------------------------------------

function generateDirections(pathNodeIds, startName, endName) {
  if (!pathNodeIds || pathNodeIds.length < 2) return [];

  const nodeToPoi = {};
  ALL_POIS.forEach(p => {
    nodeToPoi[p.nodeId] = p;
  });

  const directions = [];
  let accumulatedDist = 0;
  let segmentDist = 0;
  let prevBearing = null;
  let currentWayName = "";

  const firstEdge = GRAPH_ADJ[pathNodeIds[0]].find(e => e.node === pathNodeIds[1]);
  currentWayName = firstEdge ? firstEdge.wayName : "Jalur Pejalan Kaki";

  const initialBearing = calculateBearing(OSM_NODES[pathNodeIds[0]], OSM_NODES[pathNodeIds[1]]);
  prevBearing = initialBearing;

  directions.push({
    stepNumber: 1,
    type: "start",
    icon: "🚩",
    title: `Mulai dari ${startName}`,
    instruction: `Mulai perjalanan dari <strong>${startName}</strong>, berjalan ke arah <strong>${getCompassDirection(initialBearing)}</strong> mengikuti <strong>${currentWayName}</strong>.`,
    nodeId: pathNodeIds[0],
    coords: [OSM_NODES[pathNodeIds[0]].lat, OSM_NODES[pathNodeIds[0]].lon],
    segmentDistance: 0,
    cumulativeDistance: 0
  });

  for (let i = 0; i < pathNodeIds.length - 1; i++) {
    const u = pathNodeIds[i];
    const v = pathNodeIds[i + 1];
    const uNode = OSM_NODES[u];
    const vNode = OSM_NODES[v];

    const d = haversineDistance(uNode, vNode);
    accumulatedDist += d;
    segmentDist += d;

    const bearing = calculateBearing(uNode, vNode);
    let diff = (bearing - prevBearing + 360) % 360;
    if (diff > 180) diff -= 360;

    const edge = GRAPH_ADJ[u].find(e => e.node === v);
    const edgeWayName = edge ? edge.wayName : currentWayName;

    const isSignificantTurn = Math.abs(diff) >= 35 && segmentDist >= 15;
    const isPassingPOI = nodeToPoi[u] && i > 0 && i < pathNodeIds.length - 1;
    const isWayChange = edgeWayName !== currentWayName && segmentDist >= 20;

    if (isSignificantTurn || isPassingPOI || isWayChange) {
      const turn = getTurnInstruction(diff);
      const poiInfo = nodeToPoi[u];
      const landmarkText = poiInfo ? ` di dekat <strong>${poiInfo.name}</strong>` : '';

      directions.push({
        stepNumber: directions.length + 1,
        type: turn.type,
        icon: poiInfo ? poiInfo.icon : turn.icon,
        title: poiInfo ? `Melewati ${poiInfo.name}` : `${turn.text} (${edgeWayName})`,
        instruction: `${turn.text}${landmarkText}, lanjutkan di <strong>${edgeWayName}</strong> sejauh <strong>${segmentDist} meter</strong>.`,
        nodeId: u,
        coords: [uNode.lat, uNode.lon],
        segmentDistance: segmentDist,
        cumulativeDistance: accumulatedDist
      });

      segmentDist = 0;
      currentWayName = edgeWayName;
      prevBearing = bearing;
    }
  }

  const lastNodeId = pathNodeIds[pathNodeIds.length - 1];
  const lastNode = OSM_NODES[lastNodeId];
  directions.push({
    stepNumber: directions.length + 1,
    type: "finish",
    icon: "🏁",
    title: `Tiba di ${endName}`,
    instruction: `Tiba di lokasi tujuan: <strong>${endName}</strong>. Total perjalanan Anda <strong>${accumulatedDist} meter</strong>.`,
    nodeId: lastNodeId,
    coords: [lastNode.lat, lastNode.lon],
    segmentDistance: segmentDist,
    cumulativeDistance: accumulatedDist
  });

  return directions;
}

// -------------------------------------------------------------
// 5. KELAS UTAMA APLIKASI ZOOSMATE NAVIGATION
// -------------------------------------------------------------

class ZooSMateNavigationApp {
  constructor() {
    this.selectedStartPoi = "POI_START";
    this.selectedEndPoi = "POI_END";
    this.startNodeId = PRESET_POIS[0]?.nodeId || "10595700055";
    this.endNodeId = PRESET_POIS[1]?.nodeId || "10595699997";

    this.selectedAlgorithm = "A*";

    this.map = null;
    this.walkablePathsLayer = null;
    this.poiMarkersLayer = null;
    this.activeRouteLayer = null;
    this.walkerMarker = null;

    // Realtime GPS Tracking State
    this.isGpsActive = false;
    this.gpsWatchId = null;
    this.currentUserCoords = null; // { lat, lon, accuracy }
    this.userGpsMarker = null;
    this.userAccuracyCircle = null;

    // Mock GPS Simulation (untuk pengujian di laptop/desktop)
    this.isMockGps = false;
    this.mockGpsInterval = null;
    this.mockStepIdx = 0;

    // Proximity Detection State
    this.proximityThresholdMeters = 30; // Radius deteksi tempat terdekat
    this.activeProximityPoiId = null;
    this.proximitySoundEnabled = true;

    // Custom POI Management
    this.customPois = [];
    this.isMapPickerActive = false;

    this.showFootpaths = true;
    this.showLabels = true;
    this.isSimulating = false;
    this.simulationInterval = null;
    this.isSpeaking = false;
  }

  init() {
    this.loadCustomPoisFromStorage();
    this.rebuildPoiMap();
    this.initMap();
    this.populateDropdowns();
    this.renderWalkableNetworkLayer();
    this.renderPoiMarkers();
    this.renderCustomPoiList();
    this.bindEvents();
    this.calculateAndRenderRoute();
  }

  // -----------------------------------------------------------
  // Manajemen Custom POI (Tambah & Ganti Sendiri)
  // -----------------------------------------------------------
  loadCustomPoisFromStorage() {
    try {
      const stored = localStorage.getItem('zoosmate_custom_pois');
      if (stored) {
        this.customPois = JSON.parse(stored);
      }
    } catch (e) {
      console.warn("Gagal memuat custom POI:", e);
      this.customPois = [];
    }
  }

  saveCustomPoisToStorage() {
    try {
      localStorage.setItem('zoosmate_custom_pois', JSON.stringify(this.customPois));
    } catch (e) {
      console.warn("Gagal menyimpan custom POI:", e);
    }
  }

  rebuildPoiMap() {
    ALL_POIS = [...PRESET_POIS, ...this.customPois];
    for (const k in POI_MAP) delete POI_MAP[k];
    ALL_POIS.forEach(p => {
      POI_MAP[p.id] = p;
    });
  }

  // -----------------------------------------------------------
  // Inisialisasi Peta Leaflet
  // -----------------------------------------------------------
  initMap() {
    const startNode = OSM_NODES[this.startNodeId] || { lat: -7.299792, lon: 112.671036 };

    this.map = L.map('map', {
      zoomControl: true
    }).setView([startNode.lat, startNode.lon], 17);

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '© OpenStreetMap contributors | Jalur Pejalan Kaki Hutan UNESA'
    }).addTo(this.map);

    this.walkablePathsLayer = L.layerGroup().addTo(this.map);
    this.poiMarkersLayer = L.layerGroup().addTo(this.map);
    this.activeRouteLayer = L.layerGroup().addTo(this.map);

    setTimeout(() => {
      this.map.invalidateSize();
    }, 250);
  }

  populateDropdowns() {
    const startSelect = document.getElementById('select-start');
    const endSelect = document.getElementById('select-end');

    startSelect.innerHTML = '';
    endSelect.innerHTML = '';

    // Group 1: Tempat Wisata & Landmark Hutan UNESA
    const grpPresetStart = document.createElement('optgroup');
    grpPresetStart.label = "🌲 Tempat Resmi Hutan UNESA";
    const grpPresetEnd = document.createElement('optgroup');
    grpPresetEnd.label = "🌲 Tempat Resmi Hutan UNESA";

    PRESET_POIS.forEach(p => {
      const optS = document.createElement('option');
      optS.value = p.id;
      optS.textContent = `${p.icon} ${p.name} (${p.category})`;
      if (p.id === this.selectedStartPoi) optS.selected = true;
      grpPresetStart.appendChild(optS);

      const optE = document.createElement('option');
      optE.value = p.id;
      optE.textContent = `${p.icon} ${p.name} (${p.category})`;
      if (p.id === this.selectedEndPoi) optE.selected = true;
      grpPresetEnd.appendChild(optE);
    });

    startSelect.appendChild(grpPresetStart);
    endSelect.appendChild(grpPresetEnd);

    // Group 2: Titik Kustom Buatan Pengguna
    if (this.customPois.length > 0) {
      const grpCustomStart = document.createElement('optgroup');
      grpCustomStart.label = "⭐ Titik Kustom Anda (Bisa Diganti)";
      const grpCustomEnd = document.createElement('optgroup');
      grpCustomEnd.label = "⭐ Titik Kustom Anda (Bisa Diganti)";

      this.customPois.forEach(p => {
        const optS = document.createElement('option');
        optS.value = p.id;
        optS.textContent = `${p.icon} ${p.name} (Kustom)`;
        if (p.id === this.selectedStartPoi) optS.selected = true;
        grpCustomStart.appendChild(optS);

        const optE = document.createElement('option');
        optE.value = p.id;
        optE.textContent = `${p.icon} ${p.name} (Kustom)`;
        if (p.id === this.selectedEndPoi) optE.selected = true;
        grpCustomEnd.appendChild(optE);
      });

      startSelect.appendChild(grpCustomStart);
      endSelect.appendChild(grpCustomEnd);
    }
  }

  renderWalkableNetworkLayer() {
    this.walkablePathsLayer.clearLayers();
    if (!this.showFootpaths) return;

    OSM_WAYS.forEach(way => {
      const coords = way.nodes
        .filter(id => OSM_NODES[id])
        .map(id => [OSM_NODES[id].lat, OSM_NODES[id].lon]);

      if (coords.length < 2) return;

      const isForestPath = way.highway === 'path';
      L.polyline(coords, {
        color: isForestPath ? '#8a5c36' : '#5a6258',
        weight: isForestPath ? 3 : 2.5,
        dashArray: isForestPath ? '4, 5' : '6, 6',
        opacity: 0.65
      }).addTo(this.walkablePathsLayer);
    });
  }

  // -----------------------------------------------------------
  // Render POI Markers (Termasuk Draggable Custom Marker)
  // -----------------------------------------------------------
  renderPoiMarkers() {
    this.poiMarkersLayer.clearLayers();

    ALL_POIS.forEach(poi => {
      const node = OSM_NODES[poi.nodeId];
      if (!node) return;

      const isStart = poi.nodeId === this.startNodeId;
      const isEnd = poi.nodeId === this.endNodeId;
      const isCustom = !!poi.isCustom;

      let pinClass = "node-pin";
      if (isStart) pinClass += " is-start";
      if (isEnd) pinClass += " is-end";
      if (isCustom) pinClass += " is-custom";

      const iconHtml = `<div class="${pinClass}" title="${poi.name}">${poi.icon}</div>`;
      const customIcon = L.divIcon({
        html: iconHtml,
        className: 'custom-node-icon',
        iconSize: [28, 28],
        iconAnchor: [14, 14],
        popupAnchor: [0, -14]
      });

      // Marker kustom dapat di-drag langsung di peta untuk mengubah koordinat!
      const marker = L.marker([poi.lat || node.lat, poi.lon || node.lon], {
        icon: customIcon,
        draggable: isCustom
      });

      // Konten Popup
      let customBadge = isCustom ? `<span style="background:#f59e0b;color:#000;padding:2px 6px;border-radius:4px;font-size:10px;font-weight:bold;">TITIK KUSTOM</span>` : '';
      let actionsHtml = `
        <div class="popup-node-actions">
          <button class="popup-btn" onclick="app.setStartFromNode('${poi.nodeId}', '${poi.id}')">🚩 Titik Asal</button>
          <button class="popup-btn" onclick="app.setEndFromNode('${poi.nodeId}', '${poi.id}')">🏁 Titik Tujuan</button>
          ${isCustom ? `
            <button class="popup-btn btn-edit" onclick="app.openEditPoiModal('${poi.id}')">✏️ Edit</button>
            <button class="popup-btn btn-del" onclick="app.deleteCustomPoi('${poi.id}')">🗑️ Hapus</button>
          ` : ''}
        </div>
      `;

      const popupContent = `
        <div class="popup-node-card">
          <div class="popup-node-title">${poi.icon} ${poi.name} ${customBadge}</div>
          <div class="popup-node-desc">${poi.desc || 'Titik penanda lokasi Hutan UNESA.'}</div>
          <div style="font-size: 10px; color: #10b981; margin-bottom: 6px;">
            ✓ Sesuai jalan setapak (Lat: ${(poi.lat || node.lat).toFixed(6)}, Lng: ${(poi.lon || node.lon).toFixed(6)})
          </div>
          ${isCustom ? `<div style="font-size: 10px; color: #fbbf24; margin-bottom: 6px;">💡 Pin ini bisa digeser (drag & drop) untuk mengganti posisi</div>` : ''}
          ${actionsHtml}
        </div>
      `;
      marker.bindPopup(popupContent);

      if (this.showLabels) {
        marker.bindTooltip(poi.name, {
          permanent: false,
          direction: 'top',
          className: 'node-tooltip'
        });
      }

      // Event saat marker kustom digeser (Drag End)
      if (isCustom) {
        marker.on('dragend', (e) => {
          const newPos = e.target.getLatLng();
          this.handleCustomMarkerDragged(poi.id, newPos.lat, newPos.lng);
        });
      }

      marker.addTo(this.poiMarkersLayer);
    });
  }

  handleCustomMarkerDragged(poiId, newLat, newLng) {
    const poi = this.customPois.find(p => p.id === poiId);
    if (!poi) return;

    // Snap ke node jalan setapak terdekat
    const snap = findNearestPathNode(newLat, newLng, 300);
    if (!snap) {
      alert("Lokasi baru terlalu jauh dari jalan setapak yang ada.");
      this.renderPoiMarkers();
      return;
    }

    poi.lat = newLat;
    poi.lon = newLng;
    poi.nodeId = snap.id;

    this.saveCustomPoisToStorage();
    this.rebuildPoiMap();
    this.renderPoiMarkers();
    this.renderCustomPoiList();

    if (this.selectedStartPoi === poiId || this.selectedEndPoi === poiId) {
      if (this.selectedStartPoi === poiId) this.startNodeId = snap.id;
      if (this.selectedEndPoi === poiId) this.endNodeId = snap.id;
      this.calculateAndRenderRoute();
    }
  }

  renderCustomPoiList() {
    const listEl = document.getElementById('custom-poi-list');
    listEl.innerHTML = '';

    if (this.customPois.length === 0) {
      listEl.innerHTML = `<li style="font-size: 11px; color: var(--text-muted); font-style: italic;">Belum ada titik kustom. Klik 'Tambah' untuk menandai tempat baru.</li>`;
      return;
    }

    this.customPois.forEach(poi => {
      const li = document.createElement('li');
      li.className = 'custom-poi-item';
      li.innerHTML = `
        <div class="custom-poi-item-left" title="${poi.name} (${poi.desc || ''})">
          <span>${poi.icon}</span>
          <span style="font-weight: 600;">${poi.name}</span>
        </div>
        <div class="custom-poi-actions">
          <button class="btn-item-action" title="Pusatkan di Peta" onclick="app.panToPoi('${poi.id}')">🔍</button>
          <button class="btn-item-action" title="Jadikan Titik Awal" onclick="app.setStartFromNode('${poi.nodeId}', '${poi.id}')">🚩</button>
          <button class="btn-item-action" title="Jadikan Titik Tujuan" onclick="app.setEndFromNode('${poi.nodeId}', '${poi.id}')">🏁</button>
          <button class="btn-item-action" title="Edit Titik" onclick="app.openEditPoiModal('${poi.id}')">✏️</button>
          <button class="btn-item-action" title="Hapus Titik" onclick="app.deleteCustomPoi('${poi.id}')">🗑️</button>
        </div>
      `;
      listEl.appendChild(li);
    });
  }

  panToPoi(poiId) {
    const poi = POI_MAP[poiId];
    if (poi) {
      const lat = poi.lat || OSM_NODES[poi.nodeId].lat;
      const lon = poi.lon || OSM_NODES[poi.nodeId].lon;
      this.map.setView([lat, lon], 18, { animate: true });
    }
  }

  // -----------------------------------------------------------
  // Modal Form Tambah / Edit Titik Koordinat
  // -----------------------------------------------------------
  openAddPoiModal(presetCoords = null) {
    document.getElementById('modal-form-title').textContent = "Tambah Titik Koordinat Baru";
    document.getElementById('input-poi-id').value = "";
    document.getElementById('input-poi-name').value = "";
    document.getElementById('input-poi-desc').value = "";
    document.getElementById('input-poi-category').value = "Titik Kustom";

    if (presetCoords) {
      document.getElementById('input-poi-lat').value = presetCoords.lat.toFixed(6);
      document.getElementById('input-poi-lng').value = presetCoords.lng.toFixed(6);
    } else if (this.currentUserCoords) {
      document.getElementById('input-poi-lat').value = this.currentUserCoords.lat.toFixed(6);
      document.getElementById('input-poi-lng').value = this.currentUserCoords.lon.toFixed(6);
    } else {
      document.getElementById('input-poi-lat').value = "-7.300500";
      document.getElementById('input-poi-lng').value = "112.671000";
    }

    this.selectEmoji("📍");
    document.getElementById('modal-poi-form').classList.add('show');
  }

  openEditPoiModal(poiId) {
    const poi = this.customPois.find(p => p.id === poiId);
    if (!poi) return;

    document.getElementById('modal-form-title').textContent = "Edit Titik Koordinat";
    document.getElementById('input-poi-id').value = poi.id;
    document.getElementById('input-poi-name').value = poi.name;
    document.getElementById('input-poi-desc').value = poi.desc || "";
    document.getElementById('input-poi-category').value = poi.category || "Titik Kustom";

    const lat = poi.lat !== undefined ? poi.lat : OSM_NODES[poi.nodeId].lat;
    const lon = poi.lon !== undefined ? poi.lon : OSM_NODES[poi.nodeId].lon;

    document.getElementById('input-poi-lat').value = lat.toFixed(6);
    document.getElementById('input-poi-lng').value = lon.toFixed(6);

    this.selectEmoji(poi.icon || "⭐");
    document.getElementById('modal-poi-form').classList.add('show');
  }

  selectEmoji(emoji) {
    document.getElementById('input-poi-emoji').value = emoji;
    document.querySelectorAll('.emoji-choice-btn').forEach(btn => {
      if (btn.dataset.emoji === emoji) {
        btn.classList.add('selected');
      } else {
        btn.classList.remove('selected');
      }
    });
  }

  savePoiFromForm() {
    const id = document.getElementById('input-poi-id').value;
    const name = document.getElementById('input-poi-name').value.trim();
    const category = document.getElementById('input-poi-category').value;
    const icon = document.getElementById('input-poi-emoji').value || "📍";
    const desc = document.getElementById('input-poi-desc').value.trim();
    const lat = parseFloat(document.getElementById('input-poi-lat').value);
    const lon = parseFloat(document.getElementById('input-poi-lng').value);

    if (!name || isNaN(lat) || isNaN(lon)) {
      alert("Harap lengkapi nama tempat dan koordinat Latitude/Longitude.");
      return;
    }

    // Snap ke node jalan setapak terdekat
    const snap = findNearestPathNode(lat, lon, 400);
    if (!snap) {
      alert("Titik koordinat berada terlalu jauh dari jaringan jalan setapak Hutan UNESA. Silakan masukkan koordinat di sekitar area kampus/hutan.");
      return;
    }

    if (id) {
      // Mode Edit
      const existing = this.customPois.find(p => p.id === id);
      if (existing) {
        existing.name = name;
        existing.category = category;
        existing.icon = icon;
        existing.desc = desc;
        existing.lat = lat;
        existing.lon = lon;
        existing.nodeId = snap.id;
      }
    } else {
      // Mode Tambah Baru
      const newPoi = {
        id: "CUSTOM_" + Date.now(),
        name,
        category,
        icon,
        desc,
        lat,
        lon,
        nodeId: snap.id,
        isCustom: true
      };
      this.customPois.push(newPoi);
    }

    this.saveCustomPoisToStorage();
    this.rebuildPoiMap();
    this.populateDropdowns();
    this.renderPoiMarkers();
    this.renderCustomPoiList();

    document.getElementById('modal-poi-form').classList.remove('show');

    // Pan ke titik yang baru disimpan
    this.map.setView([lat, lon], 18, { animate: true });
  }

  deleteCustomPoi(poiId) {
    if (!confirm("Apakah Anda yakin ingin menghapus titik koordinat kustom ini?")) return;

    this.customPois = this.customPois.filter(p => p.id !== poiId);
    this.saveCustomPoisToStorage();
    this.rebuildPoiMap();
    this.populateDropdowns();
    this.renderPoiMarkers();
    this.renderCustomPoiList();

    // Jika yang dihapus sedang jadi start/end, ganti ke default
    if (this.selectedStartPoi === poiId) {
      this.selectedStartPoi = "POI_START";
      this.startNodeId = PRESET_POIS[0].nodeId;
      document.getElementById('select-start').value = "POI_START";
    }
    if (this.selectedEndPoi === poiId) {
      this.selectedEndPoi = "POI_END";
      this.endNodeId = PRESET_POIS[1].nodeId;
      document.getElementById('select-end').value = "POI_END";
    }
    this.calculateAndRenderRoute();
  }

  // -----------------------------------------------------------
  // 1. FITUR REALTIME CHECK KOORDINAT USER (LIVE GPS TRACKING)
  // -----------------------------------------------------------
  toggleGpsTracking() {
    if (this.isGpsActive) {
      this.stopGpsTracking();
    } else {
      this.startGpsTracking();
    }
  }

  startGpsTracking() {
    if (!('geolocation' in navigator)) {
      alert("Perangkat atau browser Anda tidak mendukung Geolocation API.");
      return;
    }

    this.isGpsActive = true;
    document.getElementById('btn-toggle-gps').classList.add('active');
    document.getElementById('gps-pulse-dot').classList.add('active');
    document.getElementById('gps-status-sub').textContent = "GPS Realtime Aktif (Mencari koordinat...)";

    const options = {
      enableHighAccuracy: true,
      timeout: 10000,
      maximumAge: 0
    };

    this.gpsWatchId = navigator.geolocation.watchPosition(
      (pos) => this.onGpsUpdate(pos),
      (err) => this.onGpsError(err),
      options
    );
  }

  stopGpsTracking() {
    if (this.gpsWatchId !== null) {
      navigator.geolocation.clearWatch(this.gpsWatchId);
      this.gpsWatchId = null;
    }
    if (this.isMockGps) {
      clearInterval(this.mockGpsInterval);
      this.isMockGps = false;
      document.getElementById('btn-gps-mock').textContent = "🧪 Simulasi GPS";
    }

    this.isGpsActive = false;
    document.getElementById('btn-toggle-gps').classList.remove('active');
    document.getElementById('gps-pulse-dot').classList.remove('active');
    document.getElementById('gps-status-sub').textContent = "GPS dinonaktifkan";

    if (this.userGpsMarker) {
      this.userGpsMarker.remove();
      this.userGpsMarker = null;
    }
    if (this.userAccuracyCircle) {
      this.userAccuracyCircle.remove();
      this.userAccuracyCircle = null;
    }
  }

  onGpsUpdate(pos) {
    const lat = pos.coords.latitude;
    const lon = pos.coords.longitude;
    const acc = Math.round(pos.coords.accuracy || 5);

    this.currentUserCoords = { lat, lon, accuracy: acc };

    // Update UI Status Bar
    document.getElementById('gps-coords-display').textContent = `Lat: ${lat.toFixed(6)}, Lng: ${lon.toFixed(6)}`;
    document.getElementById('gps-status-sub').textContent = `Akurasi: ±${acc}m | Bergerak di Hutan UNESA`;

    // Render Marker GPS Pengguna
    this.renderUserGpsMarker(lat, lon, acc);

    // 2. CEK PROXIMITY DETECTION (Tempat Terdekat)
    this.checkProximityToPois(lat, lon);
    this.updateNearbyList(lat, lon);
  }

  onGpsError(err) {
    console.warn("GPS Error:", err.message);
    document.getElementById('gps-status-sub').textContent = `GPS: ${err.message}. Gunakan tombol 'Simulasi GPS' untuk uji coba PC.`;
  }

  renderUserGpsMarker(lat, lon, accuracy) {
    const userIcon = L.divIcon({
      html: '<div class="user-live-marker" title="Posisi Anda Saat Ini"></div>',
      className: 'custom-node-icon',
      iconSize: [22, 22],
      iconAnchor: [11, 11]
    });

    if (!this.userGpsMarker) {
      this.userGpsMarker = L.marker([lat, lon], { icon: userIcon, zIndexOffset: 1000 }).addTo(this.map);
      this.userGpsMarker.bindPopup("<b>📍 Posisi Anda Sekarang</b><br>Koordinat GPS Realtime.");
    } else {
      this.userGpsMarker.setLatLng([lat, lon]);
    }

    if (!this.userAccuracyCircle) {
      this.userAccuracyCircle = L.circle([lat, lon], {
        radius: Math.min(accuracy, 60),
        color: '#3b82f6',
        fillColor: '#60a5fa',
        fillOpacity: 0.15,
        weight: 1
      }).addTo(this.map);
    } else {
      this.userAccuracyCircle.setLatLng([lat, lon]);
      this.userAccuracyCircle.setRadius(Math.min(accuracy, 60));
    }
  }

  recenterToUser() {
    if (this.currentUserCoords) {
      this.map.setView([this.currentUserCoords.lat, this.currentUserCoords.lon], 18, { animate: true });
    } else {
      alert("Posisi GPS belum terdeteksi. Silakan aktifkan GPS atau gunakan tombol 'Simulasi GPS'.");
    }
  }

  setGpsAsStartPoint() {
    if (!this.currentUserCoords) {
      alert("Belum ada koordinat GPS. Mengaktifkan GPS sekarang...");
      this.startGpsTracking();
      return;
    }

    const snap = findNearestPathNode(this.currentUserCoords.lat, this.currentUserCoords.lon, 400);
    if (!snap) {
      alert("Posisi GPS Anda terlalu jauh dari jaringan jalan setapak Hutan UNESA.");
      return;
    }

    this.startNodeId = snap.id;
    this.map.setView([this.currentUserCoords.lat, this.currentUserCoords.lon], 18, { animate: true });
    this.calculateAndRenderRoute();
  }

  // -----------------------------------------------------------
  // Simulasi GPS Pengunjung (Mocking untuk Laptop/PC)
  // -----------------------------------------------------------
  toggleMockGps() {
    if (this.isMockGps) {
      clearInterval(this.mockGpsInterval);
      this.isMockGps = false;
      document.getElementById('btn-gps-mock').textContent = "🧪 Simulasi GPS";
      document.getElementById('gps-status-sub').textContent = "Simulasi GPS dihentikan";
      return;
    }

    this.isMockGps = true;
    document.getElementById('btn-gps-mock').textContent = "⏹️ Stop Simulasi";
    document.getElementById('gps-pulse-dot').classList.add('active');
    document.getElementById('btn-toggle-gps').classList.add('active');

    // Rute jalan kaki simulasi melewati titik-titik Hutan UNESA
    const mockWalkCoords = [
      { lat: -7.299792, lon: 112.671036 }, // Gerbang Utama
      { lat: -7.299920, lon: 112.671066 },
      { lat: -7.300060, lon: 112.671076 },
      { lat: -7.300250, lon: 112.671070 }, // Dekat Pos Info
      { lat: -7.300619, lon: 112.671043 },
      { lat: -7.300818, lon: 112.671019 },
      { lat: -7.301092, lon: 112.670978 }, // Pintu Hutan
      { lat: -7.301233, lon: 112.671419 }, // Masuk Hutan
      { lat: -7.301330, lon: 112.671880 }  // Pusat Observasi Flora
    ];

    this.mockStepIdx = 0;
    this.onGpsUpdate({
      coords: {
        latitude: mockWalkCoords[0].lat,
        longitude: mockWalkCoords[0].lon,
        accuracy: 4
      }
    });

    this.mockGpsInterval = setInterval(() => {
      this.mockStepIdx = (this.mockStepIdx + 1) % mockWalkCoords.length;
      const pt = mockWalkCoords[this.mockStepIdx];
      this.onGpsUpdate({
        coords: {
          latitude: pt.lat,
          longitude: pt.lon,
          accuracy: 5
        }
      });
    }, 2500);
  }

  // -----------------------------------------------------------
  // 2. FITUR DETEKSI USER SEDANG DEKAT TITIK KOORDINAT (PROXIMITY)
  // -----------------------------------------------------------
  checkProximityToPois(userLat, userLon) {
    let closestPoi = null;
    let closestDist = Infinity;

    ALL_POIS.forEach(poi => {
      const node = OSM_NODES[poi.nodeId];
      if (!node) return;
      const pLat = poi.lat || node.lat;
      const pLon = poi.lon || node.lon;

      const d = haversineDistance({ lat: userLat, lon: userLon }, { lat: pLat, lon: pLon });
      if (d < closestDist) {
        closestDist = d;
        closestPoi = poi;
      }
    });

    // Jika jarak <= 30 meter: memicu Proximity Alert
    if (closestPoi && closestDist <= this.proximityThresholdMeters) {
      if (this.activeProximityPoiId !== closestPoi.id) {
        this.activeProximityPoiId = closestPoi.id;
        this.showProximityAlert(closestPoi, closestDist);
      }
    } else {
      if (closestDist > this.proximityThresholdMeters + 10) {
        this.activeProximityPoiId = null;
      }
    }
  }

  showProximityAlert(poi, distance) {
    const toast = document.getElementById('proximity-toast');
    document.getElementById('proximity-icon').textContent = poi.icon;
    document.getElementById('proximity-title').textContent = `${poi.name} (${distance}m dari Anda)`;
    document.getElementById('proximity-desc').textContent = poi.desc || "Anda sedang berada sangat dekat dengan lokasi ini.";

    // Action button
    document.getElementById('btn-prox-set-start').onclick = () => {
      this.setStartFromNode(poi.nodeId, poi.id);
      toast.classList.remove('show');
    };
    document.getElementById('btn-prox-set-end').onclick = () => {
      this.setEndFromNode(poi.nodeId, poi.id);
      toast.classList.remove('show');
    };

    toast.classList.add('show');

    // Notifikasi suara cerdas (Text to Speech)
    if (this.proximitySoundEnabled) {
      this.speakText(`Perhatian. Anda sedang berada di dekat ${poi.name}.`);
    }

    // Auto hide setelah 8 detik jika tidak disentuh
    clearTimeout(this.toastTimeout);
    this.toastTimeout = setTimeout(() => {
      toast.classList.remove('show');
    }, 8000);
  }

  updateNearbyList(userLat, userLon) {
    const listEl = document.getElementById('nearby-list');
    listEl.innerHTML = '';

    const sorted = ALL_POIS.map(p => {
      const node = OSM_NODES[p.nodeId];
      const pLat = p.lat || (node ? node.lat : 0);
      const pLon = p.lon || (node ? node.lon : 0);
      const d = haversineDistance({ lat: userLat, lon: userLon }, { lat: pLat, lon: pLon });
      return { ...p, currentDist: d };
    }).sort((a, b) => a.currentDist - b.currentDist);

    // Tampilkan 3 tempat paling dekat
    sorted.slice(0, 3).forEach(p => {
      const li = document.createElement('li');
      li.className = 'nearby-item';
      li.innerHTML = `
        <div class="nearby-item-left">
          <span>${p.icon}</span>
          <span>${p.name}</span>
        </div>
        <span class="nearby-dist-badge">${p.currentDist} m</span>
      `;
      li.onclick = () => {
        this.panToPoi(p.id);
      };
      listEl.appendChild(li);
    });
  }

  // -----------------------------------------------------------
  // Event Listeners & Binding
  // -----------------------------------------------------------
  bindEvents() {
    // Tombol Live GPS Tracking
    document.getElementById('btn-toggle-gps').addEventListener('click', () => {
      this.toggleGpsTracking();
    });

    // Tombol Pusatkan Peta ke GPS
    document.getElementById('btn-gps-recenter').addEventListener('click', () => {
      this.recenterToUser();
    });

    // Tombol Jadikan GPS sebagai Titik Awal
    document.getElementById('btn-gps-as-start').addEventListener('click', () => {
      this.setGpsAsStartPoint();
    });
    document.getElementById('btn-panel-use-my-loc').addEventListener('click', () => {
      this.setGpsAsStartPoint();
    });

    // Tombol Simulasi GPS (Laptop / PC)
    document.getElementById('btn-gps-mock').addEventListener('click', () => {
      this.toggleMockGps();
    });

    // Tombol Tutup Proximity Toast
    document.getElementById('btn-close-proximity').addEventListener('click', () => {
      document.getElementById('proximity-toast').classList.remove('show');
    });

    // Tombol Buka Modal Tambah Titik Baru
    document.getElementById('btn-open-add-poi').addEventListener('click', () => {
      this.openAddPoiModal();
    });
    document.getElementById('btn-panel-add-poi').addEventListener('click', () => {
      this.openAddPoiModal();
    });

    // Form Modal Batal & Simpan
    document.getElementById('btn-close-form').addEventListener('click', () => {
      document.getElementById('modal-poi-form').classList.remove('show');
    });
    document.getElementById('btn-cancel-form').addEventListener('click', () => {
      document.getElementById('modal-poi-form').classList.remove('show');
    });
    document.getElementById('btn-save-poi').addEventListener('click', () => {
      this.savePoiFromForm();
    });

    // Emoji Picker buttons
    document.querySelectorAll('.emoji-choice-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        this.selectEmoji(btn.dataset.emoji);
      });
    });

    // Helper "Ambil dari GPS Sekarang" di Modal
    document.getElementById('btn-helper-use-gps').addEventListener('click', () => {
      if (this.currentUserCoords) {
        document.getElementById('input-poi-lat').value = this.currentUserCoords.lat.toFixed(6);
        document.getElementById('input-poi-lng').value = this.currentUserCoords.lon.toFixed(6);
      } else {
        alert("GPS belum aktif. Mengaktifkan GPS sekarang...");
        this.startGpsTracking();
      }
    });

    // Helper "Klik Titik di Peta" di Modal
    document.getElementById('btn-helper-pick-map').addEventListener('click', () => {
      this.isMapPickerActive = true;
      document.getElementById('modal-poi-form').classList.remove('show');
      alert("Silakan klik di mana saja pada peta untuk mengambil titik koordinat baru.");
    });

    // Klik Peta Leaflet
    this.map.on('click', (e) => {
      if (this.isMapPickerActive) {
        this.isMapPickerActive = false;
        this.openAddPoiModal({ lat: e.latlng.lat, lng: e.latlng.lng });
        return;
      }

      // Normal map click: popup tanya jadikan titik awal/tujuan/buat titik baru
      const snap = findNearestPathNode(e.latlng.lat, e.latlng.lng, 300);
      const snapNode = snap ? snap.node : { lat: e.latlng.lat, lon: e.latlng.lng };

      const popup = L.popup()
        .setLatLng(e.latlng)
        .setContent(`
          <div class="popup-node-card">
            <div class="popup-node-title">📍 Koordinat Terpilih</div>
            <div class="popup-node-desc">
              Lat: ${e.latlng.lat.toFixed(6)}<br>Lng: ${e.latlng.lng.toFixed(6)}
              ${snap ? `<br><span style="color:#10b981;">✓ Snap ke jalan setapak (${Math.round(snap.dist)}m)</span>` : ''}
            </div>
            <div class="popup-node-actions">
              ${snap ? `<button class="popup-btn" onclick="app.setStartFromNode('${snap.id}')">🚩 Set Asal</button>` : ''}
              ${snap ? `<button class="popup-btn" onclick="app.setEndFromNode('${snap.id}')">🏁 Set Tujuan</button>` : ''}
              <button class="popup-btn btn-edit" onclick="app.openAddPoiModal({ lat: ${e.latlng.lat}, lng: ${e.latlng.lng} })">➕ Tandai Titik Baru</button>
            </div>
          </div>
        `)
        .openOn(this.map);
    });

    // Dropdown change
    document.getElementById('select-start').addEventListener('change', (e) => {
      this.selectedStartPoi = e.target.value;
      const p = POI_MAP[this.selectedStartPoi];
      if (p) this.startNodeId = p.nodeId;
      this.calculateAndRenderRoute();
    });

    document.getElementById('select-end').addEventListener('change', (e) => {
      this.selectedEndPoi = e.target.value;
      const p = POI_MAP[this.selectedEndPoi];
      if (p) this.endNodeId = p.nodeId;
      this.calculateAndRenderRoute();
    });

    // Swap button
    document.getElementById('btn-swap').addEventListener('click', () => {
      const tempPoi = this.selectedStartPoi;
      this.selectedStartPoi = this.selectedEndPoi;
      this.selectedEndPoi = tempPoi;

      const tempNode = this.startNodeId;
      this.startNodeId = this.endNodeId;
      this.endNodeId = tempNode;

      document.getElementById('select-start').value = this.selectedStartPoi;
      document.getElementById('select-end').value = this.selectedEndPoi;
      this.calculateAndRenderRoute();
    });

    // Pilihan Tab Algoritma
    const tabAStar = document.getElementById('tab-algo-astar');
    const tabDijkstra = document.getElementById('tab-algo-dijkstra');

    tabAStar.addEventListener('click', () => {
      this.selectedAlgorithm = "A*";
      tabAStar.classList.add('active');
      tabDijkstra.classList.remove('active');
      this.calculateAndRenderRoute();
    });

    tabDijkstra.addEventListener('click', () => {
      this.selectedAlgorithm = "Dijkstra";
      tabDijkstra.classList.add('active');
      tabAStar.classList.remove('active');
      this.calculateAndRenderRoute();
    });

    // Cari Rute Button
    document.getElementById('btn-find-route').addEventListener('click', () => {
      this.calculateAndRenderRoute();
    });

    // Bandingkan Modal
    document.getElementById('btn-compare').addEventListener('click', () => {
      this.showComparisonModal();
    });

    document.getElementById('modal-close-btn').addEventListener('click', () => {
      document.getElementById('modal-compare').classList.remove('show');
    });

    // Toggle Panel
    const navPanel = document.getElementById('nav-panel');
    document.getElementById('btn-toggle-panel').addEventListener('click', () => {
      navPanel.classList.toggle('collapsed');
    });
    document.getElementById('panel-close-btn').addEventListener('click', () => {
      navPanel.classList.add('collapsed');
    });

    // Layer Checkboxes
    document.getElementById('toggle-edges').addEventListener('change', (e) => {
      this.showFootpaths = e.target.checked;
      this.renderWalkableNetworkLayer();
    });

    document.getElementById('toggle-labels').addEventListener('change', (e) => {
      this.showLabels = e.target.checked;
      this.renderPoiMarkers();
    });

    document.getElementById('toggle-proximity-sound').addEventListener('change', (e) => {
      this.proximitySoundEnabled = e.target.checked;
    });

    // Audio & Simulasi
    document.getElementById('btn-audio-guide').addEventListener('click', () => {
      this.toggleSpeechGuide();
    });

    document.getElementById('btn-simulation').addEventListener('click', () => {
      this.startSimulation();
    });
  }

  setStartFromNode(nodeId, poiId = null) {
    this.startNodeId = nodeId;
    if (poiId) {
      this.selectedStartPoi = poiId;
      document.getElementById('select-start').value = poiId;
    }
    this.map.closePopup();
    this.calculateAndRenderRoute();
  }

  setEndFromNode(nodeId, poiId = null) {
    this.endNodeId = nodeId;
    if (poiId) {
      this.selectedEndPoi = poiId;
      document.getElementById('select-end').value = poiId;
    }
    this.map.closePopup();
    this.calculateAndRenderRoute();
  }

  // -----------------------------------------------------------
  // Hitung Rute Terpendek & Render
  // -----------------------------------------------------------
  calculateAndRenderRoute() {
    if (this.startNodeId === this.endNodeId) {
      alert("Titik awal dan tujuan berada pada lokasi yang sama.");
      return;
    }

    let result;
    if (this.selectedAlgorithm === "A*") {
      result = runAStar(this.startNodeId, this.endNodeId);
    } else {
      result = runDijkstra(this.startNodeId, this.endNodeId);
    }

    this.renderPoiMarkers();
    this.renderRouteGeometry(result);
    this.renderSummaryAndDirections(result);
  }

  renderRouteGeometry(result) {
    this.activeRouteLayer.clearLayers();
    if (this.walkerMarker) {
      this.walkerMarker.remove();
      this.walkerMarker = null;
    }

    if (!result.path || result.path.length < 2) {
      alert("Rute tidak ditemukan antara kedua titik pada jaringan jalan setapak.");
      return;
    }

    const latlngs = result.path.map(id => [OSM_NODES[id].lat, OSM_NODES[id].lon]);
    this.currentPathCoords = latlngs;

    L.polyline(latlngs, {
      color: '#004d1a',
      weight: 9,
      opacity: 0.35,
      lineCap: 'round',
      lineJoin: 'round'
    }).addTo(this.activeRouteLayer);

    const routeLine = L.polyline(latlngs, {
      color: '#10b981',
      weight: 5.5,
      opacity: 0.95,
      lineCap: 'round',
      lineJoin: 'round'
    }).addTo(this.activeRouteLayer);

    this.map.fitBounds(routeLine.getBounds(), {
      padding: [40, 40],
      maxZoom: 18
    });
  }

  renderSummaryAndDirections(result) {
    const startName = POI_MAP[this.selectedStartPoi]?.name || "Titik Asal Pejalan Kaki";
    const endName = POI_MAP[this.selectedEndPoi]?.name || "Titik Tujuan Hutan UNESA";

    const directions = generateDirections(result.path, startName, endName);
    this.currentDirections = directions;

    const distMeter = result.totalDistance;
    const estMinutes = Math.max(1, Math.round(distMeter / 75));

    document.getElementById('metric-distance').textContent = `${distMeter} m`;
    document.getElementById('metric-time').textContent = `${estMinutes} mnt`;
    document.getElementById('metric-nodes').textContent = `${result.visitedNodesCount} node`;
    document.getElementById('metric-execution').textContent = `${result.executionTimeMs} ms`;
    document.getElementById('badge-algo-name').textContent = result.algorithm;

    const stepsContainer = document.getElementById('steps-list');
    stepsContainer.innerHTML = '';

    directions.forEach((step, idx) => {
      const li = document.createElement('li');
      let itemClass = "step-item";
      if (step.type === 'start') itemClass += " is-start";
      if (step.type === 'finish') itemClass += " is-finish";
      li.className = itemClass;
      li.id = `step-item-${idx}`;

      li.innerHTML = `
        <div class="step-icon-box">${step.icon}</div>
        <div class="step-content">
          <div class="step-instruction">${step.instruction}</div>
          <div class="step-meta">
            ${step.segmentDistance > 0 ? `<span class="step-distance-badge">+${step.segmentDistance} m</span>` : ''}
            <span>Total: ${step.cumulativeDistance} m</span>
          </div>
        </div>
      `;

      li.addEventListener('click', () => {
        this.map.panTo(step.coords, { animate: true });
        this.highlightStep(idx);
      });

      stepsContainer.appendChild(li);
    });
  }

  highlightStep(stepIndex) {
    document.querySelectorAll('.step-item').forEach((el, i) => {
      if (i === stepIndex) {
        el.classList.add('active-step');
        el.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
      } else {
        el.classList.remove('active-step');
      }
    });
  }

  // -----------------------------------------------------------
  // Simulasi Navigasi & Suara
  // -----------------------------------------------------------
  startSimulation() {
    if (!this.currentPathCoords || this.currentPathCoords.length === 0) return;

    if (this.isSimulating) {
      clearInterval(this.simulationInterval);
      this.isSimulating = false;
      document.getElementById('btn-simulation').innerHTML = '🏃 Simulasi';
      return;
    }

    this.isSimulating = true;
    document.getElementById('btn-simulation').innerHTML = '⏸️ Stop';

    let coordIdx = 0;
    const totalCoords = this.currentPathCoords.length;

    const walkerIcon = L.divIcon({
      html: '<div class="walker-pin">🧭</div>',
      className: 'custom-node-icon',
      iconSize: [32, 32],
      iconAnchor: [16, 16]
    });

    if (this.walkerMarker) this.walkerMarker.remove();
    this.walkerMarker = L.marker(this.currentPathCoords[0], { icon: walkerIcon }).addTo(this.map);
    this.map.panTo(this.currentPathCoords[0], { animate: true });

    this.simulationInterval = setInterval(() => {
      coordIdx++;
      if (coordIdx >= totalCoords) {
        clearInterval(this.simulationInterval);
        this.isSimulating = false;
        document.getElementById('btn-simulation').innerHTML = '🏃 Simulasi';
        return;
      }

      const coord = this.currentPathCoords[coordIdx];
      this.walkerMarker.setLatLng(coord);

      if (this.currentDirections) {
        const closestStepIdx = this.currentDirections.findIndex(step => {
          return Math.abs(step.coords[0] - coord[0]) < 0.0001 && Math.abs(step.coords[1] - coord[1]) < 0.0001;
        });
        if (closestStepIdx !== -1) {
          this.highlightStep(closestStepIdx);
          if (this.isSpeaking) {
            this.speakText(this.currentDirections[closestStepIdx].instruction.replace(/<[^>]*>?/gm, ''));
          }
        }
      }
    }, 600);
  }

  toggleSpeechGuide() {
    const btn = document.getElementById('btn-audio-guide');
    if (!('speechSynthesis' in window)) {
      alert("Browser Anda belum mendukung Web Speech Synthesis API.");
      return;
    }

    if (window.speechSynthesis.speaking) {
      window.speechSynthesis.cancel();
      this.isSpeaking = false;
      btn.classList.remove('speaking');
      btn.innerHTML = '🔊 Suara';
      return;
    }

    this.isSpeaking = true;
    btn.classList.add('speaking');
    btn.innerHTML = '⏹️ Hening';

    let fullNarration = `Memulai navigasi jalan setapak Hutan UNESA. `;
    this.currentDirections.forEach(step => {
      const clean = step.instruction.replace(/<[^>]*>?/gm, '');
      fullNarration += `${clean}. `;
    });

    this.speakText(fullNarration, () => {
      this.isSpeaking = false;
      btn.classList.remove('speaking');
      btn.innerHTML = '🔊 Suara';
    });
  }

  speakText(text, onEnd) {
    if (!('speechSynthesis' in window)) return;
    window.speechSynthesis.cancel();

    const utterance = new SpeechSynthesisUtterance(text);
    utterance.lang = 'id-ID';
    utterance.rate = 1.0;
    if (onEnd) utterance.onend = onEnd;
    window.speechSynthesis.speak(utterance);
  }

  // -----------------------------------------------------------
  // Modal Komparasi A* vs Dijkstra
  // -----------------------------------------------------------
  showComparisonModal() {
    const resAStar = runAStar(this.startNodeId, this.endNodeId);
    const resDijkstra = runDijkstra(this.startNodeId, this.endNodeId);

    const savedNodes = resDijkstra.visitedNodesCount - resAStar.visitedNodesCount;
    const efficiencyPercent = resDijkstra.visitedNodesCount > 0
      ? Math.round((savedNodes / resDijkstra.visitedNodesCount) * 100)
      : 0;

    const modalBody = document.getElementById('modal-compare-body');
    modalBody.innerHTML = `
      <table class="compare-table">
        <thead>
          <tr>
            <th>Parameter Evaluasi</th>
            <th class="badge-a-star">⭐ Algoritma A*</th>
            <th class="badge-dijkstra">🔷 Algoritma Dijkstra</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><strong>Kesesuaian Jalur</strong></td>
            <td>100% Mengikuti Jalan Setapak OSM</td>
            <td>100% Mengikuti Jalan Setapak OSM</td>
          </tr>
          <tr>
            <td><strong>Total Jarak Rute</strong></td>
            <td><span class="badge-a-star">${resAStar.totalDistance} meter</span></td>
            <td><span class="badge-dijkstra">${resDijkstra.totalDistance} meter</span></td>
          </tr>
          <tr>
            <td><strong>Node Graf yang Dieksplorasi</strong></td>
            <td><strong class="badge-a-star">${resAStar.visitedNodesCount} node</strong></td>
            <td><strong class="badge-dijkstra">${resDijkstra.visitedNodesCount} node</strong></td>
          </tr>
          <tr>
            <td><strong>Penghematan Komputasi</strong></td>
            <td><span class="badge-a-star">Hemat ${efficiencyPercent}% pencarian</span></td>
            <td>Standar Radial Search</td>
          </tr>
          <tr>
            <td><strong>Waktu Komputasi</strong></td>
            <td>${resAStar.executionTimeMs} ms</td>
            <td>${resDijkstra.executionTimeMs} ms</td>
          </tr>
          <tr>
            <td><strong>Fungsi Evaluasi</strong></td>
            <td>f(n) = g(n) + h(n) (Heuristik Haversine Spasial)</td>
            <td>f(n) = g(n) (Uniform Cost Radial)</td>
          </tr>
        </tbody>
      </table>

      <div class="compare-insight">
        <strong>💡 Bukti Keefektifan Algoritma A*:</strong><br>
        Pada graf jalan setapak Hutan UNESA dengan ratusan simpul nyata, algoritma <strong>A* terbukti jauh lebih efektif</strong>. A* hanya memeriksa <strong>${resAStar.visitedNodesCount} node</strong> dibandingkan Dijkstra yang memeriksa <strong>${resDijkstra.visitedNodesCount} node</strong> (penghematan <strong>${efficiencyPercent}%</strong>). Hal ini karena fungsi heuristik jarak Haversine membimbing pencarian langsung ke arah tujuan.
      </div>
    `;

    document.getElementById('modal-compare').classList.add('show');
  }
}

let app;
window.addEventListener('DOMContentLoaded', () => {
  app = new ZooSMateNavigationApp();
  app.init();
});
