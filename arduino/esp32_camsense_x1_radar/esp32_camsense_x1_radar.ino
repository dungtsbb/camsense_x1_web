#include <WiFi.h>
#include <WebServer.h>

// ================= CẤU HÌNH WIFI AP (ESP32 TỰ PHÁT) =================
const char* AP_SSID = "ESP32_LiDAR_X1";
const char* AP_PASS = "12345678"; // Mật khẩu tối thiểu 8 ký tự

// ================= CẤU HÌNH UART CAMSENSE X1 =================
#define LIDAR_RX_PIN     16     // Nối với chân TX của Camsense X1
#define LIDAR_TX_PIN     -1     // Không cần nối chân TX
#define LIDAR_BAUD       115200
#define PKT_SIZE         36
#define PKT_POINTS       8
#define ANGLE_ZERO       0xA000
#define DIST_SANE_MAX_MM 12000
#define SCAN_BINS        360

WebServer server(80);

struct ScanData {
  uint16_t dist_mm[SCAN_BINS];
  uint8_t  quality[SCAN_BINS];
  uint16_t rpm;
  uint32_t seq;
};

// Buffer kép + spinlock bảo vệ đa luồng (giống hệt bản ESP-IDF)
static ScanData s_building, s_complete;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_has_scan = false;
static volatile uint32_t s_scan_count = 0;
static volatile uint32_t s_err_count  = 0;
static float s_last_start_deg = -1.0f;

// ================= GIAO DIỆN WEB (HTML5 CANVAS RADAR) =================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Camsense X1 LiDAR Map</title>
<style>
  :root {
    --bg: #0b0f19;
    --panel: #151c2c;
    --accent: #00f2fe;
    --green: #00ff87;
    --text: #e2e8f0;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: system-ui, -apple-system, sans-serif;
    display: flex;
    flex-direction: column;
    align-items: center;
    min-height: 100vh;
    padding: 12px;
  }
  header {
    width: 100%;
    max-width: 720px;
    background: var(--panel);
    padding: 12px 18px;
    border-radius: 12px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 12px;
    border: 1px solid #26334d;
    flex-wrap: wrap;
    gap: 10px;
  }
  .title {
    font-size: 1.15rem;
    font-weight: 700;
    color: var(--accent);
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .dot {
    width: 10px;
    height: 10px;
    background: var(--green);
    border-radius: 50%;
    box-shadow: 0 0 8px var(--green);
  }
  .stats {
    display: flex;
    gap: 16px;
    font-size: 0.9rem;
    flex-wrap: wrap;
  }
  .stat-item span {
    font-weight: 700;
    color: var(--green);
  }
  .controls {
    width: 100%;
    max-width: 720px;
    background: var(--panel);
    padding: 10px 18px;
    border-radius: 12px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 12px;
    border: 1px solid #26334d;
    flex-wrap: wrap;
    gap: 12px;
    font-size: 0.9rem;
  }
  select, button, input[type="range"] {
    background: #0b0f19;
    color: var(--text);
    border: 1px solid #334155;
    padding: 6px 10px;
    border-radius: 6px;
    cursor: pointer;
  }
  .canvas-wrap {
    position: relative;
    width: 100%;
    max-width: 680px;
    aspect-ratio: 1 / 1;
    background: radial-gradient(circle, #111a2e 0%, #090d16 100%);
    border-radius: 50%;
    border: 2px solid #1e293b;
    box-shadow: 0 0 30px rgba(0, 242, 254, 0.08);
    overflow: hidden;
  }
  canvas {
    width: 100%;
    height: 100%;
    display: block;
  }
  .tooltip {
    margin-top: 10px;
    font-size: 0.85rem;
    color: #94a3b8;
  }
</style>
</head>
<body>

<header>
  <div class="title"><div class="dot" id="statusDot"></div> Camsense X1 2D Map</div>
  <div class="stats">
    <div class="stat-item">Tốc độ: <span id="rpmVal">0</span> RPM</div>
    <div class="stat-item">Điểm: <span id="ptsVal">0</span>/360</div>
    <div class="stat-item">Vòng: #<span id="seqVal">0</span></div>
    <div class="stat-item">Lỗi CRC: <span id="errVal">0</span></div>
  </div>
</header>

<div class="controls">
  <div>
    <label>Bán kính tối đa: </label>
    <select id="maxRange">
      <option value="1000">1 mét</option>
      <option value="2000">2 mét</option>
      <option value="3000">3 mét</option>
      <option value="5000" selected>5 mét</option>
      <option value="8000">8 mét</option>
    </select>
  </div>
  <div>
    <label>Chế độ vẽ: </label>
    <select id="drawMode">
      <option value="points">Điểm (Point Cloud)</option>
      <option value="polygon">Đa giác (Room Contour)</option>
      <option value="rays">Tia quét (Laser Rays)</option>
    </select>
  </div>
  <div>
    <label>Cỡ điểm: </label>
    <input type="range" id="ptSize" min="1" max="6" value="3">
  </div>
</div>

<div class="canvas-wrap">
  <canvas id="radarCanvas" width="680" height="680"></canvas>
</div>
<div class="tooltip" id="hoverInfo">Di chuột hoặc chạm vào bản đồ để xem khoảng cách tại góc tương ứng</div>

<script>
const canvas = document.getElementById('radarCanvas');
const ctx = canvas.getContext('2d');
const W = canvas.width;
const H = canvas.height;
const CX = W / 2;
const CY = H / 2;
const RADIUS = (W / 2) - 24;

let distances = new Array(360).fill(0);

function drawGrid(maxDist) {
  ctx.clearRect(0, 0, W, H);
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;

  const steps = 4;
  for (let i = 1; i <= steps; i++) {
    const r = (RADIUS / steps) * i;
    ctx.beginPath();
    ctx.arc(CX, CY, r, 0, Math.PI * 2);
    ctx.stroke();

    const distLabel = ((maxDist / 1000) * (i / steps)).toFixed(2) + 'm';
    ctx.fillStyle = '#64748b';
    ctx.font = '12px sans-serif';
    ctx.fillText(distLabel, CX + 6, CY - r + 14);
  }

  for (let deg = 0; deg < 360; deg += 30) {
    const rad = (deg - 90) * Math.PI / 180;
    const x = CX + RADIUS * Math.cos(rad);
    const y = CY + RADIUS * Math.sin(rad);
    ctx.beginPath();
    ctx.moveTo(CX, CY);
    ctx.lineTo(x, y);
    ctx.stroke();

    const lx = CX + (RADIUS + 14) * Math.cos(rad);
    const ly = CY + (RADIUS + 14) * Math.sin(rad);
    ctx.fillStyle = '#94a3b8';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(deg + '°', lx, ly);
  }
  ctx.textAlign = 'start';
}

function renderMap() {
  const maxDist = parseInt(document.getElementById('maxRange').value);
  const mode = document.getElementById('drawMode').value;
  const ptSize = parseInt(document.getElementById('ptSize').value);

  drawGrid(maxDist);

  let validPts = 0;
  const coords = [];

  for (let deg = 0; deg < 360; deg++) {
    const d = distances[deg];
    if (d > 0 && d <= maxDist) {
      validPts++;
      const rad = (deg - 90) * Math.PI / 180;
      const r = (d / maxDist) * RADIUS;
      const x = CX + r * Math.cos(rad);
      const y = CY + r * Math.sin(rad);
      coords.push({ x, y, d, deg });
    }
  }

  document.getElementById('ptsVal').textContent = validPts;

  if (mode === 'polygon' && coords.length > 2) {
    ctx.beginPath();
    ctx.moveTo(coords[0].x, coords[0].y);
    for (let i = 1; i < coords.length; i++) {
      ctx.lineTo(coords[i].x, coords[i].y);
    }
    ctx.closePath();
    ctx.fillStyle = 'rgba(0, 242, 254, 0.12)';
    ctx.fill();
    ctx.strokeStyle = '#00f2fe';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  } else if (mode === 'rays') {
    ctx.strokeStyle = 'rgba(0, 255, 135, 0.25)';
    ctx.lineWidth = 1;
    for (const pt of coords) {
      ctx.beginPath();
      ctx.moveTo(CX, CY);
      ctx.lineTo(pt.x, pt.y);
      ctx.stroke();
    }
  }

  for (const pt of coords) {
    const ratio = Math.min(pt.d / maxDist, 1);
    const hue = Math.floor(ratio * 160);
    ctx.fillStyle = `hsl(${hue}, 100%, 55%)`;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, ptSize, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.fillStyle = '#ff0055';
  ctx.beginPath();
  ctx.arc(CX, CY, 5, 0, Math.PI * 2);
  ctx.fill();
}

async function fetchScanData() {
  try {
    const res = await fetch('/scan');
    if (!res.ok) throw new Error('HTTP err');
    const data = await res.json();
    if (data.d && data.d.length === 360) {
      distances = data.d;
    }
    document.getElementById('rpmVal').textContent = data.rpm || 0;
    document.getElementById('seqVal').textContent = data.seq || 0;
    document.getElementById('errVal').textContent = data.err || 0;
    document.getElementById('statusDot').style.background = '#00ff87';
    renderMap();
  } catch (e) {
    document.getElementById('statusDot').style.background = '#ff3366';
  }
  setTimeout(fetchScanData, 100);
}

canvas.addEventListener('mousemove', (e) => {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const mx = (e.clientX - rect.left) * scaleX - CX;
  const my = (e.clientY - rect.top) * scaleY - CY;

  let deg = Math.round(Math.atan2(my, mx) * 180 / Math.PI + 90);
  if (deg < 0) deg += 360;
  deg = deg % 360;

  const d = distances[deg] || 0;
  document.getElementById('hoverInfo').textContent =
    `Góc: ${deg}° | Khoảng cách thực tế: ${d > 0 ? d + ' mm (' + (d/1000).toFixed(2) + ' m)' : 'Không có vật cản'}`;
});

fetchScanData();
</script>
</body>
</html>
)rawliteral";

// ================= KIỂM TRA CHECKSUM 15-BIT VÀ GIẢI MÃ GÓI TIN =================
static uint16_t x1Checksum(const uint8_t *buf, size_t lenBytes) {
  uint32_t chk32 = 0;
  for (size_t i = 0; i < lenBytes / 2; i++) {
    uint16_t word = (uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8);
    chk32 = (chk32 << 1) + word;
  }
  uint16_t sum = (uint16_t)((chk32 & 0x7FFF) + (chk32 >> 15));
  return sum & 0x7FFF;
}

static bool packetValid(const uint8_t *pkt) {
  uint16_t stored = (uint16_t)pkt[PKT_SIZE - 2] | ((uint16_t)pkt[PKT_SIZE - 1] << 8);
  return x1Checksum(pkt, PKT_SIZE - 2) == stored;
}

static float rawToDeg(uint16_t raw) {
  return (float)(uint16_t)(raw - ANGLE_ZERO) / 64.0f;
}

static void finishScan() {
  taskENTER_CRITICAL(&s_lock);
  s_building.seq = ++s_scan_count;
  memcpy(&s_complete, &s_building, sizeof(ScanData));
  memset(&s_building, 0, sizeof(ScanData));
  taskEXIT_CRITICAL(&s_lock);
  s_has_scan = true;
}

static void handlePacket(const uint8_t *buf) {
  uint16_t rawSpeed = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
  uint16_t rawStart = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
  uint16_t rawEnd   = (uint16_t)buf[32] | ((uint16_t)buf[33] << 8);

  float start = rawToDeg(rawStart);
  float end   = rawToDeg(rawEnd);
  float span  = end - start;
  if (span < 0.0f) span += 360.0f;
  float step  = span / (PKT_POINTS - 1);

  if (s_last_start_deg >= 0.0f && start < s_last_start_deg) {
    finishScan();
  }
  s_last_start_deg = start;
  s_building.rpm = rawSpeed / 64;

  for (int i = 0; i < PKT_POINTS; i++) {
    int offset = 8 + i * 3;
    uint16_t d = (uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8);
    uint8_t  q = buf[offset + 2];
    if (d == 0 || d > DIST_SANE_MAX_MM || q == 0) continue;

    float ang = start + step * i;
    while (ang >= 360.0f) ang -= 360.0f;
    while (ang < 0.0f)    ang += 360.0f;

    int bin = (int)(ang + 0.5f) % SCAN_BINS;
    if (s_building.dist_mm[bin] == 0 || q > s_building.quality[bin]) {
      s_building.dist_mm[bin] = d;
      s_building.quality[bin] = q;
    }
  }
}

static size_t parseBuffer(const uint8_t *buf, size_t len) {
  size_t i = 0;
  while (i + PKT_SIZE <= len) {
    if (buf[i] != 0x55 || buf[i + 1] != 0xAA ||
        buf[i + 2] != 0x03 || buf[i + 3] != 0x08) {
      i++;
      continue;
    }
    if (!packetValid(buf + i)) {
      s_err_count++;
      i++;
      continue;
    }
    handlePacket(buf + i);
    i += PKT_SIZE;
  }
  return i;
}

// Task FreeRTOS ưu tiên cao chuyên đọc UART2 để không mất gói tin khi phục vụ Web
static void lidarTask(void *arg) {
  uint8_t buf[512];
  size_t len = 0;

  while (true) {
    int avail = Serial2.available();
    if (avail <= 0) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    size_t space = sizeof(buf) - len;
    size_t toRead = (size_t)avail < space ? (size_t)avail : space;
    int r = Serial2.readBytes(buf + len, toRead);
    if (r <= 0) continue;

    len += r;
    size_t used = parseBuffer(buf, len);
    if (used > 0) {
      memmove(buf, buf + used, len - used);
      len -= used;
    } else if (len == sizeof(buf)) {
      len = 0;
    }
  }
}

// ================= API TRẢ DỮ LIỆU JSON =================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleScan() {
  static ScanData snap;
  if (!s_has_scan) {
    server.send(200, "application/json", "{\"rpm\":0,\"seq\":0,\"err\":0,\"d\":[]}");
    return;
  }

  taskENTER_CRITICAL(&s_lock);
  memcpy(&snap, &s_complete, sizeof(ScanData));
  taskEXIT_CRITICAL(&s_lock);

  String json;
  json.reserve(1950);
  json = "{\"rpm\":";
  json += String(snap.rpm);
  json += ",\"seq\":";
  json += String(snap.seq);
  json += ",\"err\":";
  json += String(s_err_count);
  json += ",\"d\":[";
  for (int i = 0; i < SCAN_BINS; i++) {
    json += String(snap.dist_mm[i]);
    if (i < SCAN_BINS - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Khoi dong ESP32 Camsense X1 LiDAR Radar (WiFi AP) ---");

  Serial2.setRxBufferSize(2048);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  // Tạo task đọc LiDAR riêng biệt với độ ưu tiên cao
  xTaskCreate(lidarTask, "lidar", 4096, NULL, 13, NULL);

  // Phát WiFi chế độ Access Point (AP)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("==========================================");
  Serial.printf("Da phat WiFi AP: %s\n", AP_SSID);
  Serial.printf("Mat khau       : %s\n", AP_PASS);
  Serial.print("Truy cap Web   : http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("==========================================");

  server.on("/", handleRoot);
  server.on("/scan", handleScan);
  server.begin();
}

uint32_t lastLogMs = 0;

void loop() {
  server.handleClient();
  if (millis() - lastLogMs >= 2000) {
    lastLogMs = millis();
    Serial.printf("vong quet=%lu  goi sai checksum=%lu  rpm=%u\n",
                  (unsigned long)s_scan_count, (unsigned long)s_err_count,
                  s_complete.rpm);
  }
}
