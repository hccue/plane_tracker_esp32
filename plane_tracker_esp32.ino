/*
  ADS-B mini radar — ESP32 + KMR-1.8 (ST7735S, 160x128)
  Live aircraft from api.adsb.lol, drawn as a radar around a fixed map centre.

  Libraries (Arduino Library Manager):
    - Adafruit ST7735 and ST7789 Library
    - Adafruit GFX Library
    - ArduinoJson (v7)
  Board: "ESP32 Dev Module" (esp32 by Espressif)

  Wiring — names as printed on the back of the KMR-1.8 SPI board:
    1  GND    -> GND
    2  VCC    -> VIN / 5V   (on-board regulator U2, JP1 left open)
    6  RESET  -> GPIO4
    7  A0     -> GPIO2      (data/command)
    8  SDA    -> GPIO23     (display data, VSPI MOSI)
    9  SCL    -> GPIO18     (display clock, VSPI SCK)
    10 CS     -> GPIO5
    15 LED+   -> 3V3
    16 LED-   -> GND
    3-5 NC and 11-14 (SCK, MISO, MOSI, SD_CS = SD card slot): not connected

  Controller: ST7735S. If colours are swapped (red<->blue) or there is a
  garbage line on one edge, change INITR_BLACKTAB below to INITR_GREENTAB.
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <time.h>
#include <math.h>

// ======================= USER SETTINGS =======================
// Wi-Fi credentials live in secrets.h (git-ignored). Copy secrets.example.h.
#include "secrets.h"

// Map centre (hardcoded) and zoom
const float CENTER_LAT = -0.18f;
const float CENTER_LON = -78.47f;
const float RANGE_KM   = 25.0f;   // distance from centre to top/bottom edge
const float RING_STEP_KM = 10.0f; // range ring spacing

// Airport (SEQM - Quito Mariscal Sucre)
const char* AIRPORT_NAME   = "SEQM";
const float AIRPORT_LAT    = -0.1292f;
const float AIRPORT_LON    = -78.3575f;
const float RUNWAY_HDG     = 0.0f;    // true heading of runway 18/36
const float RUNWAY_LEN_KM  = 4.1f;
const int   AIRPORT_ELEV_FT = 7900;

// Arrival/departure highlight: within this distance of the airport
// and below this height above the airport -> amber
const float APT_ZONE_KM      = 35.0f;
const int   APT_ZONE_AGL_FT  = 10000;

const uint32_t FETCH_INTERVAL_MS = 15000;  // be polite to the free API
const uint32_t MAX_BACKOFF_MS    = 120000; // cap when rate-limited (429)
const int      UTC_OFFSET_SEC    = -5 * 3600;  // Ecuador

// Screen pins (KMR-1.8 labels). SCL->GPIO18 and SDA->GPIO23 are fixed by
// the ESP32 hardware SPI bus, so they are not set here.
#define TFT_CS   5   // CS
#define TFT_DC   2   // A0
#define TFT_RST  4   // RESET
// ============================================================

#define W 160
#define H 128
#define MAX_PLANES 48

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(W, H);

// ---------- types (defined before any function so Arduino's auto-generated prototypes can see them) ----------
struct Plane {
  char  cs[9];     // callsign
  float lat, lon;
  float gs;        // knots
  float trk;       // degrees true
  bool  hasTrk;
  int   alt;       // feet
  float seenPos;   // seconds since position was received (at fetch time)
};

struct Box  { int x, y, w, h; };
struct Draw { int x, y; float prio; bool amber; int idx; };

static inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
const uint16_t C_BG    = RGB(0, 0, 0);
const uint16_t C_GREEN = RGB(0, 255, 90);
const uint16_t C_MID   = RGB(0, 160, 70);
const uint16_t C_AMBER = RGB(255, 190, 0);
const uint16_t C_WHITE = RGB(235, 235, 235);
const uint16_t C_SWEEP = RGB(150, 255, 170);
const uint16_t C_RED   = RGB(255, 60, 60);


// Shared between fetch task (core 0) and render loop (core 1)
Plane     shared[MAX_PLANES];
int       sharedCount = 0;
uint32_t  sharedFetchedAt = 0;
volatile int lastHttpCode = 0;
SemaphoreHandle_t mtx;

// ---------- geometry ----------
const float PX_PER_KM = (H / 2.0f) / RANGE_KM;
const float KM_PER_DEG_LAT = 110.57f;
float kmPerDegLon;  // depends on latitude
const int CX = W / 2, CY = H / 2;

void project(float lat, float lon, float& x, float& y) {
  x = CX + (lon - CENTER_LON) * kmPerDegLon * PX_PER_KM;
  y = CY - (lat - CENTER_LAT) * KM_PER_DEG_LAT * PX_PER_KM;
}

float distKm(float lat1, float lon1, float lat2, float lon2) {
  float dx = (lon2 - lon1) * kmPerDegLon;
  float dy = (lat2 - lat1) * KM_PER_DEG_LAT;
  return sqrtf(dx * dx + dy * dy);
}

// ---------- WiFi ----------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  const char* ssids[] = {WIFI_SSID, WIFI_SSID2};
  const char* passes[] = {WIFI_PASS, WIFI_PASS2};
  for (int n = 0; n < 2; n++) {
    if (!ssids[n] || !ssids[n][0]) continue;
    WiFi.disconnect();
    WiFi.begin(ssids[n], passes[n]);
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) delay(250);
    if (WiFi.status() == WL_CONNECTED) return;
  }
}

// ---------- data fetch (runs on core 0) ----------
void fetchTask(void*) {
  static Plane buf[MAX_PLANES];

  // radius that covers the screen corners, in nautical miles
  float cornerKm = sqrtf((float)(CX * CX + CY * CY)) / PX_PER_KM;
  int radiusNm = (int)ceilf(cornerKm / 1.852f) + 2;

  char url[128];
  snprintf(url, sizeof(url), "https://api.adsb.lol/v2/point/%.4f/%.4f/%d",
           CENTER_LAT, CENTER_LON, radiusNm);

  JsonDocument filter;
  JsonObject f = filter["ac"].add<JsonObject>();
  f["flight"] = true; f["hex"] = true; f["r"] = true;
  f["lat"] = true; f["lon"] = true; f["alt_baro"] = true;
  f["gs"] = true; f["track"] = true; f["seen_pos"] = true;

  uint32_t delayMs = FETCH_INTERVAL_MS;
  for (;;) {
    connectWiFi();
    if (WiFi.status() == WL_CONNECTED) {
      WiFiClientSecure client;
      client.setInsecure();  // skip cert check (fine for public read-only data)
      HTTPClient http;
      http.setTimeout(8000);
      http.begin(client, url);
      http.addHeader("User-Agent", "esp32-mini-radar");
      const char* hdrs[] = {"Retry-After"};
      http.collectHeaders(hdrs, 1);
      int code = http.GET();
      lastHttpCode = code;
      if (code == 429) {
        // rate-limited: honour Retry-After if given, else double the wait
        long ra = http.header("Retry-After").toInt();
        delayMs = ra > 0 ? (uint32_t)ra * 1000UL : delayMs * 2;
        if (delayMs > MAX_BACKOFF_MS) delayMs = MAX_BACKOFF_MS;
      } else if (code == 200) {
        delayMs = FETCH_INTERVAL_MS;
        // HTTP/1.1 may use chunked encoding, so read the whole body first
        // (api.adsb.lol rejects HTTP/1.0 with 426 Upgrade Required)
        String body = http.getString();
        JsonDocument doc;
        DeserializationError err = deserializeJson(
            doc, body, DeserializationOption::Filter(filter));
        if (!err) {
          int n = 0;
          for (JsonObject a : doc["ac"].as<JsonArray>()) {
            if (n >= MAX_PLANES) break;
            if (!a["lat"].is<float>() || !a["lon"].is<float>()) continue;
            if (!a["alt_baro"].is<int>()) continue;  // "ground" or missing
            Plane& p = buf[n];
            const char* cs = a["flight"] | "";
            if (!cs[0]) cs = a["r"] | "";
            if (!cs[0]) cs = a["hex"] | "?";
            strncpy(p.cs, cs, 8); p.cs[8] = 0;
            for (int i = strlen(p.cs) - 1; i >= 0 && p.cs[i] == ' '; i--) p.cs[i] = 0;
            for (char* c = p.cs; *c; c++) *c = toupper(*c);
            p.lat = a["lat"]; p.lon = a["lon"];
            p.alt = a["alt_baro"];
            p.gs = a["gs"] | 0.0f;
            p.hasTrk = a["track"].is<float>();
            p.trk = a["track"] | 0.0f;
            p.seenPos = a["seen_pos"] | 0.0f;
            n++;
          }
          xSemaphoreTake(mtx, portMAX_DELAY);
          memcpy(shared, buf, sizeof(Plane) * n);
          sharedCount = n;
          sharedFetchedAt = millis();
          xSemaphoreGive(mtx);
        } else {
          lastHttpCode = -100;  // parse error
        }
      }
      http.end();
    }
    vTaskDelay(pdMS_TO_TICKS(delayMs));
  }
}

// ---------- drawing helpers ----------
Box placed[MAX_PLANES * 2 + 8];
int nPlaced;

bool overlaps(const Box& b) {
  if (b.x < 0 || b.y < 0 || b.x + b.w > W || b.y + b.h > H) return true;
  for (int i = 0; i < nPlaced; i++) {
    const Box& o = placed[i];
    if (b.x < o.x + o.w && b.x + b.w > o.x && b.y < o.y + o.h && b.y + b.h > o.y) return true;
  }
  return false;
}

void drawSweep(float angDeg) {
  const int TRAIL = 34;
  const float R = 110;
  for (int i = TRAIL - 1; i >= 0; i--) {
    float a0 = (angDeg - i) * DEG_TO_RAD, a1 = (angDeg - i - 1.2f) * DEG_TO_RAD;
    uint8_t g = 100 * (TRAIL - i) / TRAIL;
    canvas.fillTriangle(CX, CY, CX + R * cosf(a0), CY + R * sinf(a0),
                        CX + R * cosf(a1), CY + R * sinf(a1), RGB(0, g, g / 3));
  }
}

void drawPlaneIcon(int x, int y, float trk, bool hasTrk, uint16_t col) {
  float t = trk * DEG_TO_RAD;
  if (hasTrk) canvas.drawLine(x, y, x + 8 * sinf(t), y - 8 * cosf(t), col);
  float a = t, b = t + 140 * DEG_TO_RAD, c = t + 220 * DEG_TO_RAD;
  canvas.fillTriangle(x + 3 * sinf(a), y - 3 * cosf(a),
                      x + 2.5f * sinf(b), y - 2.5f * cosf(b),
                      x + 2.5f * sinf(c), y - 2.5f * cosf(c), col);
}

// ---------- render ----------
Plane local[MAX_PLANES];
Draw dl[MAX_PLANES];

void render() {
  int n; uint32_t fetchedAt;
  xSemaphoreTake(mtx, portMAX_DELAY);
  n = sharedCount; fetchedAt = sharedFetchedAt;
  memcpy(local, shared, sizeof(Plane) * n);
  xSemaphoreGive(mtx);

  canvas.fillScreen(C_BG);
  canvas.setTextWrap(false);
  canvas.setTextSize(1);

  // sweep: one turn every 4 s
  float sweep = fmodf(millis() / 4000.0f * 360.0f, 360.0f);
  drawSweep(sweep - 90);  // 0 = north

  // range rings + centre cross
  for (float km = RING_STEP_KM; km * PX_PER_KM < 110; km += RING_STEP_KM)
    canvas.drawCircle(CX, CY, km * PX_PER_KM, C_MID);
  canvas.drawFastHLine(CX - 2, CY, 5, C_MID);
  canvas.drawFastVLine(CX, CY - 2, 5, C_MID);
  float sr = (sweep - 90) * DEG_TO_RAD;
  canvas.drawLine(CX, CY, CX + 110 * cosf(sr), CY + 110 * sinf(sr), C_SWEEP);

  nPlaced = 0;
  canvas.setTextColor(C_MID);
  for (int k = 1; k <= 2; k++) {
    int ry = CY - k * RING_STEP_KM * PX_PER_KM - 8;
    if (ry < 0) break;
    canvas.setCursor(CX + 3, ry);
    canvas.print((int)(k * RING_STEP_KM));
    placed[nPlaced++] = {CX + 3, ry, 12, 8};
  }

  // airport + runway
  float ax, ay; project(AIRPORT_LAT, AIRPORT_LON, ax, ay);
  float rh = RUNWAY_HDG * DEG_TO_RAD, half = RUNWAY_LEN_KM / 2 * PX_PER_KM;
  float dx = sinf(rh) * half, dy = -cosf(rh) * half;
  canvas.drawLine(ax - dx, ay - dy, ax + dx, ay + dy, C_WHITE);
  canvas.drawLine(ax - dx + 1, ay - dy, ax + dx + 1, ay + dy, C_WHITE);
  int alx = ax + 4, aly = ay - 3;
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(alx, aly);
  canvas.print(AIRPORT_NAME);
  placed[nPlaced++] = {alx, aly, (int)strlen(AIRPORT_NAME) * 6, 8};

  // corner info
  canvas.fillTriangle(4, 1, 1, 5, 7, 5, C_GREEN);
  canvas.setTextColor(C_GREEN);
  canvas.setCursor(2, 7); canvas.print("N");
  placed[nPlaced++] = {0, 0, 10, 16};

  // positions, extrapolated with ground speed and track
  float age = (millis() - fetchedAt) / 1000.0f;
  if (age > 30) age = 30;
  int m = 0;
  for (int i = 0; i < n; i++) {
    Plane& p = local[i];
    float dt = age + p.seenPos;
    if (dt > 40) dt = 40;
    float lat = p.lat, lon = p.lon;
    if (p.hasTrk && p.gs > 30) {
      float km = p.gs * 1.852f / 3600.0f * dt;
      lat += km * cosf(p.trk * DEG_TO_RAD) / KM_PER_DEG_LAT;
      lon += km * sinf(p.trk * DEG_TO_RAD) / kmPerDegLon;
    }
    float x, y; project(lat, lon, x, y);
    if (x < -4 || x > W + 4 || y < -4 || y > H + 4) continue;
    bool amber = distKm(lat, lon, AIRPORT_LAT, AIRPORT_LON) < APT_ZONE_KM &&
                 p.alt < AIRPORT_ELEV_FT + APT_ZONE_AGL_FT;
    float dc = sqrtf((x - CX) * (x - CX) + (y - CY) * (y - CY));
    dl[m++] = {(int)roundf(x), (int)roundf(y), (amber ? 0 : 1000) + dc, amber, i};
  }
  // label priority: airport traffic first, then closest to centre
  for (int i = 1; i < m; i++)
    for (int j = i; j > 0 && dl[j].prio < dl[j - 1].prio; j--) { Draw t = dl[j]; dl[j] = dl[j - 1]; dl[j - 1] = t; }

  // icons first (so they always show), reserving their space
  for (int i = 0; i < m; i++) {
    Plane& p = local[dl[i].idx];
    drawPlaneIcon(dl[i].x, dl[i].y, p.trk, p.hasTrk, dl[i].amber ? C_AMBER : C_GREEN);
    if (nPlaced < (int)(sizeof(placed) / sizeof(placed[0])))
      placed[nPlaced++] = {dl[i].x - 4, dl[i].y - 4, 9, 9};
  }
  // labels where they fit
  for (int i = 0; i < m; i++) {
    Plane& p = local[dl[i].idx];
    char alt[6]; snprintf(alt, sizeof(alt), "%03d", constrain(p.alt / 100, 0, 999));
    int lw = max((int)strlen(p.cs), 3) * 6, lh = 16;
    const int offs[4][2] = {{5, 2}, {5, -18}, {-5 - lw, 2}, {-5 - lw, -18}};
    for (int k = 0; k < 4; k++) {
      Box b = {dl[i].x + offs[k][0], dl[i].y + offs[k][1], lw, lh};
      if (overlaps(b)) continue;
      uint16_t c = dl[i].amber ? C_AMBER : C_GREEN;
      canvas.setTextColor(c);
      canvas.setCursor(b.x, b.y); canvas.print(p.cs);
      canvas.setTextColor(dl[i].amber ? C_AMBER : C_MID);
      canvas.setCursor(b.x, b.y + 8); canvas.print(alt);
      placed[nPlaced++] = b;
      break;
    }
  }

  // status: count (top right), clock (bottom left), errors
  char s[16];
  snprintf(s, sizeof(s), "%d AC", m);
  canvas.setTextColor(C_MID);
  canvas.setCursor(W - strlen(s) * 6 - 1, 2); canvas.print(s);
  struct tm ti;
  if (getLocalTime(&ti, 0)) {
    strftime(s, sizeof(s), "%H:%M", &ti);
    canvas.setCursor(2, H - 9); canvas.print(s);
  }
  if (WiFi.status() != WL_CONNECTED) {
    canvas.setTextColor(C_RED); canvas.setCursor(W - 43, H - 9); canvas.print("NO WIFI");
  } else if (lastHttpCode != 200 && lastHttpCode != 0) {
    snprintf(s, sizeof(s), "ERR %d", lastHttpCode);
    canvas.setTextColor(C_RED); canvas.setCursor(W - strlen(s) * 6 - 1, H - 9); canvas.print(s);
  } else if (fetchedAt && millis() - fetchedAt > 30000) {
    canvas.setTextColor(C_RED); canvas.setCursor(W - 37, H - 9); canvas.print("STALE");
  }

  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), W, H);
}

void setup() {
  Serial.begin(115200);
  kmPerDegLon = 111.32f * cosf(CENTER_LAT * DEG_TO_RAD);

  tft.initR(INITR_BLACKTAB);  // try INITR_GREENTAB / INITR_REDTAB if colours or edges look wrong
  tft.setSPISpeed(27000000);
  tft.setRotation(1);         // landscape; use 3 to flip 180°
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(C_GREEN);
  tft.setCursor(4, 4);
  tft.print("Connecting WiFi...");

  WiFi.mode(WIFI_STA);
  connectWiFi();
  configTime(UTC_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");

  mtx = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(fetchTask, "fetch", 12288, nullptr, 1, nullptr, 0);
}

void loop() {
  uint32_t t0 = millis();
  render();
  uint32_t dt = millis() - t0;
  if (dt < 40) delay(40 - dt);  // ~25 fps
}
