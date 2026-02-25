/*
 * Alpaca RG-11 Safety Monitor – ESP32-C3 SuperMini (Arduino framework)
 *
 * Hardware:
 *   Board  : ESP32-C3 SuperMini (HW-466AB)
 *   Sensor : Hydreon RG-11 rain sensor relay output → GPIO 20
 *   Display: 0.91" SSD1306 OLED (I2C) → SCL=GPIO 4, SDA=GPIO 3
 *
 * WiFi credentials live in include/secrets.h (git-ignored).
 * Copy include/secrets.h.example → include/secrets.h and fill in your values.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AsyncUDP.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "secrets.h"

// ── Pin / display constants ───────────────────────────────────────────────────

#define RAIN_PIN        20   // GPIO 20 – RG-11 relay COM (active-LOW when raining)

#define OLED_SCL         4   // GPIO 4 – I2C SCL
#define OLED_SDA         3   // GPIO 3 – I2C SDA
#define OLED_WIDTH     128
#define OLED_HEIGHT     32
// OLED_ADDR is detected at runtime (0x3C or 0x3D) – see setup()
static uint8_t  g_oledAddr = 0x3C;   // default; overwritten by I2C scan

// ── Alpaca protocol constants ─────────────────────────────────────────────────

#define ALPACA_PORT    11111
#define DISC_PORT      32227

// ── Debounce settings ─────────────────────────────────────────────────────────

#define DEB_SAMPLES       5   // majority-vote window
#define DEB_INTERVAL_MS 100   // sample every 100 ms  → 500 ms settling time

// ── Global objects ────────────────────────────────────────────────────────────

WebServer            webServer(80);            // Human-readable status page
WebServer            httpServer(ALPACA_PORT);  // ASCOM Alpaca API
AsyncUDP             asyncUdp;
Adafruit_SSD1306     oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ── Sensor / Alpaca state (single loop – no concurrent tasks – no mutex) ──────

static bool     g_oledOk     = false;  // set after oled.begin() succeeds
static int      g_rawLevel   = 1;
static int      g_debounced  = 1;
static uint32_t g_lastChange = 0;
static uint32_t g_totalTrans = 0;
static bool     g_alpacaConn = false;
static uint32_t g_serverTxId = 1;

// Debounce ring buffer
static int  g_samples[DEB_SAMPLES];
static int  g_sampleIdx = 0;

// Loop timers
static uint32_t g_nextDebounce  = 0;
static uint32_t g_nextOled      = 0;
static uint32_t g_nextHeartbeat = 0;
static uint32_t g_nextWifiRetry = 0;

// ── Alpaca response helpers ───────────────────────────────────────────────────

static void addCors() {
    httpServer.sendHeader("Access-Control-Allow-Origin",  "*");
    httpServer.sendHeader("Access-Control-Allow-Methods", "GET, PUT, OPTIONS");
    httpServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static uint32_t clientTx() {
    String s = httpServer.arg("ClientTransactionID");
    return s.length() ? (uint32_t)s.toInt() : 0;
}

static void sendBool(bool v) {
    g_alpacaConn = true;
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":" +
        (v ? "true" : "false") + "}");
}

static void sendInt(int v) {
    g_alpacaConn = true;
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":" + String(v) + "}");
}

static void sendStr(const char *v) {
    g_alpacaConn = true;
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":\"" + String(v) + "\"}");
}

static void sendArr(const char *v) {
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":" + String(v) + "}");
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────

static void hRoot() {
    // Alpaca API is on port 11111; build the absolute base URL so the JS
    // fetch works correctly even though this page is served from port 80.
    String apiBase = "http://" + WiFi.localIP().toString() + ":" + String(ALPACA_PORT);
    String html =
        "<html><head><meta charset='UTF-8'>"
        "<style>body{font-family:sans-serif;margin:20px;background:#f5f5f5}"
        "h1{color:#333}"
        ".safe{color:green;font-weight:bold}.unsafe{color:red;font-weight:bold}"
        ".card{background:white;padding:15px;margin:10px 0;border-radius:5px;"
        "box-shadow:0 2px 5px rgba(0,0,0,.1)}</style></head>"
        "<body><h1>RG-11 Safety Monitor</h1><div class='card'>"
        "<p><b>Status:</b> <span id='s'>--</span></p>"
        "<p><b>GPIO " + String(RAIN_PIN) + ":</b> <span id='r'>--</span></p>"
        "<p><b>Transitions:</b> " + String(g_totalTrans) +
        " | <b>Alpaca:</b> " + (g_alpacaConn ? "Connected" : "Disconnected") +
        " | <b>IP:</b> " + WiFi.localIP().toString() + "</p></div>"
        "<script>var B='" + apiBase + "';"
        "function u(){"
        "fetch(B+'/api/v1/safetymonitor/0/issafe').then(r=>r.json()).then(d=>{"
        "const s=d.Value===true;"
        "document.getElementById('s').innerHTML=s?"
        "'<span class=\"safe\">SAFE</span>':'<span class=\"unsafe\">UNSAFE (RAIN)</span>';"
        "document.getElementById('r').innerHTML=s?"
        "'<span style=\"color:green\">HIGH (1)</span>'"
        ":'<span style=\"color:red\">LOW (0)</span>';"
        "}).catch(()=>{document.getElementById('s').innerHTML='Error';});}"
        "setInterval(u,500);u();</script></body></html>";
    webServer.send(200, "text/html", html);
}

static void hFavicon()          { webServer.send(404); }
static void hIsSafe()           { sendBool(g_debounced == 1); }
static void hConnectedGet()     { sendBool(true); }
static void hConnectedPut()     { g_alpacaConn = true; sendBool(true); }
static void hDescription()      { sendStr("Hydreon RG-11 Optical Rain Sensor Safety Monitor"); }
static void hDriverInfo()       { sendStr("ESP32-C3 SuperMini Alpaca Safety Monitor (Arduino)"); }
static void hDriverVersion()    { sendStr("2.0.0"); }
static void hInterfaceVersion() { sendInt(3); }
static void hName()             { sendStr("Alpaca RG-11 Safety Monitor"); }
static void hSupportedActions() { sendArr("[]"); }
static void hDeviceState()      { sendArr("[]"); }

static void hMgmtApiVersions()  { sendArr("[1]"); }

static void hMgmtDescription() {
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\","
        "\"Value\":{\"ServerName\":\"ESP32-C3 Alpaca\",\"Manufacturer\":\"DIY\","
        "\"ManufacturerVersion\":\"2.0\",\"Location\":\"Observatory\"}}");
}

static void hMgmtDevices() {
    sendArr("[{\"DeviceType\":\"SafetyMonitor\","
            "\"DeviceName\":\"RG-11 Safety Monitor\","
            "\"DeviceNumber\":0,\"UniqueID\":\"ESP32C3-RG11-SM-0\"}]");
}

static void hNotFound() {
    if (httpServer.method() == HTTP_OPTIONS) {
        addCors();
        httpServer.send(200);
        return;
    }
    httpServer.send(404, "text/plain", "Not found");
}

static void hNotFound80() {
    webServer.send(404, "text/plain", "Not found");
}

// ── Alpaca UDP discovery – set up once in setup(), runs via AsyncUDP callback ─

// ── Debounce (called every DEB_INTERVAL_MS) ───────────────────────────────────

static void runDebounce() {
    int s = digitalRead(RAIN_PIN);
    g_samples[g_sampleIdx] = s;
    g_sampleIdx = (g_sampleIdx + 1) % DEB_SAMPLES;

    int highs = 0;
    for (int i = 0; i < DEB_SAMPLES; i++) if (g_samples[i]) highs++;
    int newDeb = (highs > DEB_SAMPLES / 2) ? 1 : 0;

    g_rawLevel = s;
    if (newDeb != g_debounced) {
        g_debounced  = newDeb;
        g_lastChange = millis();
        g_totalTrans++;
        Serial.printf("State change → debounced=%d (%s)\n",
                      g_debounced, g_debounced ? "SAFE" : "UNSAFE");
    }
}

// ── OLED update (called every 1 s) ────────────────────────────────────────────
//
//  128 x 32 pixel layout:
//   y= 0 (16 px, size-2): "SAFE  " / "RAIN! "
//   y=16 ( 8 px, size-1): IP address
//   y=24 ( 8 px, size-1): Alpaca status + transition count

static void updateOled() {
    if (!g_oledOk) return;
    bool safe = (g_debounced == 1);

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.print(safe ? "SAFE  " : "RAIN! ");

    oled.setTextSize(1);
    oled.setCursor(0, 16);
    oled.print(WiFi.status() == WL_CONNECTED
               ? WiFi.localIP().toString()
               : "No WiFi");

    oled.setCursor(0, 24);
    oled.print(g_alpacaConn ? "Alpaca:OK" : "Alpaca:--");
    oled.print(" T:");
    oled.print(g_totalTrans);

    oled.display();
}

// ── setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    // Wait up to 3 s for USB CDC host to enumerate; continue anyway if no PC connected
    {
        uint32_t t = millis();
        while (!Serial && (millis() - t) < 3000) delay(10);
    }
    Serial.println("\n\n=== RG-11 Safety Monitor booting ===");

    // OLED – initialise Wire with our custom pins BEFORE calling oled.begin()
    // so the Adafruit BusIO layer doesn't reset them to the board defaults.
    Wire.begin(OLED_SDA, OLED_SCL);

    // I2C scan – auto-detect OLED address (0x3C or 0x3D)
    bool oledFound = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0 && (addr == 0x3C || addr == 0x3D) && !oledFound) {
            g_oledAddr = addr;
            oledFound  = true;
        }
    }

    g_oledOk = oled.begin(SSD1306_SWITCHCAPVCC, g_oledAddr);
    if (!g_oledOk) {
        Serial.printf("SSD1306 not found at 0x%02X – check wiring\n", g_oledAddr);
    } else {
        Serial.printf("SSD1306 OK at 0x%02X\n", g_oledAddr);
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(0, 0);
        oled.println("RG-11 Monitor");
        oled.println("WiFi connecting...");
        oled.display();
    }

    // Rain sensor GPIO
    pinMode(RAIN_PIN, INPUT_PULLUP);
    for (int i = 0; i < DEB_SAMPLES; i++) g_samples[i] = digitalRead(RAIN_PIN);

    // WiFi – 20-second timeout so a bad SSID/password doesn't hang forever
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    uint32_t wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < 20000) {
        delay(500);
        Serial.print('.');
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi FAILED – check SSID/password in secrets.h");
    } else {
        Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
    }
    if (g_oledOk) {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(0, 0);
        if (WiFi.status() == WL_CONNECTED) {
            oled.println("WiFi OK!");
            oled.println(WiFi.localIP().toString());
        } else {
            oled.println("WiFi FAILED");
            oled.println("Check secrets.h");
        }
        oled.display();
        delay(2000);
    }

    // Web status page – port 80 (human-readable, no port number needed in browser)
    webServer.on("/",            HTTP_GET, hRoot);
    webServer.on("/favicon.ico", HTTP_GET, hFavicon);
    webServer.onNotFound(hNotFound80);
    webServer.begin();
    Serial.println("Web status page on port 80");

    // Alpaca API – port 11111
    httpServer.on("/api/v1/safetymonitor/0/issafe",           HTTP_GET,  hIsSafe);
    httpServer.on("/api/v1/safetymonitor/0/connected",        HTTP_GET,  hConnectedGet);
    httpServer.on("/api/v1/safetymonitor/0/connected",        HTTP_PUT,  hConnectedPut);
    httpServer.on("/api/v1/safetymonitor/0/description",      HTTP_GET,  hDescription);
    httpServer.on("/api/v1/safetymonitor/0/driverinfo",       HTTP_GET,  hDriverInfo);
    httpServer.on("/api/v1/safetymonitor/0/driverversion",    HTTP_GET,  hDriverVersion);
    httpServer.on("/api/v1/safetymonitor/0/interfaceversion", HTTP_GET,  hInterfaceVersion);
    httpServer.on("/api/v1/safetymonitor/0/name",             HTTP_GET,  hName);
    httpServer.on("/api/v1/safetymonitor/0/supportedactions", HTTP_GET,  hSupportedActions);
    httpServer.on("/api/v1/safetymonitor/0/devicestate",      HTTP_GET,  hDeviceState);
    httpServer.on("/management/apiversions",                   HTTP_GET,  hMgmtApiVersions);
    httpServer.on("/management/v1/description",                HTTP_GET,  hMgmtDescription);
    httpServer.on("/management/v1/configureddevices",          HTTP_GET,  hMgmtDevices);
    httpServer.onNotFound(hNotFound);
    httpServer.begin();
    Serial.printf("Alpaca API on port %d\n", ALPACA_PORT);

    // Alpaca UDP discovery – AsyncUDP handles receive + reply on the same socket
    if (asyncUdp.listen(DISC_PORT)) {
        asyncUdp.onPacket([](AsyncUDPPacket packet) {
            if (packet.length() < 16) return;
            if (strncmp((char*)packet.data(), "alpacadiscovery1", 16) != 0) return;
            String resp =
                "{\"AlpacaPort\":" + String(ALPACA_PORT) +
                ",\"Devices\":[{\"DeviceType\":\"SafetyMonitor\","
                "\"DeviceName\":\"RG-11 Safety Monitor\","
                "\"DeviceNumber\":0,\"UniqueID\":\"ESP32C3-RG11-SM-0\"}]}";
            packet.print(resp);
        });
        Serial.printf("Alpaca discovery on UDP port %d\n", DISC_PORT);
    } else {
        Serial.println("AsyncUDP listen FAILED");
    }

    g_lastChange    = millis();
    g_nextDebounce  = millis() + DEB_INTERVAL_MS;
    g_nextOled      = millis() + 1000;

    Serial.println("=== Setup complete – entering loop ===");
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop() {
    uint32_t now = millis();

    webServer.handleClient();
    httpServer.handleClient();
    // Discovery is handled by AsyncUDP callback – nothing to poll here

    // WiFi watchdog – reconnect automatically if connection drops
    if (WiFi.status() != WL_CONNECTED && (int32_t)(now - g_nextWifiRetry) >= 0) {
        g_nextWifiRetry = now + 30000;   // retry at most every 30 s
        Serial.println("WiFi lost – reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

    if ((int32_t)(now - g_nextDebounce) >= 0) {
        g_nextDebounce = now + DEB_INTERVAL_MS;
        runDebounce();
    }

    if ((int32_t)(now - g_nextOled) >= 0) {
        g_nextOled = now + 1000;
        updateOled();
    }

    if ((int32_t)(now - g_nextHeartbeat) >= 0) {
        g_nextHeartbeat = now + 30000;
        Serial.printf("HB: OLED=%s(0x%02X) GPIO%d=%d safe=%s WiFi=%s\n",
                      g_oledOk ? "OK" : "FAIL", g_oledAddr,
                      RAIN_PIN, g_rawLevel,
                      g_debounced ? "Y" : "N",
                      WiFi.status() == WL_CONNECTED
                          ? WiFi.localIP().toString().c_str()
                          : "DOWN");
    }
}
