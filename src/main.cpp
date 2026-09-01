/*
 * Alpaca RG-11 Safety Monitor (Arduino framework)
 *
 * Board  : ESP32-WROOM-32 DevKit (30-pin) on a screw-terminal expansion board
 * Sensor : Hydreon RG-11 rain sensor relay  COM -> GPIO 27, NO -> GND
 *          plus a 4.7k pull-up from GPIO 27 to 3V3 (see README — the external
 *          pull-up is required, it supplies the relay's contact wetting current)
 * Display: GC9A01 1.28" round TFT, 240x240, seated directly on the first seven
 *          header pins: VCC GND SCL SDA DC CS RST -> 3V3 GND D15 D2 D4 D16 D17.
 *          Pin numbers live in platformio.ini as TFT_eSPI build flags.
 *
 * The device is unattended overnight, so it supervises itself: a task watchdog
 * catches hung handlers, a WiFi supervisor escalates from re-associate to
 * reboot, and a diagnostics record in RTC memory survives those reboots so the
 * cause is still visible afterwards at http://<ip>/.
 *
 * WiFi credentials live in include/secrets.h (git-ignored).
 * Copy include/secrets.h.example -> include/secrets.h and fill in your values.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <AsyncUDP.h>
#include <TFT_eSPI.h>
#include <esp_task_wdt.h>
#include <time.h>
#include "secrets.h"

// ── Pins ──────────────────────────────────────────────────────────────────────
// GPIO 27 has no strapping or boot function and supports the internal pull-up.
// Do NOT move this to 34/35/36/39 — those are input-only with no pull-up, so
// INPUT_PULLUP compiles and silently does nothing.
#define RAIN_PIN        27

// ── Ports ─────────────────────────────────────────────────────────────────────
#define INFO_PORT          80
#define ALPACA_PORT     11111
#define DISC_PORT       32227

// ── Debounce ──────────────────────────────────────────────────────────────────
#define DEB_SAMPLES        5   // majority-vote window
#define DEB_INTERVAL_MS  100   // sample every 100 ms -> 500 ms settling time

// ── Watchdogs ─────────────────────────────────────────────────────────────────
// 1. Hardware task watchdog — reboots if loop() stops running (hung handler,
//    deadlock, wedged network stack). Fed once per loop() iteration.
// 2. WiFi supervisor — polls the link, re-associates, and finally reboots if
//    the association can't be recovered.
// 3. Heap floor — the HTTP handlers build responses with String concatenation;
//    if that ever leaks, reboot before the allocator starts failing.
#define WDT_TIMEOUT_S             30
#define WIFI_CHECK_INTERVAL_MS    5000UL   // how often to poll link state
#define WIFI_RECONNECT_WAIT_MS   15000UL   // grace period per reconnect attempt
#define WIFI_MAX_RECONNECT_TRIES  4        // ~60 s down, then reboot
#define BOOT_WIFI_REBEGIN_TRIES   30       // 30 x 500 ms = 15 s, then re-begin
#define BOOT_WIFI_MAX_TRIES       60       // 60 x 500 ms = 30 s, then reboot
#define HEAP_FLOOR_BYTES      40000UL      // reboot below this much free heap

// 4. Poll-stall supervisor — catches the failure the WiFi supervisor can't see:
//    the association stays up (so no disconnect event ever fires) but traffic
//    to this one device blackholes for a minute, N.I.N.A's polls time out, and
//    it fails safe and shuts the observatory down. Observed 2026-08-31 17:41
//    and 2026-09-01 01:19 — zero events on the device either time, while the
//    roof controller answered fine seconds later.
//    While a client is armed (PUT connected=true or a GET issafe), silence on
//    the Alpaca API escalates: log it -> re-associate (rebuilds AP client
//    state) -> give up and disarm, so a N.I.N.A that was simply closed doesn't
//    cause reconnect flapping all night.
#define STALL_CHECK_MS         5000UL      // how often to evaluate
#define STALL_WARN_MS         35000UL      // quiet this long -> log EV_NET_STALL
#define STALL_REASSOC_MS      70000UL      // still quiet -> force re-associate
#define STALL_GIVEUP_MS      600000UL      // still quiet -> assume client gone

// ── Time (house convention: Melbourne, DST handled by configTzTime) ───────────
static const char *TZ_INFO      = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.nist.gov";
#define TIME_SYNCED_EPOCH  1704067200UL   // 2024-01-01; anything below = not synced

// ── Display geometry / palette ────────────────────────────────────────────────
#define SCR_W      240
#define SCR_H      240
#define CX         120
#define CY         120
#define R_OUTER    120
#define R_INNER     98   // leaves a 22 px coloured annulus

static constexpr uint16_t COL_BG   = TFT_BLACK;
static constexpr uint16_t COL_TEXT = 0xDEFB;  // near-white
static constexpr uint16_t COL_DIM  = 0x7BEF;  // grey
static constexpr uint16_t COL_GOOD = 0x2661;  // green
static constexpr uint16_t COL_BAD  = 0xE124;  // red

// ── Global objects ────────────────────────────────────────────────────────────
WebServer   webServer(INFO_PORT);     // human-readable status page
WebServer   httpServer(ALPACA_PORT);  // ASCOM Alpaca API
AsyncUDP    asyncUdp;

TFT_eSPI    tft;
TFT_eSprite spr(&tft);
static bool g_spriteOk = false;
// Drawing goes through this so the layout code is identical whether we render
// into the off-screen sprite or straight to the panel.
static TFT_eSPI *gfx = &tft;

// ── Sensor / Alpaca state (single loop – no concurrent tasks – no mutex) ──────
static int      g_rawLevel   = 1;
static int      g_debounced  = 1;
static uint32_t g_lastChange = 0;
static uint32_t g_totalTrans = 0;
static bool     g_alpacaConn = false;
static uint32_t g_serverTxId = 1;
static String   g_uniqueId;          // MAC-derived, built in setup()

// Debounce ring buffer
static int  g_samples[DEB_SAMPLES];
static int  g_sampleIdx = 0;

// Loop timers
static uint32_t g_nextDebounce  = 0;
static uint32_t g_nextDisplay   = 0;
static uint32_t g_nextHeartbeat = 0;

// ── Diagnostics ───────────────────────────────────────────────────────────────
// Lives in RTC memory, which survives a software reset and a watchdog reset but
// not a power cycle. That's exactly what we want: if the device reboots itself
// overnight, the reason is still on the status page in the morning.
//
// Bump the magic whenever the layout below changes, otherwise a reflash can find
// stale RTC contents that still match the old magic and decode as garbage.
#define DIAG_MAGIC   0x52473131UL   // "RG11" — rev 1
#define DIAG_EVENTS  12

enum DiagEvent : uint8_t {
    EV_NONE = 0, EV_BOOT, EV_WIFI_LOST, EV_WIFI_OK, EV_REBOOT, EV_RAIN, EV_DRY,
    EV_NET_STALL,   // client polls stopped while the link claimed to be up
    EV_NET_OK       // polls resumed; detail = outage length in 10 s units
};

struct DiagEntry {
    uint32_t epoch;      // 0 if the clock wasn't synced yet
    uint32_t uptimeSec;
    uint8_t  type;
    uint8_t  detail;     // EV_WIFI_LOST: the 802.11 disconnect reason code
};

struct DiagRecord {
    uint32_t  magic;
    uint32_t  bootCount;
    uint32_t  wifiDrops;   // cumulative across reboots, unlike s_wifiDropCount
    uint8_t   head;
    DiagEntry ev[DIAG_EVENTS];
};

RTC_NOINIT_ATTR DiagRecord s_diag;

static const char *diagEventName(uint8_t t)
{
    switch (t) {
        case EV_BOOT:      return "boot";
        case EV_WIFI_LOST: return "WiFi lost";
        case EV_WIFI_OK:   return "WiFi recovered";
        case EV_REBOOT:    return "self-reboot";
        case EV_RAIN:      return "RAIN — unsafe";
        case EV_DRY:       return "dry — safe";
        case EV_NET_STALL: return "polls stopped (link up)";
        case EV_NET_OK:    return "polls resumed";
        default:           return "";
    }
}

static const char *resetReasonName()
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_SW:        return "software restart";
        case ESP_RST_PANIC:     return "panic / exception";
        case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";
        case ESP_RST_INT_WDT:   return "interrupt watchdog";
        case ESP_RST_WDT:       return "other watchdog";
        case ESP_RST_BROWNOUT:  return "BROWNOUT (power dip)";
        case ESP_RST_EXT:       return "external reset pin";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        default:                return "unknown";
    }
}

static void diagLog(DiagEvent type, uint8_t detail = 0)
{
    DiagEntry &e = s_diag.ev[s_diag.head];
    time_t now   = time(nullptr);
    e.epoch      = (now > (time_t)TIME_SYNCED_EPOCH) ? (uint32_t)now : 0;
    e.uptimeSec  = millis() / 1000UL;
    e.type       = (uint8_t)type;
    e.detail     = detail;
    s_diag.head  = (s_diag.head + 1) % DIAG_EVENTS;
}

// The common 802.11 reason codes, so the log reads without a lookup table.
// 15 in particular means the AP's group-key rotation timed out — a classic
// cause of a single unexplained nightly drop.
static const char *wifiReasonName(uint8_t r)
{
    switch (r) {
        case 1:   return "unspecified";
        case 2:   return "auth expired";
        case 4:   return "assoc expired (AP idle timeout)";
        case 8:   return "AP deauthenticated us";
        case 15:  return "4-way handshake timeout (group rekey)";
        case 200: return "beacon timeout (AP unreachable)";
        case 201: return "no AP found";
        case 202: return "auth failed";
        case 203: return "assoc failed";
        default:  return "";
    }
}

// Local wall-clock string, or an uptime fallback when NTP hadn't synced yet.
static String diagWhen(const DiagEntry &e)
{
    if (e.epoch == 0) return "+" + String(e.uptimeSec) + "s (no clock)";
    time_t t = (time_t)e.epoch;
    struct tm tmv;
    localtime_r(&t, &tmv);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
    return String(buf);
}

// Diagnostics card for the status page — newest event first.
static String diagCardHtml()
{
    String h = "<div class='card'><h2>Diagnostics</h2>"
               "<p><strong>Last reset:</strong> " + String(resetReasonName()) + "</p>"
               "<p><strong>Boots since power-on:</strong> " + String(s_diag.bootCount) + "</p>"
               "<p><strong>WiFi drops since power-on:</strong> " + String(s_diag.wifiDrops) + "</p>";

    time_t now = time(nullptr);
    h += "<p><strong>Clock:</strong> ";
    if (now > (time_t)TIME_SYNCED_EPOCH) {
        struct tm tmv;
        localtime_r(&now, &tmv);
        char buf[24];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
        h += buf;
    } else {
        h += "not synced";
    }
    h += "</p>";

    h += "<table><tr><th align='left'>When</th><th align='left'>Event</th></tr>";
    bool any = false;
    for (int i = 0; i < DIAG_EVENTS; i++) {
        // walk backwards from the most recently written slot
        int idx = (s_diag.head - 1 - i + 2 * DIAG_EVENTS) % DIAG_EVENTS;
        const DiagEntry &e = s_diag.ev[idx];
        if (e.type == EV_NONE) continue;
        any = true;
        String what = diagEventName(e.type);
        if (e.type == EV_WIFI_LOST) {
            what += " — reason " + String(e.detail);
            const char *rn = wifiReasonName(e.detail);
            if (rn[0]) what += " (" + String(rn) + ")";
        } else if (e.type == EV_NET_OK && e.detail) {
            what += " — after ~" + String((uint32_t)e.detail * 10) + " s";
        }
        h += "<tr><td><code>" + diagWhen(e) + "</code></td><td>" + what + "</td></tr>";
    }
    if (!any) h += "<tr><td colspan='2'>no events recorded</td></tr>";
    h += "</table></div>";
    return h;
}

// ── Poll-stall supervisor state ───────────────────────────────────────────────
static bool     g_clientArmed   = false;  // a client is (or was) actively polling
static uint32_t g_lastAlpacaMs  = 0;      // last request on the Alpaca API
static uint8_t  g_stallStage    = 0;      // 0 quiet-ok, 1 warned, 2 re-associated
static uint32_t g_stallCount    = 0;      // episodes since boot (status page)
static uint32_t g_lastStallCheckMs = 0;

// Every Alpaca response funnels through here. `arms` is true only for the
// requests that prove a client session is live (issafe polls, connect PUT) —
// a passing discovery scan of the management API must not arm the detector.
static void noteAlpaca(bool arms)
{
    if (g_stallStage != 0) {
        uint32_t gap = millis() - g_lastAlpacaMs;
        diagLog(EV_NET_OK, (uint8_t)min(gap / 10000UL, 255UL));
        Serial.printf("Polls resumed after %lus (stage %u)\n",
                      (unsigned long)(gap / 1000UL), g_stallStage);
        g_stallStage = 0;
    }
    g_lastAlpacaMs = millis();
    if (arms) g_clientArmed = true;
}

// ── WiFi supervisor state ─────────────────────────────────────────────────────
static uint32_t s_lastWifiCheckMs = 0;
static uint32_t s_wifiDownSinceMs = 0;   // 0 = link is up
static uint8_t  s_wifiRetries     = 0;
static uint32_t s_wifiDropCount   = 0;   // reported on the status page

// Set from the WiFi event task, consumed in loop(). The handler must stay this
// trivial: WiFi.begin()/disconnect() must not be called from the event context,
// so all it does is raise a flag that forces the next poll to run immediately.
static volatile bool    s_wifiEvtDisconnected  = false;
static volatile bool    s_wifiEvtGotIp         = false;
static volatile uint8_t s_wifiDisconnectReason = 0;

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            s_wifiDisconnectReason = info.wifi_sta_disconnected.reason;
            s_wifiEvtDisconnected  = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            s_wifiEvtGotIp = true;
            break;
        default:
            break;
    }
}

// ── Display ───────────────────────────────────────────────────────────────────
//
//  displayInit()    – bring up the panel, returns true on success
//  displayMessage() – two-line boot-progress message
//  displayUpdate()  – periodic status screen (called every 1 s)
//
//  240 x 240 round layout:
//    a 22 px coloured annulus at the rim — green SAFE / red NOT SAFE —
//    around a black centre carrying the text.  Everything is drawn into an
//    8-bit off-screen sprite (57.6 kB) and pushed in one go, so there is no
//    flicker even though the GC9A01 itself has no frame buffer.

static bool displayInit()
{
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(COL_BG);

    spr.setColorDepth(8);
    g_spriteOk = (spr.createSprite(SCR_W, SCR_H) != nullptr);
    gfx = g_spriteOk ? (TFT_eSPI *)&spr : &tft;

    Serial.printf("GC9A01 initialised (240x240), sprite %s\n",
                  g_spriteOk ? "OK" : "FAILED - drawing direct");
    return true;
}

static void displayPush()
{
    if (g_spriteOk) spr.pushSprite(0, 0);
}

static void displayMessage(const char *l1, const String &l2)
{
    gfx->fillScreen(COL_BG);
    gfx->setTextDatum(MC_DATUM);
    gfx->setTextColor(COL_TEXT, COL_BG);
    gfx->setTextFont(4);
    gfx->drawString(l1, CX, CY - 16);
    gfx->setTextFont(2);
    gfx->drawString(l2, CX, CY + 16);
    displayPush();
}

static void displayUpdate()
{
    const bool     safe = (g_debounced == 1);
    const uint16_t ring = safe ? COL_GOOD : COL_BAD;
    const bool     up   = (WiFi.status() == WL_CONNECTED);

    gfx->fillScreen(COL_BG);
    gfx->fillCircle(CX, CY, R_OUTER, ring);
    gfx->fillCircle(CX, CY, R_INNER, COL_BG);

    gfx->setTextDatum(MC_DATUM);

    // Headline — FreeSans is the largest built-in face with a full alphabet
    // (fonts 6/7/8 are digits-only, so they can't render "SAFE").
    gfx->setTextColor(safe ? COL_GOOD : COL_BAD, COL_BG);
    gfx->setFreeFont(&FreeSansBold18pt7b);
    gfx->drawString(safe ? "SAFE" : "NOT SAFE", CX, 95);
    gfx->setTextFont(2);   // back to the built-in fonts

    gfx->setTextColor(COL_TEXT, COL_BG);
    gfx->drawString(up ? WiFi.localIP().toString() : String("No WiFi"), CX, 142);

    // "Alpaca:OK" now means a client polled within the last 30 s, so the screen
    // itself shows when N.I.N.A's polls stop reaching us — the failure mode
    // that used to be invisible here.
    const bool polled = g_alpacaConn && (millis() - g_lastAlpacaMs < 30000UL);
    gfx->setTextColor(COL_DIM, COL_BG);
    gfx->drawString(String(polled ? "Alpaca:OK" : "Alpaca:--") +
                    "  T:" + String(g_totalTrans), CX, 168);

    gfx->setTextFont(1);
    char foot[32];
    snprintf(foot, sizeof(foot), "%ddBm  up %lum",
             up ? WiFi.RSSI() : 0, (unsigned long)(millis() / 60000UL));
    gfx->drawString(foot, CX, 192);

    displayPush();
}

// ── Restart helper ────────────────────────────────────────────────────────────
// A reboot is only ever triggered when the device is already unreachable (WiFi
// down) or already broken (heap exhausted), so unlike the roof controller there
// is nothing to defer to — no client can observe the gap either way.
static void safeRestart(const char *reason)
{
    Serial.printf("Restarting: %s\n", reason);
    diagLog(EV_REBOOT);
    displayMessage("Restarting", reason);
    Serial.flush();
    delay(1000);
    ESP.restart();
}

// ── Alpaca UDP discovery ──────────────────────────────────────────────────────
// AsyncUDP handles receive + reply on the same socket. WiFiUDP cannot: sending
// on the socket it is listening on returns ENOMEM. Re-armed after a WiFi
// recovery because the bound socket does not reliably survive re-association.
static void startDiscovery()
{
    asyncUdp.close();
    if (!asyncUdp.listen(DISC_PORT)) {
        Serial.println("AsyncUDP listen FAILED");
        return;
    }
    asyncUdp.onPacket([](AsyncUDPPacket packet) {
        if (packet.length() < 16) return;
        if (strncmp((char *)packet.data(), "alpacadiscovery1", 16) != 0) return;
        // The discovery reply MUST contain only AlpacaPort. Adding a "Devices"
        // array here made N.I.N.A silently drop the device — clients fetch the
        // device list from /management/v1/configureddevices over HTTP once they
        // know the port.
        packet.print("{\"AlpacaPort\":" + String(ALPACA_PORT) + "}");
    });
    Serial.printf("Alpaca discovery on UDP port %d\n", DISC_PORT);
}

// ── WiFi supervisor ───────────────────────────────────────────────────────────
// Polled from loop(). Escalates: notice the drop -> re-associate -> reboot.
static void maintainWifi()
{
    // The driver knows about a drop long before a 5 s poll would notice. Rather
    // than duplicate the recovery logic, just force the poll below to run now —
    // shrinking a ~10 s outage to ~2 s, which may be short enough that clients
    // like N.I.N.A never see a failed request.
    if (s_wifiEvtDisconnected || s_wifiEvtGotIp) {
        s_wifiEvtDisconnected = false;
        s_wifiEvtGotIp        = false;
        s_lastWifiCheckMs     = 0;   // millis() - 0 always exceeds the interval
    }

    if (millis() - s_lastWifiCheckMs < WIFI_CHECK_INTERVAL_MS) return;
    s_lastWifiCheckMs = millis();

    if (WiFi.status() == WL_CONNECTED) {
        if (s_wifiDownSinceMs != 0) {
            Serial.print("WiFi recovered, IP: ");
            Serial.println(WiFi.localIP());
            diagLog(EV_WIFI_OK);
            startDiscovery();   // rebind the discovery socket
        }
        s_wifiDownSinceMs = 0;
        s_wifiRetries     = 0;
        return;
    }

    // First time we've seen the link down — kick off a reconnect immediately.
    if (s_wifiDownSinceMs == 0) {
        s_wifiDownSinceMs = millis();
        s_wifiRetries     = 0;
        s_wifiDropCount++;
        s_diag.wifiDrops++;
        uint8_t reason = s_wifiDisconnectReason;
        Serial.printf("WiFi link lost (reason %u %s) — reconnecting\n",
                      reason, wifiReasonName(reason));
        diagLog(EV_WIFI_LOST, reason);
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        return;
    }

    // Give the current attempt its full grace period before doing anything else.
    if (millis() - s_wifiDownSinceMs < WIFI_RECONNECT_WAIT_MS) return;

    if (s_wifiRetries >= WIFI_MAX_RECONNECT_TRIES) {
        safeRestart("WiFi down");
        return;
    }

    s_wifiRetries++;
    Serial.printf("WiFi reconnect attempt %u/%u\n", s_wifiRetries, WIFI_MAX_RECONNECT_TRIES);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    s_wifiDownSinceMs = millis();
}

// ── Poll-stall supervisor ─────────────────────────────────────────────────────
// Polled from loop(). Only acts while the link *claims* to be up — when it is
// actually down the WiFi supervisor owns recovery.
static void maintainStall()
{
    if (millis() - g_lastStallCheckMs < STALL_CHECK_MS) return;
    g_lastStallCheckMs = millis();

    if (!g_clientArmed || WiFi.status() != WL_CONNECTED) return;

    uint32_t quiet = millis() - g_lastAlpacaMs;

    if (g_stallStage == 0 && quiet > STALL_WARN_MS) {
        g_stallStage = 1;
        g_stallCount++;
        diagLog(EV_NET_STALL);
        Serial.printf("Client polls stopped %lus ago but WiFi is up\n",
                      (unsigned long)(quiet / 1000UL));
    } else if (g_stallStage == 1 && quiet > STALL_REASSOC_MS) {
        // Re-associate: rebuilds the AP's client state and renegotiates rates,
        // which is the strongest repair available short of a reboot. The WiFi
        // supervisor sees the drop and drives the reconnect + discovery re-arm.
        g_stallStage = 2;
        Serial.println("Still no polls — forcing re-association");
        WiFi.disconnect();
    } else if (g_stallStage == 2 && quiet > STALL_GIVEUP_MS) {
        // Client is genuinely gone (N.I.N.A closed or the PC is off). Disarm so
        // an idle device doesn't re-associate all night.
        g_stallStage  = 0;
        g_clientArmed = false;
        Serial.println("No polls for 10 min — client presumed gone, disarming");
    }
}

// ── Alpaca response helpers ───────────────────────────────────────────────────

static void addCors()
{
    httpServer.sendHeader("Access-Control-Allow-Origin",  "*");
    httpServer.sendHeader("Access-Control-Allow-Methods", "GET, PUT, OPTIONS");
    httpServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static uint32_t clientTx()
{
    String s = httpServer.arg("ClientTransactionID");
    return s.length() ? (uint32_t)s.toInt() : 0;
}

static void sendBool(bool v)
{
    g_alpacaConn = true;
    noteAlpaca(false);
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":" +
        (v ? "true" : "false") + "}");
}

static void sendInt(int v)
{
    g_alpacaConn = true;
    noteAlpaca(false);
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":" + String(v) + "}");
}

static void sendStr(const char *v)
{
    g_alpacaConn = true;
    noteAlpaca(false);
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":\"" + String(v) + "\"}");
}

static void sendArr(const String &v)
{
    noteAlpaca(false);
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\",\"Value\":" + v + "}");
}

// ── Status page (port 80) ─────────────────────────────────────────────────────

// Live data for the status page. Served on port 80 so browsers never touch the
// Alpaca port — N.I.N.A must be the only client on 11111. (The page previously
// fetched issafe cross-port at 2 Hz, competing with N.I.N.A for the
// one-client-at-a-time WebServer.)
static void hStatusJson()
{
    String j = "{\"safe\":" + String(g_debounced == 1 ? "true" : "false") +
        ",\"gpio\":" + String(g_rawLevel) +
        ",\"lastPollSec\":" +
        (g_alpacaConn ? String((millis() - g_lastAlpacaMs) / 1000UL) : String(-1)) +
        ",\"clientArmed\":" + (g_clientArmed ? "true" : "false") +
        ",\"stalls\":" + String(g_stallCount) +
        ",\"rssi\":" + String(WiFi.RSSI()) +
        ",\"heap\":" + String(ESP.getFreeHeap()) + "}";
    webServer.send(200, "application/json", j);
}

static void hRoot()
{
    String html =
        "<html><head><meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='10'>"
        "<style>body{font-family:Arial,sans-serif;margin:24px;background:#f5f5f5}"
        "h1{color:#333}h2{margin:0 0 8px;font-size:1.1em;color:#555}"
        ".safe{color:green;font-weight:bold}.unsafe{color:red;font-weight:bold}"
        ".card{background:#fff;padding:16px;margin:12px 0;border-radius:8px;"
        "box-shadow:0 2px 6px rgba(0,0,0,.1)}"
        "table{border-collapse:collapse;width:100%}"
        "th,td{padding:4px 8px;border-bottom:1px solid #eee;font-size:.9em}"
        "</style></head>"
        "<body><h1>RG-11 Safety Monitor</h1>"

        "<div class='card'><h2>Status</h2>"
        "<p><strong>Status:</strong> <span id='s'>--</span></p>"
        "<p><strong>GPIO " + String(RAIN_PIN) + ":</strong> <span id='r'>--</span></p>"
        "<p><strong>Transitions:</strong> " + String(g_totalTrans) +
        " | <strong>Last Alpaca poll:</strong> <span id='p'>--</span>"
        " | <strong>Client:</strong> " + (g_clientArmed ? "armed" : "idle") +
        " | <strong>IP:</strong> " + WiFi.localIP().toString() + "</p></div>"

        "<div class='card'><h2>Link</h2>"
        "<p><strong>Uptime:</strong> " + String(millis() / 60000UL) + " min</p>"
        "<p><strong>WiFi RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>"
        "<p><strong>WiFi drops since boot:</strong> " + String(s_wifiDropCount) + "</p>"
        "<p><strong>Poll stalls since boot:</strong> " + String(g_stallCount) + "</p>"
        "<p><strong>Free heap:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>"
        "</div>"

        + diagCardHtml() +

        "<script>function u(){"
        "fetch('/status.json').then(r=>r.json()).then(d=>{"
        "document.getElementById('s').innerHTML=d.safe?"
        "'<span class=\"safe\">SAFE</span>':'<span class=\"unsafe\">NOT SAFE (RAIN)</span>';"
        "document.getElementById('r').innerHTML=d.safe?"
        "'<span style=\"color:green\">HIGH (1)</span>'"
        ":'<span style=\"color:red\">LOW (0)</span>';"
        "document.getElementById('p').textContent="
        "d.lastPollSec<0?'never':d.lastPollSec+' s ago';"
        "}).catch(()=>{document.getElementById('s').innerHTML='Error';});}"
        "setInterval(u,2000);u();</script></body></html>";
    webServer.send(200, "text/html", html);
}

// ── Alpaca handlers ───────────────────────────────────────────────────────────

static void hFavicon()          { webServer.send(404); }
static void hIsSafe()           { noteAlpaca(true); sendBool(g_debounced == 1); }
static void hConnectedGet()     { noteAlpaca(true); sendBool(true); }

static void hConnectedPut()
{
    // Connected=False is the client saying goodbye — disarm the poll-stall
    // supervisor so the ensuing silence isn't treated as a network failure.
    String v = httpServer.arg("Connected");
    bool connecting = !v.equalsIgnoreCase("false");
    noteAlpaca(connecting);
    if (!connecting) {
        g_clientArmed = false;
        g_stallStage  = 0;
        Serial.println("Client disconnected (PUT Connected=False)");
    }
    g_alpacaConn = true;
    sendBool(true);
}
static void hDescription()      { sendStr("Hydreon RG-11 Optical Rain Sensor Safety Monitor"); }
static void hDriverInfo()       { sendStr("ESP32 Alpaca Safety Monitor (Arduino)"); }
static void hDriverVersion()    { sendStr("3.0.0"); }
// ISafetyMonitorV2 – the classic Connected property, which is what this
// firmware implements. Claiming 3 (Platform 7) would promise the async
// Connect/Disconnect/Connecting methods that are not served here.
static void hInterfaceVersion() { sendInt(2); }
static void hName()             { sendStr("Alpaca RG-11 Safety Monitor"); }
static void hSupportedActions() { sendArr("[]"); }
static void hDeviceState()      { sendArr("[]"); }

static void hMgmtApiVersions()  { sendArr("[1]"); }

static void hMgmtDescription()
{
    uint32_t tx = clientTx(), stx = g_serverTxId++;
    addCors();
    httpServer.send(200, "application/json",
        "{\"ClientTransactionID\":" + String(tx) +
        ",\"ServerTransactionID\":" + String(stx) +
        ",\"ErrorNumber\":0,\"ErrorMessage\":\"\","
        "\"Value\":{\"ServerName\":\"RG-11 Safety Monitor\",\"Manufacturer\":\"DIY\","
        "\"ManufacturerVersion\":\"3.0\",\"Location\":\"Observatory\"}}");
}

static void hMgmtDevices()
{
    sendArr("[{\"DeviceType\":\"SafetyMonitor\","
            "\"DeviceName\":\"RG-11 Safety Monitor\","
            "\"DeviceNumber\":0,\"UniqueID\":\"" + g_uniqueId + "\"}]");
}

static void hNotFound()
{
    if (httpServer.method() == HTTP_OPTIONS) {
        addCors();
        httpServer.send(200);
        return;
    }
    httpServer.send(404, "text/plain", "Not found");
}

static void hNotFound80() { webServer.send(404, "text/plain", "Not found"); }

// ── Debounce (called every DEB_INTERVAL_MS) ───────────────────────────────────

static void runDebounce()
{
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
        diagLog(g_debounced ? EV_DRY : EV_RAIN);
        Serial.printf("State change → debounced=%d (%s)\n",
                      g_debounced, g_debounced ? "SAFE" : "UNSAFE");
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    Serial.println("\n\n=== RG-11 Safety Monitor booting ===");

    // Diagnostics record — RTC memory is garbage after a power cycle, so gate on
    // both the magic and the reset reason before trusting it.
    esp_reset_reason_t rr = esp_reset_reason();
    if (s_diag.magic != DIAG_MAGIC || rr == ESP_RST_POWERON) {
        memset(&s_diag, 0, sizeof(s_diag));
        s_diag.magic = DIAG_MAGIC;
    }
    s_diag.bootCount++;
    Serial.printf("Reset reason: %s (boot #%lu)\n", resetReasonName(),
                  (unsigned long)s_diag.bootCount);

    displayInit();
    displayMessage("RG-11 Monitor", "WiFi connecting...");

    // Rain sensor GPIO. Seed the debounce state from the real pin level so the
    // device never reports SAFE during the first debounce window while it is
    // actually raining.
    pinMode(RAIN_PIN, INPUT_PULLUP);
    int initial = digitalRead(RAIN_PIN);
    for (int i = 0; i < DEB_SAMPLES; i++) g_samples[i] = initial;
    g_rawLevel  = initial;
    g_debounced = initial;

    // WiFi — re-begin at 15 s, restart at 30 s
    WiFi.persistent(false);        // don't wear out flash rewriting the same creds
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // modem sleep makes the link flaky under polling
    WiFi.setAutoReconnect(true);   // first line of defence; maintainWifi() backs it up
    WiFi.setHostname("rg11");
    WiFi.onEvent(onWiFiEvent);     // must be registered before begin()
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    {
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print('.');
            ++attempts;

            // Half way — tear the association down and start over. Cheaper than
            // a reboot and clears a stuck WPA handshake.
            if (attempts == BOOT_WIFI_REBEGIN_TRIES) {
                Serial.print(" retrying");
                WiFi.disconnect(true);
                delay(200);
                WiFi.mode(WIFI_STA);
                WiFi.begin(WIFI_SSID, WIFI_PASS);
            }

            if (attempts >= BOOT_WIFI_MAX_TRIES) {
                Serial.println("\nWiFi failed — restarting in 3 s");
                displayMessage("WiFi FAILED", "Restarting...");
                delay(3000);
                ESP.restart();
            }
        }
    }
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
    displayMessage("WiFi OK!", WiFi.localIP().toString());

    // Stable per-device Alpaca identity. A hardcoded UniqueID would collide with
    // the unit this one replaces if both are powered on during changeover.
    g_uniqueId = "RG11-SM-" + WiFi.macAddress();
    g_uniqueId.replace(":", "");
    Serial.printf("Alpaca UniqueID: %s\n", g_uniqueId.c_str());

    // NTP — non-blocking, syncs in the background. Only needed so the event log
    // carries wall-clock times that line up with N.I.N.A's log.
    configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);
    diagLog(EV_BOOT);

    delay(1500);   // let the boot message be read

    // Web status page – port 80 (human-readable, no port number needed)
    webServer.on("/",            HTTP_GET, hRoot);
    webServer.on("/status.json", HTTP_GET, hStatusJson);
    webServer.on("/favicon.ico", HTTP_GET, hFavicon);
    webServer.onNotFound(hNotFound80);
    webServer.begin();
    Serial.printf("Web status page on port %d\n", INFO_PORT);

    // Alpaca API – port 11111
    httpServer.on("/api/v1/safetymonitor/0/issafe",           HTTP_GET, hIsSafe);
    httpServer.on("/api/v1/safetymonitor/0/connected",        HTTP_GET, hConnectedGet);
    httpServer.on("/api/v1/safetymonitor/0/connected",        HTTP_PUT, hConnectedPut);
    httpServer.on("/api/v1/safetymonitor/0/description",      HTTP_GET, hDescription);
    httpServer.on("/api/v1/safetymonitor/0/driverinfo",       HTTP_GET, hDriverInfo);
    httpServer.on("/api/v1/safetymonitor/0/driverversion",    HTTP_GET, hDriverVersion);
    httpServer.on("/api/v1/safetymonitor/0/interfaceversion", HTTP_GET, hInterfaceVersion);
    httpServer.on("/api/v1/safetymonitor/0/name",             HTTP_GET, hName);
    httpServer.on("/api/v1/safetymonitor/0/supportedactions", HTTP_GET, hSupportedActions);
    httpServer.on("/api/v1/safetymonitor/0/devicestate",      HTTP_GET, hDeviceState);
    httpServer.on("/management/apiversions",                  HTTP_GET, hMgmtApiVersions);
    httpServer.on("/management/v1/description",               HTTP_GET, hMgmtDescription);
    httpServer.on("/management/v1/configureddevices",         HTTP_GET, hMgmtDevices);
    httpServer.onNotFound(hNotFound);
    httpServer.begin();
    Serial.printf("Alpaca API on port %d\n", ALPACA_PORT);

    startDiscovery();

    g_lastChange   = millis();
    g_nextDebounce = millis() + DEB_INTERVAL_MS;
    g_nextDisplay  = millis() + 1000;

    // Hardware task watchdog — armed last so a slow setup() can't trip it.
    // If loop() stops feeding it for WDT_TIMEOUT_S the chip resets.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wdtCfg = {
        .timeout_ms     = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    // Core 3.x may already have initialised the TWDT — reconfigure in that case.
    if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_reconfigure(&wdtCfg);
    }
#else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
    esp_task_wdt_add(NULL);   // watch the Arduino loop task
    Serial.printf("Task watchdog armed (%d s)\n", WDT_TIMEOUT_S);

    Serial.println("=== Setup complete – entering loop ===");
    displayUpdate();
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop()
{
    esp_task_wdt_reset();   // feed the hardware watchdog

    uint32_t now = millis();

    webServer.handleClient();
    httpServer.handleClient();
    // Discovery is handled by the AsyncUDP callback – nothing to poll here

    maintainWifi();
    maintainStall();

    if ((int32_t)(now - g_nextDebounce) >= 0) {
        g_nextDebounce = now + DEB_INTERVAL_MS;
        runDebounce();
    }

    if ((int32_t)(now - g_nextDisplay) >= 0) {
        g_nextDisplay = now + 1000;
        displayUpdate();
    }

    if ((int32_t)(now - g_nextHeartbeat) >= 0) {
        g_nextHeartbeat = now + 30000;
        Serial.printf("HB: GPIO%d=%d safe=%s WiFi=%s RSSI=%d heap=%u drops=%lu\n",
                      RAIN_PIN, g_rawLevel,
                      g_debounced ? "Y" : "N",
                      WiFi.status() == WL_CONNECTED
                          ? WiFi.localIP().toString().c_str()
                          : "DOWN",
                      WiFi.RSSI(), (unsigned)ESP.getFreeHeap(),
                      (unsigned long)s_wifiDropCount);

        // Heap floor — the String-building handlers are the only plausible leak
        // source; reboot before the allocator starts failing requests.
        if (ESP.getFreeHeap() < HEAP_FLOOR_BYTES) {
            safeRestart("low heap");
        }
    }
}
