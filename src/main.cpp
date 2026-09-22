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
#define WIFI_REBOOT_AFTER_MS     90000UL   // link down this long (boot included) -> reboot
#define HEAP_FLOOR_BYTES      40000UL      // reboot below this much free heap

// ── WiFi AP selection & roaming ───────────────────────────────────────────────
// Several UniFi APs broadcast the same SSID. The ESP32 default (fast scan) joins
// the first match in channel order and never roams, which parked this device on
// the Study AP at -74..-81 dBm with the observatory U6+ at -49 dBm. Every
// connect now scans all channels and tries BSSIDs strongest-first; a sustained
// weak link triggers a cautious roam. Any drop makes N.I.N.A close the roof, so
// roaming errs heavily on the side of staying put.
//
// Each value can be overridden with a -D build flag (see [env:roamtest]).
#ifndef WIFI_MIN_RSSI
#define WIFI_MIN_RSSI            -75       // dBm; weaker APs are tried only after every stronger one refused us
#endif
#ifndef ROAM_TRIGGER_RSSI
#define ROAM_TRIGGER_RSSI        -72       // dBm; consider roaming only below this...
#endif
#ifndef ROAM_SUSTAIN_MS
#define ROAM_SUSTAIN_MS       300000UL     // ...on every sample for this long (5 min)
#endif
#ifndef ROAM_HYSTERESIS_DB
#define ROAM_HYSTERESIS_DB        10       // target must beat the current AP by this much
#endif
#ifndef ROAM_COOLDOWN_MS
#define ROAM_COOLDOWN_MS      900000UL     // no roam attempt within 15 min of the last one
#endif
#ifndef ROAM_CHECK_INTERVAL_MS
#define ROAM_CHECK_INTERVAL_MS 60000UL     // RSSI sample period
#endif

// Mechanics — not tuning knobs.
#define WIFI_SCAN_MS_PER_CHAN    300       // active dwell; measured 1.7-3.2 s for 13 channels (core times out at 20x = 6 s)
#define WIFI_SCAN_SETTLE_MS      500UL     // gap between WiFi.disconnect() and starting a scan
#define WIFI_SCAN_GIVEUP_MS    12000UL     // wait this long for a late SCAN_DONE past the core's timeout
#define WIFI_MAX_CANDIDATES        3       // BSSIDs tried individually before a plain connect
#define WIFI_BSSID_WAIT_MS      8000UL     // per-BSSID connect timeout
#define WIFI_ANY_WAIT_MS       15000UL     // plain connect timeout (driver picks the AP)
#define ROAM_POLL_ALIGN_MS     10000UL     // max wait for a client poll to slot a roam step behind
#ifndef ROAM_LEAVE_SETTLE_MS
#define ROAM_LEAVE_SETTLE_MS     300UL     // after the old AP's disconnect event, before begin() on the new one
#endif
#ifndef ROAM_LEAVE_MAX_MS
#define ROAM_LEAVE_MAX_MS       1500UL     // begin() anyway if no disconnect event arrives by then
#endif
#define ROAM_REJECT_HOLDOFF_MS 3600000UL   // don't roam back to a BSSID that refused us for 1 h
#define ROAM_SKIP_RELOG_MS     3600000UL   // repeat an identical "roam skipped" log at most hourly

// 4. Poll-stall supervisor — catches the failure the WiFi supervisor can't see:
//    the association stays up (so no disconnect event ever fires) but traffic
//    to this one device blackholes for a minute, N.I.N.A's polls time out, and
//    it fails safe and shuts the observatory down. Observed 2026-08-31 17:41
//    and 2026-09-01 01:19 — zero events on the device either time, while the
//    roof controller answered fine seconds later.
//    While a client is armed (PUT connected=true or a GET issafe), silence on
//    the Alpaca API escalates: log it -> re-associate (rebuilds AP client
//    state) -> give up and disarm, so a N.I.N.A that was simply closed doesn't
//    cause reconnect flapping all night. Giving up is logged too, and so is the
//    full length of the gap when polls come back. The armed flag survives a
//    self-reboot (it lives in the RTC diagnostics record), because a reboot in
//    the middle of an outage used to erase all trace of it.
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
#define DIAG_MAGIC   0x52473132UL   // "RG12" — rev 2: AP fields, roam/client events
#define DIAG_EVENTS  24

enum DiagEvent : uint8_t {
    EV_NONE = 0, EV_BOOT, EV_WIFI_LOST, EV_WIFI_OK, EV_REBOOT, EV_RAIN, EV_DRY,
    EV_NET_STALL,    // client polls stopped while the link claimed to be up
    EV_NET_OK,       // polls resumed; value = silence in seconds
    EV_CONNECT_FAIL, // detail = reason (0 = timeout); ap = BSSID tried (zero = plain connect)
    EV_ROAM_SKIP,    // detail = RoamSkip; ap = best other AP, from = current AP
    EV_ROAM,         // from -> ap; value = seconds the link had been weak
    EV_CLIENT_GONE,  // stall supervisor gave up; value = silence in seconds
    EV_CLIENT_BYE    // PUT Connected=false
};

// Why a sustained weak link did not lead to a roam.
enum RoamSkip : uint8_t {
    RS_NONE = 0, RS_SCAN_FAILED, RS_NO_OTHER_AP, RS_NOT_BETTER, RS_REJECTED, RS_COOLDOWN
};

struct DiagEntry {
    uint32_t epoch;      // 0 if the clock wasn't synced yet
    uint32_t uptimeSec;
    uint32_t value;      // durations in seconds (see DiagEvent)
    uint8_t  type;
    uint8_t  detail;     // 802.11 reason code, or RoamSkip
    uint8_t  chan;       // AP events: channel of `ap`
    int8_t   rssi;       // AP events: RSSI of `ap`, dBm
    uint8_t  ap[6];      // AP events: the BSSID connected / roamed to / tried
    uint8_t  from[6];    // EV_ROAM: the BSSID left; EV_ROAM_SKIP: the current one
    int8_t   fromRssi;
};

struct DiagRecord {
    uint32_t  magic;
    uint32_t  bootCount;
    uint32_t  wifiDrops;   // cumulative across reboots, unlike s_wifiDropCount
    uint32_t  stalls;      // cumulative across reboots, unlike g_stallCount
    uint32_t  roams;       // cumulative across reboots, unlike s_roamCount
    uint8_t   clientArmed; // re-arm the stall supervisor after a self-reboot
    uint8_t   head;
    DiagEntry ev[DIAG_EVENTS];
};

RTC_NOINIT_ATTR DiagRecord s_diag;

static const char *diagEventName(uint8_t t)
{
    switch (t) {
        case EV_BOOT:         return "boot";
        case EV_WIFI_LOST:    return "WiFi lost";
        case EV_WIFI_OK:      return "WiFi connected";
        case EV_REBOOT:       return "self-reboot";
        case EV_RAIN:         return "RAIN — unsafe";
        case EV_DRY:          return "dry — safe";
        case EV_NET_STALL:    return "polls stopped (link up)";
        case EV_NET_OK:       return "polls resumed";
        case EV_CONNECT_FAIL: return "connect failed";
        case EV_ROAM_SKIP:    return "roam skipped";
        case EV_ROAM:         return "roamed";
        case EV_CLIENT_GONE:  return "client presumed gone";
        case EV_CLIENT_BYE:   return "client disconnected (Connected=false)";
        default:              return "";
    }
}

static const char *roamSkipName(uint8_t r)
{
    switch (r) {
        case RS_SCAN_FAILED: return "scan failed";
        case RS_NO_OTHER_AP: return "no other AP with this SSID";
        case RS_NOT_BETTER:  return "no AP enough stronger";
        case RS_REJECTED:    return "stronger AP refused us within the last hour";
        case RS_COOLDOWN:    return "cooldown after the last roam";
        default:             return "";
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

// Returns the new entry so callers can fill in the AP / duration fields.
static DiagEntry &diagLog(DiagEvent type, uint8_t detail = 0)
{
    DiagEntry &e = s_diag.ev[s_diag.head];
    memset(&e, 0, sizeof(e));
    time_t now   = time(nullptr);
    e.epoch      = (now > (time_t)TIME_SYNCED_EPOCH) ? (uint32_t)now : 0;
    e.uptimeSec  = millis() / 1000UL;
    e.type       = (uint8_t)type;
    e.detail     = detail;
    s_diag.head  = (s_diag.head + 1) % DIAG_EVENTS;
    return e;
}

// The common 802.11 reason codes, so the log reads without a lookup table.
// 15 in particular means the AP's group-key rotation timed out — a classic
// cause of a single unexplained nightly drop.
static const char *wifiReasonName(uint8_t r)
{
    switch (r) {
        case 1:   return "unspecified";
        case 2:   return "auth expired";
        case 3:   return "deauth — we left";
        case 4:   return "assoc expired (AP idle timeout)";
        case 6:   return "not authenticated";
        case 7:   return "not associated";
        case 8:   return "left voluntarily";
        case 15:  return "4-way handshake timeout (group rekey)";
        case 200: return "beacon timeout (AP unreachable)";
        case 201: return "no AP found";
        case 202: return "auth failed";
        case 203: return "assoc failed";
        case 204: return "handshake timeout";
        case 39:  return "timeout (AP gave up on us)";
        case 205: return "connection failed";
        default:  return "";
    }
}

static String macStr(const uint8_t *m)
{
    char buf[18];
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             m[0], m[1], m[2], m[3], m[4], m[5]);
    return String(buf);
}

static String fmtDur(uint32_t s)
{
    if (s < 120)  return String(s) + " s";
    if (s < 7200) return String(s / 60) + " min";
    return String(s / 3600) + " h " + String((s % 3600) / 60) + " min";
}

// "aa:bb:cc:dd:ee:ff ch 6, -49 dBm"
static String apStr(const uint8_t *bssid, uint8_t chan, int8_t rssi)
{
    String s = macStr(bssid);
    if (chan) s += " ch " + String(chan);
    return s + ", " + String(rssi) + " dBm";
}

static bool macIsZero(const uint8_t *m)
{
    for (int i = 0; i < 6; i++) if (m[i]) return false;
    return true;
}

// Human-readable detail for one diagnostics row.
static String diagWhat(const DiagEntry &e)
{
    String what = diagEventName(e.type);
    switch (e.type) {
        case EV_WIFI_LOST: {
            what += " — reason " + String(e.detail);
            const char *rn = wifiReasonName(e.detail);
            if (rn[0]) what += " (" + String(rn) + ")";
            break;
        }
        case EV_WIFI_OK:
            what += " — " + apStr(e.ap, e.chan, e.rssi);
            if (e.rssi < WIFI_MIN_RSSI) what += " (below min RSSI — best that accepted us)";
            break;
        case EV_CONNECT_FAIL: {
            what += macIsZero(e.ap) ? String(" — any AP") : " — " + apStr(e.ap, e.chan, e.rssi);
            if (e.detail == 0) {
                what += " — timeout";
            } else {
                what += " — reason " + String(e.detail);
                const char *rn = wifiReasonName(e.detail);
                if (rn[0]) what += " (" + String(rn) + ")";
            }
            break;
        }
        case EV_ROAM_SKIP:
            what += " — " + String(roamSkipName(e.detail)) +
                    "; on " + macStr(e.from) + " at " + String(e.fromRssi) + " dBm";
            if (!macIsZero(e.ap)) what += ", best other " + apStr(e.ap, e.chan, e.rssi);
            if (e.detail == RS_NOT_BETTER)
                what += " (needs ≥ " + String(e.fromRssi + ROAM_HYSTERESIS_DB) + ")";
            if (e.value) what += "; weak for " + fmtDur(e.value);
            break;
        case EV_ROAM:
            what += " " + macStr(e.from) + " (" + String(e.fromRssi) + " dBm) → " +
                    apStr(e.ap, e.chan, e.rssi);
            if (e.value) what += "; was weak for " + fmtDur(e.value);
            break;
        case EV_NET_OK:
            if (e.value) what += " — after " + fmtDur(e.value);
            break;
        case EV_CLIENT_GONE:
            what += " — no polls for " + fmtDur(e.value) + ", stall supervisor disarmed";
            break;
        default:
            break;
    }
    return what;
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
               "<p><strong>WiFi drops since power-on:</strong> " + String(s_diag.wifiDrops) + "</p>"
               "<p><strong>Poll stalls since power-on:</strong> " + String(s_diag.stalls) + "</p>"
               "<p><strong>Roams since power-on:</strong> " + String(s_diag.roams) + "</p>";

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
        h += "<tr><td><code>" + diagWhen(e) + "</code></td><td>" + diagWhat(e) + "</td></tr>";
    }
    if (!any) h += "<tr><td colspan='2'>no events recorded</td></tr>";
    h += "</table></div>";
    return h;
}

// ── Poll-stall supervisor state ───────────────────────────────────────────────
static bool     g_clientArmed   = false;  // a client is (or was) actively polling
static uint32_t g_lastAlpacaMs  = 0;      // last request on the Alpaca API
static uint8_t  g_stallStage    = 0;      // 0 quiet-ok, 1 warned, 2 re-associated,
                                          // 3 gave up (disarmed; gap still logged on resume)
static uint32_t g_stallCount    = 0;      // episodes since boot (status page)
static uint32_t g_lastStallCheckMs = 0;

// Mirrored into RTC memory so a self-reboot mid-outage doesn't forget that a
// client was polling — that used to leave "Client: idle, 0 stalls" on the page
// after a whole night of missed polls.
static void setClientArmed(bool armed)
{
    g_clientArmed      = armed;
    s_diag.clientArmed = armed ? 1 : 0;
}

// Every Alpaca response funnels through here. `arms` is true only for the
// requests that prove a client session is live (issafe polls, connect PUT) —
// a passing discovery scan of the management API must not arm the detector.
static void noteAlpaca(bool arms)
{
    if (g_stallStage != 0) {
        uint32_t gap = millis() - g_lastAlpacaMs;
        diagLog(EV_NET_OK).value = gap / 1000UL;
        Serial.printf("Polls resumed after %lus (stage %u)\n",
                      (unsigned long)(gap / 1000UL), g_stallStage);
        g_stallStage = 0;
    }
    g_lastAlpacaMs = millis();
    if (arms && !g_clientArmed) setClientArmed(true);
}

// ── WiFi supervisor state ─────────────────────────────────────────────────────
static uint32_t s_lastWifiCheckMs = 0;
static uint32_t s_wifiDownSinceMs = 0;   // 0 = link is up (a roam's gap counts as down)
static uint32_t s_wifiDropCount   = 0;   // reported on the status page
static uint32_t s_linkUpMs        = 0;   // when the current association came up
static bool     s_wifiEverUp      = false;

// Connect sequencer. Every (re)connect — boot, drop, stall-forced, roam — goes
// through the same steps: all-channel scan -> each BSSID strongest-first ->
// plain connect (driver scans all channels and picks by signal). Trying the
// BSSIDs in turn is what copes with UniFi "Lock to AP": an AP that refuses us
// is skipped rather than retried forever.
enum LinkState : uint8_t {
    LS_UP,          // associated, or down and waiting for the next supervisor poll
    LS_SCAN,        // async scan before a (re)connect; link is down
    LS_TRY_BSSID,   // connecting to s_cand[s_candIdx]
    LS_TRY_ANY,     // plain connect, the driver chooses
    LS_ROAM_SCAN,   // async scan while still associated
    LS_ROAM_WAIT,   // roam decided; waiting for the gap after a client poll
    LS_ROAM_LEAVE   // roam under way: old AP dropped, waiting for it to settle
};

struct ApCand {
    uint8_t bssid[6];
    uint8_t chan;
    int8_t  rssi;
};

static LinkState s_link       = LS_UP;
static ApCand    s_cand[WIFI_MAX_CANDIDATES];
static uint8_t   s_candCount  = 0;
static uint8_t   s_candIdx    = 0;       // next candidate to try
static uint32_t  s_linkStepMs = 0;       // when the current scan / attempt started
static bool      s_tryGotIp   = false;   // GOT_IP seen during the current attempt
static bool      s_scanStarted = false;  // LS_SCAN: settle gap over, scan running
static bool      s_roamLeft    = false;  // LS_ROAM_LEAVE: the old AP's disconnect event has arrived
static uint32_t  s_roamLeftMs  = 0;      // ...at this time

// Roaming
static bool      s_roaming         = false;  // current attempt is a planned roam
static ApCand    s_roamFrom;                 // AP we left, for the log
static uint32_t  s_roamWeakSec     = 0;      // how long the link had been weak
static uint32_t  s_roamCount       = 0;      // since boot (status page)
static uint32_t  s_roamSkipCount   = 0;      // since boot (status page)
static uint32_t  s_lastRoamCheckMs = 0;
static uint32_t  s_weakSinceMs     = 0;      // 0 = RSSI healthy
static uint32_t  s_lastRoamMs      = 0;      // last roam attempt; 0 = none yet
static bool      s_roamPending     = false;  // sustained weak; scan at the next poll gap
static uint32_t  s_roamPendingMs   = 0;
static uint8_t   s_rejectBssid[6]  = {0};    // last BSSID that refused a connect
static uint32_t  s_rejectMs        = 0;
static uint8_t   s_lastSkipReason  = RS_NONE;
static uint32_t  s_lastSkipLogMs   = 0;

// Set from the WiFi event task, consumed in loop(). The handler must stay this
// trivial: WiFi.begin()/disconnect() must not be called from the event context,
// so all it does is bump a counter that loop() compares against what it has
// already seen. Counters (not flags) so the sequencer can tell an event that
// arrived during the current attempt from one it already handled.
static volatile uint32_t s_evtDiscSeq           = 0;
static volatile uint32_t s_evtIpSeq             = 0;
static volatile uint8_t  s_wifiDisconnectReason = 0;
static uint32_t          s_seenDiscSeq          = 0;
static uint32_t          s_seenIpSeq            = 0;

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            s_wifiDisconnectReason = info.wifi_sta_disconnected.reason;
            s_evtDiscSeq++;
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            s_evtIpSeq++;
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
    snprintf(foot, sizeof(foot), "%ddBm ch%d  up %lum",
             up ? WiFi.RSSI() : 0, up ? WiFi.channel() : 0,
             (unsigned long)(millis() / 60000UL));
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

// ── WiFi connect sequencer ────────────────────────────────────────────────────
// Nothing here blocks: scans are async and WiFi.begin() returns immediately, so
// loop() keeps serving HTTP, sampling the rain pin and feeding the watchdog
// through a scan or a roam.

static void currentAp(ApCand &a)
{
    const uint8_t *b = WiFi.BSSID();
    if (b) memcpy(a.bssid, b, 6); else memset(a.bssid, 0, 6);
    a.chan = (uint8_t)WiFi.channel();
    a.rssi = (int8_t)WiFi.RSSI();
}

static bool recentlyRejected(const uint8_t *bssid)
{
    return s_rejectMs && millis() - s_rejectMs < ROAM_REJECT_HOLDOFF_MS &&
           memcmp(bssid, s_rejectBssid, 6) == 0;
}

// True when a roam step can run without landing on top of a client poll:
// nobody is polling, or we are 0.3-2 s past the last poll (the response has
// left and N.I.N.A's next 5 s poll is ~3 s away), or polls are too irregular
// to wait for.
static bool pollGapOk(uint32_t waitingSinceMs)
{
    if (!g_clientArmed) return true;
    uint32_t since = millis() - g_lastAlpacaMs;
    if (since >= 300 && since < 2000) return true;
    return millis() - waitingSinceMs > ROAM_POLL_ALIGN_MS;
}

// Start an async scan for our SSID only. False if the driver refused.
static bool wifiStartScan()
{
    int16_t r = WiFi.scanNetworks(true, false, false, WIFI_SCAN_MS_PER_CHAN, 0, WIFI_SSID);
    s_linkStepMs = millis();
    return r != WIFI_SCAN_FAILED;
}

// Poll the running scan. The core reports FAILED once 20x the dwell has passed,
// but a scan taken while associated keeps hopping back to the home channel and
// can run past that; its SCAN_DONE still lands and delivers the results. So
// treat FAILED as "still running" until WIFI_SCAN_GIVEUP_MS.
static int16_t wifiScanPoll()
{
    int16_t n = WiFi.scanComplete();
    if (n == WIFI_SCAN_FAILED && millis() - s_linkStepMs < WIFI_SCAN_GIVEUP_MS) return WIFI_SCAN_RUNNING;
    if (n != WIFI_SCAN_RUNNING) {
        Serial.printf("WiFi: scan result %d after %lu ms\n", n,
                      (unsigned long)(millis() - s_linkStepMs));
    }
    return n;
}

// Copy a finished scan into s_cand[], strongest first, keeping the top
// WIFI_MAX_CANDIDATES. Candidates below WIFI_MIN_RSSI sort after every stronger
// one, so they are only tried once the stronger APs have refused us.
static void wifiCollectCandidates(int16_t n)
{
    s_candCount = 0;
    s_candIdx   = 0;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) != WIFI_SSID) continue;
        ApCand c;
        memcpy(c.bssid, WiFi.BSSID(i), 6);
        c.chan = (uint8_t)WiFi.channel(i);
        c.rssi = (int8_t)WiFi.RSSI(i);

        int pos = s_candCount;
        while (pos > 0 && s_cand[pos - 1].rssi < c.rssi) pos--;
        if (pos >= WIFI_MAX_CANDIDATES) continue;
        int last = min((int)s_candCount, WIFI_MAX_CANDIDATES - 1);
        for (int j = last; j > pos; j--) s_cand[j] = s_cand[j - 1];
        s_cand[pos] = c;
        if (s_candCount < WIFI_MAX_CANDIDATES) s_candCount++;
    }
    WiFi.scanDelete();

    Serial.printf("Scan: %u AP(s) for \"%s\"\n", s_candCount, WIFI_SSID);
    for (int i = 0; i < s_candCount; i++) {
        Serial.printf("  %s%s\n", apStr(s_cand[i].bssid, s_cand[i].chan, s_cand[i].rssi).c_str(),
                      s_cand[i].rssi < WIFI_MIN_RSSI ? "  (below min RSSI)" : "");
    }
}

// Issue the next connect attempt: the next BSSID, else a plain connect.
static void wifiTryNext()
{
    s_tryGotIp   = false;
    s_linkStepMs = millis();
    if (s_candIdx < s_candCount) {
        const ApCand &c = s_cand[s_candIdx];
        Serial.printf("WiFi: trying %s\n", apStr(c.bssid, c.chan, c.rssi).c_str());
        // The AP is already chosen from our own all-channel scan, so let the
        // driver go straight to it on its channel — the shortest possible gap.
        WiFi.setScanMethod(WIFI_FAST_SCAN);
        WiFi.begin(WIFI_SSID, WIFI_PASS, c.chan, c.bssid);
        s_link = LS_TRY_BSSID;
    } else {
        Serial.println("WiFi: plain connect (driver picks strongest AP)");
        WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        s_link = LS_TRY_ANY;
    }
}

// Begin a full (re)connect pass with the link down. The scan itself starts
// WIFI_SCAN_SETTLE_MS later from maintainWifi(): an async scan started right
// after WiFi.disconnect() is silently aborted by it — no SCAN_DONE ever
// arrives (measured on core 2.0.17).
static void wifiStartConnect()
{
    s_candCount   = 0;
    s_candIdx     = 0;
    WiFi.disconnect();   // abandon any attempt in flight, or the scan is refused
    s_scanStarted = false;
    s_link        = LS_SCAN;
    s_linkStepMs  = millis();
}

// The attempt in flight failed (reason 0 = timed out). Log it, then move on.
static void wifiAttemptFailed(uint8_t reason)
{
    DiagEntry &e = diagLog(EV_CONNECT_FAIL, reason);
    if (s_link == LS_TRY_BSSID) {
        const ApCand &c = s_cand[s_candIdx];
        memcpy(e.ap, c.bssid, 6);
        e.chan = c.chan;
        e.rssi = c.rssi;
        memcpy(s_rejectBssid, c.bssid, 6);   // keep roaming away from it for a while
        s_rejectMs = millis() | 1;
        Serial.printf("WiFi: %s failed (reason %u %s)\n", macStr(c.bssid).c_str(),
                      reason, reason ? wifiReasonName(reason) : "timeout");
        s_candIdx++;
        wifiTryNext();
    } else {
        Serial.printf("WiFi: plain connect failed (reason %u %s) — new pass\n",
                      reason, reason ? wifiReasonName(reason) : "timeout");
        wifiStartConnect();
    }
}

static void wifiOnConnected()
{
    ApCand now;
    currentAp(now);
    Serial.printf("WiFi up: %s, IP %s\n", apStr(now.bssid, now.chan, now.rssi).c_str(),
                  WiFi.localIP().toString().c_str());

    if (s_roaming && memcmp(now.bssid, s_roamFrom.bssid, 6) != 0) {
        DiagEntry &e = diagLog(EV_ROAM);
        memcpy(e.ap, now.bssid, 6);
        e.chan = now.chan;
        e.rssi = now.rssi;
        memcpy(e.from, s_roamFrom.bssid, 6);
        e.fromRssi = s_roamFrom.rssi;
        e.value    = s_roamWeakSec;
        s_roamCount++;
        s_diag.roams++;
        Serial.printf("Roamed in %lu ms\n", (unsigned long)(millis() - s_wifiDownSinceMs));
    } else {
        DiagEntry &e = diagLog(EV_WIFI_OK);
        memcpy(e.ap, now.bssid, 6);
        e.chan = now.chan;
        e.rssi = now.rssi;
    }

    s_roaming         = false;
    s_link            = LS_UP;
    s_wifiDownSinceMs = 0;
    s_linkUpMs        = millis();
    s_weakSinceMs     = 0;
    s_roamPending     = false;
    s_lastRoamCheckMs = millis();
    if (s_wifiEverUp) startDiscovery();   // rebind the discovery socket; setup() does the first
    s_wifiEverUp = true;
}

// Evaluate a scan taken while associated and either commit to a roam or log
// why not. s_cand[] is sorted strongest first with refusing BSSIDs removed.
static void roamDecide(int16_t n)
{
    ApCand cur;
    currentAp(cur);
    uint32_t weakSec = s_weakSinceMs ? (millis() - s_weakSinceMs) / 1000UL : 0;

    uint8_t skip = RS_NONE;
    int  bestIdx = -1;
    bool rejectedBetter = false;

    if (n < 0) {
        skip = RS_SCAN_FAILED;
        WiFi.scanDelete();
        s_candCount = 0;
    } else {
        wifiCollectCandidates(n);
        // Prefer the scan's reading of our own AP so both sides of the
        // comparison come from the same measurement.
        for (int i = 0; i < s_candCount; i++)
            if (memcmp(s_cand[i].bssid, cur.bssid, 6) == 0) cur.rssi = s_cand[i].rssi;
        // Drop BSSIDs that refused us recently (UniFi "Lock to AP").
        int k = 0;
        for (int i = 0; i < s_candCount; i++) {
            if (recentlyRejected(s_cand[i].bssid)) {
                if (s_cand[i].rssi >= cur.rssi + ROAM_HYSTERESIS_DB) rejectedBetter = true;
                continue;
            }
            s_cand[k++] = s_cand[i];
        }
        s_candCount = k;
        for (int i = 0; i < s_candCount; i++) {
            if (memcmp(s_cand[i].bssid, cur.bssid, 6) != 0) { bestIdx = i; break; }
        }
        if (bestIdx < 0)
            skip = rejectedBetter ? RS_REJECTED : RS_NO_OTHER_AP;
        else if (s_cand[bestIdx].rssi < cur.rssi + ROAM_HYSTERESIS_DB)
            skip = rejectedBetter ? RS_REJECTED : RS_NOT_BETTER;
    }

    if (skip == RS_NONE) {
        // Target first; the rest (including the current AP) stay as fallbacks.
        ApCand t = s_cand[bestIdx];
        for (int j = bestIdx; j > 0; j--) s_cand[j] = s_cand[j - 1];
        s_cand[0] = t;
        const ApCand *best = &s_cand[0];
        Serial.printf("Roam: %s -> %s after a gap in polls\n",
                      apStr(cur.bssid, cur.chan, cur.rssi).c_str(),
                      apStr(best->bssid, best->chan, best->rssi).c_str());
        s_roamFrom    = cur;
        s_roamWeakSec = weakSec;
        s_link        = LS_ROAM_WAIT;
        s_linkStepMs  = millis();
        return;
    }

    s_roamSkipCount++;
    Serial.printf("Roam skipped: %s (current %d dBm)\n", roamSkipName(skip), cur.rssi);
    if (skip != s_lastSkipReason || millis() - s_lastSkipLogMs > ROAM_SKIP_RELOG_MS) {
        DiagEntry &e = diagLog(EV_ROAM_SKIP, skip);
        memcpy(e.from, cur.bssid, 6);
        e.fromRssi = cur.rssi;
        e.value    = weakSec;
        if (bestIdx >= 0) {
            memcpy(e.ap, s_cand[bestIdx].bssid, 6);
            e.chan = s_cand[bestIdx].chan;
            e.rssi = s_cand[bestIdx].rssi;
        }
        s_lastSkipReason = skip;
        s_lastSkipLogMs  = millis();
    }
    // Stay put, and require another full sustain period before scanning again.
    s_link        = LS_UP;
    s_weakSinceMs = millis() | 1;
}

// Roam watch — only while associated and idle. Samples RSSI every
// ROAM_CHECK_INTERVAL_MS; any sample at or above the trigger resets the clock.
static void maintainRoam()
{
    if (millis() - s_lastRoamCheckMs >= ROAM_CHECK_INTERVAL_MS) {
        s_lastRoamCheckMs = millis();
        int rssi = WiFi.RSSI();
        if (rssi == 0 || rssi >= ROAM_TRIGGER_RSSI) {   // 0 = driver had no reading
            s_weakSinceMs = 0;
            s_roamPending = false;
        } else if (s_weakSinceMs == 0) {
            s_weakSinceMs = millis() | 1;
            Serial.printf("Roam watch: RSSI %d dBm below %d — timing\n", rssi, ROAM_TRIGGER_RSSI);
        } else if (!s_roamPending && millis() - s_weakSinceMs >= ROAM_SUSTAIN_MS) {
            if (s_lastRoamMs && millis() - s_lastRoamMs < ROAM_COOLDOWN_MS) {
                if (s_lastSkipReason != RS_COOLDOWN) {
                    DiagEntry &e = diagLog(EV_ROAM_SKIP, RS_COOLDOWN);
                    ApCand cur;
                    currentAp(cur);
                    memcpy(e.from, cur.bssid, 6);
                    e.fromRssi = cur.rssi;
                    e.value    = (millis() - s_weakSinceMs) / 1000UL;
                    s_lastSkipReason = RS_COOLDOWN;
                    s_lastSkipLogMs  = millis();
                    s_roamSkipCount++;
                }
            } else {
                Serial.printf("Roam watch: weak for %lus — scanning at next poll gap\n",
                              (unsigned long)((millis() - s_weakSinceMs) / 1000UL));
                s_roamPending   = true;
                s_roamPendingMs = millis();
            }
        }
    }

    if (s_roamPending && pollGapOk(s_roamPendingMs)) {
        s_roamPending = false;
        if (wifiStartScan()) {
            s_link = LS_ROAM_SCAN;
        } else {
            roamDecide(-1);   // logs RS_SCAN_FAILED and restarts the sustain clock
        }
    }
}

// ── WiFi supervisor ───────────────────────────────────────────────────────────
// Polled from loop(). Escalates: notice the drop -> connect pass (repeated) ->
// reboot once the link has been down WIFI_REBOOT_AFTER_MS.
static void maintainWifi()
{
    const bool    evDisc = (s_evtDiscSeq != s_seenDiscSeq);
    const bool    evIp   = (s_evtIpSeq   != s_seenIpSeq);
    const uint8_t reason = s_wifiDisconnectReason;
    s_seenDiscSeq = s_evtDiscSeq;
    s_seenIpSeq   = s_evtIpSeq;

    if (s_wifiDownSinceMs && millis() - s_wifiDownSinceMs > WIFI_REBOOT_AFTER_MS) {
        safeRestart("WiFi down");
        return;
    }

    switch (s_link) {
        case LS_SCAN: {
            if (!s_scanStarted) {
                if (millis() - s_linkStepMs < WIFI_SCAN_SETTLE_MS) return;
                if (!wifiStartScan()) {
                    Serial.println("WiFi: scan refused — plain connect");
                    wifiTryNext();
                    return;
                }
                s_scanStarted = true;
                return;
            }
            int16_t n = wifiScanPoll();
            if (n == WIFI_SCAN_RUNNING) return;
            if (n > 0) {
                wifiCollectCandidates(n);
                // A BSSID that just refused us goes last rather than first.
                for (int i = 0; i + 1 < s_candCount; i++) {
                    if (recentlyRejected(s_cand[i].bssid)) {
                        ApCand t = s_cand[i];
                        for (int j = i; j + 1 < s_candCount; j++) s_cand[j] = s_cand[j + 1];
                        s_cand[s_candCount - 1] = t;
                        break;   // only one BSSID is ever remembered
                    }
                }
            } else {
                WiFi.scanDelete();
                s_candCount = 0;
                s_candIdx   = 0;
            }
            wifiTryNext();
            return;
        }

        case LS_TRY_BSSID:
        case LS_TRY_ANY:
            if (evIp) s_tryGotIp = true;
            if (s_tryGotIp && WiFi.status() == WL_CONNECTED) {
                wifiOnConnected();
                return;
            }
            // Reason 8 is our own begin() tearing down the old association.
            if (evDisc && reason != WIFI_REASON_ASSOC_LEAVE) {
                wifiAttemptFailed(reason);
                return;
            }
            if (millis() - s_linkStepMs >
                (s_link == LS_TRY_BSSID ? WIFI_BSSID_WAIT_MS : WIFI_ANY_WAIT_MS)) {
                wifiAttemptFailed(0);
            }
            return;

        case LS_ROAM_SCAN:
        case LS_ROAM_WAIT:
            if (WiFi.status() != WL_CONNECTED) {
                // Dropped on its own mid-roam — hand over to normal recovery.
                if (s_link == LS_ROAM_SCAN) WiFi.scanDelete();
                s_link = LS_UP;
                s_lastWifiCheckMs = 0;
                break;
            }
            if (s_link == LS_ROAM_SCAN) {
                int16_t n = wifiScanPoll();
                if (n == WIFI_SCAN_RUNNING) return;
                roamDecide(n);
                return;
            }
            if (pollGapOk(s_linkStepMs)) {
                s_roaming         = true;
                s_lastRoamMs      = millis() | 1;   // cooldown runs from the attempt, success or not
                s_lastSkipReason  = RS_NONE;        // so this cooldown's skip gets logged
                s_wifiDownSinceMs = millis();
                s_candIdx         = 0;
                // Leave the old AP first and let it settle. Core 2.0.17's
                // begin() disconnects and reconnects back to back, which from a
                // live association races the teardown. Ported from the roof
                // controller as a precaution: it did not fix the roof's bench
                // roam failures (targets there were -71..-88 dBm), and no roam
                // has yet been seen to succeed on either device.
                WiFi.disconnect();
                s_roamLeft   = false;
                s_linkStepMs = millis();
                s_link       = LS_ROAM_LEAVE;
            }
            return;

        case LS_ROAM_LEAVE:
            if (evDisc && !s_roamLeft) { s_roamLeft = true; s_roamLeftMs = millis(); }
            if ((s_roamLeft && millis() - s_roamLeftMs >= ROAM_LEAVE_SETTLE_MS) ||
                millis() - s_linkStepMs >= ROAM_LEAVE_MAX_MS) {
                Serial.printf("Roam: left old AP (disconnect event %s), %lu ms since leaving\n",
                              s_roamLeft ? "seen" : "NOT seen",
                              (unsigned long)(millis() - s_linkStepMs));
                wifiTryNext();   // target first, then the other candidates incl. the old AP
            }
            return;

        case LS_UP:
            break;
    }

    // LS_UP. The driver knows about a drop long before a 5 s poll would notice,
    // so an event forces the check to run now — shrinking the outage enough
    // that N.I.N.A may never see a failed request.
    if (evDisc || evIp) s_lastWifiCheckMs = 0;

    if (WiFi.status() == WL_CONNECTED) {
        maintainRoam();
        return;
    }

    if (millis() - s_lastWifiCheckMs < WIFI_CHECK_INTERVAL_MS) return;
    s_lastWifiCheckMs = millis();

    s_wifiDownSinceMs = millis();
    s_wifiDropCount++;
    s_diag.wifiDrops++;
    Serial.printf("WiFi link lost (reason %u %s) — reconnecting\n",
                  reason, wifiReasonName(reason));
    diagLog(EV_WIFI_LOST, reason);
    wifiStartConnect();
}

// ── Poll-stall supervisor ─────────────────────────────────────────────────────
// Polled from loop(). Only acts while the link *claims* to be up — when it is
// actually down the WiFi supervisor owns recovery.
static void maintainStall()
{
    if (millis() - g_lastStallCheckMs < STALL_CHECK_MS) return;
    g_lastStallCheckMs = millis();

    // A scan or roam in progress owns the link; don't pull it out from under it.
    if (!g_clientArmed || s_link != LS_UP || WiFi.status() != WL_CONNECTED) return;

    // Escalate on silence since the later of the last poll and the link coming
    // up. Measured from the last poll alone, the first check after any WiFi
    // recovery saw >70 s of "silence" and forced another disconnect before the
    // client had a chance to poll again.
    uint32_t quiet   = millis() - g_lastAlpacaMs;
    uint32_t sinceUp = millis() - s_linkUpMs;
    uint32_t escal   = min(quiet, sinceUp);

    if ((g_stallStage == 1 || g_stallStage == 2) && quiet > STALL_GIVEUP_MS) {
        // Client is genuinely gone (N.I.N.A closed or the PC is off). Disarm so
        // an idle device doesn't re-associate all night, but log it, and stay
        // in stage 3 so the full gap is logged if polls ever come back. Checked
        // from stage 1 too: a link that keeps recovering can hold off stage 2.
        g_stallStage = 3;
        setClientArmed(false);
        diagLog(EV_CLIENT_GONE).value = quiet / 1000UL;
        Serial.println("No polls for 10 min — client presumed gone, disarming");
    } else if (g_stallStage == 0 && escal > STALL_WARN_MS) {
        g_stallStage = 1;
        g_stallCount++;
        s_diag.stalls++;
        diagLog(EV_NET_STALL);
        Serial.printf("Client polls stopped %lus ago but WiFi is up\n",
                      (unsigned long)(quiet / 1000UL));
    } else if (g_stallStage == 1 && escal > STALL_REASSOC_MS) {
        // Re-associate: rebuilds the AP's client state and renegotiates rates,
        // which is the strongest repair available short of a reboot. The WiFi
        // supervisor sees the drop and drives the reconnect + discovery re-arm.
        g_stallStage = 2;
        Serial.println("Still no polls — forcing re-association");
        WiFi.disconnect();
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
    ApCand ap;
    currentAp(ap);
    String j = "{\"safe\":" + String(g_debounced == 1 ? "true" : "false") +
        ",\"gpio\":" + String(g_rawLevel) +
        ",\"lastPollSec\":" +
        (g_alpacaConn ? String((millis() - g_lastAlpacaMs) / 1000UL) : String(-1)) +
        ",\"clientArmed\":" + (g_clientArmed ? "true" : "false") +
        ",\"stalls\":" + String(g_stallCount) +
        ",\"rssi\":" + String(WiFi.RSSI()) +
        ",\"bssid\":\"" + macStr(ap.bssid) + "\"" +
        ",\"channel\":" + String(ap.chan) +
        ",\"roams\":" + String(s_roamCount) +
        ",\"heap\":" + String(ESP.getFreeHeap()) + "}";
    webServer.send(200, "application/json", j);
}

// "armed", "armed — no polls for 3 min", "gone — no polls for 9 h 5 min", "idle"
static String clientStateStr()
{
    uint32_t quietSec = (millis() - g_lastAlpacaMs) / 1000UL;
    if (g_clientArmed) {
        return g_stallStage ? "armed — no polls for " + fmtDur(quietSec) : String("armed");
    }
    if (g_stallStage == 3) return "gone — no polls for " + fmtDur(quietSec);
    return "idle";
}

// What the roam watch is doing right now, for the Link card.
static String roamStateStr()
{
    switch (s_link) {
        case LS_ROAM_SCAN: return "scanning";
        case LS_ROAM_WAIT: return "roaming at next poll gap";
        case LS_ROAM_LEAVE: return "roaming";
        case LS_UP:        break;
        default:           return "connecting";
    }
    String s;
    if (s_weakSinceMs) {
        s = "weak for " + fmtDur((millis() - s_weakSinceMs) / 1000UL) +
            " (roam check at " + fmtDur(ROAM_SUSTAIN_MS / 1000UL) + ")";
    } else {
        s = "signal OK";
    }
    if (s_lastRoamMs && millis() - s_lastRoamMs < ROAM_COOLDOWN_MS) {
        s += "; cooldown " + fmtDur((ROAM_COOLDOWN_MS - (millis() - s_lastRoamMs)) / 1000UL) + " left";
    }
    return s;
}

static void hRoot()
{
    ApCand ap;
    currentAp(ap);
    const bool up = (WiFi.status() == WL_CONNECTED);
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
        " | <strong>Client:</strong> " + clientStateStr() +
        " | <strong>IP:</strong> " + WiFi.localIP().toString() + "</p></div>"

        "<div class='card'><h2>Link</h2>"
        "<p><strong>Uptime:</strong> " + String(millis() / 60000UL) + " min</p>"
        "<p><strong>WiFi RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>"
        "<p><strong>Access point:</strong> " +
        (up ? "<code>" + macStr(ap.bssid) + "</code> · channel " + String(ap.chan) +
              " · " + String(ap.rssi) + " dBm"
            : String("not connected")) + "</p>"
        "<p><strong>Roam watch:</strong> " + roamStateStr() + "</p>"
        "<p><strong>Roams since boot:</strong> " + String(s_roamCount) +
        " | <strong>Roam checks skipped:</strong> " + String(s_roamSkipCount) + "</p>"
        "<p><strong>WiFi drops since boot:</strong> " + String(s_wifiDropCount) + "</p>"
        "<p><strong>Poll stalls since boot:</strong> " + String(g_stallCount) + "</p>"
        "<p><small>Roam policy: below " + String(ROAM_TRIGGER_RSSI) + " dBm for " +
        fmtDur(ROAM_SUSTAIN_MS / 1000UL) + ", target ≥ " + String(ROAM_HYSTERESIS_DB) +
        " dB stronger, cooldown " + fmtDur(ROAM_COOLDOWN_MS / 1000UL) +
        ", RSSI check every " + fmtDur(ROAM_CHECK_INTERVAL_MS / 1000UL) +
        ", min connect RSSI " + String(WIFI_MIN_RSSI) + " dBm</small></p>"
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
        if (g_clientArmed) diagLog(EV_CLIENT_BYE);   // so a deliberate goodbye is visible too
        setClientArmed(false);
        g_stallStage = 0;
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
    diagLog(EV_BOOT);
    // A client was polling when we self-rebooted — keep watching for it, so an
    // outage that spans the reboot is still logged as a stall.
    g_clientArmed = (s_diag.clientArmed != 0);

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

    // WiFi. The supervisor owns every (re)connect so it can choose the AP. The
    // core's auto-reconnect is off because it re-begin()s the last config —
    // including a pinned BSSID that may be refusing us (UniFi "Lock to AP").
    WiFi.persistent(false);        // don't wear out flash rewriting the same creds
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // modem sleep makes the link flaky under polling
    WiFi.setAutoReconnect(false);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);      // not the first AP found in channel order
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);  // ...but the strongest
    WiFi.setHostname("rg11");
    WiFi.onEvent(onWiFiEvent);     // must be registered before begin()
    Serial.println("Connecting to WiFi (all-channel scan, strongest AP first)");
    // Same sequencer as every later reconnect. maintainWifi() reboots if the
    // link isn't up within WIFI_REBOOT_AFTER_MS (the watchdog isn't armed yet).
    s_wifiDownSinceMs = millis();
    wifiStartConnect();
    while (s_link != LS_UP || WiFi.status() != WL_CONNECTED) {
        delay(50);
        maintainWifi();
    }
    Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    displayMessage("WiFi OK!", WiFi.localIP().toString());

    // Stable per-device Alpaca identity. A hardcoded UniqueID would collide with
    // the unit this one replaces if both are powered on during changeover.
    g_uniqueId = "RG11-SM-" + WiFi.macAddress();
    g_uniqueId.replace(":", "");
    Serial.printf("Alpaca UniqueID: %s\n", g_uniqueId.c_str());

    // NTP — non-blocking, syncs in the background. Only needed so the event log
    // carries wall-clock times that line up with N.I.N.A's log.
    configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);

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
        Serial.printf("HB: GPIO%d=%d safe=%s WiFi=%s RSSI=%d ch=%d heap=%u drops=%lu roams=%lu\n",
                      RAIN_PIN, g_rawLevel,
                      g_debounced ? "Y" : "N",
                      WiFi.status() == WL_CONNECTED
                          ? WiFi.localIP().toString().c_str()
                          : "DOWN",
                      WiFi.RSSI(), WiFi.channel(), (unsigned)ESP.getFreeHeap(),
                      (unsigned long)s_wifiDropCount, (unsigned long)s_roamCount);

        // Heap floor — the String-building handlers are the only plausible leak
        // source; reboot before the allocator starts failing requests.
        if (ESP.getFreeHeap() < HEAP_FLOOR_BYTES) {
            safeRestart("low heap");
        }
    }
}
