# CLAUDE.md

See [README.md](README.md) for the full overview (wiring, endpoint tables,
display layout, troubleshooting). This file is the quick map + gotchas.

## What this is

ASCOM Alpaca **SafetyMonitor** driver for a Hydreon RG-11 optical rain sensor.
The RG-11's relay closes to GND when raining → `issafe` = false for astronomy
clients (N.I.N.A, SGPro). Entire firmware is one file, `src/main.cpp`, one
board, no `#ifdef` branching. This folder IS a git repo (unlike most siblings).

**Board:** ESP32-WROOM-32 DevKit (30-pin) on a screw-terminal expansion board.
**Display:** GC9A01 1.28″ round TFT 240×240, seated directly on the first seven
header pins (VCC GND SCL SDA DC CS RST → 3V3 GND D15 D2 D4 D16 D17).
**Rain pin:** GPIO 27.

```bash
pio run -t upload
pio device monitor        # 115200 baud, exception decoder on
```

## Sibling projects this borrows from

Both live in `C:\Projects\devices`. Check them before reinventing anything here.

- **`ESP32 GC9A01`** — same DevKit, same round panel. The TFT_eSPI build-flag
  block in `platformio.ini` was copied from it verbatim, including the 8-bit
  full-screen sprite pattern.
- **`ESP32 Rolloff`** — the roof controller. The diagnostics record, WiFi
  supervisor, task watchdog and event log are ports of its implementation, so
  both devices' status pages read the same. Keep them in sync.

## Architecture (all in src/main.cpp)

- Three servers: `webServer` port **80** (Status + Link + Diagnostics cards),
  `httpServer` port **11111** (Alpaca REST), `asyncUdp` port **32227**
  (Alpaca UDP discovery via an `onPacket` callback — nothing polled in `loop()`).
  AsyncUDP rather than WiFiUDP because sending on the socket it listens on
  returns ENOMEM. `startDiscovery()` re-arms it after a WiFi recovery.
- Debounce: 5-sample majority vote, one sample per 100 ms → 500 ms settling
  (`DEB_SAMPLES` / `DEB_INTERVAL_MS`). `RAIN_PIN` is `INPUT_PULLUP`,
  active-LOW when raining: HIGH = SAFE, LOW = UNSAFE.
- Single loop, no tasks, no mutexes — keep it that way. The only other context
  is the WiFi event handler, which must stay trivial (see below).

## Reliability layers — don't remove these

- **Task watchdog** armed at the *end* of `setup()` (so a slow boot can't trip
  it), fed by `esp_task_wdt_reset()` as the first statement in `loop()`.
  The `ESP_ARDUINO_VERSION_MAJOR >= 3` guard is deliberate: the
  `esp_task_wdt_init()` signature changed between cores. Installed core is
  **2.0.17**, so the `#else` branch is what actually compiles today.
- **WiFi supervisor** `maintainWifi()`: event-driven detection → re-associate →
  reboot after ~60 s down. `onWiFiEvent()` only raises flags —
  **never call `WiFi.begin()`/`disconnect()` from the event task context.**
- **Heap floor** at 40 kB. The HTTP handlers build responses with `String`
  concatenation, which is the only plausible leak source.
- **Debounce seeded from the real pin level** in `setup()`. It used to default
  to SAFE, which meant reporting SAFE for the first ~100 ms of every boot even
  while raining. This is a safety monitor — keep the seeding.
- Diagnostics live in `RTC_NOINIT_ATTR` memory: survives a software/watchdog
  reset, not a power cycle. Gated on **both** `DIAG_MAGIC` and
  `ESP_RST_POWERON`. Bump the magic whenever `DiagRecord` changes, or a reflash
  can decode stale RTC contents as garbage.

## Hard-won hardware facts

- **The 4.7 kΩ pull-up from GPIO 27 to 3V3 is required**, for relay contact
  wetting current — the internal ~45 kΩ pull-up only gives ~73 µA, which is
  dry-circuit territory for a contact that switches a few times a year.
  Older docs in this repo claimed an external resistor "caused problems"; that
  was a resistor loading the *I²C bus* on the retired OLED build, wrongly
  generalised. The deployed hardware always had one.
- **Pull up to 3V3, never VIN.** The old build pulled to 5 V; the GPIO ESD diode
  clamps it and it works, but VIN rises before the 3.3 V regulator at power-on,
  injecting current into an unpowered rail — a latch-up path.
- **GPIO 34/35/36(VP)/39(VN) are input-only with no internal pull-up.**
  `INPUT_PULLUP` compiles and silently does nothing. This is the trap on a
  WROOM-32. D12 (MTDI) and D0/D5 are strapping pins.
- **GPIO 2 (MOSI) shares the onboard D2 LED.** That's why `SPI_FREQUENCY` is
  pinned to 27 MHz. The LED flickers constantly — expected, not a fault.
- GPIO 2 and 15 are strapping pins but safe here: all display signals are
  inputs on the module, so they're high-Z at reset.
- RG-11 relay: COM → GPIO 27, NO → GND. 12 V from a separate supply.

## Build config gotchas

- **TFT_eSPI is configured entirely through `build_flags`** in
  `platformio.ini` — never edit `User_Setup.h`, the config must travel with
  the repo.
- **Fonts 6, 7 and 8 are digits-only** in TFT_eSPI and render nothing for
  "SAFE". The headline uses a GFXFF free font (`FreeSansBold18pt7b`); font 4 is
  the largest built-in with a full alphabet.
- The display is drawn into an **8-bit 240×240 sprite** (57.6 kB heap) and
  pushed in one operation — the GC9A01 has no frame buffer, so drawing straight
  to the panel flickers. `displayInit()` falls back to direct drawing if
  `createSprite()` fails.
- `build_type = debug` so `esp32_exception_decoder` can resolve symbols. No
  IRAM pressure on ESP32, so this is free — keep it.

## Secrets

Copy `include/secrets.h.example` → `include/secrets.h` and set
`WIFI_SSID` / `WIFI_PASS`. The real file is gitignored.
