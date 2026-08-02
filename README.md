# ESP32 ASCOM Alpaca Safety Monitor – Hydreon RG-11 Rain Sensor

An ASCOM Alpaca-compatible Safety Monitor that interfaces with a Hydreon RG-11
optical rain sensor and shows live status on a 1.28″ round colour display.

Built for unattended overnight operation: it supervises its own WiFi link,
reboots itself out of states it can't recover from, and keeps a diagnostics
record in RTC memory so the cause of an overnight reboot is still visible on
the status page in the morning.

| | |
|---|---|
| **Board** | ESP32-WROOM-32 DevKit (30-pin) on a screw-terminal expansion board |
| **Display** | GC9A01 1.28″ round IPS TFT, 240×240, SPI |
| **Sensor** | Hydreon RG-11 optical rain sensor (relay output) |
| **Power** | 5 V via USB-C; RG-11 needs its own 12 V supply |

## Features

- **ASCOM Alpaca Protocol** – full compatibility with N.I.N.A, SGPro, etc.
- **UDP Discovery** – automatic device discovery on port 32227
- **GPIO Debouncing** – 5-sample majority voting (500 ms settling time)
- **Round status display** – green SAFE / red NOT SAFE ring, IP, link health
- **Self-healing** – task watchdog, escalating WiFi supervisor, heap floor guard
- **Diagnostics** – reset reason, boot count, WiFi drop counts, event log with
  wall-clock timestamps, all surviving a self-reboot
- **WiFi credentials in `secrets.h`** – never committed to Git

---

## Hardware

### Wiring – GC9A01 display

The display **seats directly onto the first seven header pins** — no jumpers.
Its header order matches the board silkscreen one-to-one.

> The module labels its SPI pins with I²C names: **`SCL` is SCK** and
> **`SDA` is MOSI**. It is SPI, not I²C. There is no `BLK` pin, so the
> backlight is permanently on.

```
Display pin  →  ESP32 header pin  (GPIO)
──────────────────────────────────────────
VCC          →  3V3
GND          →  GND
SCL          →  D15              (GPIO 15, SCK)
SDA          →  D2               (GPIO 2,  MOSI)
DC           →  D4               (GPIO 4)
CS           →  D16              (GPIO 16)
RST          →  D17              (GPIO 17)
```

Pin numbers are set in [platformio.ini](platformio.ini) as TFT_eSPI build
flags, so no `User_Setup.h` edit is needed and the config travels with the repo.

### Wiring – RG-11 rain sensor

All three connections land on screw terminals, so no part of the sensor path
relies on a jumper.

```
RG-11 terminal   →  ESP32 terminal
────────────────────────────────────────────
Relay COM        →  D27      (right-hand block)
Relay NO         →  G (GND)  (right-hand block)
RG-11 12 V / GND →  external 12 V supply

              ┌── D27
Relay COM ────┤
              └── 4.7 kΩ ──► 3V3   (left-hand block, terminal 1)
```

**Logic:**
- GPIO 27 = HIGH (1) → relay open → **SAFE** (no rain)
- GPIO 27 = LOW  (0) → relay closed to GND → **UNSAFE** (rain detected)

#### The 4.7 kΩ pull-up is required

Fit it. The internal pull-up alone is ~45 kΩ, which puts only
3.3 V / 45 kΩ ≈ **73 µA** through the RG-11's relay contacts — dry-circuit
territory for a contact that sits unchanged for months and only switches when
it rains. Relay contacts need a minimum **wetting current** to break through the
oxide film that forms over time. 4.7 kΩ gives ~700 µA. It also drops the input's
source impedance about 10×, which matters on a long outdoor cable run.

**Pull up to 3V3, not to VIN.** A pull-up to 5 V works — the GPIO's ESD diode
clamps the pin to ~3.6 V — but it is out of spec. The problem isn't the
steady-state leakage, it's power sequencing: VIN rises before the onboard 3.3 V
regulator settles, so every power-on briefly injects current through the ESD
diode into an unpowered VDD rail, which is a latch-up path. 3V3 is on the
adjacent terminal and costs nothing.

Optional: a 100 nF from D27 to GND for RF filtering. Fit it only if the
heartbeat shows the raw level flapping — decide from data, not up front.

#### Pins to avoid on this board

| Pin | Why |
|-----|-----|
| **D34, D35, VP (36), VN (39)** | Input-only with **no internal pull-up**. `INPUT_PULLUP` compiles and silently does nothing — the pin floats and the reading is meaningless. |
| **D12** | MTDI strapping; high at boot selects the wrong flash voltage. |
| **D0, D5** | Strapping pins. |
| **D2, D15** | Strapping too, but consumed by the display (safe — see below). |

Three notes on the display pins:

- **GPIO 2 carries the onboard D2 LED**, so MOSI has an LED and resistor
  hanging off it. This is why `SPI_FREQUENCY` is pinned to 27 MHz rather than
  taking the library default. The LED flickers continuously in normal
  operation — expected, not a fault.
- **GPIO 2 and GPIO 15 are strapping pins**, but boot is safe because all five
  display signals are *inputs* on the module and therefore high-Z at reset.
  Don't repurpose them for anything that drives them at reset.
- **None of the five are VSPI IOMUX pins**, so SPI routes through the GPIO
  matrix — another reason for the conservative clock.

---

## Software Setup

### 1. Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)

### 2. WiFi Credentials

```bash
cp include/secrets.h.example include/secrets.h
```

Edit `include/secrets.h` and set your SSID and password:

```cpp
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourPassword"
```

> **`include/secrets.h` is listed in `.gitignore` and will never be committed.**

### 3. Build & Upload

```bash
pio run                 # compile
pio run -t upload       # compile + flash
pio device monitor      # 115200 baud, exception decoder enabled
```

The serial monitor shows the reset reason, boot count and assigned IP on boot.

### 4. Timezone

Event-log timestamps use the house Melbourne timezone, set in
[src/main.cpp](src/main.cpp) as `TZ_INFO`:

```cpp
static const char *TZ_INFO = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
```

Change this if the observatory is elsewhere. NTP sync is non-blocking; events
logged before the clock syncs render as `+123s (no clock)`.

---

## Reliability

Three independent layers, so no single stuck component leaves the device
unreachable until someone power-cycles it.

| Layer | Trigger | Action |
|-------|---------|--------|
| **Task watchdog** | `loop()` stops running for 30 s (hung handler, wedged network stack) | Chip resets; next boot reports `Last reset: TASK WATCHDOG` |
| **WiFi supervisor** | Link drops | Event-driven detection → immediate re-associate → 4 retries × 15 s grace → reboot after ~60 s down |
| **Boot WiFi timeout** | Can't associate at startup | Re-`begin()` at 15 s, reboot at 30 s |
| **Heap floor** | Free heap < 40 kB | Reboot before the allocator starts failing requests |

Supporting settings: `WiFi.setSleep(false)` (modem sleep makes the link flaky
under the status page's 500 ms polling), `WiFi.setAutoReconnect(true)`,
`WiFi.persistent(false)` (avoids flash wear rewriting the same credentials).

The debounce state is seeded from the actual pin level at boot, so the device
never reports SAFE during the first debounce window while it is actually
raining.

---

## Status page (port 80)

Open `http://<IP>/` in any browser — no port number needed. The page
auto-refreshes every 10 s, and the live SAFE/NOT SAFE line updates every 500 ms.

```
Status        SAFE / NOT SAFE · GPIO 27 level · transitions · Alpaca · IP
Link          Uptime · WiFi RSSI · WiFi drops since boot · Free heap
Diagnostics   Last reset · Boots since power-on · WiFi drops since power-on
              Clock · event table (When | Event), newest first
```

The diagnostics record lives in `RTC_NOINIT_ATTR` memory, which **survives a
software or watchdog reset but not a power cycle**. That is deliberate: an
overnight self-reboot keeps its history, while pulling the plug gives a clean
slate. "Since power-on" counters mean exactly what they say.

Logged events: `boot`, `WiFi lost` (with the 802.11 reason code decoded),
`WiFi recovered`, `self-reboot`, `RAIN — unsafe`, `dry — safe`. The rain events
give a usable overnight rain history.

---

## ASCOM Client Integration (N.I.N.A)

1. Open **Equipment → Safety Monitor**
2. Click **Scan for Devices**
3. Select **RG-11 Safety Monitor** and click connect

The Alpaca `UniqueID` is derived from the board's MAC address, so running two
of these on one network (for example during a hardware changeover) does not
confuse the client.

---

## Endpoints

### Status page (port 80)

| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Live status, link health and diagnostics |

### ASCOM Alpaca API (port 11111)

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/v1/safetymonitor/0/issafe` | Current safety status |
| GET | `/api/v1/safetymonitor/0/connected` | Connection state |
| PUT | `/api/v1/safetymonitor/0/connected` | Set connection state |
| GET | `/api/v1/safetymonitor/0/description` | Device description |
| GET | `/api/v1/safetymonitor/0/name` | Device name |
| GET | `/api/v1/safetymonitor/0/driverinfo` | Driver info |
| GET | `/api/v1/safetymonitor/0/driverversion` | Driver version |
| GET | `/api/v1/safetymonitor/0/interfaceversion` | ASCOM interface version |
| GET | `/api/v1/safetymonitor/0/supportedactions` | Supported actions |
| GET | `/api/v1/safetymonitor/0/devicestate` | Device state |
| GET | `/management/apiversions` | Supported API versions |
| GET | `/management/v1/description` | Server description |
| GET | `/management/v1/configureddevices` | Configured devices |

---

## Display Layout

240 × 240 round panel: a 22 px coloured annulus at the rim — **green SAFE /
red NOT SAFE** — around a black centre carrying the text.

```
        ╭───────────╮
      ╭███████████████╮
    ╭███─────────────███╮
   │███               ███│
   │██     SAFE        ██│   headline (FreeSansBold 18pt)
   │██  192.168.1.45   ██│   IP address
   │███ Alpaca:OK T:3 ███│   Alpaca state + transition count
    ╰███ -38dBm up 12m ██╯   link health
      ╰███████████████╯
        ╰───────────╯
```

The GC9A01 has no frame buffer, so the whole frame is composed in an 8-bit
off-screen sprite (240×240 = 57.6 kB of heap) and pushed in one operation.
That removes flicker entirely — do not switch to drawing straight to the panel
unless `createSprite()` fails, which the firmware handles by falling back
automatically.

Fonts 6, 7 and 8 are digits-only in TFT_eSPI and cannot render "SAFE", which is
why the headline uses a GFXFF free font rather than a built-in one.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Display black | Check the module is seated on the *first seven* pins starting at 3V3. Remember SCL=SCK and SDA=MOSI — it is SPI, not I²C |
| Display glitching / corrupt pixels | Lower `SPI_FREQUENCY` in [platformio.ini](platformio.ini) below 27 MHz. GPIO 2 shares MOSI with the onboard D2 LED, which loads the line |
| Display flickers | `createSprite()` failed and it fell back to direct drawing — check free heap on the status page |
| Always shows NOT SAFE | `digitalRead(RAIN_PIN)` should be 1 when dry. Check the 4.7 kΩ pull-up to **3V3**, and that the sensor is on D27, not one of the input-only pins |
| Device not found in discovery | Confirm device and PC are on the same subnet; check UDP port 32227. Discovery is re-armed automatically after a WiFi recovery |
| WiFi not connecting | Verify `secrets.h` credentials; confirm 2.4 GHz network. The device reboots itself after 30 s of failed association |
| Unexplained reboots | Check **Last reset** on the status page. `BROWNOUT (power dip)` means the 5 V supply is marginal — the WROOM-32 draws ~160 mA peak |
| Nightly single WiFi drop | Check the event log's reason code. Reason 15 is the AP's group-key rotation timing out — an AP-side setting, not a device fault |

---

## File Structure

```
Alpaca-RG11-ESP32-SafetyMonitor/
├── platformio.ini           # Build config + TFT_eSPI pin flags
├── src/
│   └── main.cpp             # Full driver
├── include/
│   ├── secrets.h            # WiFi credentials – git-ignored
│   └── secrets.h.example    # Template – committed to Git
├── .gitignore               # Excludes .pio/, secrets.h
├── rg11.jpg                 # Hydreon RG-11 sensor
└── README.md                # This file
```

---

## Version History

### v3.0.0 (2026-08-02)
- **Single board.** Rewritten for the ESP32-WROOM-32 DevKit + GC9A01 round
  display. The `hw364a` (ESP8266), `esp32s3-tft` and `esp32c3-supermini`
  environments and all their `#ifdef` branching are removed
- Round 240×240 ring layout via TFT_eSPI, composed in an off-screen sprite
- **Self-healing:** task watchdog, escalating WiFi supervisor, boot WiFi
  timeout, heap floor guard
- **Diagnostics page:** reset reason, boot and WiFi-drop counters in RTC memory,
  NTP-timestamped event log
- Alpaca `UniqueID` now derived from the MAC instead of hardcoded
- Debounce state seeded from the real pin level at boot (was hardcoded SAFE)
- Corrected the wiring guidance: the external pull-up is **required** (relay
  wetting current), and belongs on 3V3 rather than VIN

### v2.1.0 (2026-07-09)
- Added ESP32-S3 TFT Feather and HW-364A ESP8266 support
- Display layer refactored behind `displayInit()` / `displayMessage()` /
  `displayUpdate()`

### v1.0.0 (2026-02-25)
- Arduino framework, ESP32-C3 SuperMini (HW-466AB), 0.91″ SSD1306 OLED

---

## License

Open source – use and modify freely for your astronomy setup.
