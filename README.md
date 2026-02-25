# ESP32-C3 ASCOM Alpaca Safety Monitor – Hydreon RG-11 Rain Sensor

An ASCOM Alpaca-compatible Safety Monitor for the **ESP32-C3 SuperMini** board
that interfaces with a Hydreon RG-11 optical rain sensor and shows live status on
a 0.91″ OLED display.

## Features

- **ASCOM Alpaca Protocol** – full compatibility with N.I.N.A, SGPro, etc.
- **UDP Discovery** – automatic device discovery on port 32227
- **GPIO Debouncing** – 5-sample majority voting (500 ms settling time)
- **0.91″ OLED Display** – shows SAFE/RAIN status, IP address, Alpaca connection
- **Real-time Web Interface** – live status page at `http://<IP>/`
- **WiFi credentials in `secrets.h`** – never committed to Git

---

## Hardware

### Components

| Part | Details |
|------|---------|
| MCU  | ESP32-C3 SuperMini (HW-466AB) |
| Rain sensor | Hydreon RG-11 optical rain sensor |
| Display | 0.91″ SSD1306 OLED (128×32, I2C) |
| Power | 5 V via USB-C to SuperMini |

---

### Wiring

#### OLED Display (0.91″ SSD1306, I2C)

```
OLED pin  →  ESP32-C3 SuperMini pin
─────────────────────────────────────
GND       →  G   (GND)
VCC       →  3V3 (3.3 V)
SCL       →  GPIO 4
SDA       →  GPIO 3
```

#### RG-11 Rain Sensor

```
RG-11 terminal  →  ESP32-C3 SuperMini pin
─────────────────────────────────────────
Relay COM       →  GPIO 20  (right side)
Relay NO        →  G  (GND)
                   (internal pull-up is sufficient; testing shows problems if an external 10 kΩ is used)
RG-11 12 V / GND → external 12 V power supply
```

**Logic:**
- GPIO 20 = HIGH (1) → relay open → **SAFE** (no rain)
- GPIO 20 = LOW  (0) → relay closed to GND → **UNSAFE** (rain detected)

#### ESP32-C3 SuperMini Pinout Reference

All OLED wires connect to consecutive left-side pins matching the OLED's own
GND → VCC → SCL → SDA pin order.  Rain sensor uses GPIO 20 on the right side.

```
        USB-C
  ┌─────┤├──────┐
  │ 5V        5 │
  │ G  ←GND   6 │  ← OLED GND
  │ 3V3←VCC   7 │  ← OLED VCC
  │ 4  ←SCL   8 │  ← OLED SCL
  │ 3  ←SDA   9 │  ← OLED SDA
  │ 2        10 │
  │ 1        20 │←RAIN (Rain sensor)
  │ 0        21 │
  └─────────────┘
```

> **Board variant note:** The HW-466AB left side has 8 pins:
> 5V, G, 3V3, GPIO 4, GPIO 3, GPIO 2, GPIO 1, GPIO 0.
> GPIO 4 **is** broken out on this board revision.

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
platformio run               # compile
platformio run -t upload     # compile + flash
platformio device monitor    # open serial monitor
```

The serial monitor shows the assigned IP address on first boot.

### 4. Build Configuration (`platformio.ini`)

```ini
[env:esp32c3-supermini]
platform  = espressif32
board     = esp32-c3-devkitm-1
framework = arduino
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
lib_deps =
    adafruit/Adafruit SSD1306@^2.5.10
    adafruit/Adafruit GFX Library@^1.11.9
```

> The two `build_flags` enable **USB CDC Serial** on the ESP32-C3's built-in
> USB-C port so `Serial.print()` works without a separate UART adapter.

---

## ASCOM Client Integration (N.I.N.A)

1. Open **Equipment → Safety Monitor**
2. Click **Scan for Devices**
3. Select **RG-11 Safety Monitor** and click connect

---

## Endpoints

### Status page (port 80)

| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Live status web page – open in any browser, no port needed |

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
| GET | `/management/apiversions` | Supported API versions |
| GET | `/management/v1/description` | Server description |
| GET | `/management/v1/configureddevices` | Configured devices |

---

## OLED Display Layout

```
┌────────────────────────────┐
│ SAFE                       │  ← size-2 text (16 px)
│ 192.168.1.45               │  ← IP address  ( 8 px)
│ Alpaca:OK  T:3             │  ← status+count( 8 px)
└────────────────────────────┘
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| OLED blank | Check SDA/SCL wiring (SCL=GPIO4, SDA=GPIO3). Do **not** add external pull-up resistors near OLED wires — they can load the I2C bus. The module has onboard pull-ups. Try `OLED_ADDR 0x3D` if heartbeat shows FAIL |
| Serial not showing in monitor | Ensure `ARDUINO_USB_CDC_ON_BOOT=1` build flag is set |
| Device not found in discovery | Confirm device and PC are on same subnet; check UDP port 32227 |
| Always shows UNSAFE | Check pull-up wiring; `digitalRead(20)` should be 1 when dry. GPIOs 1 and 10 have board-level pull-downs — use GPIO 20 |
| WiFi not connecting | Verify `secrets.h` credentials; confirm 2.4 GHz network |

---

## File Structure

```
Alpaca-RG11-ESP32-SafetyMonitor/
├── platformio.ini           # Build config (Arduino, esp32-c3-devkitm-1)
├── src/
│   └── main.cpp             # Full driver (~260 lines)
├── include/
│   ├── secrets.h            # WiFi credentials – git-ignored
│   └── secrets.h.example    # Template – committed to Git
├── .gitignore               # Excludes .pio/, secrets.h
├── hardware-front.jpeg      # Photo of hardware
├── hardware-back.jpeg       # Photo of hardware (board labels)
└── README.md                # This file
```

---

## Version History

### v1.0.0 (2026-02-25)
- Arduino framework, ESP32-C3 SuperMini (HW-466AB)
- 0.91″ SSD1306 OLED display
- WiFi credentials in git-ignored `secrets.h`

---

## License

Open source – use and modify freely for your astronomy setup.
