# CanSat 2026

A student CanSat project for the 2026 competition. The system consists of a CanSat satellite unit that collects and transmits environmental and navigation telemetry, a ground station receiver that captures the radio packets, and a Python dashboard for real-time display and data logging.

---

## Repository Structure

```
Onelastdecent_CanSat2026/
├── Hardware/
│   ├── CanSat 2026.step          # CanSat mechanical design
│   └── Ground Station.step       # Ground station enclosure design
└── Software/
    ├── CanSat_Main_Transmitter/  # CanSat onboard firmware (Pico 2 W)
    └── CanSat_Main_Reciever/     # Ground station firmware (Pico 2) + dashboard
        └── dashboard/            # Python ground station dashboard
```

---

## System Overview

### CanSat (Transmitter)

**Hardware**
| Component | Part | Interface |
|-----------|------|-----------|
| Microcontroller | Raspberry Pi Pico 2 W (RP2350) | — |
| Environmental sensor | BME680 | I2C0 — SDA GP4, SCL GP5 |
| Radio | E220-900T30D (LoRa 915 MHz, 30 dBm) | UART0 — TX GP0, RX GP1, M0 GP2, M1 GP3, AUX GP6 |
| Flight controller | MicoAir743V2 (ArduPilot) — GPS + IMU | UART1 — TX GP8, RX GP9 |
| Status LED | WS2812B NeoPixel | PIO — GP7 |

The CanSat firmware runs at 1 Hz. Each loop it reads the BME680, polls the flight controller for GPS position and attitude over MAVLink, builds a CSV telemetry packet, and transmits it via the E220 radio. Sensor values are also logged to the flight controller's DataFlash via `NAMED_VALUE_FLOAT` MAVLink messages.

**NeoPixel status codes**

| Colour | Meaning |
|--------|---------|
| Green | All sensors valid, radio OK |
| Yellow | BME680 invalid, radio OK |
| Orange | Radio transmit timeout |
| Red | BME680 not found at boot |

---

### Ground Station (Receiver)

**Hardware**
| Component | Part | Interface |
|-----------|------|-----------|
| Microcontroller | Raspberry Pi Pico 2 (RP2350) | — |
| Radio | E220-900T30D (LoRa 915 MHz, 30 dBm) | UART0 — TX GP0, RX GP1, M0 GP2, M1 GP3, AUX GP6 |
| GPS | HGLRC M100-5883 | UART1 — TX GP4, RX GP5 |
| Status LED | WS2812B NeoPixel | PIO — GP7 |
| Onboard LED | GP25 | GPIO |

The receiver firmware forwards received `$CANSAT` packets verbatim to USB serial. It also parses NMEA GGA sentences from the ground station GPS and emits `$GSPOS` lines to USB so the dashboard can display the ground station's own position and calculate distance/bearing to the CanSat.

**NeoPixel status codes**

| Colour | Meaning |
|--------|---------|
| Green flash | Packet received |
| Red flash | No packet in the last 2 s |
| Solid orange | E220 configuration in progress |

---

## Telemetry Packet Format

All packets are newline-terminated ASCII sent over USB serial at any baud rate.

**CanSat telemetry**
```
$CANSAT,<ms>,<temp>,<press>,<hum>,<gas>,<lat_e7>,<lon_e7>,<alt_mm>,<roll>,<pitch>,<yaw>,<fc_bytes>,<stx_v1>,<stx_v2>
```

| Field | Unit | Notes |
|-------|------|-------|
| `ms` | ms | Time since boot |
| `temp` | °C | BME680 temperature (PCB offset applied) |
| `press` | hPa | Barometric pressure |
| `hum` | % | Relative humidity |
| `gas` | Ω | Gas resistance |
| `lat_e7` / `lon_e7` | deg × 10⁷ | GPS position from ArduPilot FC |
| `alt_mm` | mm | GPS altitude |
| `roll` / `pitch` / `yaw` | deg | Attitude from ArduPilot FC |
| `fc_bytes` / `stx_v1` / `stx_v2` | — | FC UART debug counters |

**Ground station GPS**
```
$GSPOS,<lat_e7>,<lon_e7>,<alt_mm>,<hdop_100>,<fix>
```

**Radio frequency broadcast** (sent by CanSat every ~10 s)
```
$FREQ,<freq_mhz>,<channel>
```

---

## Radio Configuration

Both E220 modules are configured at boot to:

| Parameter | Value |
|-----------|-------|
| Frequency | 915.125 MHz (channel 65) |
| Air data rate | 2.4 kbps |
| UART baud | 9600 |
| TX power | 30 dBm |
| Sub-packet size | 200 B |
| Mode | Transparent |

---

## Ground Station Dashboard

A Python/Tkinter application providing real-time telemetry display, live graphs, a GPS map view, and CSV data recording.

### Setup

```bash
cd Software/CanSat_Main_Reciever/dashboard
pip install -r requirements.txt
python3 ground_station.py
```

**Dependencies:** `pyserial`, `matplotlib`, `tkintermapview` (optional — required for tile map view)

### Features

- Connect to any serial port at 9600 or 115200 baud
- Live telemetry panel: environmental, navigation, and attitude data
- Six real-time graphs: temperature, pressure, humidity, altitude, orientation, gas resistance
- GPS map tab with tile map (online) and simple offline scatter plot
- Ground station GPS overlay showing position, distance, and bearing to CanSat
- Radio frequency badge updated automatically from `$FREQ` packets
- CSV recording and export
- Signal loss overlay when no packets are received for > 5 s

---

## Firmware Build Instructions

Both firmware projects use the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) and are configured for VS Code with the Pico extension.

**Prerequisites:** CMake ≥ 3.13, ARM GCC toolchain, Pico SDK 2.2.0

```bash
cd Software/CanSat_Main_Transmitter   # or CanSat_Main_Reciever
mkdir build && cd build
cmake ..
make -j4
```

Flash the resulting `.uf2` file by holding BOOTSEL on the Pico while connecting via USB, then copying the file to the mounted drive.
