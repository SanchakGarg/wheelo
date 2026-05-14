# Wheelo: Scissored CMG Balancing Robot

Wheelo is an open-source, single-axis balancing robot that uses a **Scissored Control Moment Gyroscope (CMG)** system for stabilization. Unlike traditional balancing robots that use wheel acceleration, Wheelo stays upright by harnessing gyroscopic precession torque.

## Videos

| Demo | Working Principle |
|------|-------------------|
| [![Wheelo Demo](https://img.youtube.com/vi/aaRE76FrkvM/hqdefault.jpg)](https://www.youtube.com/watch?v=aaRE76FrkvM) | [![Wheelo Working](https://img.youtube.com/vi/aowIbTuXIWc/hqdefault.jpg)](https://www.youtube.com/watch?v=aowIbTuXIWc) |

## Key Features

- **Scissored CMG Design:** Two counter-rotating flywheels geared together to provide pure torque and cancel parasitic yaw/roll.
- **XIAO ESP32-C3 Core:** Compact, powerful microcontroller with WiFi/OTA capabilities.
- **High-Speed Actuation:** ST3215 1 Mbps Serial Bus Servo for precise gimbal control.
- **Advanced Filtering:** Mahony filter with Median and Low-Pass stages for stable orientation sensing.
- **Web Dashboard:** Real-time PID tuning, telemetry, and calibration via a built-in web server.
- **OTA Updates:** Flash new firmware over WiFi — no USB cable needed after first flash.

## Repository Structure

```
Wheelo/
├── firmware/main/          # PlatformIO ESP32-C3 project
│   ├── platformio.ini      # Build environments (usb + ota)
│   └── src/
│       ├── config.h        # All pin assignments and tunable constants
│       ├── main.cpp        # Entry point — wires up modules, runs 10 ms PID loop
│       ├── BalancePID.cpp/h     # XRobots-V2 style angle PID with setpoint accumulator
│       ├── MPUSensor.cpp/h     # MPU6050/6500 driver + Mahony filter
│       ├── ServoController.cpp/h  # ST3215 half-duplex UART servo
│       ├── ESCController.cpp/h # BLDC ESC PWM driver (capped at 30%)
│       └── WebUI.cpp/h         # HTTP server — dashboard, PID tuning, telemetry
├── hardware/
│   └── Gyroscope Assembly.3mf  # 3D print files (Bambu/PrusaSlicer)
├── tools/
│   ├── sim_filter.py       # Off-robot Mahony filter simulation against recorded data
│   └── telemetry.py        # Real-time WiFi telemetry logger
└── docs/
    └── sim_filter_results.png  # Filter validation output
```

## Hardware

### Components

| Role | Part |
|------|------|
| MCU | Seeed Studio XIAO ESP32-C3 |
| IMU | MPU6050 / MPU6500 |
| Gimbal Servo | Waveshare ST3215 (Serial Bus Servo, 1 Mbps) |
| Flywheel Motor | BLDC Motor + ESC (2S–3S LiPo) |

### Pin Map

| Signal | GPIO | Board Label |
|--------|------|-------------|
| ESC PWM | 10 | D10 |
| Servo TX | 20 | D7 |
| Servo RX | 21 | D6 |
| MPU SDA | 6 | D4 |
| MPU SCL | 7 | D5 |

### CAD

The mechanical design is available on Onshape: **[Wheelo CAD](https://cad.onshape.com/documents/1c7d4ff26d1dc69175c3ae98/w/5cd2b66e5aeb4751e0f29f10/e/76622d04818cc6738350398c)**

## Firmware Architecture

The firmware is split into six modules, each a self-contained C++ class:

| Module | File | Responsibility |
|--------|------|----------------|
| **BalancePID** | `BalancePID.cpp` | XRobots-V2 direct-position PID. Outputs gimbal position commands. Includes setpoint accumulator to absorb chassis lean bias and prevent gimbal saturation. |
| **MPUSensor** | `MPUSensor.cpp` | Reads MPU6050/6500 over I2C. Applies a Mahony complementary filter plus median and low-pass stages. Exposes `pitch` and `gyroY` in degrees. |
| **ServoController** | `ServoController.cpp` | Drives the ST3215 over half-duplex UART at 1 Mbps. Reads back live gimbal position every 50 ms for closed-loop velocity computation. |
| **ESCController** | `ESCController.cpp` | Generates standard 1000–2000 µs PWM for the BLDC ESC. Throttle is capped at 30% for safety. |
| **WebUI** | `WebUI.cpp` | Serves a single-page dashboard on port 80. Endpoints for PID gains, servo manual control, IMU calibration, balance start/stop, and live telemetry. |
| **main** | `main.cpp` | Wires modules together, connects to WiFi, starts OTA, and runs a 10 ms PID tick in `loop()`. |

### PID Loop

The balance loop runs at **100 Hz** (every 10 ms). Each tick:
1. `mpu.read()` — fetches and filters a new IMU sample.
2. `pid.compute()` — runs the PID, moves the gimbal via `servo`.
3. `servo.readPos()` — runs at 50 ms cadence, keeps the position feedback fresh.

### Setpoint Accumulator

The setpoint accumulator (`accumEnabled`) slowly drifts the balance target toward the chassis's natural lean angle. This prevents the gimbal from parking at the edge of its travel and saturating on any second disturbance.

## Firmware Setup

### Prerequisites

- [PlatformIO Core](https://platformio.org/) (CLI or VS Code extension)
- Python 3.x (for `tools/`)

### Configuration

Edit `firmware/main/src/config.h` before building:

```cpp
#define WIFI_SSID     "YourNetwork"
#define WIFI_PASS     "YourPassword"
#define OTA_PASSWORD  "wheelo123"   // change for production
```

### Build & Flash

**First flash (USB):**
```bash
cd firmware/main
pio run -e usb -t upload
```

**Subsequent flashes (OTA — robot must be on WiFi):**
```bash
pio run -e ota -t upload
```

Serial monitor at 115 200 baud:
```bash
pio device monitor
```

## Web Dashboard

Once connected to WiFi, the robot advertises itself as `wheelo.local`. Open `http://wheelo.local` in a browser.

Available controls:
- **PID gains** (Kp / Ki / Kd / Trim) — live update with save to NVS
- **Setpoint Accumulator** toggle
- **Servo manual position** control
- **IMU calibration** (zero gyro bias, noise floor measurement)
- **Balance start / stop**
- **Live telemetry** (angle, gimbal position, PID output)

## Tools

| Script | Usage |
|--------|-------|
| `tools/sim_filter.py` | Replay recorded IMU CSV through the Mahony filter off-robot to validate tuning. |
| `tools/telemetry.py` | Connect over WiFi and stream live telemetry to a CSV for post-analysis. |

## License

This project is open-source under the MIT License. Feel free to build, modify, and share!
