# 🐭 MicroMouse — IEEE Maze Solver

An autonomous maze-solving robot firmware for **ESP32-S3**, built to navigate a standard IEEE 16×16 micromouse maze using the **Flood Fill algorithm**. The robot discovers walls with three VL53L0X Time-of-Flight sensors and finds the shortest path to the center.

---

## 📋 Table of Contents

- [How It Works](#-how-it-works)
- [Hardware](#-hardware)
- [Pin Assignment](#-pin-assignment)
- [Software Architecture](#-software-architecture)
- [Getting Started](#-getting-started)
- [Calibration](#-calibration)
- [Configuration](#-configuration)
- [Testing Order](#-testing-order)
- [Troubleshooting](#-troubleshooting)
- [Known Assumptions](#-known-assumptions)

---

## 🧠 How It Works

The robot operates in two phases:

### Phase 1 — Exploration
The robot starts at cell `(0, 0)` facing North. It uses three ToF sensors (left, front, right) to detect walls, updates its internal map, and runs a BFS flood fill to determine the next best move toward the goal zone (cells `[7–8][7–8]`). Once the goal is reached, it returns to start.

### Phase 2 — Speed Run
With the maze fully mapped, the robot runs the optimal path at ~40% higher speed.

```
┌─────────┐     ┌─────────┐     ┌─────────┐     ┌──────────┐     ┌──────────┐
│  IDLE   │────▶│ EXPLORE │────▶│ RETURN  │────▶│ SPEEDRUN │────▶│   DONE   │
│ (wait)  │     │ (map)   │     │ (go back)│    │ (fast!)  │     │ (flash!) │
└─────────┘     └─────────┘     └─────────┘     └──────────┘     └──────────┘
     │
     │  Serial 'C'
     ▼
┌───────────┐
│ CALIBRATE │
│ (tune)    │
└───────────┘
```

### Key Algorithms & Techniques

| Feature | Implementation |
|---|---|
| Pathfinding | BFS Flood Fill on 16×16 grid |
| Wall storage | 4-bit bitmask per cell (N/E/S/W) with auto-mirroring |
| Speed control | PID per motor (setpoint = counts/tick) |
| Wall centering | Steering PID using ToF sensor differential |
| Single-wall follow | Maintains 90mm from one wall when only one is visible |
| Heading correction | Square-up scan (±5° sweep to find perpendicular to front wall) |
| Motion profile | Trapezoidal (20mm accel ramp + 20mm decel ramp) |
| Encoder decoding | Hardware PCNT (ESP32Encoder) — zero CPU load |

---

## 🔧 Hardware

| Component | Model / Spec | Notes |
|---|---|---|
| Microcontroller | ESP32-S3 N16R8 | 16 MB Flash, 8 MB PSRAM |
| Motor Driver | TB6612FNG | Dual H-bridge |
| Motors | N20 with encoder | 200:1 gear ratio, 3 PPR motor-shaft |
| Battery | 1500mAh 12V 3S LiPo | Powers everything |
| Buck Converter | LM2596 | Steps 12V → 5V |
| ToF Sensors (×3) | VL53L0X | Left, Front, Right on shared I2C |

### Power Architecture

```
LiPo 12V ──▶ LM2596 ──▶ 5V ──▶ TB6612FNG (VM)
                              ──▶ ESP32-S3 (Vin → onboard 3.3V reg)
                                      │
                                      ▼ 3.3V
                              ┌───────┼───────┐
                              │       │       │
                           VL53L0X VL53L0X VL53L0X
                           (Left)  (Front) (Right)
```

### Robot Dimensions

| Parameter | Value |
|---|---|
| Wheel diameter | 44 mm |
| Wheel base (center-to-center) | 92 mm |
| Gear ratio | 200:1 |
| Encoder PPR (motor shaft) | 3 |
| Counts per wheel revolution | 2400 |
| Distance per encoder count | ~0.0576 mm |

---

## 📌 Pin Assignment

```
 ┌──────────────────────────────────────────┐
 │            ESP32-S3 GPIO Map             │
 ├──────────────────────────────────────────┤
 │  Motor Driver (TB6612FNG)                │
 │    GPIO  1  ── PWMA  (Left speed)        │
 │    GPIO  2  ── AIN1  (Left dir)          │
 │    GPIO 15  ── AIN2  (Left dir)          │
 │    GPIO  4  ── PWMB  (Right speed)       │
 │    GPIO  5  ── BIN1  (Right dir)         │
 │    GPIO  6  ── BIN2  (Right dir)         │
 │    GPIO  7  ── STBY  (Enable)            │
 │                                          │
 │  Encoders                                │
 │    GPIO  8  ── Left  Ch-A                │
 │    GPIO  9  ── Left  Ch-B                │
 │    GPIO 10  ── Right Ch-A                │
 │    GPIO 11  ── Right Ch-B                │
 │                                          │
 │  VL53L0X XSHUT                           │
 │    GPIO 12  ── Left  (addr 0x30)         │
 │    GPIO 13  ── Front (addr 0x31)         │
 │    GPIO 14  ── Right (addr 0x32)         │
 │                                          │
 │  I2C Bus                                 │
 │    GPIO 17  ── SDA                       │
 │    GPIO 18  ── SCL                       │
 │                                          │
 │  User Interface                          │
 │    GPIO  0  ── Boot button (start)       │
 │    GPIO 48  ── NeoPixel LED              │
 └──────────────────────────────────────────┘
```

> ⚠️ **Note:** GPIO 3 was intentionally avoided for motor control — it is an ESP32-S3 strapping pin that can cause boot failures.

---

## 🏗 Software Architecture

```
micromouse/
├── micromouse.ino           ← Arduino entry point
├── config.h                 ← All constants & pin defines
├── motor.h / motor.cpp      ← TB6612FNG + LEDC PWM
├── encoder.h / encoder.cpp  ← ESP32Encoder (PCNT hardware)
├── tof.h / tof.cpp          ← VL53L0X init + readings
├── pid.h / pid.cpp          ← Generic PID controller
├── motion.h / motion.cpp    ← Forward / turn / square-up
├── maze.h / maze.cpp        ← Wall map + BFS flood fill
├── solver.h / solver.cpp    ← State machine & navigation
└── led.h / led.cpp          ← NeoPixel status animations
```

### Module Dependency Graph

```
config.h ──────┬──▶ motor ──────┐
               ├──▶ encoder ────┤
               ├──▶ tof ────────┼──▶ motion ──┐
               ├──▶ pid ────────┘             │
               └──▶ led ──────────────────────┼──▶ solver ──▶ main.ino
                                              │
                              maze ───────────┘
```

### LED Status Indicators

| State | LED Pattern |
|---|---|
| IDLE | Slow white pulse (breathing) |
| CALIBRATE | Slow magenta pulse |
| EXPLORE | Solid blue |
| RETURN | Solid yellow |
| SPEEDRUN | Solid green |
| DONE | Rainbow flash → solid green |
| Boot error | Fast red blink (ToF init failed) |

---

## 🚀 Getting Started

### Prerequisites

1. **Arduino IDE 2.x** (or later)
2. **ESP32 Board Package** — Install via Boards Manager:
   - URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - Package: `esp32` by Espressif Systems

3. **Libraries** — Install via Arduino Library Manager:

   | Library | Author | Required |
   |---|---|---|
   | VL53L0X | Pololu | ✅ |
   | Adafruit NeoPixel | Adafruit | ✅ |
   | ESP32Encoder | Kevin Harrington | ✅ |

### Board Configuration

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| PSRAM | OPI PSRAM |
| Flash Size | 16MB |
| USB CDC On Boot | Enabled |
| Upload Speed | 921600 |

### Upload & Run

1. Open `micromouse/micromouse.ino` in Arduino IDE
2. Select the board and port
3. Click **Upload**
4. Open Serial Monitor at **115200 baud**
5. You should see the boot banner:
   ```
   ╔══════════════════════════════════╗
   ║       MicroMouse  v1.0           ║
   ║  ESP32-S3 · Flood Fill Solver    ║
   ╚══════════════════════════════════╝
   [BOOT] Motors OK
   [BOOT] Encoders OK
   [BOOT] I2C OK
   [TOF] Left  sensor → 0x30
   [TOF] Front sensor → 0x31
   [TOF] Right sensor → 0x32
   [BOOT] ToF OK
   [BOOT] ✓ Ready. Press BOOT to start or send 'C' for calibration.
   ```
6. **Press the BOOT button** to start exploration, or **send `C`** for calibration mode.

---

## 🔬 Calibration

Enter calibration mode by sending `C` over Serial while the robot is in IDLE state.

### Calibration Menu

```
╔══════════════════════════════════╗
║      CALIBRATION MODE            ║
╠══════════════════════════════════╣
║  1 — Forward 1 cell (180mm)      ║
║  2 — 360° rotation (4×right)     ║
║  3 — Print encoder/speed info    ║
║  4 — Print ToF readings          ║
║  5 — Test square-up              ║
║  Q — Quit calibration → IDLE     ║
╚══════════════════════════════════╝
```

### Calibration Procedure

| Step | Command | What to Check | Constant to Adjust |
|---|---|---|---|
| 1 | `1` | Robot moves exactly 180mm (measure with ruler) | `GEAR_RATIO` |
| 2 | `2` | Robot completes exact 360° (ends at same heading) | `WHEEL_BASE_MM` |
| 3 | `3` | Print computed constants for verification | — |
| 4 | `4` | Place wall at 80mm, verify `wall = true` | `TOF_WALL_PRESENT` |
| 5 | `5` | Robot aligns perpendicular to front wall | `SQUARE_UP_SCAN_STEPS` |

After calibrating, send `Q` to return to IDLE, then press BOOT to start.

---

## ⚙️ Configuration

All tunable constants are in [`config.h`](micromouse/config.h). Key parameters:

### Physical (measure and update first!)

```cpp
#define WHEEL_DIAMETER_MM   44.0f    // Measure your wheel
#define WHEEL_BASE_MM       92.0f    // Center-to-center of wheels
#define GEAR_RATIO          200.0f   // From motor datasheet
```

### PID Gains (tune on hardware)

```cpp
// Speed PID (per motor)
#define PID_SPEED_KP        2.0f
#define PID_SPEED_KI        0.5f
#define PID_SPEED_KD        0.1f

// Steering PID (wall centering)
#define PID_STEER_KP        0.3f
#define PID_STEER_KI        0.0f
#define PID_STEER_KD        0.05f
```

### Speed

```cpp
#define TARGET_SPEED_COUNTS     13.0f   // Exploration speed (counts/tick)
#define SPEEDRUN_SPEED_COUNTS   18.0f   // Speed-run (~40% faster)
```

### Wall Detection

```cpp
#define TOF_WALL_PRESENT    100    // Wall present if < 100mm
#define TOF_NO_WALL         130    // No wall if > 130mm
```

---

## 🧪 Testing Order

**Follow this order strictly. Do not skip steps.**

| # | Test | What to Verify |
|---|---|---|
| 1 | `motor.cpp` | Both motors spin forward/reverse/brake via calibration menu |
| 2 | `encoder.cpp` | Roll wheels by hand → print counts → verify direction sign |
| 3 | `motion.cpp` (forward) | Move 180mm → measure with ruler → adjust `GEAR_RATIO` |
| 4 | `motion.cpp` (turns) | 4× right turn → ends at same heading → adjust `WHEEL_BASE_MM` |
| 5 | `tof.cpp` | All 3 addresses init → print readings → verify wall thresholds |
| 6 | `pid.cpp` | Straight line holds → tune speed PID, then steering PID |
| 7 | `maze.cpp` | Flood fill serial print with known maze (no motors needed) |
| 8 | `solver.cpp` | Run in a small physical maze or hallway |
| 9 | **Full integration** | Complete maze exploration + speed run |

---

## 🔥 Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| LED blinks red on boot | ToF sensor init failed | Check I2C wiring (SDA=17, SCL=18), verify 3.3V power |
| Robot won't boot / enters download mode | Strapping pin conflict | Verify GPIO 3 is NOT connected to motor driver |
| Motors don't spin | STBY pin not HIGH | Check GPIO 7 connection to TB6612 STBY |
| Robot overshoots turns | `WHEEL_BASE_MM` too small | Increase value, re-test with 4× right turn |
| Robot undershoots turns | `WHEEL_BASE_MM` too large | Decrease value |
| Forward distance wrong | `GEAR_RATIO` incorrect | Check motor datasheet, use calibration step 1 |
| Encoder counts = 0 | Encoder wires swapped or disconnected | Check GPIO 8-11, try `ESP32Encoder` example sketch |
| Robot oscillates in corridor | Steering PID too aggressive | Reduce `PID_STEER_KP`, increase `PID_STEER_KD` |
| `65535` from ToF sensor | Sensor timeout / no wall | Normal for open space — treated as "no wall" |
| Compile error: `ledcSetup` not found | ESP32 Arduino Core v3.x | Code auto-detects — ensure `esp32` board package is updated |

---

## 📐 Known Assumptions

| # | Assumption | Action |
|---|---|---|
| 1 | Encoder 3 PPR is on **motor shaft** (before gearbox) | If on output shaft, set `GEAR_RATIO = 1.0` |
| 2 | Maze is standard IEEE 16×16 with 180mm cells | Confirm competition rules |
| 3 | Robot starts at cell (0,0) facing North | Confirm starting orientation |
| 4 | LM2596 output is 5V | Verify with multimeter before first power-on |
| 5 | No IMU/gyroscope available | Turns rely on encoder counts + square-up correction |

---

## 📄 License

This project is for educational and competition use.

---

*Built with ❤️ for IEEE MicroMouse competition.*
