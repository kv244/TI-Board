# LiDAR Servo Scanner — Project README

## System Overview

A 2D scanning LiDAR system built from three components:

```
Garmin LIDAR-Lite v3
      ↓ I2C
TMS320F28069M LaunchPad  ──UART──→  Arduino UNO R4 WiFi
  + RC Servo (sweep)                      ↓ I2C
                                    0.96" SSD1306 OLED
                                    (2D polar map display)
```

**Files in this package:**

| File | Description |
|---|---|
| `lidar_servo_scan.c` | F28069M firmware — sweeps servo, reads LiDAR, streams data |
| `lidar_display.ino` | Arduino sketch — receives data, renders polar map on OLED |
| `README.md` | This file |

---

## Hardware Required

- TI LAUNCHXL-F28069M LaunchPad
- Garmin LIDAR-Lite v3
- RC hobby servo (any standard 3-wire servo)
- Arduino UNO R4 WiFi
- 0.96" SSD1306 OLED display (I2C, 128x64)
- External 5V power supply (for servo + LiDAR)
- 680µF electrolytic capacitor (for LiDAR power filtering)
- Jumper wires

---

## Wiring Instructions

### 1. LIDAR-Lite v3 → F28069M LaunchPad

| LIDAR Wire | Connects To | Notes |
|---|---|---|
| Red (VCC) | External 5V supply | Place 680µF cap between 5V and GND as close to LiDAR as possible |
| Black (GND) | Common GND | Must share GND with LaunchPad and Arduino |
| Green (SDA) | LaunchPad GPIO32 | I2C data line |
| Blue (SCL) | LaunchPad GPIO33 | I2C clock line |
| Yellow (MODE) | LaunchPad GPIO1 | Hold HIGH = I2C mode. Connect 10K resistor to 3.3V as pull-up |
| Orange (MONITOR) | Not connected | Leave floating |

> ⚠️ The LIDAR-Lite v3 requires 4.5V–5.5V supply. Do NOT power from the LaunchPad 3.3V rail.
> ✅ The I2C lines (SDA/SCL) operate at 3.3V internally — no level shifter needed.

---

### 2. RC Servo → F28069M LaunchPad

| Servo Wire | Connects To | Notes |
|---|---|---|
| Red (VCC) | External 5V supply | Same supply as LiDAR is fine |
| Brown/Black (GND) | Common GND | Must share GND with LaunchPad |
| Orange/Yellow (Signal) | LaunchPad GPIO0 (ePWM1A) | 3.3V signal is sufficient |

> ⚠️ Never power the servo from the LaunchPad 3.3V pin — servos draw too much current.

---

### 3. F28069M LaunchPad → Arduino UNO R4 WiFi

| F28069M Pin | Arduino Pin | Notes |
|---|---|---|
| GPIO28 (SCI-A TX) | Pin 0 (RX) | Direct connection — both 3.3V logic, no level shifter needed |
| GND | GND | Common ground |

> ✅ The Arduino UNO R4 uses the RA4M1 chip at 3.3V logic — safe for direct connection to F28069M.
> ⚠️ Disconnect Arduino Pin 0 before uploading sketches via USB, then reconnect afterwards.

---

### 4. SSD1306 OLED → Arduino UNO R4 WiFi

| OLED Pin | Arduino Pin | Notes |
|---|---|---|
| VCC | 3.3V | The SSD1306 module accepts 3.3V or 5V |
| GND | GND | Common ground |
| SDA | A4 | I2C data |
| SCL | A5 | I2C clock |

---

### 5. Power — Common Ground

All GND pins must be connected together:

```
External 5V Supply GND
        ↓
LaunchPad GND ──── Arduino GND ──── OLED GND
```

---

## Complete Wiring Diagram (Text)

```
External 5V Supply
  (+5V) ──┬──── LIDAR Red (VCC) ──[680µF to GND]
           └──── Servo Red (VCC)
  (GND)  ──┬──── LIDAR Black (GND)
            ├──── Servo Black (GND)
            ├──── LaunchPad GND
            └──── Arduino GND

LaunchPad
  GPIO0  ──── Servo Signal (Orange)
  GPIO1  ──── LIDAR Yellow (MODE) ──[10K pull-up to 3.3V]
  GPIO28 ──── Arduino Pin 0 (RX)
  GPIO32 ──── LIDAR Green (SDA)
  GPIO33 ──── LIDAR Blue (SCL)

Arduino UNO R4 WiFi
  A4     ──── OLED SDA
  A5     ──── OLED SCL
  3.3V   ──── OLED VCC
  GND    ──── OLED GND
```

---

## Software Setup

### F28069M (CCS)

1. Open **Code Composer Studio**
2. In Resource Explorer, select board: `LAUNCHXL F28069M`
3. Create a new CCS project
4. Add `lidar_servo_scan.c` to the project
5. Add required C2000Ware source files (same as LED blink example):
   - `F2806x_CodeStartBranch.asm`
   - `F2806x_CpuTimers.c`
   - `F2806x_DefaultIsr.c`
   - `F2806x_GlobalVariableDefs.c`
   - `F2806x_PieCtrl.c`
   - `F2806x_PieVect.c`
   - `F2806x_SysCtrl.c`
   - `F2806x_usDelay.asm`
6. In Project Properties → General:
   - Connection: `Texas Instruments XDS100v2 USB Debug Probe`
7. Ensure S1 switches on LaunchPad: **S1-1 UP, S1-2 UP, S1-3 UP**
8. Build and Debug (F11)

### Arduino UNO R4 WiFi (Arduino IDE)

1. Open Arduino IDE
2. Select board: **Arduino UNO R4 WiFi**
3. Install libraries via **Sketch → Include Library → Manage Libraries**:
   - `Adafruit SSD1306`
   - `Adafruit GFX Library`
4. Open `lidar_display.ino`
5. **Disconnect Arduino Pin 0 (RX)** from the LaunchPad
6. Upload sketch
7. Reconnect Pin 0 to LaunchPad GPIO28

---

## Configuration — Key `#define` Values

### lidar_servo_scan.c (F28069M)

```c
// Servo pulse width limits — adjust if your servo doesn't reach full travel
#define SERVO_MIN_TICKS  625    // 1.0ms = 0°   — increase if arm hits stop
#define SERVO_MID_TICKS  937    // 1.5ms = 90°  — centre position
#define SERVO_MAX_TICKS  1250   // 2.0ms = 180° — decrease if arm hits stop

// Sweep speed — time between each 1° step
#define STEP_DELAY_MS    30     // 30ms = ~11 second full sweep
                                // Increase for more accurate LiDAR readings
                                // Decrease for faster sweep (less accurate)

// LiDAR I2C address — only change if you have modified the LiDAR address
#define LIDAR_I2C_ADDR   0x62   // Default Garmin LIDAR-Lite v3 address

// UART baud rate — must match Arduino sketch
#define BAUD_RATE        115200UL
```

### lidar_display.ino (Arduino)

```cpp
// OLED I2C address — change to 0x3D if display stays blank
#define OLED_ADDRESS    0x3C

// Maximum distance shown on polar map
#define MAX_DISPLAY_CM  200     // 200cm = 2 metres full scale
                                // 400 = 4 metres, 100 = 1 metre
                                // LiDAR-Lite v3 max range = 4000cm

// Polar map radius in pixels (max = ~30 to fit OLED)
#define MAP_RADIUS      30      // Pixels from origin to outer arc

// Sweep line and scan point display
// Objects beyond MAX_DISPLAY_CM are not plotted (out of range)
// Objects at 0xFFFF (65535) are LiDAR errors — also not plotted
```

---

## Packet Protocol

Each measurement is sent as a **5-byte packet**:

```
Byte 1: 0xFF          — Sync byte (marks start of packet)
Byte 2: angle         — Servo angle 0–180 degrees
Byte 3: dist_hi       — Distance high byte (cm)
Byte 4: dist_lo       — Distance low byte (cm)
Byte 5: checksum      — angle XOR dist_hi XOR dist_lo
```

The Arduino verifies the checksum before plotting. Corrupted packets are silently discarded.

**Example:** Angle=90°, Distance=150cm (0x0096)
```
0xFF  0x5A  0x00  0x96  0xCC
sync  90°   hi    lo    checksum (0x5A ^ 0x00 ^ 0x96 = 0xCC)
```

---

## Troubleshooting

| Problem | Likely Cause | Fix |
|---|---|---|
| CCS Error -1135 | S1-3 switch is OFF | Flip S1-3 to UP position |
| OLED stays blank | Wrong I2C address | Change `OLED_ADDRESS` to `0x3D` |
| No data on OLED | Pin 0 still disconnected | Reconnect GPIO28 → Arduino Pin 0 after upload |
| LiDAR always returns 0xFFFF | I2C wiring issue | Check GPIO32/33 connections and 10K pull-up on GPIO1 |
| Servo doesn't reach full travel | Pulse width out of range | Adjust `SERVO_MIN_TICKS` / `SERVO_MAX_TICKS` |
| Servo jitters | Insufficient power | Use a dedicated 5V supply, not USB power bank |
| Scan points scattered | Servo moving too fast | Increase `STEP_DELAY_MS` to 50 or more |

---

## Expected Behaviour

Once everything is running:

1. The servo sweeps smoothly left and right (0°–180°–0°)
2. At each degree, the LiDAR fires and measures distance
3. The F28069M sends a packet to the Arduino every ~30ms
4. The OLED updates with each new reading, building up a polar map
5. Objects in the scan path appear as dot clusters on the map
6. The sweep line rotates in sync with the physical servo

---

*Built with TMS320F28069M (C2000 DSP), Garmin LIDAR-Lite v3, Arduino UNO R4 WiFi*
