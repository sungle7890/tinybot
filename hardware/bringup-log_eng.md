# Bring-up log

> Korean: [bringup-log.md](bringup-log.md)

Only values actually measured on hardware. Estimates are marked as estimates.

## 2026-09-15 — Stage 1: sensors (USB power, no motors)

**Board**: Arduino UNO R4 WiFi. Uploaded with pyOCD (CMSIS-DAP), bootloader kept.

### Distance sensors (VL53L0X · Adafruit 3317, six-pin STEMMA QT revision)

| Sensor | XSHUT | Detected | Hand test | Cover test |
|---|---|---|---|---|
| Front | A0 | ✅ | ✅ | 49–57 mm when covered; the other two unchanged |
| Left | A1 | ✅ | ✅ | 43–52 mm when covered; the other two unchanged |
| Right | A2 | ✅ | ✅ | 48–53 mm when covered; the other two unchanged |

XSHUT sequential readdressing works for all three. Covering one leaves the others
unchanged, so they are distinct sensors. Out of range reads 8190 mm.

### 6-axis IMU (MPU-6050 · Adafruit 3886)

- `WHO_AM_I` = `0x68` ✅
- At rest (flat, chip up, 10 reads): x −0.025 g, y −0.013 g, **z +0.871 g**, rotation +0.11 °/s, shock 3 mg
- Flipped (chip down, 10 reads): **z −1.175 g**
- Derived: **z offset −0.152 g**, **sensitivity 1.023**
  - If absolute acceleration is ever needed: `z_true = (z + 0.152) / 1.023`
  - No effect on impact detection (deviation from baseline) or the gyro
- A cable propped the board about 20° at first, giving x −0.33 g. **Mount it flat**

### Impact threshold (0.6 g) — ⚠️ not validated

Three desk taps, 40 s recorded at 50 Hz: **one event ≥ 50 mg (peak 68 mg)**, none above 0.6 g.

Likely cause (estimate): the IMU is read once per 20 ms control tick and its DLPF is
44 Hz, so a vibration lasting a few ms falls between samples or is filtered out. A
real collision decelerates over tens of ms and may behave differently. **Re-tune at
stage 7 by driving the robot into a wall.** If still missed, consider multiple reads
between ticks or the MPU-6050 motion-detect interrupt. Bumpers are the primary
collision detector.

### Control-loop jitter (telemetry on, 60 s)

| Configuration | Tick period | Min / max | Overruns | Telemetry |
|---|---|---|---|---|
| 1 distance sensor | 20000.0 ± 2.3 µs | 19995 / 20005 µs | 0 | 3001 lines, 0 dropped |
| 3 distance sensors | 20000.0 ± 2.4 µs | 19994 / 20005 µs | 0 | 3001 lines, 0 dropped |

Not yet re-measured with the IMU added. Final measurement at stage 7 with motors attached.

### Found and fixed
- `%f` printed blank on the R4 → `-Wl,-u,_printf_float`
- All telemetry dropped: the Renesas UART's `availableForWrite()` always returns 0 → decide by time left before the next tick
- Default uploader bossac is x86_64-only → pyOCD

**Stage 1 passed.**
