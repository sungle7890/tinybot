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

### Bumper switches (snap-action switch · Pololu 1405)

| State | Result |
|---|---|
| At rest | `ok (reason 0x00)` ✅ |
| Left pressed (A3) | `TRIPPED (reason 0x01)` · `mode : SAFETY-STOP` ✅ |
| Right pressed (A4) | `TRIPPED (reason 0x02)` ✅ |
| Latch | Holds after release, cleared with `c` ✅ |

Left and right are distinguished. Nothing trips at rest, so both are wired
`COM–NO` correctly (an `NC` wire would read as permanently pressed). The right
one did not respond on the first attempt and worked after the wiring was
re-seated, which points to a loose contact. **Check that jumper pins are pushed
fully home.**

### Control-loop jitter (telemetry on, 60 s)

| Configuration | Tick period | Min / max | Overruns | Telemetry |
|---|---|---|---|---|
| 1 distance sensor | 20000.0 ± 2.3 µs | 19995 / 20005 µs | 0 | 3001 lines, 0 dropped |
| 3 distance sensors | 20000.0 ± 2.4 µs | 19994 / 20005 µs | 0 | 3001 lines, 0 dropped |

Not yet re-measured with the IMU added. Final measurement at stage 7 with motors attached.

### Wi-Fi link (2026-09-16)

Commands and telemetry now go over HTTP through the R4's onboard radio. It joins
the house network, answers 6/6 consecutive requests, and responds in 130-630 ms.

**Each call into the radio costs 7-17 ms** (measured with the `net` command):

| Call | Worst |
|---|---|
| accept | 6.8 ms |
| connected/available | 11.8 ms |
| read | 13.8 ms |
| write 128 B | 16.6 ms |

With a 20 ms control period on a single core, the ±2 ms budget cannot hold while
the radio is in use. Measured while streaming telemetry: 20491 ± 3972 µs, max
48 ms, **225 of 2928 ticks over budget - FAIL**. Chunked writes and bulk reads
barely moved it (230 → 225).

**What this affects**
- A handful of commands (`d 1000`, `e`, `st`) is fine
- **Encoders count in interrupts, so calibration accuracy is unaffected by jitter**
- Continuous telemetry streaming keeps pushing the loop late

**Real fix (Phase 2)**: move the control step into a hardware timer interrupt and
take the sensor I2C reads out of it, so blocking in loop() cannot delay a tick.

### Found and fixed
- `%f` printed blank on the R4 → `-Wl,-u,_printf_float`
- All telemetry dropped: the Renesas UART's `availableForWrite()` always returns 0 → decide by time left before the next tick
- Default uploader bossac is x86_64-only → pyOCD

**Stage 1 passed.** All four sensors and both bumpers verified.

## 2026-09-16 — Phase 1 complete (motors, encoders, gates)

### Motors and encoders
- Both directions and duty control work. The pack is 9 V against motors rated
  3-6 V, so a **duty ceiling of 666/1000** now caps every command
- Getting the encoder signs right took several attempts: each time the wiring was
  touched, one side died or flipped. **The jumper contacts are vibration-prone.**
  Swapping the right encoder's `A`/`B` finally gave both sides positive
- A dead encoder mid-drive once **made the robot drive away**: the firmware
  believed the bad reading and kept commanding forward. Guards added (375acea)

### Gate ① odometry calibration — ✅ PASS
```
commanded 1.000 m, measured 1.016 m (40 in), error +1.6%
counts/m 3274.0 -> 3222.4, saved to EEPROM
```
The figure derived from the datasheet was within 1.6% of reality.

### Gate ② control-loop jitter — ✅ PASS
Motors and encoders attached, Wi-Fi up, telemetry over USB for 60 s:
```
mean 20000.6 ± 30.8 µs · min/max 19989/21676 · overruns 0 · 2945 telemetry lines
```

It first measured 6 overruns at ±486 µs. The cause was not the motors but
**polling the radio for waiting connections**, one call of which costs up to
6.8 ms. Limiting that poll to once per 100 ms took overruns to zero and the
deviation from 486 to 31 µs.

### Open problem: it curves right
The 1 m run ended **about 10 cm to the right** (~5.7°), while the encoders
disagreed by only 0.2%. Today's heading hold equalises wheel counts, so it
cannot see this: unequal effective wheel diameter, slip, or caster drag. The fix
is **heading correction from the gyro**, kept for Phase 2.

**Phase 1 complete.**
