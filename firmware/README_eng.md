# firmware — Phase 1 (hardware bring-up)

> Korean: [README.md](README.md)

No learning code here.
Phase 1 answers exactly one question: **are these sensors and this odometry good
enough to trust as a reward signal later?**

## Two targets

Every platform difference lives behind `src/hal/`; everything above it is shared.

| Environment | Board | Status | Control loop |
|---|---|---|---|
| `uno_r4_wifi` | Arduino UNO R4 WiFi | **Default. What we are using** | Cooperative, off `micros()` from `loop()` |
| `esp32s3` | ESP32-S3-DevKitC-1 | Deferred to Phase 4 | RTOS task pinned to core 0 |

```bash
pio run                      # build the default target (R4)
pio run -e esp32s3           # build for the ESP32-S3
pio run -t upload            # flash
pio device monitor           # console at 115200
```

> **⚠️ Apple Silicon.** The `renesas-ra` platform's default toolchain (1.70201.0)
> ships macOS x86_64 binaries only and dies with `Bad CPU type in executable`.
> `platformio.ini` pins `~1.100301.0`, the oldest release with a darwin_arm64
> build.

> **⚠️ R4 uploads go through pyOCD (one-time setup).** The default uploader,
> bossac, is also x86_64-only and will not run without Rosetta, and OpenOCD has
> no RA4M1 flash driver. Upload instead through the board's onboard CMSIS-DAP
> probe with pyOCD and Renesas' pack:
> ```bash
> uv tool install pyocd
> pyocd pack install r7fa4m1ab
> ```
> After that, `pio run -t upload` erases and writes by sector from `0x4000`
> only, leaving the bootloader (`0x0000`–`0x3FFF`) and the EEPROM data flash
> untouched. Verified on hardware 2026-09-15.

## Execution model

| | UNO R4 WiFi | ESP32-S3 |
|---|---|---|
| Cores | 1 | 2 |
| Control step | Cooperative, from `loop()` | Its own RTOS task on core 0 |
| Console + telemetry | The same `loop()` | Core 1 |
| Consequence | **Blocking `loop()` slips a tick** | Serial cannot touch control |

The R4 has one core, so physical separation is impossible. Instead **everything
in `loop()` must be non-blocking** — telemetry printing checks
`Serial.availableForWrite()` first and **drops** the line if the buffer is too
full (and counts the drop).

Losing telemetry is acceptable; a late control tick is not —
[docs/02-architecture_eng.md](../docs/02-architecture_eng.md) design rule 1.

> **Whether cooperative scheduling holds the ±2 ms budget is a measurement, not
> an assumption.** The `j` command decides. If it fails, swap
> `controlLoopBegin` in `hal_r4.cpp` for an FspTimer implementation — nothing
> outside that file changes.

## Console commands

| Command | Effect |
|---|---|
| `?` | Help |
| `st` | Status (mode, sensors present, calibration, loop) |
| `m <l> <r>` | Set left/right duty directly, −1000..1000 |
| `f <duty>` | Both wheels at one duty |
| `s` / `b` | Stop (coast) / brake |
| `e` / `z` | Read encoders / zero them |
| `t` / `i` | Read ToF / IMU |
| `j` / `jz` | Loop jitter stats / reset |
| `d <mm>` | Closed-loop straight drive |
| `cal <mm>` | After `d`, report the measured distance; rescales counts/m into NVS |
| `cpm <v>` | Set counts/m directly |
| `v` | Toggle the telemetry CSV stream |
| `c` | Clear a latched safety stop |

## Bring-up order

**Follow the order.** Applying motor power first means a wiring mistake burns parts.

### 1. Board only, motor power OFF
```
pio run -t upload && pio device monitor
```
Check `st` shows 3 ToF and the IMU. If not, start with I2C wiring and XSHUT pins.

> On the R4 the sensors hang off **Qwiic (`Wire1`, 3.3 V)**. The Adafruit ToF and
> IMU boards are STEMMA QT, so they daisy-chain with no I2C soldering.
> **Putting 5 V on Qwiic damages the board.** The three XSHUT wires still go to
> A0/A1/A2 individually.

### 2. Sensors alone
`t` — wave a hand in front of each; out of range reads 8190 mm.
`i` — tilt the board and watch accel change; tap it and watch shock spike.

### 3. Motors — **with the wheels off the ground**
Motor power ON. `f 300` and confirm both wheels turn **forward**.
If one runs backwards, swap that motor's two wires. Do not fix it in code.
`s` to stop.

### 4. Encoder direction
`z`, then roll the wheels forward by hand. `e` must show **both counts rising**.
If one falls, swap that encoder's A/B wires.
(The right-hand ISR in `src/drive/encoders.cpp` already accounts for the mirrored
mounting.)

### 5. Odometry calibration — Phase 1 gate ①
Put it on the floor and mark the start line.
```
d 1000          # drive 1 m
                # measure the real distance with a ruler
cal 970         # e.g. if it actually went 970 mm
```
counts/m is rescaled and saved to NVS. Repeat until within ±5 %.

> Rushing this makes the phase-3 reward lie. See "Why encoders are mandatory" in
> [docs/03-hardware-bom_eng.md](../docs/03-hardware-bom_eng.md).

### 6. Loop jitter — Phase 1 gate ②
```
jz              # reset stats
v               # telemetry on, so the loop is under load
                # leave it for at least 60 s
j
```
It must print `PHASE 1 GATE : PASS` (zero overruns, within ±2 ms).

## What is not verified yet

**It compiles. Nothing has run on hardware.** No hardware exists yet, so that is
expected. These in particular are **guesses** until measured.

| Item | Location | Note |
|---|---|---|
| Pin map | `include/pins.h` | Check against the physical board |
| `kDutyDeadband` = 120 | `include/config.h` | Chassis-specific; confirm in step 3 |
| `kDefaultCountsPerMeter` = 3000 | `include/config.h` | Pure placeholder; `cal` overwrites it |
| `kHeadingKp` = 1.2 | `include/config.h` | Lower it if the drive oscillates |
| IMU impact threshold 0.6 g | `src/sense/imu.cpp` | Desk taps peaked at only 68 mg — tune against a real collision at stage 7. See [bringup-log](../hardware/bringup-log_eng.md) |
| Right encoder sign flip | `src/drive/encoders.cpp` | Confirm in step 4 |
| R4 loop jitter | `src/hal/hal_r4.cpp` | **First measurement PASS** (2026-09-15, one distance sensor, no motors, telemetry on, 60 s): 20000.0 ± 2.3 µs, min/max 19995/20005 µs, 0 overruns. Re-measure at stage 7 with every sensor and the motors attached |
| **R4 PWM frequency** | `src/hal/hal_r4.cpp` | The R4 core exposes no frequency control, so the carrier is audible. Expect motor whine |
| R4 interrupt pins D2/D3 | `include/pins_r4.h` | Per official docs; verify on hardware |
