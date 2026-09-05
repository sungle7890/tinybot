# firmware — Phase 1 (hardware bring-up)

> Korean: [README.md](README.md)

ESP32-S3 standalone. No learning code here.
Phase 1 answers exactly one question: **are these sensors and this odometry good
enough to trust as a reward signal later?**

## Build

```bash
pio run                 # build
pio run -t upload       # flash
pio device monitor      # console at 115200
```

Pinned to `platform = espressif32@6.12.0` (Arduino core 2.0.17). The LEDC calls
in `src/drive/motors.cpp` are version-guarded so Arduino core 3.x also builds.

## Core split

| Core | Responsibility |
|---|---|
| 0 | Control task. Fixed 50 Hz. No `delay()`, no `Serial.print()` |
| 1 | Arduino `loop()` — serial console, telemetry printing |

The point is that serial traffic cannot jitter the control period. Telemetry is
**dropped** when the queue fills (and the drops are counted). Losing telemetry is
acceptable; a late control tick is not — [docs/02-architecture_eng.md](../docs/02-architecture_eng.md)
design rule 1.

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
| IMU impact threshold 0.6 g | `src/sense/imu.cpp` | Tune against a real collision |
| Right encoder sign flip | `src/drive/encoders.cpp` | Confirm in step 4 |
