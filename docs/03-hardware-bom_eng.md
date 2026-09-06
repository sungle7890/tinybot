# 03. Hardware BOM

> Korean: [03-hardware-bom.md](03-hardware-bom.md)

Prices are rough, as of 2026. Re-check before ordering.

## Board options

| Board | Spec | Verdict |
|---|---|---|
| Uno R3 (ATmega328P) | 16 MHz, 2 KB SRAM | Q-learning works, TinyML does not. No headroom |
| **Nano 33 BLE Sense Rev2** | nRF52840 M4F 64 MHz, 256 KB RAM, onboard IMU/mic/proximity | Official Arduino TinyML board. Built-in sensors cut wiring |
| Uno R4 WiFi | RA4M1 48 MHz + ESP32-S3 (WiFi) | Fine. Convenient for wireless telemetry |
| **ESP32-S3** | Dual-core 240 MHz, PSRAM, ESP-NN acceleration | Best value. Camera and WiFi included. One core can be pinned to the control loop |
| Raspberry Pi Zero 2 W | Linux, quad-core | For putting the host tier on the robot itself |

**Recommendation**: ESP32-S3 as the MCU tier. Dual cores let us physically
separate the control loop (core 0) from communication (core 1) — which
dissolves the loop-jitter problem from the architecture doc at design time.

## Drivetrain

| Part | Choice | Note |
|---|---|---|
| Chassis | 2WD robot car kit | 4WD slips badly when turning, which ruins odometry |
| Motor driver | **TB6612FNG** or DRV8833 | **Avoid L298N** — over 1.4 V drop, runs hot |
| Motors | N20 geared DC with **encoders** | Encoders are not optional. See below |
| Wheels | Kit wheels plus a caster | |

### Why encoders are mandatory

If the reward is "+1 for moving forward" but we cannot tell whether the robot
actually moved, the reward lies. Pressed against a wall with wheels spinning
freely, the robot keeps collecting +1, and learning converges on "hug the wall."
Doing this project without encoders guarantees meeting that bug.

## Sensors

| Part | Purpose | Note |
|---|---|---|
| **VL53L0X ToF** × 2–3 | Front/left/right distance | More accurate and faster than HC-SR04 ultrasonic. Watch I2C address collisions — sequence startup via XSHUT pins |
| MPU6050 or BMI270 IMU | Attitude, collision detection | Sudden deceleration = collision, so collision reward works without a bumper |
| Bumper switches × 2 | Hard collision | Wire to interrupts. A safety device, not something the policy learns |
| (optional) Cliff IR × 2 | Fall prevention | Mandatory if training on a table |

## Power — the most common failure point

| Part | Spec |
|---|---|
| Battery | 2× 18650 (7.4 V) plus holder |
| Logic regulator | Battery → 5 V/3.3 V, **separate** from motor power |
| Capacitors | ≥1000 µF on the motor rail, 0.1 µF across each motor terminal |
| Protection | Battery with built-in BMS, or a separate protection circuit |

> **⚠️ Do not power motors from the board's 5 V pin.**
> Motor inrush current drags the rail down and triggers a brownout reset.
> A reset mid-training wipes the in-RAM Q-table, and the symptom presents as
> "why isn't it learning?" — the kind of bug that costs days to trace.
> Keep motor and logic power separate; tie only the grounds together.

## Order list (US, checked 2026-09-05)

Unit prices at the time of checking. Re-check before ordering.

### Stage 1 — sensors (Adafruit): order now, start without the chassis

| Part | SKU | Unit | Qty | Subtotal |
|---|---|---|---|---|
| VL53L0X ToF distance sensor | [Adafruit 3317](https://www.adafruit.com/product/3317) | $14.95 | 3 | $44.85 |
| MPU-6050 6-DoF IMU | [Adafruit 3886](https://www.adafruit.com/product/3886) | $12.95 | 1 | $12.95 |
| STEMMA QT / Qwiic cable, 100 mm | [Adafruit 4210](https://www.adafruit.com/product/4210) | $0.95 | 4 | $3.80 |
| | | | **Subtotal** | **$61.60** |

Four cables chain R4 → ToF1 → ToF2 → ToF3 → IMU. Qwiic and STEMMA QT are the
same JST-SH 4-pin standard, so they plug straight in.

### Stage 2 — chassis, drivetrain, power (Pololu)

| Part | SKU | Unit | Qty | Subtotal |
|---|---|---|---|---|
| Romi Chassis Kit (chassis, 2 motors, wheels, casters, 6×AA holder, contacts) | [Pololu 3502](https://www.pololu.com/product/3502) | $39.95 | 1 | $39.95 |
| Romi Encoder Pair Kit | [Pololu 3542](https://www.pololu.com/product/3542) | $9.95 | 1 | $9.95 |
| **Power Distribution Board for Romi** | [Pololu 3541](https://www.pololu.com/product/3541) | $14.95 | 1 | $14.95 |
| TB6612FNG Dual Motor Driver Carrier | [Pololu 713](https://www.pololu.com/product/713) | $4.95 | 1 | $4.95 |
| Snap-action switch, 18.5 mm lever (bumpers) | [Pololu 1405](https://www.pololu.com/product/1405) | $2.37 | 2 | $4.74 |
| | | | **Subtotal** | **$74.54** |

### Stage 3 — power and consumables (Amazon or similar)

| Part | Approx |
|---|---|
| 8× AA NiMH plus charger (Eneloop or similar) | ~$35 |
| Jumper wires (M-F, F-F) | ~$8 |
| Header pin strips | ~$5 |
| 22 AWG wire | ~$8 |
| Capacitors (1000 µF electrolytic, 0.1 µF ceramic) | ~$10 |
| | **~$66** |

### Total

| | |
|---|---|
| Stage 1 sensors | $61.60 |
| Stage 2 chassis and drivetrain | $74.54 |
| Stage 3 consumables | ~$66 |
| **Parts total** | **~$202** |
| MCU | **$0** — UNO R4 WiFi already owned |

Tools, if not owned: soldering iron and solder ~$40, cutters/strippers ~$15,
multimeter ~$20.

## Rationale

### Why #3541 rather than #3543

The earlier recommendation was [#3543](https://www.pololu.com/product/3543),
the Motor Driver **and** Power Distribution Board at $34.95. Moving to the R4
changed the arithmetic.

| | #3543 | **#3541 + TB6612** |
|---|---|---|
| Price | $34.95 | **$19.90** |
| Motor driver | DRV8838 (DIR+PWM) | TB6612FNG (IN1/IN2+PWM) |
| 5 V regulator | 2 A onboard | None — **the R4's own is enough** |
| Reverse protection, power switch, battery contacts | ✅ | ✅ |
| **Firmware change** | **Required** | **None** |

Two things decide it. **The R4 takes 6–24 V on VIN and regulates 5 V/3.3 V
itself**, so there is no reason to pay for #3543's regulator. And #3543's
DRV8838 is DIR+PWM, which would mean rewriting both `motors.cpp` and
`pins_r4.h`; #3541 handles power only, so the TB6612FNG stays and the code is
untouched.

$15 cheaper, and no work.

### ⚠️ XSHUT soldering is unavoidable

Adafruit's ToF boards ship with headers loose, not soldered. All three VL53L0X
boot at 0x29, so readdressing needs **individual control of XSHUT** (labelled
`SHDN` on these boards). At minimum that one pin must be soldered per board.

A Qwiic I2C multiplexer (TCA9548A) would dodge the address clash without
soldering, but it means rewriting `tof.cpp`, and an iron is needed for the
motors, encoders and battery contacts regardless. Not recommended.

### Shipping

Adafruit and Pololu are separate vendors, so shipping is paid twice (estimate
$8–15 each). Combining saves money, but **taking delivery of stage 1 first is
worth more** — readdressing three ToF sensors is the fiddliest part of Phase 1,
and solving it before motors exist makes later fault isolation much easier.
