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

## Purchase list (US sourcing, checked 2026-09-04)

Prices are unit prices at the time of checking. Stock and pricing move; re-check
before ordering.

### Option 1 — Pololu Romi, integrated (recommended)

| # | Part | Vendor / SKU | Unit | Qty | Subtotal |
|---|---|---|---|---|---|
| 1 | Romi Chassis Kit (chassis, 2 motors, wheels, ball casters, 6×AA holder) | Pololu [#3502](https://www.pololu.com/product/3502) | $39.95 | 1 | $39.95 |
| 2 | Romi Encoder Pair Kit | Pololu [#3542](https://www.pololu.com/product/3542) | $9.95 | 1 | $9.95 |
| 3 | Motor Driver and Power Distribution Board for Romi | Pololu [#3543](https://www.pololu.com/product/3543) | $34.95 | 1 | $34.95 |
| 4 | ESP32-S3-DevKitC-1-N8 | DigiKey [15199021](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-S3-DEVKITC-1-N8/15199021) / Mouser / Amazon | ~$15 | 1 | ~$15 |
| 5 | VL53L0X ToF breakout | Adafruit [#3317](https://www.adafruit.com/product/3317) | $14.95 | 3 | $44.85 |
| 6 | MPU-6050 6-DoF IMU | Adafruit [#3886](https://www.adafruit.com/product/3886) | $12.95 | 1 | $12.95 |
| 7 | Snap-action switch, 18.5 mm lever (bumpers) | Pololu [#1405](https://www.pololu.com/product/1405) | $2.37 | 2 | $4.74 |
| 8 | AA NiMH cells (Eneloop or similar) + charger | Amazon | ~$30 | 1 | ~$30 |
| 9 | Jumper wires, standoffs, perfboard, USB-C cable | Amazon / Adafruit | ~$25 | 1 | ~$25 |
| | | | | **Total** | **~$218** |

### Option 2 — discrete parts (TB6612FNG)

Replace line 3 above with:

| Part | Vendor / SKU | Unit |
|---|---|---|
| TB6612FNG Dual Motor Driver Carrier | Pololu [#713](https://www.pololu.com/product/713) | $4.95 |
| 5 V step-down regulator, 1 A or better | Pololu D24V10F5 or similar | ~$10 |
| Power switch | Pololu / Amazon | ~$3 |

About $201 total. **Saving $17** costs reverse-voltage protection, 2 A
regulation, a power switch, and battery-contact wiring, all now done by hand.
Not recommended.

### Option 3 — budget build (~$70)

A generic 2WD kit plus N20 encoder motors, a GY-530 (VL53L0X) 3-pack, a GY-521
(MPU6050) and a TB6612 module from AliExpress or Amazon. Less than half the
price, but encoder quality varies a lot and XSHUT breakout differs per listing.
`firmware/src/sense/tof.cpp` requires XSHUT, so **confirm it before buying**.

## Why option 1

The [power pitfall](#power--the-most-common-failure-point) is this project's
biggest failure mode, and Pololu #3543 removes it wholesale — reverse-voltage
protection, a 2 A 5 V switching regulator, a power switch, battery contacts and
the motor drivers on one board that drops into the chassis. No hand-wired motor
power means no brownout resets.

## What option 1 implies

### ⚠️ It requires a firmware change

#3543 uses two **DRV8838** drivers with a DIR + PWM (phase/enable) interface,
not the TB6612FNG's IN1/IN2 + PWM that `firmware/src/drive/motors.cpp` currently
implements. It is a simplification — two pins per motor instead of three — but
the code has to change. Confirm against the #3543 pinout drawing before wiring.

### Expected counts/m

The Romi encoders give 12 counts per motor-shaft revolution counting both edges
of both channels (4x). The firmware decodes 2x, counting both edges of channel A
only, so 6 counts per motor revolution.

```
6 counts/rev x 120:1 gearbox = 720 counts per wheel revolution
70 mm wheel -> 0.2199 m circumference
720 / 0.2199 = about 3274 counts/m
```

Close to the 3000 placeholder in `cfg::kDefaultCountsPerMeter`.
Still **calibrate with `cal`.** The computed number is a starting point only.

### Use NiMH, not alkaline

Six alkaline AAs is 9 V, above the rating of the Romi's mini plastic gearmotors,
and shortens their life. Six NiMH gives a comfortable 7.2 V — and learning runs
are long enough that rechargeables are needed regardless.

### Stock

Adafruit's resale of the ESP32-S3-DevKitC-1 ([#5312](https://www.adafruit.com/product/5312))
was out of stock when checked. DigiKey, Mouser or Amazon are better bets.
N8 or N8R8 both work — the PSRAM goes unused. Note that N8R8's octal PSRAM
occupies GPIO 33–37, which `include/pins.h` already avoids; keep it that way.
