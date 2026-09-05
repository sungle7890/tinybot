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

## Rough budget

| Item | KRW |
|---|---|
| ESP32-S3 dev board | 15,000 |
| 2WD chassis + encoder motors | 30,000 |
| TB6612FNG | 5,000 |
| VL53L0X × 3 | 25,000 |
| IMU, bumpers, wiring, battery, caps | 30,000 |
| **Total** | **~100,000–120,000** |

About a fifth of Microduck ($399 ≈ ₩550,000). No bipedal walking in exchange.
