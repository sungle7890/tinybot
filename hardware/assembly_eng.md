# Assembly and wiring guide — UNO R4 WiFi + Romi

> Korean: [assembly.md](assembly.md)

Pin numbers come from `firmware/include/pins_r4.h`. If this document disagrees,
the code is right.

**Do not skip stages.** Each assumes the previous one is verified. The point of
the order is to narrow "which stage broke it" when something goes wrong.

## Safety rules throughout

- **Power off** whenever rewiring (unplug USB, switch off with the #3541 button)
- **Qwiic is 3.3 V only.** 5 V on it destroys the R4
- Burning smell, a hot part, or the R4 LED going dark → cut power at #3541 now
- First motor run is **with the wheels off the ground**

---

## Stage 1 — sensors only (USB power, no battery, no motors)

### Soldering
Solder the included headers onto all three VL53L0X boards. Only `SHDN` is
strictly needed, but the full header holds the board mechanically.
The MPU-6050 uses Qwiic only; no soldering.

### Wiring

| Connection | How |
|---|---|
| R4 Qwiic → VL53L0X → VL53L0X → VL53L0X → MPU-6050 | QT-to-QT cable chain |
| **Front** VL53L0X `SHDN` → R4 **A0** | jumper |
| **Left** VL53L0X `SHDN` → R4 **A1** | jumper |
| **Right** VL53L0X `SHDN` → R4 **A2** | jumper |

Chain order does not matter electrically — addresses follow **XSHUT order**, not
chain position. **Label each sensor front/left/right** anyway; mixed up later,
they are hard to tell apart.

### Verify
```bash
cd firmware
pio run -t upload
pio device monitor
```
Expected:
```
bring-up: 3/3 ToF, IMU ok
```
Then `t` (range changes with a hand in front) and `i` (accel changes on tilt,
shock spikes on a tap).

| Symptom | Suspect |
|---|---|
| `0/3 ToF`, IMU MISSING | The whole Qwiic chain. Cables fully seated? |
| `0/3 ToF`, IMU ok | All three SHDN wires, or A0 stuck LOW |
| `1/3`, `2/3` | The SHDN wire of the missing sensor |
| ToF stuck at 8190 | Normal — out of range. Bring a hand within 20 cm |

**Pass: `3/3 ToF, IMU ok`, and all three sensors respond to a hand.**

---

## Stage 2 — motors and encoders (no power)

### Encoder boards (each side)
1. Fit the encoder board **directly onto the motor's two terminals**
2. **Tack one pin first**, check the board sits flat and aligned, then solder the other
3. ⚠️ **Do not heat the motor pins for long.** It can deform the case or brushes (Pololu warning)

### Magnetic disc
Set the disc on a flat surface and **press the motor down onto it** until the
shaft tip touches the surface.

### Encoder board's six pins
`M1` `M2` `VCC` `GND` `A` `B` — solder wires or a header here.

> **⚠️ Two encoder facts**
> - **VCC is 3.5 V minimum.** The 3.3 V rail will not do → use R4 **5 V**
> - **A/B are open-drain** and need pull-ups. The firmware enables the R4's
>   internal pull-ups via `INPUT_PULLUP`. If counts are erratic, add external
>   4.7–10 kΩ pull-ups from each of A/B to 5 V

### Mount the motors
Fix them with the Romi motor clips. Fit the wheels.

---

## Stage 3 — chassis and power wiring (no batteries in)

### Order matters
1. **Mount the #3541 power board on the chassis first**
2. **Then solder the four battery contacts** — Pololu says **after** mounting
3. Fix the TB6612FNG and R4 to the chassis (standoffs or double-sided foam tape.
   Whether the R4's holes line up with the Romi's has not been checked)

### Full connection table

**Power**

| From | To | Note |
|---|---|---|
| #3541 `VSW` | TB6612 `VM` | Motor supply, after the switch and reverse protection |
| #3541 `VSW` | R4 `VIN` | Logic supply; the R4 regulates 5 V/3.3 V |
| #3541 `GND` | common GND | |
| R4 `5V` | TB6612 `VCC` | Driver logic |
| R4 `5V` | left and right encoder `VCC` | **Not 3.3 V** |
| R4 `GND` | common GND | **Every GND to one point** |

**Motor driver control**

| R4 | TB6612 |
|---|---|
| D5 | `PWMA` |
| D7 | `AIN1` |
| D8 | `AIN2` |
| D6 | `PWMB` |
| D9 | `BIN1` |
| D10 | `BIN2` |
| D11 | `STBY` |

**Motor outputs**

| TB6612 | Encoder board |
|---|---|
| `AO1`, `AO2` | **left** `M1`, `M2` |
| `BO1`, `BO2` | **right** `M1`, `M2` |

**Encoder signals**

| Encoder | R4 |
|---|---|
| left `A` | **D2** (interrupt) |
| left `B` | D4 |
| right `A` | **D3** (interrupt) |
| right `B` | D12 |

**Bumpers** (use only `COM` and `NO` of the three snap-action pins)

| Switch | Connect to |
|---|---|
| left `COM` | GND |
| left `NO` | R4 **A3** |
| right `COM` | GND |
| right `NO` | R4 **A4** |

**Sensors** — as in stage 1.

### Capacitor (recommended)
The TB6612 carrier already has filter capacitors. Add a **1000 µF electrolytic
across VM–GND** close to the driver anyway, for the Romi motors' inrush.
⚠️ Electrolytics are **polarised.** The striped (−) lead goes to GND.

---

## Stage 4 — checks before power (multimeter)

**Before** inserting batteries, with power off:

| Measure | Expect | Otherwise |
|---|---|---|
| `VSW` ↔ `GND` resistance | hundreds of Ω or more, not 0 Ω | **Short. Do not insert batteries** |
| R4 `5V` ↔ `GND` resistance | not 0 Ω | Short |
| Encoder `VCC` reaches 5 V | continuity | Rewire |
| Battery orientation | matches holder marks | #3541 has reverse protection; do not rely on it |

Insert six **fully charged** NiMH cells; see "Known limits" for why.

---

## Stage 5 — first power (wheels up)

1. Rest the chassis on a box so **the wheels spin free**
2. Power on with the #3541 button; check the R4 LED
3. Connect USB, `pio device monitor`
4. `st` — 3/3 ToF, IMU ok, mode idle
5. `f 300` → **both wheels turn forward.** If one runs backwards, swap that side's `M1`/`M2` (do not fix it in code)
6. `s` to stop

## Stage 6 — encoder direction

`z` → roll the wheels **forward by hand** → `e`.
**Both counts must rise.** If one falls, swap that encoder's `A`/`B` wires.

## Stage 7 — the Phase 1 gates

Steps 5 and 6 of [firmware/README_eng.md](../firmware/README_eng.md):
calibration (`d 1000` → measure → `cal`) and jitter (`jz` → `v` → 60 s → `j`).

---

## Known limits

**End-of-discharge voltage.** The R4's minimum VIN is 6 V. Six NiMH are 7.2 V
nominal but fall to about 1.0 V per cell — **6.0 V** — near empty, so the R4
may reset as the pack runs down. Charge fully before long runs.

A reset mid-training in Phase 3 loses everything since the last checkpoint. A
**voltage divider on the spare A5** would let the firmware checkpoint and stop
before brown-out — add it when that starts to matter.
