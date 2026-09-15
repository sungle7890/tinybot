# Wiring

> Korean: [wiring.md](wiring.md)

**The primary board is the Arduino UNO R4 WiFi** (decided 2026-09-05). The
ESP32-S3 wiring is kept below for Phase 4.

Must stay in sync with `firmware/include/pins_r4.h` / `pins_esp32.h`.

---

# A. UNO R4 WiFi (current)

## Two constraints shaped this layout

1. **Only D2 and D3 are interrupt-capable.** So the encoder A channels take
   them and the bumpers are polled once per control tick. A switch held against
   an obstacle stays closed far longer than 20 ms, so nothing is missed.
2. **I2C runs on Qwiic.** Qwiic is a second bus (`Wire1`) and 3.3 V only. It
   uses no header pins, which frees A4/A5 — and that is what makes 18 pins fit.

## Pin assignment (R4 WiFi)

| Pin | Connection | Note |
|---|---|---|
| D0, D1 | — | UART0 (USB console). Do not use |
| **D2** | Left encoder A | **interrupt** |
| **D3** | Right encoder A | **interrupt** |
| D4 | Left encoder B | |
| D5 | Left motor PWMA | PWM |
| D6 | Right motor PWMB | PWM |
| D7 | Left AIN1 | |
| D8 | Left AIN2 | |
| D9 | Right BIN1 | |
| D10 | Right BIN2 | |
| D11 | Motor STBY | |
| D12 | Right encoder B | |
| D13 | (spare) | Drives the onboard LED; poor as an input |
| A0 | ToF XSHUT front | |
| A1 | ToF XSHUT left | |
| A2 | ToF XSHUT right | |
| A3 | Left bumper → GND | Polled, INPUT_PULLUP |
| A4 | Right bumper → GND | Free because I2C is on Qwiic |
| A5 | (spare) | |
| **Qwiic** | 3× VL53L0X + MPU-6050 | `Wire1`, **3.3 V only** |

**18 used / 18 available, plus 2 spare.**

## Qwiic chain

Adafruit VL53L0X (#3317) and MPU-6050 (#3886) carry STEMMA QT connectors, which
are Qwiic-compatible. **No I2C soldering.**

```
R4 Qwiic ─ VL53L0X(front) ─ VL53L0X(left) ─ VL53L0X(right) ─ MPU-6050
```

⚠️ XSHUT is not on the Qwiic cable. Each sensor still needs its own wire to
A0/A1/A2.

⚠️ **Putting 5 V on Qwiic damages the board.** It is 3.3 V only.

> To use the 5 V header bus on A4/A5 instead, set `cfg::kUseQwiic` false and
> move the right bumper from A4 to D13.

## Power (R4 WiFi)

```
Romi 6×AA NiMH (7.2 V) ─→ #3541 power board (reverse protection + switch)
                              └─ VSW ─┬─→ TB6612FNG VM   (motors)
                                      └─→ R4 VIN         (logic; the R4 regulates)
R4 5V ─→ TB6612 VCC, encoder VCC (3.5 V minimum, so not 3.3 V)
All grounds common
```

Full connection table and build order: **[assembly_eng.md](assembly_eng.md)**.

> **⚠️ Do not power motors from the R4's 5 V pin**, for the same reason given in
> the power section below.

---

# B. ESP32-S3 (deferred to Phase 4)

## Pin assignment (provisional — verify against the physical board)

### TB6612FNG motor driver

| ESP32-S3 | TB6612FNG | Note |
|---|---|---|
| GPIO 17 | STBY | LOW disables the outputs |
| GPIO 4 | PWMA | Left |
| GPIO 5 | AIN1 | |
| GPIO 6 | AIN2 | |
| GPIO 7 | PWMB | Right |
| GPIO 15 | BIN1 | |
| GPIO 16 | BIN2 | |
| 3V3 | VCC | Logic |
| — | VM | **Straight from the motor battery, not the board's 5 V** |
| GND | GND | Logic and motor grounds tied together |

### Encoders

| ESP32-S3 | Signal |
|---|---|
| GPIO 8 | Left A (interrupt) |
| GPIO 9 | Left B |
| GPIO 10 | Right A (interrupt) |
| GPIO 11 | Right B |

Internal pull-ups are enabled; fine for open-collector or push-pull encoders.

### I2C

| ESP32-S3 | Signal |
|---|---|
| GPIO 13 | SDA |
| GPIO 14 | SCL |

400 kHz. 4.7 kΩ pull-ups — most breakout boards already carry them.
**Three breakouts in parallel makes the pull-up far too strong.** Remove all but
one, or check the actual waveform before leaving them.

### VL53L0X XSHUT

| ESP32-S3 | Sensor | Address after boot |
|---|---|---|
| GPIO 1 | Front | 0x30 |
| GPIO 2 | Left | 0x31 |
| GPIO 42 | Right | 0x32 |

All three boot at 0x29, so they are released one at a time and readdressed
(`firmware/src/sense/tof.cpp`).

### Bumpers

| ESP32-S3 | Signal |
|---|---|
| GPIO 41 | Left bumper → GND |
| GPIO 40 | Right bumper → GND |

Normally open, INPUT_PULLUP, FALLING interrupt, 30 ms software debounce.

## Pins avoided, and why

| Pin | Reason |
|---|---|
| 0, 3, 45, 46 | Strapping pins |
| 19, 20 | Native USB D−/D+ |
| 26–32 | SPI flash |
| 33–37 | Octal PSRAM on N8R8/N16R8 modules |
| 43, 44 | UART0 (the console) |

## Power

```
2x 18650 (7.4 V)
   ├─→ TB6612FNG VM        (motors)
   └─→ 5 V regulator ─→ ESP32-S3 5 V pin   (logic)
Grounds tied together
```

- ≥1000 µF on TB6612FNG VM, 0.1 µF ceramic across each motor terminal
- Use batteries with a built-in BMS

> **⚠️ Do not power motors from the board's 5 V pin.**
> Inrush current drags the rail down into a brownout reset. When that happens in
> phase 3 it wipes the in-RAM Q-table, and the only symptom is "it isn't
> learning." That is the kind of bug that costs days to trace.

## Verification status

**Unverified.** Nothing has been wired yet.
Update this line once it has been checked against real hardware.
