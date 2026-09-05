# Wiring

> Korean: [wiring.md](wiring.md)

Must stay in sync with `firmware/include/pins.h`. Never change only one side.

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
