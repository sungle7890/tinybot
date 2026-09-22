# tinybot

Building a small robot that **learns** how to move, from the ground up.

> 한국어: [README_ko.md](README_ko.md)

## Goal

A robot that moves autonomously without commands, improves its behavior through
trial and error, and keeps what it learned across power cycles. The priority is
not impressive performance but a setup where **"is learning actually happening?"**
is a verifiable question.

## Premise

Training a neural network **on** an Arduino-class MCU is not realistic.
Two approaches are combined instead.

1. **On-device learning** — tabular reinforcement learning (Q-learning). A few
   hundred bytes is enough; it genuinely runs on an ATmega328P. This alone
   satisfies the definition of a learning robot.
2. **Off-device training → on-device inference** — train a policy in simulation
   or on a PC, quantize it, deploy to the board. This is what shipping products,
   Microduck included, actually do.

Rationale: [docs/01-microduck-analysis_eng.md](docs/01-microduck-analysis_eng.md).

## Documents

| Doc | Contents |
|---|---|
| [01 Microduck analysis](docs/01-microduck-analysis_eng.md) | How Microduck actually runs AI |
| [02 Architecture](docs/02-architecture_eng.md) | The two-tier split and why |
| [03 Hardware BOM](docs/03-hardware-bom_eng.md) | Board/part options and pitfalls |
| [04 Roadmap](docs/04-roadmap_eng.md) | Five phases with exit criteria |
| [05 Open questions](docs/05-open-questions_eng.md) | Decisions to make first |

## Layout

```
firmware/   MCU firmware (real-time control loop, on-device learning)
host/       Host tools (live dashboard; later simulation and policy training)
hardware/   Wiring, chassis, measured part notes
docs/       Design documents
```

## Status

**Phase 3 — on-device Q-learning, in progress.** Runs on a real robot (Arduino UNO R4 WiFi).

- **Phase 1 done** — 50 Hz control loop, all three sensor types and encoder calibration
  (+1.6 % over 1 m) verified on hardware. Every measurement is in
  [hardware/bringup-log_eng.md](hardware/bringup-log_eng.md)
- **Phase 2 partly done** — rule-based roaming (`a`). The exit criterion (10 minutes, ≤3
  collisions) is recorded as not met: with no bumper fitted there is nothing to count
  collisions with
- **Phase 3 in progress** — the robot learns on the floor (`l`), and what it learns survives a
  power cycle. Whether it beats the rule-based baseline is **not yet compared**
- Known limit: the range sensors are mounted so they see the floor, leaving 13.5 cm of warning
  ([cause and measurements](hardware/bringup-log_eng.md))
- `host/dashboard/` — live dashboard over USB or Wi-Fi, including the learning curve
- Learner unit tests: `cd firmware && pio test -e native`

The board was planned as an ESP32-S3 ([05 open questions](docs/05-open-questions_eng.md)); the
project went ahead on the UNO R4 WiFi already on hand. The firmware builds for both.

## License

[MIT](LICENSE)
