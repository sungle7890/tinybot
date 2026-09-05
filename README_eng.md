# tinybot

Building a small robot that **learns** how to move, from the ground up.

> Korean docs: [README.md](README.md)

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
host/       Host-side Python (telemetry, simulation, policy training)
hardware/   Wiring, chassis, measured part notes
docs/       Design documents
```

## Status

**Phase 1 — firmware written, hardware verification pending.**

- Phase 0 closed. Board is the ESP32-S3 standalone, option A — [docs/05-open-questions_eng.md](docs/05-open-questions_eng.md)
- `firmware/` compiles (`pio run` succeeds, no warnings under `-Wall -Wextra`)
- **No parts yet, so nothing is verified on hardware.** Bring-up procedure: [firmware/README_eng.md](firmware/README_eng.md)
- `host/` starts in phase 2
