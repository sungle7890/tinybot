# 04. Roadmap

> Korean: [04-roadmap.md](04-roadmap.md)

Every phase needs an explicit **exit criterion** before moving on.
Advancing on "seems to work" turns into an untraceable bug later.

## Phase 0 — Design and parts ✅

- [x] Board picked — option A, ESP32-S3 standalone ([05-open-questions_eng.md](05-open-questions_eng.md))
- [x] Wiring diagram drawn ([../hardware/wiring_eng.md](../hardware/wiring_eng.md))
- [ ] Finalize BOM and order

**Exit**: parts on the desk, and a drawing of what plugs in where.
→ Only the order is left.

## Phase 1 — Hardware bring-up ← **current**

> Firmware is written and compiles. Every item below is a **hardware
> verification** item and cannot be checked until parts arrive. Procedure:
> [../firmware/README_eng.md](../firmware/README_eng.md).

- [x] PlatformIO project, build pipeline (`pio run` succeeds)
- [ ] Upload verified
- [ ] Motors driven both directions with PWM speed control
- [ ] Encoder counts read and converted to real distance (mm)
- [ ] Three ToF sensors running together (I2C address reassignment)
- [ ] IMU readout
- [ ] Fixed-rate 50 Hz loop, with measured jitter recorded

**Exit**: commanding 1 m straight lands within ±5 % measured.
Loop jitter within ±2 ms of the 50 Hz period.

> If odometry is inaccurate here, every phase-3 reward becomes a lie.
> This is the one phase not to rush.

## Phase 2 — Rule-based autonomy

- [ ] Obstacle avoidance via Braitenberg / subsumption
- [ ] Hard-interrupt safety stop on bumpers and cliff sensors
- [ ] Serial telemetry streaming, plotted from `host/`

**Exit**: roams a room for 10 minutes with no commands and ≤3 collisions.

> **The "moves autonomously without commands" goal is already met here.**
> Autonomy does not require AI. Confirming that before moving on is what makes
> it possible to honestly judge what AI adds in phase 3.
> Phase 2 performance becomes the phase-3 baseline.

## Phase 3 — On-device Q-learning ★ the core of the project

- [ ] Design and implement state discretization
- [ ] Q-table plus ε-greedy exploration with a decay schedule
- [ ] Reward function using encoder-measured actual forward motion
- [ ] EEPROM/Flash checkpointing, verified across a power cycle
- [ ] Live learning-curve plot on the host

**Exit**:
1. The learning curve trends upward and reproduces across 3 different seeds.
2. Post-training performance **beats the phase-2 rule-based baseline.**
3. Learned behavior survives a power cycle.

> Criterion 2 is the real gate. Failing it means "it learned, but worse than
> hand-written rules" — which is still an honest result. In that case, suspect
> the state representation or the reward design.

## Phase 4 — TinyML inference (optional)

- [ ] Edge Impulse or TFLite Micro pipeline
- [ ] Gesture or voice command recognition wired to robot behavior
- [ ] Measure inference latency; confirm the 50 Hz loop is unaffected

**Exit**: ≥90 % recognition accuracy while holding the phase-1 jitter budget.

## Phase 5 — Sim-to-real (optional, the Microduck approach)

- [ ] Model the robot in MuJoCo or PyBullet
- [ ] Train a policy with PPO
- [ ] Export ONNX → quantize to a C array for the MCU
- [ ] Deploy to hardware, measure the sim-to-real gap

**Exit**: a policy trained purely in simulation works on the real robot
(degradation acceptable, outright failure not).

---

## Where to stop

**Phases 0–3 are the project.** Phases 4–5 happen only if interest remains.
Finishing phase 3 properly means physically holding a robot that learns its
behavior and moves autonomously without commands. That was the original goal.
