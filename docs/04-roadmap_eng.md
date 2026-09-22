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

## Phase 1 — Hardware bring-up ✅ complete (2026-09-16)

> Firmware is written and compiles. Every item below is a **hardware
> verification** item and cannot be checked until parts arrive. Procedure:
> [../firmware/README_eng.md](../firmware/README_eng.md).

- [x] PlatformIO project, build pipeline (`pio run` succeeds)
- [x] Upload verified (pyOCD, 2026-09-15)
- [x] Motors driven both directions with PWM speed control
- [x] Encoder counts read and converted to real distance (mm)
- [x] Three ToF sensors running together (I2C address reassignment)
- [x] IMU readout
- [x] Fixed-rate 50 Hz loop, with measured jitter recorded

**Exit**: commanding 1 m straight lands within ±5 % measured.
Loop jitter within ±2 ms of the 50 Hz period.

> If odometry is inaccurate here, every phase-3 reward becomes a lie.
> This is the one phase not to rush.

## Phase 2 — Rule-based autonomy ✅ partly done (2026-09-17)

- [x] Rule-based obstacle avoidance (cruise / turn / backup state machine)
- [x] Self-imposed stops (10 s without progress, 180 s session cap) — the radio is not a dependable stop
- [x] Escape when pressed against a wall: all three ranges still for 2 s counts as a collision
- [~] ~~Bumper safety stop~~ — **dropped.** Switches only; the bumper frame that would press them was never fitted
- [~] ~~Gyro heading correction~~ — **removed** (the 10 cm/m drift to the right stays a known limitation)
- [ ] Tilt the range sensors up 10 degrees — **deferred.** They see the floor, leaving 13.5 cm of warning (`hardware/bringup-log_eng.md`)
- [ ] Serial telemetry streaming, plotted from `host/`

**Exit**: roams a room for 10 minutes with no commands and ≤3 collisions.

> **Recorded as not met, and moving on.** With no bumper there is nothing to count collisions
> with, and sensors that see the floor mean frequent wall contact. Measured: 7.5 s cruising, then
> boxed into a corner and self-stopped at 17 s; 143 s of roaming on the earlier settings.
> The phase-3 baseline therefore uses **encoder forward distance and time until self-stop**
> instead of a collision count.

> **The "moves autonomously without commands" goal is already met here.**
> Autonomy does not require AI. Confirming that before moving on is what makes
> it possible to honestly judge what AI adds in phase 3.
> Phase 2 performance becomes the phase-3 baseline.

## Phase 3 — On-device Q-learning ★ the core of the project ← **current**

- [x] State discretization — 3 front bands × 3 side bands × 4 previous actions = 36 states
- [x] Q-table plus ε-greedy exploration (0.30 → 0.05, ×0.999 per step, stored with the table)
- [x] Draft reward — +30 per encoder metre forward, −0.1 per turn, −1 too close, −10 contact
- [x] EEPROM checkpoint, **verified across a reboot** (85 steps and ε 0.276 restored intact)
- [x] Host learning curve — the dashboard plots windowed mean reward live over Wi-Fi
- [x] 20 unit tests for the learner (`pio test -e native`)
- [x] **A learning run on the floor** — learning and driving both work on the robot
- [x] Compared against rule-based roaming (`a`) under one set of conditions — 2026-09-22, same
      battery and area, 122 s each: policy 7.81 m/min with 0 contacts vs rules 5.84 m/min with 1
      (**exit criterion 2 met**)
- [x] A policy-only mode (`lr`) that never explores, so the comparison is fair
- [ ] Exit criterion 1: reproduced from a blank table on three separate runs

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
