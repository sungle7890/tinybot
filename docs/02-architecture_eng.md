# 02. Architecture

> Korean: [02-architecture.md](02-architecture.md)

## Two-tier split

```
┌─────────────────────────────────────────┐
│  Host  (PC / Raspberry Pi)               │
│  · Heavy training (PPO, simulation)      │
│  · Telemetry capture and visualization   │
│  · Policy compile → deploy to board      │
└────────────────┬────────────────────────┘
                 │ USB serial / WiFi  (non-realtime, may drop)
┌────────────────┴────────────────────────┐
│  MCU                                     │
│  · Fixed-rate control loop (50 Hz)       │
│  · Read sensors → run policy → drive     │
│  · On-device Q-learning                  │
│  · Safety stop (works if host dies)      │
└─────────────────────────────────────────┘
```

## Why split

**One MCU doing everything hits a wall.** Add a camera and memory runs out; the
WiFi stack jitters the control period. Jitter breaks a learned policy on real
hardware — which is exactly why Microduck pins its loop at 50 Hz.

**One host doing everything is unsafe.** While WiFi drops or Python pauses for
GC, the robot keeps driving into the wall. The safety stop must live on the MCU.

**It makes swaps cheap.** With the heavy parts upstairs, going from phase 3 to
phase 5 means changing the host, not the firmware.

## Contract between tiers

The MCU is the source of truth. The host is only an advisor.

| Direction | Payload | Rate |
|---|---|---|
| MCU → Host | State, action, reward, raw sensors (telemetry) | Every step |
| Host → MCU | New policy weights / Q-table, hyperparameters | Ad hoc |
| Host → MCU | Manual drive commands (debugging) | Ad hoc |

If nothing arrives from the host for N ms, the MCU stays in autonomous mode.
Loss of connection is a **normal state**, not an error.

## Two learning modes

### A. On-device (Q-learning)

```c
// 36 states × 4 actions × int16 (hundredths) + header = 304 bytes.
Q[s][a] += alpha * (reward + gamma * max_a2(Q[s2][a2]) - Q[s][a]);   // alpha 0.2, gamma 0.9
```

Implementation: `firmware/src/learn/qlearn.cpp` (pure logic, no hardware), wired to driving in
`stepLearn()` in `control_loop.cpp`. Console: `l` learn / `q` table / `qs` save / `qz` reset.

- State: front range in 3 bands (135 / 300 mm) × side (open / wall left / wall right) × previous action
  - ⚠️ While the sensors see the floor (~180 mm), the front sits in the middle band almost
    always, so all the learner can really tell apart is "very close / not". If learning looks
    weak, suspect the sensor angle first.
- Actions: forward / turn left / turn right / reverse — each held for 200 ms
- Reward (draft): +30 per encoder metre forward, −0.1 per turn, −1 front too close, −10 contact
  - With no bumper, **contact = "all three ranges moved less than 25 mm over 2 s"**
  - ⚠️ Do not trust the encoders alone — a wheel slipping against a wall still counts
    forward. A step that ended in contact therefore earns no forward credit. A full
    cross-check needs the sensor angle fixed, so the front range sees real objects.
- Exploration: ε from 0.30, ×0.999 per step, floor 0.05; stored with the table so it carries across reboots
- Stuck: after 10 s without progress a scripted backup-and-turn frees the robot and learning
  carries on (nothing is learned from the escape itself)
- Persistence: checkpoint to EEPROM every 60 s and when a session ends
  - ⚠️ The R4's EEPROM costs **about 44 ms per changed byte** (measured). Written in one go
    it froze the control loop for seconds (2.8 s; 13.5 s for the first write over blank flash).
    Checkpoints therefore go out **one changed byte per loop pass**, skipping unchanged bytes.

This alone produces a robot that moves without commands and gets better on its own.

### B. Off-device (sim-to-real)

```
Train with PPO in simulation → quantize → C array or ONNX → MCU runs inference
```

The Microduck approach. Introduce it in phases 4–5, when a neural net is
actually needed.

## Design rules

1. **The control loop never blocks.** No `delay()`; logging is non-blocking too.
2. **Compute reward on the MCU.** Delegating it to the host lets latency
   contaminate the learning signal.
3. **Put physical guards around exploration.** ε-greedy will happily walk the
   robot off a table. Cliff sensors and bumpers are hard interrupts, not
   something the policy gets to learn.
4. **Record the seed for every run.** A learning result that does not reproduce
   is not a result.
