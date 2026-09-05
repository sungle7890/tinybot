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
// 32 states × 4 actions × int8 = 128 bytes. Runs on an Uno.
Q[s][a] += alpha * (reward + gamma * max_a2(Q[s2][a2]) - Q[s][a]);
```

- State: distance-sensor bucket × left/right difference × previous action
- Actions: forward / turn left / turn right / reverse
- Reward: collision −10, forward +1, spinning in place −0.1 (draft, needs tuning)
- Persistence: checkpoint the Q-table to EEPROM/Flash so learning survives power cycles

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
