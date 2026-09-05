# 01. How Microduck actually runs AI

> Korean: [01-microduck-analysis.md](01-microduck-analysis.md)

Sources: [pollen-robotics.com/microduck](https://pollen-robotics.com/microduck/),
[github.com/pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)

## The correction that matters

**Microduck is not a product that runs AI on a microcontroller.**
There is no Arduino and no secondary MCU anywhere in the design. A single
Linux-capable ARM SoC drives the servo bus directly and handles everything.

This changes our premise. The takeaway is not "AI runs on tiny boards like that"
but "a small **Linux computer** runs inference on pre-trained policies."

## Measured specs

| Item | Value |
|---|---|
| Compute | Rockchip RK3566 (ARM SoC, Linux) |
| Size / weight | 25 cm / 800 g |
| Actuators | 15 servos on a servo bus |
| Sensors | Camera, 8×8 ToF depth sensor, 2× IMU |
| Control loop | 50 Hz onboard policy loop |
| Policy format | ONNX |
| Firmware language | Rust (single workspace, several daemons) |
| Training | MuJoCo + PPO (separate `microduck_rl` repo) |
| License | Apache-2.0; all 7 shipped policies published and retrainable |
| Price | $399 introductory, before tax and shipping |

## Software structure

Daemons talk over a JSON-RPC contract on Unix sockets.

| Daemon | Role |
|---|---|
| `robotd` | Motor control loop, servo bus management |
| `updaterd` | Signed release install and rollback |
| `configd` | WiFi / identity configuration |
| `btd` / `padd` | Bluetooth / gamepad input |
| `mediad` | WebRTC camera streaming |
| `tofd` | Depth sensor |

## Pipeline

```
Train with PPO in MuJoCo simulation
        ↓
Export to ONNX
        ↓
Deploy to hardware → inference only, at 50 Hz
```

It is **sim-to-real**. No backpropagation runs on the robot.
Walking, sit/stand, kicking, ground pick, roller-skating and self-recovery are
all neural policies produced this way.

## What we take from it

- **The sim-to-real pipeline.** Trial and error on real hardware breaks hardware.
  Run millions of steps in simulation, then transfer only the result.
- **A fixed-rate policy loop.** 50 Hz is a good reference. Let the control period
  jitter and a learned policy falls apart on real hardware.
- **ONNX as the policy exchange format**, decoupling training framework from
  runtime.
- **Separated daemons.** Do not put the control loop in the same process as the
  camera or the network. Apache-2.0 means we can study the structure freely.

## What we cannot take

- 15-servo bipedal walking. That is several difficulty steps above a first
  project. Two wheels lets us focus on the learning itself.
- RK3566-class compute. Choosing that means this is a Raspberry Pi project, not
  an Arduino project — a legitimate choice, but a choice.

## Conclusion

Microduck is not evidence that "AI runs on an MCU." It is a good reference design
for **"small Linux board + pre-trained policy inference."**
And since it is fully open source, we can read and learn from the code without
buying one.
