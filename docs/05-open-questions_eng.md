# 05. Open questions

> Korean: [05-open-questions.md](05-open-questions.md)

Decisions needed before phase 1. Once settled, record the decision and its
rationale in this document.

## Q1. Board — how far do we go? ★ most important

| Option | What it is | Gains | Costs |
|---|---|---|---|
| **A. MCU only** (ESP32-S3) | Keep it an Arduino project throughout | Cheap; real understanding of power and real-time control | No vision; phase 5 is hard |
| **B. MCU + host PC** | MCU controls, laptop trains | A's gains plus heavy training | Depends on wireless; robot tethered to a PC |
| **C. MCU + Pi Zero 2 W** | Host rides on the robot | Fully autonomous plus vision | More cost, weight, power draw |
| **D. Buy a Microduck** | $399 | A state-of-the-art RL robot immediately | None of the building; shallower learning |

**Decided: A (ESP32-S3 only).** 2026-09-04.
B and C are deferred, not discarded. A→B→C extends on the same hardware, which is exactly why
[02-architecture_eng.md](02-architecture_eng.md) fixes the two-tier split now.
Starting at A wastes nothing on the way to C.

D is "using," not "building," so it does not match this project's goal. Since it
is open source, though, **reading the code without buying one** is strongly
recommended.

## Q2. Is vision needed?

- If no → ESP32-S3 or Nano 33 BLE Sense, either is fine
- If yes → ESP32-S3 (low resolution only) or a Pi-class board is effectively forced

**Decided: not for now.** 2026-09-04.
Phase-3 Q-learning needs no camera. Having picked the ESP32-S3 keeps the option
of a low-resolution camera open later.

## Q3. Locomotion — wheels or legs?

Microduck is a 15-servo biped. Attractive, but:

- Bipedal walking cannot be learned by trial and error on real hardware (the
  robot breaks)
- It mandates a simulator first, pulling phase 5 to the front
- Most of the time goes into mechanics and control, not learning

**Decided: 2WD wheels.** 2026-09-04.
 That keeps the focus on verifying
that learning happens. Legs can be a separate project after phase 3.

## Q4. Firmware language

| Option | Verdict |
|---|---|
| **C++ / Arduino (PlatformIO)** | Overwhelming library and reference support. The default |
| Rust (esp-hal) | Microduck's choice. Safe, but a steep embedded-ecosystem learning cost |
| MicroPython | Fast to prototype, unsuitable for a deterministic 50 Hz loop |

**Decided: PlatformIO + C++.** 2026-09-04.
Learning a new language during a first
embedded project makes it hard to tell "is this my logic or the language?" while
debugging.

## Q5. Budget ceiling

~₩100,000–120,000 for option A. The Pi Zero 2 W is deferred, so no extra cost.

**Undecided — confirm before ordering.**

---

## Decision log

| Date | Decision | Rationale |
|---|---|---|
| 2026-09-04 | Repo created, two-tier architecture adopted | A structure that is not wasted whichever board is chosen |
| 2026-09-04 | **Q1 = A: ESP32-S3 only** | Dual cores physically separate the control loop from comms. Extending to B or C keeps A's firmware intact |
| 2026-09-04 | **Q2 = camera deferred** | Not needed through phase 3. The ESP32-S3 leaves room to add one later |
| 2026-09-04 | **Q3 = 2WD wheels** | Bipeds cannot be trained by trial and error on real hardware, which would force the simulator first |
| 2026-09-04 | **Q4 = PlatformIO + C++** | Learning a new language during a first embedded project makes it impossible to isolate causes while debugging |
| 2026-09-04 | Arduino core 2.x via the official espressif32 platform | A proven combination. LEDC calls are version-guarded so core 3.x also builds |
