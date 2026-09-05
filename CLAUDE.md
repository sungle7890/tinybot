# CLAUDE.md

Guidance for Claude Code working in this repository.

## What this is

A from-scratch robotics project: a small robot that learns to move autonomously.
Not related to any other repo in `~/Workspace/repos`. See `README.md`.

## Ground rules specific to this project

- **Docs live in `docs/`, numbered, and exist in both Korean and English**
  (`NN-name.md` Korean, `NN-name_eng.md` English). Update both or neither.
- **Decisions go in `docs/05-open-questions.md`** under the decision log, with a
  date and rationale. Do not silently resolve an open question in code.
- **Phase discipline.** `docs/04-roadmap.md` defines exit criteria per phase.
  Do not start phase N+1 work while phase N's exit criteria are unmet; say so
  instead.

## Firmware conventions (once `firmware/` exists)

- PlatformIO, C++. Not Arduino IDE.
- The control loop is fixed-rate and never blocks. No `delay()` in the loop path.
- Safety interrupts (bumper, cliff) are not policy inputs. They preempt.
- Any change touching the control loop must state its effect on loop jitter.

## Host conventions (once `host/` exists)

- Python with `uv`, `pytest`, `ruff`.
- Telemetry parsing must tolerate dropped and corrupt serial frames — the link
  is expected to be lossy.

## Honesty requirements

This project's whole point is answering "is learning actually happening?"

- Report measured numbers, never "should work."
- A learning run that does not reproduce across seeds is not a result.
- If a learned policy loses to the phase-2 hand-written baseline, say that
  plainly rather than tuning until the graph looks good.
