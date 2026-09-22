# progress — driving videos

> Korean: [README.md](README.md)

What the robot actually did, which the numbers do not show: the twitching, the pushing against a
wall, the dithering in a corner.

The originals (1080×1920, 175 MB together) stay in `progress/originals/`, out of the repository.
What is here is 360×640, 20 fps, no audio - 12 MB together. To make them:

```bash
ffmpeg -i original.mp4 -vf "scale=-2:640,fps=20" -c:v libx264 -preset medium -crf 32 \
       -pix_fmt yuv420p -movflags +faststart -an out.mp4
```

| file | mode | matching run |
|---|---|---|
| `rule-based.mp4` | rule-based (`a`) | log entry 10 — 122 s, 11.88 m, 5.84 m/min, 1 contact |
| `learning.mp4` | learning (`l`) | log entry 8 — 122 s, 12.47 m, 6.13 m/min, 0 contacts, 2 stuck |
| `learn-based.mp4` | policy (`lr`) | log entry 9 — 122 s, 15.88 m, 7.81 m/min, 0 contacts, 0 stuck |

All three were filmed on 2026-09-22, back to back on one battery in one part of the room: the
comparison where learning first beat the rules - see
[hardware/bringup-log_eng.md](../hardware/bringup-log_eng.md).

**The mapping is inferred from the clip lengths.** Correct this table if it is wrong.
