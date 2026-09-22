# progress — 주행 영상

> English: [README_eng.md](README_eng.md)

로봇이 실제로 어떻게 움직였는지 담은 영상. 숫자로는 안 보이는 것들이 여기 남는다 —
움찔거림, 벽에 붙어 버티는 모습, 코너에서 헤매는 모습.

원본(1080×1920, 합계 175 MB)은 `progress/originals/`에 두고 저장소에는 넣지 않는다.
여기 있는 것은 360×640·20 fps·소리 없음으로 줄인 것이다 (합계 12 MB). 줄이는 명령:

```bash
ffmpeg -i 원본.mp4 -vf "scale=-2:640,fps=20" -c:v libx264 -preset medium -crf 32 \
       -pix_fmt yuv420p -movflags +faststart -an 결과.mp4
```

| 파일 | 모드 | 대응하는 주행 기록 |
|---|---|---|
| `rule-based.mp4` | 규칙 기반 (`a`) | 이력 10번 — 122초, 11.88 m, 5.84 m/분, 접촉 1 |
| `learning.mp4` | 학습 (`l`) | 이력 8번 — 122초, 12.47 m, 6.13 m/분, 접촉 0, 갇힘 2 |
| `learn-based.mp4` | 배운 대로 (`lr`) | 이력 9번 — 122초, 15.88 m, 7.81 m/분, 접촉 0, 갇힘 0 |

세 주행은 2026-09-22에 같은 배터리·같은 구역에서 연달아 찍었다. 학습이 규칙 기반을 처음
넘어선 그 비교다 — [hardware/bringup-log.md](../hardware/bringup-log.md) 참조.

**대응 관계는 영상 길이로 추정한 것이다.** 다르면 이 표를 고칠 것.
