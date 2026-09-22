# tinybot

스스로 움직이는 법을 **학습하는** 소형 로봇을 밑바닥부터 만드는 프로젝트.

> English: [README.md](README.md)

## 목표

명령 없이 자율적으로 움직이고, 시행착오를 통해 동작을 개선하며, 배운 것을
전원을 껐다 켜도 유지하는 로봇. 화려한 성능보다 **"진짜로 학습이 일어나는가"**를
검증 가능한 형태로 만드는 것이 우선.

## 이 프로젝트의 전제

MCU(아두이노급)에서 신경망을 **학습**시키는 것은 현실적이지 않다.
대신 두 가지를 조합한다.

1. **온디바이스 학습** — 표 기반 강화학습(Q-learning). 수백 바이트면 되고,
   ATmega328P에서도 실제로 돈다. "학습하는 로봇"의 요건은 이것으로 충족된다.
2. **오프디바이스 학습 → 온디바이스 추론** — 시뮬레이션/PC에서 학습한 정책을
   양자화해 보드에 올린다. Microduck을 포함한 상용 제품들이 쓰는 방식.

자세한 근거는 [docs/01-microduck-analysis.md](docs/01-microduck-analysis.md) 참조.

## 문서

| 문서 | 내용 |
|---|---|
| [01 Microduck 분석](docs/01-microduck-analysis.md) | Microduck이 실제로 어떻게 AI를 돌리는가 |
| [02 아키텍처](docs/02-architecture.md) | 2계층 구조와 그 이유 |
| [03 하드웨어 BOM](docs/03-hardware-bom.md) | 보드/부품 선택지와 함정 |
| [04 로드맵](docs/04-roadmap.md) | 5단계, 각 단계의 완료 조건 |
| [05 미결정 사항](docs/05-open-questions.md) | 먼저 정해야 하는 것들 |

## 디렉터리

```
firmware/   MCU 펌웨어 (실시간 제어 루프, 온디바이스 학습)
host/       호스트 도구 (실시간 대시보드; 이후 시뮬레이션·정책 학습)
hardware/   배선도, 섀시, 부품 실측 메모
docs/       설계 문서
```

## 상태

**Phase 3 — 온디바이스 Q-learning 진행 중.** 실제 로봇(Arduino UNO R4 WiFi)에서 돌아간다.

- **Phase 1 완료** — 50 Hz 제어 루프, 센서 3종, 엔코더 보정(1 m 오차 +1.6 %)을 실기로 검증.
  측정값은 전부 [hardware/bringup-log.md](hardware/bringup-log.md)에 있다
- **Phase 2 부분 완료** — 규칙 기반 자율 주행(`a`). 완료 조건(10분·충돌 3회 이하)은
  범퍼가 없어 측정할 수 없어 미달성으로 기록
- **Phase 3 진행 중** — 로봇이 바닥에서 직접 학습(`l`)하고, 학습 결과는 전원을 껐다 켜도 남는다.
  규칙 기반보다 나은지는 **아직 비교 전**
- 알려진 한계: 거리센서가 바닥을 보도록 달려 있어 감지 거리가 13.5 cm뿐이다
  ([원인과 측정](hardware/bringup-log.md))
- `host/dashboard/` — USB 또는 Wi-Fi로 붙는 실시간 대시보드 (학습 곡선 포함)
- 학습 로직 단위 테스트: `cd firmware && pio test -e native`

보드는 원래 ESP32-S3로 계획했지만([05 미결정 사항](docs/05-open-questions.md)), 이미 있던
UNO R4 WiFi로 진행했다. 펌웨어는 두 보드 모두 빌드된다.

## 라이선스

[MIT](LICENSE)
