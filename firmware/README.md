# firmware — Phase 1 (하드웨어 브링업)

> English: [README_eng.md](README_eng.md)

학습 코드는 여기에 없다.
Phase 1의 목적은 하나다 — **나중에 보상 신호로 믿을 만한 센서·오도메트리인가?**

## 두 개의 타깃

플랫폼 차이는 전부 `src/hal/` 뒤에 있다. 그 위는 공유된다.

| 환경 | 보드 | 상태 | 제어 루프 |
|---|---|---|---|
| `uno_r4_wifi` | Arduino UNO R4 WiFi | **기본. 현재 쓰는 것** | 협조적 (`micros()` 기준, `loop()`에서 구동) |
| `esp32s3` | ESP32-S3-DevKitC-1 | Phase 4용 보류 | 코어 0에 핀된 RTOS 태스크 |

```bash
pio run                      # 기본 타깃(R4) 빌드
pio run -e esp32s3           # ESP32-S3 빌드
pio run -t upload            # 업로드
pio device monitor           # 콘솔 (115200)
```

> **⚠️ Apple Silicon 주의.** `renesas-ra` 플랫폼의 기본 툴체인(1.70201.0)은
> macOS x86_64 바이너리뿐이라 `Bad CPU type in executable`로 죽는다.
> `platformio.ini`에서 arm64 빌드가 있는 `~1.100301.0`으로 고정해뒀다.

## 실행 모델

| | UNO R4 WiFi | ESP32-S3 |
|---|---|---|
| 코어 | 1개 | 2개 |
| 제어 스텝 | `loop()`에서 협조적 구동 | 코어 0 전용 RTOS 태스크 |
| 콘솔·텔레메트리 | 같은 `loop()` | 코어 1 |
| 결과 | **`loop()`를 막으면 틱이 밀린다** | 시리얼이 제어를 못 건드림 |

R4는 단일 코어라 물리적 분리가 불가능하다. 대신 **`loop()` 안의 모든 것이
논블로킹**이어야 한다 — 텔레메트리 출력은 `Serial.availableForWrite()`를 먼저
확인하고, 버퍼가 모자라면 그 줄을 **버린다**(드롭 카운트로 기록).

텔레메트리 유실은 허용, 제어 틱 지연은 불허 —
[docs/02-architecture.md](../docs/02-architecture.md) 설계 규칙 1.

> **협조적 스케줄링이 ±2 ms 예산을 지키는지는 실측해야 안다.**
> `j` 명령이 판정한다. 못 지키면 `hal_r4.cpp`의 `controlLoopBegin`을
> FspTimer 기반으로 바꾸면 되고, 그 파일 밖은 건드릴 필요가 없다.

## 콘솔 명령

| 명령 | 동작 |
|---|---|
| `?` | 도움말 |
| `st` | 상태 (모드, 센서 유무, 캘리브레이션, 루프) |
| `m <l> <r>` | 좌우 duty 직접 지정, −1000..1000 |
| `f <duty>` | 양쪽 동일 duty |
| `s` / `b` | 정지(coast) / 브레이크 |
| `e` / `z` | 엔코더 값 읽기 / 0으로 리셋 |
| `t` / `i` | ToF / IMU 읽기 |
| `j` / `jz` | 루프 지터 통계 / 리셋 |
| `d <mm>` | 엔코더 폐루프 직진 |
| `cal <mm>` | `d` 직후 실측값을 알려주면 counts/m 재계산 → NVS 저장 |
| `cpm <v>` | counts/m 직접 지정 |
| `v` | 텔레메트리 CSV 스트림 토글 |
| `c` | 래치된 안전 정지 해제 |

## 브링업 순서

**순서를 지킬 것.** 모터 전원을 먼저 넣으면 배선 실수가 부품을 태운다.

### 1. 보드만 (모터 전원 OFF)
```
pio run -t upload && pio device monitor
```
`st`로 ToF 3개와 IMU가 잡히는지 확인. 안 잡히면 I2C 배선과 XSHUT 핀부터.

> R4는 센서를 **Qwiic(`Wire1`, 3.3 V)**으로 물린다. Adafruit ToF/IMU는
> STEMMA QT라 데이지체인만 하면 되고 I2C 납땜이 없다. **Qwiic에 5 V를 넣으면
> 보드가 망가진다.** XSHUT 3가닥은 여전히 A0/A1/A2로 개별 배선해야 한다.

### 2. 센서 단독 확인
`t` — 손을 대었다 떼며 값이 변하는지. 범위 밖은 8190 mm로 나온다.
`i` — 보드를 기울이면 accel이 변하고, 툭 치면 shock이 튀는지.

### 3. 모터 — **바퀴를 공중에 띄운 상태로**
모터 전원 ON. `f 300` → 양쪽이 **전진** 방향으로 도는지 확인.
반대로 돌면 그 쪽 모터 배선 두 가닥을 바꾼다. 코드를 고치지 말 것.
`s`로 정지.

### 4. 엔코더 방향
`z` 후 바퀴를 손으로 전진 방향으로 굴린다. `e`에서 **양쪽 모두 증가**해야 한다.
한쪽이 감소하면 그 쪽 엔코더 A/B 두 가닥을 바꾼다.
(`src/drive/encoders.cpp`의 우측 ISR은 반대 방향 장착을 이미 반영해 두었다.)

### 5. 오도메트리 캘리브레이션 — Phase 1 관문 ①
바닥에 내려놓고 출발선 표시.
```
d 1000          # 1 m 직진
                # 실제 이동거리를 자로 측정
cal 970         # 예: 970 mm 나왔으면
```
`counts/m`이 재계산되어 NVS에 저장된다. 오차 ±5 % 이내가 될 때까지 반복.

> 여기서 대충 넘어가면 Phase 3에서 보상이 거짓말을 한다.
> [docs/03-hardware-bom.md](../docs/03-hardware-bom.md)의 "엔코더가 필수인 이유" 참조.

### 6. 루프 지터 — Phase 1 관문 ②
```
jz              # 통계 리셋
v               # 텔레메트리 켜서 부하를 준 상태로
                # 60초 이상 방치
j
```
`PHASE 1 GATE : PASS`가 떠야 한다 (overruns 0, ±2 ms 이내).

## 아직 검증되지 않은 것

**컴파일만 확인된 상태다. 실기 검증 0.** 하드웨어가 없으므로 당연하다.
특히 다음은 실측 전까지 **추측값**이다.

| 항목 | 위치 | 비고 |
|---|---|---|
| 핀 맵 | `include/pins.h` | 보드 실물과 대조 필요 |
| `kDutyDeadband` = 120 | `include/config.h` | 섀시마다 다름. 3단계에서 확인 |
| `kDefaultCountsPerMeter` = 3000 | `include/config.h` | 순수 자리표시자. `cal`로 덮어쓸 것 |
| `kHeadingKp` = 1.2 | `include/config.h` | 진동하면 낮출 것 |
| IMU 충돌 임계값 0.6 g | `src/sense/imu.cpp` | 실제 충돌 데이터로 조정 |
| 우측 엔코더 부호 반전 | `src/drive/encoders.cpp` | 4번 단계에서 확인 |
| **R4 루프 지터** | `src/hal/hal_r4.cpp` | 협조적 스케줄링. `j`로 실측 필요 |
| **R4 PWM 주파수** | `src/hal/hal_r4.cpp` | R4 코어가 주파수 제어를 노출하지 않아 가청 대역. 모터 소음 예상 |
| R4 인터럽트 핀 D2/D3 | `include/pins_r4.h` | 공식 문서 기준. 실측 확인 |
