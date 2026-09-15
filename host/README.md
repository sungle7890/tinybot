# host — PC 쪽 도구

> English: [README_eng.md](README_eng.md)

## 텔레메트리 대시보드 (`dashboard/index.html`)

보드의 50 Hz 텔레메트리를 실시간으로 보는 웹 페이지. 설치 없음 — 브라우저가 USB
시리얼을 직접 읽는다(Web Serial).

**보여주는 것**: 위에서 본 거리(전방·좌·우 광선), 거리·충격·회전·틱 주기·엔코더·모드
타일, 최근 10초 그래프 3개(거리 / 충격과 충돌 기준선 / 틱 주기와 ±2 ms 예산 띠),
명령 콘솔, CSV 기록.

### 여는 법

1. `pio device monitor` 등 **시리얼 포트를 쓰는 다른 프로그램을 모두 닫는다.**
   포트는 한 번에 한 프로그램만 열 수 있다.
2. **Chrome 또는 Edge**로 연다. Safari·Firefox는 Web Serial을 지원하지 않는다.
   ```bash
   open -a "Google Chrome" host/dashboard/index.html
   ```
3. **보드 연결** → 목록에서 `UNO WiFi R4` 선택.
4. 텔레메트리가 꺼져 있으면 대시보드가 알아서 `v`를 보내 켜고, 연결을 끊을 때 다시 끈다.

연결 전에는 **예시 데이터**가 돌아간다. 화면에 예시라고 표시된다.

### 파일로 열었는데 "Web Serial을 지원하지 않는다"고 나오면

Chrome은 보통 `file://`에서도 Web Serial을 허용하지만, 막히면 로컬 서버로 연다.
```bash
cd host/dashboard && python3 -m http.server 8000
```
그리고 Chrome에서 `http://localhost:8000`.

### 주의

- **콘솔 명령을 보내면 R4의 제어 루프가 잠깐 멈춘다.** R4의 `Serial.write()`는 전송이
  끝날 때까지 기다리기 때문이다. 틱 주기 그래프에 튀는 점이 생기면 그 때문이다.
  지터를 잴 때는 `jz` 이후 명령을 보내지 말 것.
- 텔레메트리에는 거리센서의 "미연결"과 "범위 밖"이 둘 다 8190으로 들어온다. 그래서
  대시보드는 둘을 구분하지 않고 "범위 밖 또는 미연결"로 보여준다. 어느 쪽인지는 `st`로 확인.
- 충돌 기준선 600 mg는 아직 검증되지 않은 값이다([bringup-log](../hardware/bringup-log.md)).
