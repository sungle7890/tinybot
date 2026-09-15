# host — PC-side tools

> Korean: [README.md](README.md)

## Telemetry dashboard (`dashboard/index.html`)

A web page showing the board's 50 Hz telemetry live. No install — the browser
reads USB serial directly (Web Serial).

**Shows**: distances from above (front, left, right rays); tiles for range,
shock, rotation, tick period, encoders and mode; three 10-second charts (range /
shock with the impact threshold / tick period with the ±2 ms budget band); a
command console; CSV recording.

### Opening it

1. **Close every other program using the serial port**, `pio device monitor`
   included. Only one program can hold the port.
2. Open it in **Chrome or Edge**. Safari and Firefox do not support Web Serial.
   ```bash
   open -a "Google Chrome" host/dashboard/index.html
   ```
3. **Connect board** → pick `UNO WiFi R4`.
4. If telemetry is off, the dashboard sends `v` to turn it on, and turns it off
   again on disconnect.

Before connecting, **example data** plays, marked as such on screen.

### If opening the file says Web Serial is unsupported

Chrome normally allows Web Serial on `file://`; if it is blocked, serve it locally:
```bash
cd host/dashboard && python3 -m http.server 8000
```
then open `http://localhost:8000` in Chrome.

### Caveats

- **Sending a console command briefly stalls the R4's control loop**, because its
  `Serial.write()` waits until transmission finishes. That is what a spike on the
  tick chart means. When measuring jitter, send nothing after `jz`.
- Telemetry reports both "distance sensor absent" and "out of range" as 8190, so
  the dashboard shows them together as "out of range or absent". Use `st` to tell.
- The 600 mg impact line is not yet validated ([bringup-log](../hardware/bringup-log_eng.md)).
