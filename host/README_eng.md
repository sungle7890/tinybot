# host — PC-side tools

> Korean: [README.md](README.md)

## Firmware over Wi-Fi (`ota/make_ota.py`)

Updates the firmware without a cable. **The first time still needs USB** — the robot has to be
running firmware that has the OTA command in it.

```bash
cd firmware && pio run -e uno_r4_wifi && cd ..
python3 host/ota/make_ota.py firmware/.pio/build/uno_r4_wifi/firmware.bin --serve
```

Send the robot the command it prints (`ota http://<computer IP>:8765/tinybot.ota`), from the
dashboard console or a browser address bar (`/api/cmd?c=ota%20http://...`).

- The robot only takes it **while idle**: the transfer blocks the control loop
- On success the robot reboots; the `firmware : built ...` time in `st` changes
- On failure the `ota` line in `st` says which step failed. USB can always reflash it
- If macOS asks whether python may accept incoming connections, allow it, or the robot cannot fetch the file

The format is Arduino's `.ota` (LZSS-compressed, board id, CRC). The compressed output matches
Arduino's own encoder byte for byte, and Arduino's decoder restores the firmware exactly.

## Telemetry dashboard (`dashboard/index.html`)

A web page showing the board's 50 Hz telemetry live. No install. Two ways in:

- **USB** — the browser reads USB serial directly (Web Serial, Chrome/Edge)
- **Wi-Fi** — enter the robot's IP and press **Connect Wi-Fi**. For watching untethered
  driving and learning. Any browser works, but a page opened over `https://` cannot
  reach the robot (http)

**Shows**: distances from above (front, left, right rays); tiles for range,
shock, rotation, tick period, encoders and mode; three 10-second charts (range /
shock with the impact threshold / tick period with the ±2 ms budget band); a
command console; CSV recording; a **session and learning panel** (tiles for forward
distance, contacts, escapes, learning steps and ε, plus a mean-reward-per-step curve).

### Over Wi-Fi

1. Open the file as is: `open host/dashboard/index.html`
2. Type the robot's IP in the top field (e.g. `192.168.1.223`, printed on USB serial at
   boot) → **Connect Wi-Fi**
3. The firmware switches telemetry on by itself, and off again 3 s after requests stop
4. The session panel refreshes from `st` every 2 s; each point on the learning curve is
   the mean reward of the decisions taken in those 2 s

Requests go out one at a time — the R4's Wi-Fi module wedges under parallel connections
(found in Phase 1). A command reply occasionally gets lost; the console then shows
"(no reply)" and resending works. Polling raises control-loop overruns, so **do not
connect over Wi-Fi while measuring jitter.**

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
