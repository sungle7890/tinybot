// tinybot - Phase 1 firmware entry point.
//
// Targets both the UNO R4 WiFi already on hand and the ESP32-S3 in the BOM.
// Everything platform-specific is behind src/hal/; see docs/02-architecture.md.
//
//   ESP32-S3   control loop is an RTOS task pinned to core 0; loop() owns
//              core 1 for the console and telemetry
//   UNO R4     one core, so the control step is driven cooperatively from
//              loop(). Nothing in loop() may block - telemetry printing skips
//              a line rather than waiting on the UART
//
// Phase 1 asked whether the sensors and odometry are good enough to trust a
// reward signal. Phase 3 now uses that signal: `l` learns on the robot itself.

#include <Arduino.h>

#include "comms/console.h"
#include "comms/fmt.h"
#include "comms/telemetry.h"
#include "comms/wifi_link.h"
#include "config.h"
#include "control/control_loop.h"
#include "drive/encoders.h"
#include "drive/motors.h"
#include "hal/hal.h"
#include "learn/history.h"
#include "learn/qlearn.h"
#include "safety/safety.h"
#include "sense/imu.h"
#include "sense/tof.h"

void setup() {
  // Before anything else: the flags say why the last run ended, and only the
  // first read of them is the truth.
  hal::captureResetCause();

  Serial.begin(cfg::kSerialBaud);
  delay(200);

  console::begin();
  console::printBanner();

  motors::begin();
  motors::coast();
  encoders::begin();
  safety::begin();
  telemetry::begin();

  hal::i2cBegin(cfg::kI2cFreqHz);

  const uint8_t tofFound = tof::begin();
  const bool imuFound = imu::begin();

  fmt::printf("\nbring-up: %u/%u ToF, IMU %s\n", tofFound, tof::kCount,
                imuFound ? "ok" : "MISSING");
  if (tofFound < tof::kCount || !imuFound) {
    // Not fatal on purpose: partial hardware still needs to be testable.
    Serial.println(F("WARNING: incomplete sensor set - check wiring and "
                     "hardware/wiring.md"));
  }
  Serial.println();

  // Before the control loop starts: joining a network can take seconds, and a
  // loop already ticking would record that wait as one enormous overrun.
  if (wifi_link::begin()) {
    fmt::printf("wifi          : %s at http://%s\n",
                wifi_link::isAccessPoint() ? "own network (tinybot)" : "joined",
                wifi_link::ipAddress());
  } else {
    fmt::printf("wifi          : off (no secrets.h, or the join failed)\n");
  }

  // After the Wi-Fi join, whose duration varies, so the random seed does too.
  learn::begin();
  history::begin();
  fmt::printf("q-table       : %s\n",
              learn::loadedFromCheckpoint() ? "loaded from checkpoint" : "blank");

  control::begin();
  // The robot is standing still at boot, which is exactly when the gyro bias
  // can be measured. `gc` repeats it later; the bias moves with temperature.
  imu::calibrate();
  console::printStatus();
}

void loop() {
  // Serviced first and every iteration: on the R4 this is what runs the
  // control step, so anything that delays it shows up as loop jitter.
  hal::controlLoopService();

  console::poll();
  wifi_link::service();
  // Checkpoint writes, never from inside a control tick. One byte per pass:
  // a whole-table write measured 2.8 s of frozen control loop on the R4.
  learn::service();
  history::service();

  // Only one consumer may drain the queue, or they split the samples between
  // them and each sees a fraction of the stream.
  if (!wifi_link::streamingTelemetry()) {
    telemetry::Sample sample;
    if (telemetry::pop(sample)) telemetry::printSample(sample);
  }
}
