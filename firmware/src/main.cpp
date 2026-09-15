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
// Phase 1 exists to answer one question: are the sensors and odometry good
// enough to trust a reward signal later? Nothing here learns anything.

#include <Arduino.h>

#include "comms/console.h"
#include "comms/fmt.h"
#include "comms/telemetry.h"
#include "config.h"
#include "control/control_loop.h"
#include "drive/encoders.h"
#include "drive/motors.h"
#include "hal/hal.h"
#include "safety/safety.h"
#include "sense/imu.h"
#include "sense/tof.h"

void setup() {
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

  control::begin();
  console::printStatus();
}

void loop() {
  // Serviced first and every iteration: on the R4 this is what runs the
  // control step, so anything that delays it shows up as loop jitter.
  hal::controlLoopService();

  console::poll();

  telemetry::Sample sample;
  if (telemetry::pop(sample)) {
    telemetry::printSample(sample);
  }
}
