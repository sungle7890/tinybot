// tinybot - Phase 1 firmware entry point.
//
// Core split (docs/02-architecture.md):
//   core 0  control task, fixed 50 Hz, never blocks
//   core 1  Arduino loop() - serial console and telemetry printing
//
// Phase 1 exists to answer one question: are the sensors and odometry good
// enough to trust a reward signal later? Nothing here learns anything.

#include <Arduino.h>
#include <Wire.h>

#include "comms/console.h"
#include "comms/telemetry.h"
#include "config.h"
#include "control/control_loop.h"
#include "drive/encoders.h"
#include "drive/motors.h"
#include "pins.h"
#include "safety/safety.h"
#include "sense/imu.h"
#include "sense/tof.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  console::begin();
  console::printBanner();

  motors::begin();
  motors::coast();
  encoders::begin();
  safety::begin();
  telemetry::begin();

  Wire.begin(pins::kI2cSda, pins::kI2cScl, cfg::kI2cFreqHz);

  const uint8_t tofFound = tof::begin();
  const bool imuFound = imu::begin();

  Serial.printf("\nbring-up: %u/%u ToF, IMU %s\n", tofFound, tof::kCount,
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
  console::poll();

  telemetry::Sample sample;
  while (telemetry::pop(sample)) {
    telemetry::printSample(sample);
  }

  delay(2);
}
