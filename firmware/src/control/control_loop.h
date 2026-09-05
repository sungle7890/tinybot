#pragma once
#include <stdint.h>

// The fixed-rate control loop. Runs as its own FreeRTOS task pinned to
// cfg::kControlCore; the Arduino loop() (console, telemetry printing) stays on
// the other core so serial traffic cannot jitter the control period.
//
// Nothing in here blocks. No delay(), no Serial.print().
namespace control {

enum class Mode : uint8_t {
  kIdle = 0,           // motors coasting
  kManual = 1,         // duties set from the console
  kDriveDistance = 2,  // closed-loop straight line, for calibration
  kSafetyStop = 3,     // bumper latched; needs an explicit clear
};

struct LoopStats {
  uint32_t ticks;
  uint32_t minUs;
  uint32_t maxUs;
  float meanUs;
  float stdevUs;
  uint32_t overruns;  // ticks outside cfg::kJitterBudgetUs
};

void begin();

void requestIdle();
void requestManual(int16_t leftDuty, int16_t rightDuty);
bool requestDriveDistance(float meters);

Mode mode();
const char* modeName(Mode m);

// Result of the last kDriveDistance run.
bool driveActive();
float driveCommandedMeters();
float driveTravelledMeters();

LoopStats stats();
void resetStats();

}  // namespace control
