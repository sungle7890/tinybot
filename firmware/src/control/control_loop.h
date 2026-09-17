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
  kAuto = 4,           // roaming on hand-written rules (Phase 2 baseline)
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

// Start roaming. Stops on `s`, or on a fault the rules cannot recover from.
bool requestAuto();

// Why roaming last ended, for the console to report. nullptr until it has run.
// Roaming gives up on its own because the radio link is not a reliable stop.
const char* autoStopReason();

struct RoamStatus {
  const char* state;         // cruise / turn / backup
  uint16_t cruiseRunTicks;   // consecutive cruising ticks so far
  uint32_t sinceProgressMs;  // age of the stuck watchdog
  uint32_t elapsedMs;        // age of the session cap
};
RoamStatus roamStatus();

Mode mode();
const char* modeName(Mode m);

// Result of the last kDriveDistance run.
bool driveActive();
float driveCommandedMeters();
float driveTravelledMeters();

LoopStats stats();
void resetStats();

}  // namespace control
