#include "drive/encoders.h"

#include <Arduino.h>

#include "config.h"
#include "hal/hal.h"
#include "pins.h"

namespace encoders {
namespace {

// Bumped whenever the layout changes, so a stale blob is ignored rather than
// read as garbage calibration.
constexpr uint32_t kPersistMagic = 0x54424B31;  // "TBK1"

struct PersistBlob {
  uint32_t magic;
  float countsPerMeter;
};

volatile int32_t g_leftCount = 0;
volatile int32_t g_rightCount = 0;

float g_countsPerMeter = cfg::kDefaultCountsPerMeter;

void HAL_ISR onLeftEdge() {
  const int delta =
      (hal::fastRead(pins::kLeftEncA) == hal::fastRead(pins::kLeftEncB)) ? 1 : -1;
  hal::CriticalSectionIsr lock;
  g_leftCount += delta;
}

void HAL_ISR onRightEdge() {
  // Mirrored: the right motor faces the opposite way on the chassis, so the
  // same physical forward motion produces the opposite phase relationship.
  const int delta =
      (hal::fastRead(pins::kRightEncA) == hal::fastRead(pins::kRightEncB)) ? -1 : 1;
  hal::CriticalSectionIsr lock;
  g_rightCount += delta;
}

}  // namespace

void begin() {
  pinMode(pins::kLeftEncA, INPUT_PULLUP);
  pinMode(pins::kLeftEncB, INPUT_PULLUP);
  pinMode(pins::kRightEncA, INPUT_PULLUP);
  pinMode(pins::kRightEncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pins::kLeftEncA), onLeftEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::kRightEncA), onRightEdge, CHANGE);

  PersistBlob blob{};
  if (hal::persistLoad(&blob, sizeof(blob)) && blob.magic == kPersistMagic &&
      blob.countsPerMeter > 1.0f) {
    g_countsPerMeter = blob.countsPerMeter;
  }
}

int32_t leftCount() {
  hal::CriticalSection lock;
  return g_leftCount;
}

int32_t rightCount() {
  hal::CriticalSection lock;
  return g_rightCount;
}

void reset() {
  hal::CriticalSection lock;
  g_leftCount = 0;
  g_rightCount = 0;
}

float leftMeters() { return static_cast<float>(leftCount()) / g_countsPerMeter; }
float rightMeters() { return static_cast<float>(rightCount()) / g_countsPerMeter; }
float meters() { return 0.5f * (leftMeters() + rightMeters()); }

float countsPerMeter() { return g_countsPerMeter; }

bool setCountsPerMeter(float value) {
  if (!(value > 1.0f) || value > 1.0e6f) return false;
  g_countsPerMeter = value;

  PersistBlob blob{kPersistMagic, value};
  return hal::persistSave(&blob, sizeof(blob));
}

bool calibrateFrom(float commandedMeters, float measuredMeters) {
  if (!(commandedMeters > 0.0f) || !(measuredMeters > 0.0f)) return false;
  // The encoders reported commandedMeters worth of counts while the robot
  // actually travelled measuredMeters, so counts-per-meter was off by exactly
  // that ratio.
  return setCountsPerMeter(g_countsPerMeter * commandedMeters / measuredMeters);
}

}  // namespace encoders
