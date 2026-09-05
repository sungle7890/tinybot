#include "drive/encoders.h"

#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>

#include "config.h"
#include "pins.h"

namespace encoders {
namespace {

constexpr char kNvsNamespace[] = "tinybot";
constexpr char kNvsKeyCpm[] = "enc_cpm";

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

volatile int32_t g_leftCount = 0;
volatile int32_t g_rightCount = 0;

float g_countsPerMeter = cfg::kDefaultCountsPerMeter;

// gpio_get_level() is IRAM-resident; digitalRead() is not guaranteed to be.
inline int level(uint8_t pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin));
}

void IRAM_ATTR onLeftEdge() {
  const int delta = (level(pins::kLeftEncA) == level(pins::kLeftEncB)) ? 1 : -1;
  portENTER_CRITICAL_ISR(&g_mux);
  g_leftCount += delta;
  portEXIT_CRITICAL_ISR(&g_mux);
}

void IRAM_ATTR onRightEdge() {
  // Mirrored: the right motor faces the opposite way on the chassis, so the
  // same physical forward motion produces the opposite phase relationship.
  const int delta = (level(pins::kRightEncA) == level(pins::kRightEncB)) ? -1 : 1;
  portENTER_CRITICAL_ISR(&g_mux);
  g_rightCount += delta;
  portEXIT_CRITICAL_ISR(&g_mux);
}

}  // namespace

void begin() {
  pinMode(pins::kLeftEncA, INPUT_PULLUP);
  pinMode(pins::kLeftEncB, INPUT_PULLUP);
  pinMode(pins::kRightEncA, INPUT_PULLUP);
  pinMode(pins::kRightEncB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pins::kLeftEncA), onLeftEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::kRightEncA), onRightEdge, CHANGE);

  Preferences prefs;
  if (prefs.begin(kNvsNamespace, /*readOnly=*/true)) {
    g_countsPerMeter =
        prefs.getFloat(kNvsKeyCpm, cfg::kDefaultCountsPerMeter);
    prefs.end();
  }
  if (!(g_countsPerMeter > 1.0f)) {
    g_countsPerMeter = cfg::kDefaultCountsPerMeter;
  }
}

int32_t leftCount() {
  portENTER_CRITICAL(&g_mux);
  const int32_t value = g_leftCount;
  portEXIT_CRITICAL(&g_mux);
  return value;
}

int32_t rightCount() {
  portENTER_CRITICAL(&g_mux);
  const int32_t value = g_rightCount;
  portEXIT_CRITICAL(&g_mux);
  return value;
}

void reset() {
  portENTER_CRITICAL(&g_mux);
  g_leftCount = 0;
  g_rightCount = 0;
  portEXIT_CRITICAL(&g_mux);
}

float leftMeters() { return static_cast<float>(leftCount()) / g_countsPerMeter; }
float rightMeters() { return static_cast<float>(rightCount()) / g_countsPerMeter; }
float meters() { return 0.5f * (leftMeters() + rightMeters()); }

float countsPerMeter() { return g_countsPerMeter; }

bool setCountsPerMeter(float value) {
  if (!(value > 1.0f) || value > 1.0e6f) return false;
  g_countsPerMeter = value;

  Preferences prefs;
  if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) return false;
  const bool ok = prefs.putFloat(kNvsKeyCpm, value) > 0;
  prefs.end();
  return ok;
}

bool calibrateFrom(float commandedMeters, float measuredMeters) {
  if (!(commandedMeters > 0.0f) || !(measuredMeters > 0.0f)) return false;
  // The encoders reported commandedMeters worth of counts while the robot
  // actually travelled measuredMeters, so counts-per-meter was off by exactly
  // that ratio.
  return setCountsPerMeter(g_countsPerMeter * commandedMeters / measuredMeters);
}

}  // namespace encoders
