#include "sense/tof.h"

#include <Arduino.h>
#include <VL53L0X.h>

#include "config.h"
#include "hal/hal.h"
#include "pins.h"

namespace tof {
namespace {

struct Slot {
  uint8_t xshutPin;
  uint8_t i2cAddress;
  const char* name;
};

constexpr Slot kSlots[kCount] = {
    {pins::kTofXshutFront, 0x30, "front"},
    {pins::kTofXshutLeft, 0x31, "left"},
    {pins::kTofXshutRight, 0x32, "right"},
};

VL53L0X g_sensors[kCount];
bool g_present[kCount] = {false, false, false};
uint16_t g_rangeMm[kCount] = {cfg::kTofOutOfRangeMm, cfg::kTofOutOfRangeMm,
                              cfg::kTofOutOfRangeMm};
uint32_t g_lastReadMs[kCount] = {0, 0, 0};

// VL53L0X RESULT_INTERRUPT_STATUS: low three bits are the data-ready flags.
bool measurementReady(VL53L0X& sensor) {
  return (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

}  // namespace

uint8_t begin() {
  // Hold every sensor in reset first, otherwise the ones still at 0x29 answer
  // over the top of the one being configured.
  for (uint8_t i = 0; i < kCount; ++i) {
    pinMode(kSlots[i].xshutPin, OUTPUT);
    digitalWrite(kSlots[i].xshutPin, LOW);
  }
  delay(10);

  uint8_t found = 0;
  for (uint8_t i = 0; i < kCount; ++i) {
    digitalWrite(kSlots[i].xshutPin, HIGH);
    delay(10);  // boot time before the sensor answers on 0x29

    g_sensors[i].setBus(&hal::i2c());
    g_sensors[i].setTimeout(100);
    if (!g_sensors[i].init()) {
      // Put it back in reset so a half-initialised sensor cannot squat on 0x29.
      digitalWrite(kSlots[i].xshutPin, LOW);
      continue;
    }

    g_sensors[i].setAddress(kSlots[i].i2cAddress);
    g_sensors[i].setMeasurementTimingBudget(cfg::kTofTimingBudgetUs);
    g_sensors[i].startContinuous();  // back-to-back, as fast as the budget allows

    g_present[i] = true;
    g_lastReadMs[i] = millis();
    ++found;
  }
  return found;
}

void update() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < kCount; ++i) {
    if (!g_present[i]) continue;
    if (!measurementReady(g_sensors[i])) continue;  // no new data; keep the old

    const uint16_t mm = g_sensors[i].readRangeContinuousMillimeters();
    if (g_sensors[i].timeoutOccurred()) continue;

    g_rangeMm[i] = mm;
    g_lastReadMs[i] = now;
  }
}

bool present(Index i) { return i < kCount && g_present[i]; }
uint16_t rangeMm(Index i) { return i < kCount ? g_rangeMm[i] : cfg::kTofOutOfRangeMm; }

uint32_t ageMs(Index i) {
  if (i >= kCount) return UINT32_MAX;
  return millis() - g_lastReadMs[i];
}

bool fresh(Index i) { return present(i) && ageMs(i) <= cfg::kSensorStaleMs; }

const char* name(Index i) { return i < kCount ? kSlots[i].name : "?"; }

}  // namespace tof
