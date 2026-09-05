#pragma once
#include <stdint.h>

// Three VL53L0X on one I2C bus. They all boot at 0x29, so they are released
// from reset one at a time via XSHUT and readdressed.
//
// Sensors are optional at bring-up: a missing one is reported, not fatal, so
// the rest of the robot can still be tested.
namespace tof {

enum Index : uint8_t { kFront = 0, kLeft = 1, kRight = 2, kCount = 3 };

// Returns how many sensors answered. Check present() for which.
uint8_t begin();

// Non-blocking: reads only sensors that have a measurement ready.
void update();

bool present(Index i);
uint16_t rangeMm(Index i);      // cfg::kTofOutOfRangeMm when nothing in range
uint32_t ageMs(Index i);        // since the last successful read
bool fresh(Index i);            // present, and newer than cfg::kSensorStaleMs

const char* name(Index i);

}  // namespace tof
