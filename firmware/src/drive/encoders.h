#pragma once
#include <stdint.h>

// Quadrature encoders, 2x decoded: interrupt on both edges of channel A,
// channel B sampled in the ISR for direction.
//
// Counts-per-meter is NOT a constant you can look up - gearbox tolerance and
// wheel diameter make it chassis-specific. Calibrate it (console `cal`) and it
// is stored in NVS. Everything downstream depends on this number being right;
// see docs/03-hardware-bom.md on why encoders are mandatory.
namespace encoders {

void begin();

int32_t leftCount();
int32_t rightCount();
void reset();

float leftMeters();
float rightMeters();
float meters();  // mean of both sides

float countsPerMeter();
// Persists to NVS. Returns false if the value is not usable.
bool setCountsPerMeter(float value);

// Rescales the stored calibration from a commanded vs. measured distance.
bool calibrateFrom(float commandedMeters, float measuredMeters);

}  // namespace encoders
