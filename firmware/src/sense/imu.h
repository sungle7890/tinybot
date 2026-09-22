#pragma once
#include <stdint.h>

// Minimal MPU6050 driver - raw register reads, no external dependency.
// Only what the robot needs: acceleration for impact detection and yaw rate
// for heading.
namespace imu {

bool begin();
bool present();
uint8_t whoAmI();

void update();

float accelX();  // g
float accelY();
float accelZ();
float yawRateDps();     // bias removed once calibrated

// Heading, in degrees, integrated from the yaw rate since the last zero.
// Positive is one way round, negative the other; which is which comes from
// the measurements, not from a guess here. It drifts over minutes, which is
// fine for holding a line over a metre and not fine as a compass.
float headingDeg();
void zeroHeading();

// Measure the bias with the robot still. Takes cfg::kGyroBiasSamples ticks;
// calibrating() is true until then, and it fails if the robot was moving.
void calibrate();
bool calibrating();
bool calibrated();
float biasDps();

// Deviation of the current acceleration vector from its slow-moving baseline,
// in g. Gravity cancels out, so this reads ~0 at rest at any mounting angle
// and spikes on impact.
float shock();

// shock() above cfg-independent threshold. THRESHOLD IS A GUESS until it has
// been checked against a real collision; see firmware/README.md.
bool impact();

}  // namespace imu
