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
float yawRateDps();

// Deviation of the current acceleration vector from its slow-moving baseline,
// in g. Gravity cancels out, so this reads ~0 at rest at any mounting angle
// and spikes on impact.
float shock();

// shock() above cfg-independent threshold. THRESHOLD IS A GUESS until it has
// been checked against a real collision; see firmware/README.md.
bool impact();

}  // namespace imu
