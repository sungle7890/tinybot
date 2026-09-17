#pragma once
// Minimal stand-in for the Arduino core, for host-side tests only. Anything
// the tests need to control (time, randomness) is defined by the test itself.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

uint32_t millis();
uint32_t micros();
long random(long howBig);
long random(long howSmall, long howBig);
void randomSeed(unsigned long seed);
