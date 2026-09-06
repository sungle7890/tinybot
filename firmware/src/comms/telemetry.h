#pragma once
#include <stdint.h>

// One record per control tick, handed from the control step to the console
// through a lock-free ring buffer. The control step never blocks on this: if
// the buffer is full the sample is dropped and counted (docs/02-architecture.md
// design rule 1). Dropped telemetry is acceptable; a late control tick is not.
namespace telemetry {

struct Sample {
  uint32_t seq;
  uint32_t tickUs;      // measured period of the tick that produced this
  int32_t encLeft;
  int32_t encRight;
  uint16_t tofFront;
  uint16_t tofLeft;
  uint16_t tofRight;
  int16_t dutyLeft;
  int16_t dutyRight;
  int16_t shockMilliG;
  int16_t yawRateDps;
  uint8_t mode;
  uint8_t safetyReason;
};

void begin();

// Called from the control step. Non-blocking; returns false when dropped.
bool push(const Sample& sample);

// Called from loop(). Returns false when nothing is waiting.
bool pop(Sample& out);

uint32_t dropped();

void setEnabled(bool on);
bool enabled();

void printHeader();
// Non-blocking: skips (and counts) the line if the UART buffer is too full.
bool printSample(const Sample& sample);

}  // namespace telemetry
