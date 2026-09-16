#pragma once
#include <stdint.h>

// Bumpers. Per docs/02-architecture.md design rule 3 these preempt the control
// loop - they are never policy inputs, so no learned behaviour can ever choose
// to ignore them.
namespace safety {

enum Reason : uint8_t {
  kNone = 0,
  kBumperLeft = 1 << 0,
  kBumperRight = 1 << 1,
  kEncoderFault = 1 << 2,   // impossible counts, or one side not counting
  kStalled = 1 << 3,        // driving, but nothing is turning
};

void begin();

// Sample the switches. Called once per control tick.
void poll();

// Latch a fault found elsewhere (the control loop watching the encoders).
void raise(uint8_t reason);

// Latched: stays set until clear(), so a bounce-length contact cannot be missed
// between two loop ticks.
bool tripped();
uint8_t reason();
uint32_t trippedAtMs();
void clear();

}  // namespace safety
