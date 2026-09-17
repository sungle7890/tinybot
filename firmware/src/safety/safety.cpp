#include "safety/safety.h"

#include <Arduino.h>

#include "config.h"
#include "hal/hal.h"
#include "pins.h"

namespace safety {
namespace {

constexpr uint32_t kDebounceMs = 30;

volatile uint8_t g_reason = kNone;
volatile uint32_t g_trippedAtMs = 0;
uint32_t g_lastEdgeMs = 0;

void latch(uint8_t bit, uint32_t now) {
  hal::CriticalSection lock;
  if (g_reason == kNone) g_trippedAtMs = now;
  g_reason |= bit;
}

}  // namespace

void begin() {
  if (!cfg::kBumpersFitted) return;
  pinMode(pins::kBumperLeft, INPUT_PULLUP);
  pinMode(pins::kBumperRight, INPUT_PULLUP);
}

// Called once per control tick. The bumpers are polled rather than
// interrupt-driven because the UNO R4 WiFi only exposes two external-interrupt
// pins and the encoders need both. Polling is sound here: a chassis pressed
// against an obstacle holds the switch closed for far longer than one 20 ms
// tick, so no contact is missed. The latch below still makes the trip sticky.
void poll() {
  if (!cfg::kBumpersFitted) return;
  const uint32_t now = millis();
  if (now - g_lastEdgeMs < kDebounceMs) return;

  const bool left = digitalRead(pins::kBumperLeft) == LOW;
  const bool right = digitalRead(pins::kBumperRight) == LOW;
  if (!left && !right) return;

  g_lastEdgeMs = now;
  if (left) latch(kBumperLeft, now);
  if (right) latch(kBumperRight, now);
}

void raise(uint8_t bit) { latch(bit, millis()); }

bool tripped() { return reason() != kNone; }

uint8_t reason() {
  hal::CriticalSection lock;
  return g_reason;
}

uint32_t trippedAtMs() {
  hal::CriticalSection lock;
  return g_trippedAtMs;
}

void clear() {
  hal::CriticalSection lock;
  g_reason = kNone;
}

}  // namespace safety
