#include "safety/safety.h"

#include <Arduino.h>

#include "pins.h"

namespace safety {
namespace {

constexpr uint32_t kDebounceMs = 30;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t g_reason = kNone;
volatile uint32_t g_trippedAtMs = 0;
volatile uint32_t g_lastEdgeMs = 0;

void IRAM_ATTR latch(uint8_t bit) {
  const uint32_t now = millis();
  portENTER_CRITICAL_ISR(&g_mux);
  if (now - g_lastEdgeMs >= kDebounceMs) {
    g_lastEdgeMs = now;
    if (g_reason == kNone) g_trippedAtMs = now;
    g_reason |= bit;
  }
  portEXIT_CRITICAL_ISR(&g_mux);
}

void IRAM_ATTR onLeft() { latch(kBumperLeft); }
void IRAM_ATTR onRight() { latch(kBumperRight); }

}  // namespace

void begin() {
  pinMode(pins::kBumperLeft, INPUT_PULLUP);
  pinMode(pins::kBumperRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pins::kBumperLeft), onLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(pins::kBumperRight), onRight, FALLING);
}

bool tripped() { return reason() != kNone; }

uint8_t reason() {
  portENTER_CRITICAL(&g_mux);
  const uint8_t value = g_reason;
  portEXIT_CRITICAL(&g_mux);
  return value;
}

uint32_t trippedAtMs() {
  portENTER_CRITICAL(&g_mux);
  const uint32_t value = g_trippedAtMs;
  portEXIT_CRITICAL(&g_mux);
  return value;
}

void clear() {
  portENTER_CRITICAL(&g_mux);
  g_reason = kNone;
  portEXIT_CRITICAL(&g_mux);
}

}  // namespace safety
