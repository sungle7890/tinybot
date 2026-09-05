#include "comms/telemetry.h"

#include <Arduino.h>

#include "config.h"

namespace telemetry {
namespace {

QueueHandle_t g_queue = nullptr;
volatile uint32_t g_dropped = 0;
volatile bool g_enabled = false;

}  // namespace

void begin() {
  g_queue = xQueueCreate(cfg::kTelemetryQueueLen, sizeof(Sample));
}

bool push(const Sample& sample) {
  if (g_queue == nullptr || !g_enabled) return false;
  if (xQueueSend(g_queue, &sample, 0) == pdTRUE) return true;
  ++g_dropped;
  return false;
}

bool pop(Sample& out) {
  if (g_queue == nullptr) return false;
  return xQueueReceive(g_queue, &out, 0) == pdTRUE;
}

uint32_t dropped() { return g_dropped; }

void setEnabled(bool on) {
  g_enabled = on;
  if (on) printHeader();
}

bool enabled() { return g_enabled; }

void printHeader() {
  Serial.println(F("# seq,tick_us,enc_l,enc_r,tof_f,tof_l,tof_r,duty_l,duty_r,"
                   "shock_mg,yaw_dps,mode,safety"));
}

void printSample(const Sample& s) {
  char line[128];
  snprintf(line, sizeof(line), "%lu,%lu,%ld,%ld,%u,%u,%u,%d,%d,%d,%d,%u,%u",
           static_cast<unsigned long>(s.seq),
           static_cast<unsigned long>(s.tickUs),
           static_cast<long>(s.encLeft), static_cast<long>(s.encRight),
           s.tofFront, s.tofLeft, s.tofRight, s.dutyLeft, s.dutyRight,
           s.shockMilliG, s.yawRateDps, s.mode, s.safetyReason);
  Serial.println(line);
}

}  // namespace telemetry
