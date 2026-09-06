#include "comms/telemetry.h"

#include <Arduino.h>
#include <stdio.h>

#include "config.h"
#include "hal/ring_buffer.h"

namespace telemetry {
namespace {

hal::RingBuffer<Sample, cfg::kTelemetryQueueLen> g_queue;
uint32_t g_dropped = 0;
bool g_enabled = false;

}  // namespace

void begin() {}

bool push(const Sample& sample) {
  if (!g_enabled) return false;
  if (g_queue.push(sample)) return true;
  ++g_dropped;
  return false;
}

bool pop(Sample& out) { return g_queue.pop(out); }

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

// Writes only when the UART buffer can take the whole line without blocking.
// A blocking write here would stall loop(), and on the R4 loop() is what drives
// the control tick - dropping a telemetry line is much cheaper than slipping a
// control period.
bool printSample(const Sample& s) {
  char line[cfg::kTelemetryLineMax];
  const int len =
      snprintf(line, sizeof(line), "%lu,%lu,%ld,%ld,%u,%u,%u,%d,%d,%d,%d,%u,%u\n",
               static_cast<unsigned long>(s.seq),
               static_cast<unsigned long>(s.tickUs),
               static_cast<long>(s.encLeft), static_cast<long>(s.encRight),
               s.tofFront, s.tofLeft, s.tofRight, s.dutyLeft, s.dutyRight,
               s.shockMilliG, s.yawRateDps, s.mode, s.safetyReason);
  if (len <= 0) return false;

  if (Serial.availableForWrite() < len) {
    ++g_dropped;
    return false;
  }
  Serial.write(line, static_cast<size_t>(len));
  return true;
}

}  // namespace telemetry
