#include "control/control_loop.h"

#include <Arduino.h>
#include <math.h>

#include "comms/telemetry.h"
#include "config.h"
#include "drive/encoders.h"
#include "drive/motors.h"
#include "safety/safety.h"
#include "sense/imu.h"
#include "sense/tof.h"

namespace control {
namespace {

// Distance over which the drive ramps down to the deadband, so the robot stops
// on the target instead of coasting past it.
constexpr float kApproachMeters = 0.15f;
constexpr float kArrivedMeters = 0.005f;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

Mode g_mode = Mode::kIdle;
int16_t g_reqLeft = 0;
int16_t g_reqRight = 0;

float g_driveCommanded = 0.0f;
float g_driveTravelled = 0.0f;
uint32_t g_driveStartedMs = 0;

// Welford accumulators for the tick period.
uint32_t g_ticks = 0;
uint32_t g_minUs = UINT32_MAX;
uint32_t g_maxUs = 0;
double g_mean = 0.0;
double g_m2 = 0.0;
uint32_t g_overruns = 0;

void recordTick(uint32_t periodUs) {
  ++g_ticks;
  if (periodUs < g_minUs) g_minUs = periodUs;
  if (periodUs > g_maxUs) g_maxUs = periodUs;

  const double delta = static_cast<double>(periodUs) - g_mean;
  g_mean += delta / static_cast<double>(g_ticks);
  g_m2 += delta * (static_cast<double>(periodUs) - g_mean);

  const int32_t error =
      static_cast<int32_t>(periodUs) - static_cast<int32_t>(cfg::kLoopPeriodMs * 1000);
  if (error > static_cast<int32_t>(cfg::kJitterBudgetUs) ||
      error < -static_cast<int32_t>(cfg::kJitterBudgetUs)) {
    ++g_overruns;
  }
}

int16_t clampCorrection(float value) {
  if (value > cfg::kHeadingCorrMax) return cfg::kHeadingCorrMax;
  if (value < -cfg::kHeadingCorrMax) return -cfg::kHeadingCorrMax;
  return static_cast<int16_t>(value);
}

// Ends a drive. Separate so the mode write is the only thing under the lock.
void finishDrive(float travelled) {
  motors::brake();
  portENTER_CRITICAL(&g_mux);
  g_driveTravelled = travelled;
  g_mode = Mode::kIdle;
  portEXIT_CRITICAL(&g_mux);
}

// Straight-line drive with a proportional heading hold on encoder divergence.
// This is a calibration aid, not the robot's real locomotion.
void stepDriveDistance() {
  const float travelled = encoders::meters();
  const float remaining = g_driveCommanded - travelled;

  const bool timedOut = millis() - g_driveStartedMs > cfg::kDriveTimeoutMs;
  if (fabsf(remaining) <= kArrivedMeters || timedOut) {
    finishDrive(travelled);
    return;
  }

  const float ramp =
      fminf(1.0f, fabsf(remaining) / kApproachMeters);
  const float direction = remaining > 0.0f ? 1.0f : -1.0f;
  const float base = direction * ramp * cfg::kDriveDuty;

  // Positive divergence means the left wheel ran ahead, so slow it down.
  const float divergence =
      static_cast<float>(encoders::leftCount() - encoders::rightCount());
  const int16_t correction = clampCorrection(cfg::kHeadingKp * divergence);

  motors::set(static_cast<int16_t>(base) - correction,
              static_cast<int16_t>(base) + correction);
  g_driveTravelled = travelled;
}


void controlTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(cfg::kLoopPeriodMs);

  uint32_t lastTickUs = micros();
  uint32_t seq = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    const uint32_t nowUs = micros();
    const uint32_t periodUs = nowUs - lastTickUs;
    lastTickUs = nowUs;

    tof::update();
    imu::update();

    // Latch a bumper trip into the mode before deciding what to do. safety::
    // takes its own spinlock, so it is read outside ours.
    const bool bumped = safety::tripped();

    // Snapshot under the lock, act outside it. Motor and I2C calls take locks
    // of their own; running them inside a critical section risks deadlock and
    // stretches the window with interrupts disabled.
    Mode modeNow;
    int16_t reqLeft;
    int16_t reqRight;
    portENTER_CRITICAL(&g_mux);
    if (bumped && g_mode != Mode::kSafetyStop) g_mode = Mode::kSafetyStop;
    modeNow = g_mode;
    reqLeft = g_reqLeft;
    reqRight = g_reqRight;
    portEXIT_CRITICAL(&g_mux);

    switch (modeNow) {
      case Mode::kIdle:
        motors::coast();
        break;
      case Mode::kManual:
        motors::set(reqLeft, reqRight);
        break;
      case Mode::kDriveDistance:
        stepDriveDistance();  // may call finishDrive() and change the mode
        break;
      case Mode::kSafetyStop:
        motors::brake();
        break;
    }

    // Skip the very first tick: lastTickUs was seeded before the loop, so its
    // period is meaningless and would poison min/mean.
    if (seq > 0) {
      portENTER_CRITICAL(&g_mux);
      recordTick(periodUs);
      portEXIT_CRITICAL(&g_mux);
    }

    telemetry::Sample sample{};
    sample.seq = seq++;
    sample.tickUs = periodUs;
    sample.encLeft = encoders::leftCount();
    sample.encRight = encoders::rightCount();
    sample.tofFront = tof::rangeMm(tof::kFront);
    sample.tofLeft = tof::rangeMm(tof::kLeft);
    sample.tofRight = tof::rangeMm(tof::kRight);
    sample.dutyLeft = motors::leftDuty();
    sample.dutyRight = motors::rightDuty();
    sample.shockMilliG = static_cast<int16_t>(imu::shock() * 1000.0f);
    sample.yawRateDps = static_cast<int16_t>(imu::yawRateDps());
    sample.mode = static_cast<uint8_t>(mode());
    sample.safetyReason = safety::reason();
    telemetry::push(sample);
  }
}

}  // namespace

void begin() {
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                          configMAX_PRIORITIES - 2, nullptr, cfg::kControlCore);
}

void requestIdle() {
  portENTER_CRITICAL(&g_mux);
  g_mode = safety::tripped() ? Mode::kSafetyStop : Mode::kIdle;
  g_reqLeft = 0;
  g_reqRight = 0;
  portEXIT_CRITICAL(&g_mux);
}

void requestManual(int16_t leftDuty, int16_t rightDuty) {
  if (safety::tripped()) return;
  portENTER_CRITICAL(&g_mux);
  g_reqLeft = leftDuty;
  g_reqRight = rightDuty;
  g_mode = Mode::kManual;
  portEXIT_CRITICAL(&g_mux);
}

bool requestDriveDistance(float meters) {
  if (safety::tripped()) return false;
  if (!(fabsf(meters) > kArrivedMeters)) return false;

  encoders::reset();
  portENTER_CRITICAL(&g_mux);
  g_driveCommanded = meters;
  g_driveTravelled = 0.0f;
  g_driveStartedMs = millis();
  g_mode = Mode::kDriveDistance;
  portEXIT_CRITICAL(&g_mux);
  return true;
}

Mode mode() {
  portENTER_CRITICAL(&g_mux);
  const Mode value = g_mode;
  portEXIT_CRITICAL(&g_mux);
  return value;
}

const char* modeName(Mode m) {
  switch (m) {
    case Mode::kIdle: return "idle";
    case Mode::kManual: return "manual";
    case Mode::kDriveDistance: return "drive";
    case Mode::kSafetyStop: return "SAFETY-STOP";
  }
  return "?";
}

bool driveActive() { return mode() == Mode::kDriveDistance; }
float driveCommandedMeters() { return g_driveCommanded; }
float driveTravelledMeters() { return g_driveTravelled; }

LoopStats stats() {
  LoopStats out{};
  portENTER_CRITICAL(&g_mux);
  out.ticks = g_ticks;
  out.minUs = g_ticks ? g_minUs : 0;
  out.maxUs = g_maxUs;
  out.meanUs = static_cast<float>(g_mean);
  out.stdevUs =
      g_ticks > 1 ? sqrtf(static_cast<float>(g_m2 / (g_ticks - 1))) : 0.0f;
  out.overruns = g_overruns;
  portEXIT_CRITICAL(&g_mux);
  return out;
}

void resetStats() {
  portENTER_CRITICAL(&g_mux);
  g_ticks = 0;
  g_minUs = UINT32_MAX;
  g_maxUs = 0;
  g_mean = 0.0;
  g_m2 = 0.0;
  g_overruns = 0;
  portEXIT_CRITICAL(&g_mux);
}

}  // namespace control
