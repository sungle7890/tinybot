#include "control/control_loop.h"

#include <Arduino.h>
#include <math.h>

#include "comms/telemetry.h"
#include "config.h"
#include "drive/encoders.h"
#include "drive/motors.h"
#include "hal/hal.h"
#include "safety/safety.h"
#include "sense/imu.h"
#include "sense/tof.h"

namespace control {
namespace {

// Distance over which the drive ramps down to the deadband, so the robot stops
// on the target instead of coasting past it.
constexpr float kApproachMeters = 0.15f;
constexpr float kArrivedMeters = 0.005f;

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

uint32_t g_lastTickUs = 0;
uint32_t g_seq = 0;

int32_t g_prevLeft = 0, g_prevRight = 0;
uint16_t g_stillTicks = 0;
uint32_t g_seenResetGeneration = 0;

void recordTick(uint32_t periodUs) {
  ++g_ticks;
  if (periodUs < g_minUs) g_minUs = periodUs;
  if (periodUs > g_maxUs) g_maxUs = periodUs;

  const double delta = static_cast<double>(periodUs) - g_mean;
  g_mean += delta / static_cast<double>(g_ticks);
  g_m2 += delta * (static_cast<double>(periodUs) - g_mean);

  const int32_t error = static_cast<int32_t>(periodUs) -
                        static_cast<int32_t>(cfg::kLoopPeriodMs * 1000);
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

// Watches the encoders for readings that cannot be real motion, so a loose
// wire cannot make the robot drive away: a drive that believes it has gone
// backwards keeps commanding forward until it times out. Faults latch through
// safety::, which the step below turns into a stop.
void checkEncoders(int32_t left, int32_t right, Mode mode) {
  // A zeroing moves the counts by however far the robot had travelled, which
  // is indistinguishable from a fault by size alone. Re-baseline and skip.
  const uint32_t generation = encoders::resetGeneration();
  if (generation != g_seenResetGeneration) {
    g_seenResetGeneration = generation;
    g_prevLeft = left;
    g_prevRight = right;
    g_stillTicks = 0;
    return;
  }

  const int32_t dLeft = left - g_prevLeft;
  const int32_t dRight = right - g_prevRight;
  g_prevLeft = left;
  g_prevRight = right;

  if (abs(dLeft) > cfg::kMaxCountsPerTick || abs(dRight) > cfg::kMaxCountsPerTick) {
    safety::raise(safety::kEncoderFault);
    return;
  }

  const bool driving = (mode == Mode::kManual || mode == Mode::kDriveDistance) &&
                       (abs(motors::leftDuty()) > cfg::kDutyDeadband ||
                        abs(motors::rightDuty()) > cfg::kDutyDeadband);
  if (driving && abs(dLeft) <= cfg::kStallCountsPerTick &&
      abs(dRight) <= cfg::kStallCountsPerTick) {
    if (++g_stillTicks >= cfg::kStallTicks) safety::raise(safety::kStalled);
  } else {
    g_stillTicks = 0;
  }

  // Only meaningful while driving straight; a turn diverges on purpose.
  if (mode == Mode::kDriveDistance &&
      abs(left - right) > cfg::kDriveMismatchCounts) {
    safety::raise(safety::kEncoderFault);
  }
}

// Ends a drive. Separate so the mode write is the only thing under the lock.
void finishDrive(float travelled) {
  motors::brake();
  hal::CriticalSection lock;
  g_driveTravelled = travelled;
  g_mode = Mode::kIdle;
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

  const float ramp = fminf(1.0f, fabsf(remaining) / kApproachMeters);
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

// One control tick. Invoked by hal::controlLoopBegin's scheduler - as a pinned
// RTOS task on the ESP32, cooperatively from loop() on the R4.
void step() {
  const uint32_t nowUs = micros();
  const uint32_t periodUs = nowUs - g_lastTickUs;
  g_lastTickUs = nowUs;

  safety::poll();
  checkEncoders(encoders::leftCount(), encoders::rightCount(), mode());
  tof::update();
  imu::update();

  const bool bumped = safety::tripped();

  // Snapshot under the lock, act outside it. Motor and I2C calls take locks of
  // their own; running them inside a critical section risks deadlock and
  // stretches the window with interrupts disabled.
  Mode modeNow;
  int16_t reqLeft;
  int16_t reqRight;
  {
    hal::CriticalSection lock;
    if (bumped && g_mode != Mode::kSafetyStop) g_mode = Mode::kSafetyStop;
    modeNow = g_mode;
    reqLeft = g_reqLeft;
    reqRight = g_reqRight;
  }

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

  // Skip the very first tick: g_lastTickUs was seeded before the loop started,
  // so its period is meaningless and would poison min/mean.
  if (g_seq > 0) {
    hal::CriticalSection lock;
    recordTick(periodUs);
  }

  telemetry::Sample sample{};
  sample.seq = g_seq++;
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

}  // namespace

void begin() {
  g_lastTickUs = micros();
  hal::controlLoopBegin(step, cfg::kLoopHz);
}

void requestIdle() {
  // Read the latch before taking the lock: safety:: takes a lock of its own,
  // and nesting critical sections is a deadlock waiting to happen.
  const bool bumped = safety::tripped();
  hal::CriticalSection lock;
  g_mode = bumped ? Mode::kSafetyStop : Mode::kIdle;
  g_reqLeft = 0;
  g_reqRight = 0;
}

void requestManual(int16_t leftDuty, int16_t rightDuty) {
  if (safety::tripped()) return;
  hal::CriticalSection lock;
  g_reqLeft = leftDuty;
  g_reqRight = rightDuty;
  g_mode = Mode::kManual;
}

bool requestDriveDistance(float meters) {
  if (safety::tripped()) return false;
  if (!(fabsf(meters) > kArrivedMeters)) return false;

  encoders::reset();
  hal::CriticalSection lock;
  g_driveCommanded = meters;
  g_driveTravelled = 0.0f;
  g_driveStartedMs = millis();
  g_mode = Mode::kDriveDistance;
  return true;
}

Mode mode() {
  hal::CriticalSection lock;
  return g_mode;
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
  hal::CriticalSection lock;
  out.ticks = g_ticks;
  out.minUs = g_ticks ? g_minUs : 0;
  out.maxUs = g_maxUs;
  out.meanUs = static_cast<float>(g_mean);
  out.stdevUs =
      g_ticks > 1 ? sqrtf(static_cast<float>(g_m2 / (g_ticks - 1))) : 0.0f;
  out.overruns = g_overruns;
  return out;
}

void resetStats() {
  hal::CriticalSection lock;
  g_ticks = 0;
  g_minUs = UINT32_MAX;
  g_maxUs = 0;
  g_mean = 0.0;
  g_m2 = 0.0;
  g_overruns = 0;
}

}  // namespace control
