#include "control/control_loop.h"

#include <Arduino.h>
#include <math.h>

#include "comms/telemetry.h"
#include "comms/wifi_link.h"
#include "config.h"
#include "drive/encoders.h"
#include "drive/motors.h"
#include "hal/hal.h"
#include "learn/history.h"
#include "learn/qlearn.h"
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

// Roaming state. A manoeuvre runs until its deadline so the robot commits to a
// decision instead of dithering on the threshold.
enum class Roam : uint8_t { kCruise, kTurn, kBackup };
Roam g_roam = Roam::kCruise;
uint32_t g_roamUntilMs = 0;
bool g_turnLeft = false;

uint32_t g_autoStartedMs = 0;
uint32_t g_lastCruiseMs = 0;
uint16_t g_cruiseRunTicks = 0;
const char* g_autoStopReason = nullptr;

// Session measurement, shared by roaming and learning.
Mode g_sessMode = Mode::kIdle;
uint32_t g_sessStartedMs = 0;
uint32_t g_sessEndedMs = 0;
int32_t g_sessPrevLeft = 0, g_sessPrevRight = 0;
uint32_t g_sessGeneration = 0;
int64_t g_sessForwardCounts = 0;
uint16_t g_sessContacts = 0;
uint16_t g_sessEscapes = 0;
uint32_t g_sessSteps = 0;
float g_sessRewardSum = 0.0f;
// The session mean flattens out and hides what the robot is doing now, and a
// host cannot recover the recent mean from a rounded printout. So the window
// is kept here, where the rewards are.
float g_rewardWindow[cfg::kRewardWindow] = {};
uint16_t g_rewardAt = 0;
uint16_t g_rewardFilled = 0;
uint8_t g_sessStopReason = history::kStopUnknown;

// Learning step in progress. A decision holds for cfg::kLearnStepMs, and its
// reward is only known once that time has passed.
bool g_stepActive = false;
uint8_t g_stepState = 0;
learn::Action g_stepAction = learn::kForward;
learn::Action g_prevAction = learn::kForward;
uint32_t g_stepStartedMs = 0;
int32_t g_stepStartLeft = 0, g_stepStartRight = 0;
bool g_stepContact = false;
uint32_t g_learnProgressMs = 0;
uint32_t g_lastCheckpointMs = 0;
uint32_t g_escapeBackUntilMs = 0;
uint32_t g_escapeTurnUntilMs = 0;
bool g_escapeLeft = false;

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

  // Heading is zeroed when the drive starts, so the error is the heading
  // itself. Clockwise reads negative (measured), and veering clockwise means
  // the left wheel is running ahead - the same correction the encoder
  // fallback computes, from a signal that can actually see the error.
  int16_t correction;
  if (imu::present() && imu::calibrated()) {
    correction = clampCorrection(-cfg::kGyroHeadingKp * imu::headingDeg());
  } else {
    const float divergence =
        static_cast<float>(encoders::leftCount() - encoders::rightCount());
    correction = clampCorrection(cfg::kHeadingKp * divergence);
  }

  motors::set(static_cast<int16_t>(base) - correction,
              static_cast<int16_t>(base) + correction);
  g_driveTravelled = travelled;
}

// Picks the side with more room. Out-of-range counts as open, which is what
// we want: nothing seen that way means nothing in the way.
bool moreRoomOnLeft() {
  return tof::rangeMm(tof::kLeft) >= tof::rangeMm(tof::kRight);
}

// A ToF sensor meeting a wall at a shallow angle reflects the beam away and
// reports "clear" while the robot is pressed against it. With no bumper fitted
// (cfg::kBumpersFitted) nothing else notices the contact - except that the
// world stops changing. Watch all three ranges over a window and call it a
// collision when none of them moves while something is near.
uint32_t g_windowStartMs = 0;
uint16_t g_windowMin[tof::kCount];
uint16_t g_windowMax[tof::kCount];

void resetRangeWindow(uint32_t now) {
  g_windowStartMs = now;
  for (uint8_t i = 0; i < tof::kCount; ++i) {
    g_windowMin[i] = UINT16_MAX;
    g_windowMax[i] = 0;
  }
}

bool worldStoppedChanging(uint32_t now) {
  for (uint8_t i = 0; i < tof::kCount; ++i) {
    const uint16_t mm = tof::rangeMm(static_cast<tof::Index>(i));
    if (mm < g_windowMin[i]) g_windowMin[i] = mm;
    if (mm > g_windowMax[i]) g_windowMax[i] = mm;
  }
  if (now - g_windowStartMs < cfg::kNoChangeWindowMs) return false;

  bool still = tof::rangeMm(tof::kFront) < cfg::kNoChangeNearMm;
  for (uint8_t i = 0; still && i < tof::kCount; ++i) {
    if (g_windowMax[i] - g_windowMin[i] > cfg::kNoChangeSpreadMm) still = false;
  }
  resetRangeWindow(now);
  return still;
}

void beginSession(Mode mode, uint32_t now) {
  g_sessMode = mode;
  g_sessStartedMs = now;
  g_sessEndedMs = 0;
  g_sessPrevLeft = encoders::leftCount();
  g_sessPrevRight = encoders::rightCount();
  g_sessGeneration = encoders::resetGeneration();
  g_sessForwardCounts = 0;
  g_sessContacts = 0;
  g_sessEscapes = 0;
  g_sessSteps = 0;
  g_sessRewardSum = 0.0f;
  g_rewardAt = 0;
  g_rewardFilled = 0;
  g_sessStopReason = history::kStopUnknown;
}

float recentRewardMean() {
  if (!g_rewardFilled) return 0.0f;
  float sum = 0.0f;
  for (uint16_t i = 0; i < g_rewardFilled; ++i) sum += g_rewardWindow[i];
  return sum / static_cast<float>(g_rewardFilled);
}

// Writes the run into the robot's own log. Called once, when a session ends:
// the radio drops and batteries die, so a run that is only in the host's
// scrollback is a run that can be lost.
void endSession(uint32_t now, uint8_t reason) {
  if (g_sessEndedMs != 0 || g_sessMode == Mode::kIdle) return;
  g_sessEndedMs = now;
  g_sessStopReason = reason;

  history::Run run{};
  run.mode = static_cast<uint8_t>(g_sessMode);
  run.contacts = g_sessContacts > 255 ? 255 : static_cast<uint8_t>(g_sessContacts);
  run.escapes = g_sessEscapes > 255 ? 255 : static_cast<uint8_t>(g_sessEscapes);
  run.stopReason = reason;
  run.seconds = static_cast<uint16_t>((now - g_sessStartedMs) / 1000);
  run.steps = g_sessSteps > 65535 ? 65535 : static_cast<uint16_t>(g_sessSteps);
  run.forwardMeters =
      static_cast<float>(g_sessForwardCounts) / encoders::countsPerMeter();
  run.meanReward =
      g_sessSteps ? g_sessRewardSum / static_cast<float>(g_sessSteps) : 0.0f;
  run.epsilonEnd = g_sessMode == Mode::kLearn ? learn::epsilon() : 0.0f;
  history::record(run);
}

// Per tick: count only forward travel. Turning in place nets to ~zero and
// reversing is not progress, so neither can inflate the score.
void trackSession() {
  const int32_t left = encoders::leftCount();
  const int32_t right = encoders::rightCount();
  const uint32_t generation = encoders::resetGeneration();
  if (generation != g_sessGeneration) {
    g_sessGeneration = generation;  // someone zeroed them: re-baseline
  } else {
    const int32_t mean = ((left - g_sessPrevLeft) + (right - g_sessPrevRight)) / 2;
    if (mean > 0) g_sessForwardCounts += mean;
  }
  g_sessPrevLeft = left;
  g_sessPrevRight = right;
}

// Ends a learning session from inside the control loop, and keeps what was
// learned: a session cut short by the cap is still worth checkpointing.
void finishLearn(const char* reason, uint8_t stopReason) {
  motors::brake();
  learn::requestSave();
  endSession(millis(), stopReason);
  hal::CriticalSection lock;
  g_autoStopReason = reason;
  g_mode = Mode::kIdle;
}

// Ends roaming from inside the control loop. Mirrors finishDrive: brake first,
// then take the lock only for the mode write.
void finishAuto(const char* reason, uint8_t stopReason) {
  motors::brake();
  endSession(millis(), stopReason);
  hal::CriticalSection lock;
  g_autoStopReason = reason;
  g_mode = Mode::kIdle;
}

void beginTurn(bool left, uint32_t ms) {
  g_roam = Roam::kTurn;
  g_turnLeft = left;
  g_roamUntilMs = millis() + ms;
}

// One pass of the hand-written avoidance rules. No learning here: this is the
// baseline Phase 3 has to beat (docs/04-roadmap.md).
void stepAuto(uint8_t bumped) {
  const uint32_t now = millis();

  // Escape hatches first. Everything below this point can loop forever on its
  // own terms - a turn that never finds a clear front just becomes a backup,
  // which becomes another turn - and the radio link is not a dependable stop.
  if (now - g_lastCruiseMs > cfg::kAutoStuckMs) {
    finishAuto("stuck: no clear path found, evading too long", history::kStopStuck);
    return;
  }
  if (now - g_autoStartedMs > cfg::kAutoMaxRunMs) {
    finishAuto("session time limit reached", history::kStopTimeCap);
    return;
  }

  // Zero the run up front so every path that is not cruising breaks the run by
  // doing nothing; only the cruise branch below restores it from runBefore.
  const uint16_t runBefore = g_cruiseRunTicks;
  g_cruiseRunTicks = 0;

  // A bump outranks everything: back off, then turn away from the side hit.
  if (bumped != safety::kNone) {
    safety::clear();
    g_roam = Roam::kBackup;
    g_turnLeft = (bumped & safety::kBumperRight) != 0;
    g_roamUntilMs = now + cfg::kBackupMs;
  }

  switch (g_roam) {
    case Roam::kBackup:
      // Only cruising fills the window; keep it anchored to now so a resumed
      // cruise starts measuring from scratch instead of from a stale mark.
      resetRangeWindow(now);
      motors::set(-cfg::kAutoBackDuty, -cfg::kAutoBackDuty);
      if (static_cast<int32_t>(now - g_roamUntilMs) >= 0) {
        beginTurn(g_turnLeft, cfg::kTurnMinMs * 2);
      }
      return;

    case Roam::kTurn: {
      resetRangeWindow(now);
      const int16_t duty = cfg::kAutoTurnDuty;
      motors::set(g_turnLeft ? -duty : duty, g_turnLeft ? duty : -duty);
      const bool committed = static_cast<int32_t>(now - g_roamUntilMs) >= 0;
      const bool clear = tof::fresh(tof::kFront) &&
                         tof::rangeMm(tof::kFront) > cfg::kFrontClearMm;
      if (committed && clear) g_roam = Roam::kCruise;
      // Turning forever means it is boxed in; back out and try again.
      if (static_cast<int32_t>(now - g_roamUntilMs) > static_cast<int32_t>(cfg::kTurnMaxMs)) {
        g_roam = Roam::kBackup;
        g_roamUntilMs = now + cfg::kBackupMs;
      }
      return;
    }

    case Roam::kCruise:
    default: {
      // A sensor that has stopped reporting is not "nothing ahead": stop.
      if (!tof::fresh(tof::kFront)) {
        motors::coast();
        return;
      }
      const uint16_t front = tof::rangeMm(tof::kFront);
      if (front < cfg::kFrontBlockedMm) {
        resetRangeWindow(now);
        beginTurn(moreRoomOnLeft(), cfg::kTurnMinMs);
        return;
      }
      // Cruising, front says clear, and nothing is moving: it is against
      // something the beam cannot see. Back off and turn, as if bumped.
      if (worldStoppedChanging(now)) {
        ++g_sessContacts;
        g_roam = Roam::kBackup;
        g_turnLeft = moreRoomOnLeft();
        g_roamUntilMs = now + cfg::kBackupMs;
        return;
      }
      // Ease away from a wall that is close on one side.
      int16_t bias = 0;
      if (tof::fresh(tof::kLeft) && tof::rangeMm(tof::kLeft) < cfg::kSideNearMm) {
        bias = cfg::kSteerBias;
      } else if (tof::fresh(tof::kRight) &&
                 tof::rangeMm(tof::kRight) < cfg::kSideNearMm) {
        bias = -cfg::kSteerBias;
      }
      // Steering must not park the inner wheel below the deadband; see
      // kMinMovingDuty. Give back to the inner wheel what it cannot use, so
      // the robot keeps rolling forward while it leans away from the wall.
      int16_t inner = cfg::kAutoCruiseDuty - abs(bias);
      if (inner < cfg::kMinMovingDuty) inner = cfg::kMinMovingDuty;
      const int16_t outer = cfg::kAutoCruiseDuty + abs(bias);

      // Progress is a sustained run, not a single tick: see kCruiseRunTicks.
      g_cruiseRunTicks = runBefore + 1;
      if (g_cruiseRunTicks >= cfg::kCruiseRunTicks) g_lastCruiseMs = now;
      if (bias > 0) {
        motors::set(outer, inner);  // wall on the left: lean right
      } else if (bias < 0) {
        motors::set(inner, outer);
      } else {
        motors::set(cfg::kAutoCruiseDuty, cfg::kAutoCruiseDuty);
      }
      return;
    }
  }
}

void driveAction(learn::Action action) {
  switch (action) {
    case learn::kForward:
      motors::set(cfg::kAutoCruiseDuty, cfg::kAutoCruiseDuty);
      break;
    case learn::kTurnLeft:
      motors::set(-cfg::kAutoTurnDuty, cfg::kAutoTurnDuty);
      break;
    case learn::kTurnRight:
      motors::set(cfg::kAutoTurnDuty, -cfg::kAutoTurnDuty);
      break;
    case learn::kBack:
    default:
      motors::set(-cfg::kAutoBackDuty, -cfg::kAutoBackDuty);
      break;
  }
}

bool reverses(learn::Action previous, learn::Action action) {
  switch (action) {
    case learn::kTurnLeft: return previous == learn::kTurnRight;
    case learn::kTurnRight: return previous == learn::kTurnLeft;
    case learn::kForward: return previous == learn::kBack;
    case learn::kBack: return previous == learn::kForward;
  }
  return false;
}

float rewardFor(learn::Action previous, learn::Action action, float meters,
                uint16_t frontMm, bool contact, bool stuck) {
  // Encoders count wheel turns, not travel: pinned against a wall, a slipping
  // wheel still reports progress. On a step that ended in contact the ranges
  // have already said the robot went nowhere, so forward distance earns
  // nothing - otherwise pushing into a wall pays for part of its own penalty.
  if (contact && meters > 0.0f) meters = 0.0f;
  float reward = cfg::kRewardPerMeter * meters;
  if (action == learn::kTurnLeft || action == learn::kTurnRight) reward -= cfg::kTurnCost;
  if (frontMm < cfg::kFrontBlockedMm) reward -= cfg::kNearCost;
  if (contact) reward -= cfg::kContactCost;
  if (stuck) reward -= cfg::kStuckCost;
  if (reverses(previous, action)) reward -= cfg::kReverseCost;
  return reward;
}

// One tick of learning. A decision is taken, held for kLearnStepMs, scored
// from the encoders and ranges, and fed back before the next one is taken.
// With `learning` false the same loop only follows the table: best action,
// no updates, no checkpoints - the scoring still runs so the run is measured
// on exactly the same terms.
void stepLearn(bool learning) {
  const uint32_t now = millis();

  if (now - g_sessStartedMs > cfg::kLearnMaxRunMs) {
    finishLearn("session time limit reached", history::kStopTimeCap);
    return;
  }
  // No front range means no state. Abandon the step rather than score it on
  // numbers that are not there.
  if (!tof::fresh(tof::kFront)) {
    motors::coast();
    g_stepActive = false;
    return;
  }

  // Scripted escape: not a decision, so nothing is learned from it.
  if (now < g_escapeTurnUntilMs) {
    if (now < g_escapeBackUntilMs) {
      motors::set(-cfg::kAutoBackDuty, -cfg::kAutoBackDuty);
    } else {
      const int16_t duty = cfg::kAutoTurnDuty;
      motors::set(g_escapeLeft ? -duty : duty, g_escapeLeft ? duty : -duty);
    }
    resetRangeWindow(now);
    g_learnProgressMs = now;
    return;
  }

  const uint16_t front = tof::rangeMm(tof::kFront);
  const uint16_t left = tof::rangeMm(tof::kLeft);
  const uint16_t right = tof::rangeMm(tof::kRight);

  if (!g_stepActive) {
    g_stepState = learn::encodeState(front, left, right, g_prevAction);
    g_stepAction = learning ? learn::choose(g_stepState) : learn::best(g_stepState);
    g_stepStartedMs = now;
    g_stepStartLeft = encoders::leftCount();
    g_stepStartRight = encoders::rightCount();
    g_stepContact = false;
    g_stepActive = true;
  }
  driveAction(g_stepAction);
  if (worldStoppedChanging(now)) g_stepContact = true;

  if (now - g_stepStartedMs < cfg::kLearnStepMs) return;

  // Step over: score it.
  const int32_t dCounts = ((encoders::leftCount() - g_stepStartLeft) +
                           (encoders::rightCount() - g_stepStartRight)) / 2;
  const float meters = static_cast<float>(dCounts) / encoders::countsPerMeter();
  if (meters > cfg::kLearnProgressM) g_learnProgressMs = now;
  // Decided before scoring, so the step that ran the clock out pays for it.
  const bool stuck = now - g_learnProgressMs > cfg::kLearnStuckMs;
  const float reward =
      rewardFor(g_prevAction, g_stepAction, meters, front, g_stepContact, stuck);
  const uint8_t next = learn::encodeState(front, left, right, g_stepAction);
  if (learning) learn::update(g_stepState, g_stepAction, reward, next);

  ++g_sessSteps;
  g_sessRewardSum += reward;
  g_rewardWindow[g_rewardAt] = reward;
  g_rewardAt = (g_rewardAt + 1) % cfg::kRewardWindow;
  if (g_rewardFilled < cfg::kRewardWindow) ++g_rewardFilled;
  if (g_stepContact) ++g_sessContacts;
  g_prevAction = g_stepAction;
  g_stepActive = false;

  // Wedged: free it with a scripted backup and turn, and carry on.
  if (stuck) {
    ++g_sessEscapes;
    g_escapeLeft = tof::rangeMm(tof::kLeft) >= tof::rangeMm(tof::kRight);
    g_escapeBackUntilMs = now + cfg::kLearnEscapeBackMs;
    g_escapeTurnUntilMs = g_escapeBackUntilMs + cfg::kLearnEscapeTurnMs;
    return;
  }

  if (learning && now - g_lastCheckpointMs > cfg::kLearnCheckpointMs) {
    g_lastCheckpointMs = now;
    learn::requestSave();
  }
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
  uint8_t bumpReason = 0;
  Mode stoppedSession = Mode::kIdle;
  {
    hal::CriticalSection lock;
    // Roaming handles a bump itself by backing off; every other mode stops.
    if (bumped && g_mode == Mode::kAuto) {
      bumpReason = safety::reason() & (safety::kBumperLeft | safety::kBumperRight);
      // A broken encoder is not something the rules can drive out of.
      if ((safety::reason() & ~(safety::kBumperLeft | safety::kBumperRight)) != 0) {
        stoppedSession = Mode::kAuto;
        g_mode = Mode::kSafetyStop;
      }
    } else if (bumped && g_mode != Mode::kSafetyStop) {
      if (g_mode == Mode::kAuto || g_mode == Mode::kLearn || g_mode == Mode::kPolicy) {
        stoppedSession = g_mode;
      }
      g_mode = Mode::kSafetyStop;
    }
    modeNow = g_mode;
    reqLeft = g_reqLeft;
    reqRight = g_reqRight;
  }

  // A fault that ends a session must still close it and keep what was learned;
  // a broken encoder wire is exactly when nobody is about to type `qs`.
  if (stoppedSession == Mode::kLearn) learn::requestSave();
  if (stoppedSession != Mode::kIdle) endSession(millis(), history::kStopSafety);

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
    // A session is only started over the radio or the cable, and the radio is
    // how it gets stopped. If the link drops mid-run, stop rather than drive on
    // with nothing left that can say stop.
    case Mode::kAuto:
      trackSession();
      if (wifi_link::linkLost()) {
        finishAuto("wi-fi link lost", history::kStopLinkLost);
        break;
      }
      stepAuto(bumpReason);
      break;
    case Mode::kLearn:
    case Mode::kPolicy:
      trackSession();
      if (wifi_link::linkLost()) {
        finishLearn("wi-fi link lost", history::kStopLinkLost);
        break;
      }
      stepLearn(modeNow == Mode::kLearn);
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
  const Mode was = mode();
  if (was == Mode::kLearn) learn::requestSave();
  if (was == Mode::kAuto || was == Mode::kLearn || was == Mode::kPolicy) {
    endSession(millis(), history::kStopCommand);
  }
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

bool requestAuto() {
  if (safety::tripped()) return false;
  const uint32_t now = millis();
  // Outside the lock: it reads the encoders, which take a lock of their own.
  beginSession(Mode::kAuto, now);
  hal::CriticalSection lock;
  g_roam = Roam::kCruise;
  g_roamUntilMs = now;
  g_autoStartedMs = now;
  g_lastCruiseMs = now;
  g_cruiseRunTicks = 0;
  g_autoStopReason = nullptr;
  resetRangeWindow(now);
  g_mode = Mode::kAuto;
  return true;
}

namespace {
bool startTableRun(Mode mode) {
  if (safety::tripped()) return false;
  const uint32_t now = millis();
  // Outside the lock: it reads the encoders, which take a lock of their own.
  beginSession(mode, now);
  hal::CriticalSection lock;
  g_stepActive = false;
  g_prevAction = learn::kForward;
  g_learnProgressMs = now;
  g_lastCheckpointMs = now;
  g_escapeBackUntilMs = 0;
  g_escapeTurnUntilMs = 0;
  g_autoStopReason = nullptr;
  resetRangeWindow(now);
  g_mode = mode;
  return true;
}
}  // namespace

bool requestLearn() { return startTableRun(Mode::kLearn); }
bool requestPolicy() { return startTableRun(Mode::kPolicy); }

SessionStats sessionStats() {
  const uint32_t now = millis();
  SessionStats s{};
  hal::CriticalSection lock;
  s.mode = g_sessMode;
  s.running = g_sessMode != Mode::kIdle && g_sessEndedMs == 0;
  s.elapsedMs = (s.running ? now : g_sessEndedMs) - g_sessStartedMs;
  s.forwardMeters = static_cast<float>(g_sessForwardCounts) / encoders::countsPerMeter();
  s.contacts = g_sessContacts;
  s.escapes = g_sessEscapes;
  s.learnSteps = g_sessSteps;
  s.meanReward = g_sessSteps ? g_sessRewardSum / static_cast<float>(g_sessSteps) : 0.0f;
  s.recentReward = recentRewardMean();
  return s;
}

const char* autoStopReason() { return g_autoStopReason; }

RoamStatus roamStatus() {
  const uint32_t now = millis();
  RoamStatus s{};
  switch (g_roam) {
    case Roam::kTurn: s.state = "turn"; break;
    case Roam::kBackup: s.state = "backup"; break;
    default: s.state = "cruise"; break;
  }
  s.cruiseRunTicks = g_cruiseRunTicks;
  s.sinceProgressMs = now - g_lastCruiseMs;
  s.elapsedMs = now - g_autoStartedMs;
  return s;
}

bool requestDriveDistance(float meters) {
  if (safety::tripped()) return false;
  if (!(fabsf(meters) > kArrivedMeters)) return false;

  encoders::reset();
  imu::zeroHeading();  // the line to hold is wherever it points now
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
    case Mode::kAuto: return "auto";
    case Mode::kLearn: return "learn";
    case Mode::kPolicy: return "policy";
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
