#pragma once
#include <stdint.h>

namespace cfg {

// --- Control loop -----------------------------------------------------------
// 50 Hz matches the reference design in docs/01-microduck-analysis.md.
constexpr uint32_t kLoopHz       = 50;
constexpr uint32_t kLoopPeriodMs = 1000 / kLoopHz;  // 20 ms
constexpr int      kControlCore  = 0;  // ESP32 only; the R4 has one core

// Phase 1 exit criterion: jitter within +/-2 ms of the period.
constexpr uint32_t kJitterBudgetUs = 2000;

// --- Board options ----------------------------------------------------------
#if defined(TINYBOT_PLATFORM_R4)
// Qwiic is a second I2C bus (Wire1) at 3.3 V. Using it costs no header pins,
// which is what lets the pin map fit. Set false to fall back to the 5 V header
// bus on A4/A5 - and then move kBumperRight off A4 (see pins_r4.h).
constexpr bool kUseQwiic = true;
#endif

// --- Motors -----------------------------------------------------------------
// Duty is a platform-independent fraction; hal::pwmWrite hides the resolution.
constexpr int16_t kDutyMax = 1000;   // public API range is -1000..+1000

// The pack drives the motors directly, so duty sets their average voltage:
// average = supply x duty. The Romi's 120:1 mini plastic gearmotors (Pololu
// 1520) are rated 4.5 V and specified to run comfortably over 3-6 V, while six
// 1.5 V cells give 9 V, so full duty would be half again over the rating -
// hotter motors, faster brush wear, and a stall current that climbs with it.
// kDutyCeiling caps every motor command so the average stays within
// kMotorMaxVolts. Re-measure kSupplyVolts at the driver and it re-derives.
constexpr float kSupplyVolts = 9.0f;
constexpr float kMotorMaxVolts = 6.0f;

constexpr int16_t dutyCeilingFor(float supply, float motorMax) {
  return supply <= motorMax
             ? kDutyMax
             : static_cast<int16_t>(static_cast<float>(kDutyMax) * motorMax / supply);
}
constexpr int16_t kDutyCeiling = dutyCeilingFor(kSupplyVolts, kMotorMaxVolts);

// Below this the motors whine without turning. Measured per chassis; the
// default is a starting guess to be corrected during bring-up.
constexpr int16_t kDutyDeadband = 120;

// --- Encoders ---------------------------------------------------------------
// Romi encoders give 12 counts per motor revolution counting both edges of both
// channels. This firmware decodes 2x (both edges of channel A only), so:
//   6 counts/motor-rev x 120:1 gearbox = 720 per wheel revolution
//   70 mm wheel -> 0.2199 m circumference
//   720 / 0.2199 = 3274 counts/m
// A computed starting point, not a measurement. Run `cal` and overwrite it.
constexpr float kDefaultCountsPerMeter = 3274.0f;

// --- Encoder sanity ---------------------------------------------------------
// The motors top out near 200 rpm at the wheel, which is about 48 counts per
// 20 ms tick, so anything far above that is noise on a floating input rather
// than motion - exactly what a disconnected encoder ground produces.
constexpr int32_t kMaxCountsPerTick = 150;

// Duty applied but nothing turning: either the robot is stalled against
// something or an encoder signal has come loose. Both want the motors off.
constexpr uint16_t kStallTicks = 25;          // 0.5 s at 50 Hz
constexpr int32_t kStallCountsPerTick = 2;

// During a straight drive the wheels track each other closely. This much
// divergence (about 12 cm) means one side is not being measured.
constexpr int32_t kDriveMismatchCounts = 400;

// --- Straight-line drive (calibration aid) ----------------------------------
constexpr int16_t  kDriveDuty      = 400;   // gentle; calibration is not a race
constexpr float    kHeadingKp      = 1.2f;  // duty per count of L/R divergence
constexpr int16_t  kHeadingCorrMax = 250;
constexpr uint32_t kDriveTimeoutMs = 15000;

// --- Rule-based roaming (Phase 2) -------------------------------------------
// Deliberately simple: this is the baseline that Phase 3's learned policy has
// to beat, so it must be honest hand-written behaviour, not a tuned showpiece.
// TEMPORARY, 2026-09-16. The three ToF sensors are mounted level about 40 mm
// up, so the lower edge of their 25-degree cone lands on the floor and every
// one of them reports 177-235 mm in an empty room. With the real thresholds
// the robot is permanently "blocked" and only ever spins in place - measured,
// not theorised. Until the sensors are tilted up ~10 degrees the thresholds
// have to duck *under* the floor reading, which leaves barely 50 mm of warning,
// so the robot also has to crawl. This is a stopgap that mostly hands obstacle
// avoidance to the bumpers; it is not the Phase 3 baseline.
//
// Once the mounting is fixed, restore:
//   kAutoCruiseDuty 320, kAutoTurnDuty 300, kAutoBackDuty 280, kSteerBias 120,
//   kFrontBlockedMm 220, kFrontClearMm 320, kSideNearMm 200.
constexpr int16_t  kAutoCruiseDuty = 190;
constexpr int16_t  kAutoTurnDuty   = 250;   // still has to break static friction
constexpr int16_t  kAutoBackDuty   = 200;

constexpr uint16_t kFrontBlockedMm = 135;   // turn away below this
constexpr uint16_t kFrontClearMm   = 155;   // resume cruising above this
constexpr uint16_t kSideNearMm     = 130;   // steer away from a close wall
constexpr int16_t  kSteerBias      = 70;    // duty difference while steering

// Steering subtracts kSteerBias from the inner wheel, and a wheel below the
// deadband does not turn at all: at cruise 190 and bias 70 the inner wheel sat
// on exactly kDutyDeadband, so every "ease away from the wall" became a pivot
// on one wheel - which is how the robot ended up pushing itself along a wall.
// Whatever the duties are, the inner wheel has to keep driving.
constexpr int16_t kMinMovingDuty = kDutyDeadband + 40;

// The bumper switches are wired but have no bumper: the mechanical frame that
// would press them was never fitted, so they cannot fire and the robot has no
// contact sensing at all. Polling them only risks a spurious latch.
constexpr bool kBumpersFitted = false;

// Contact sensing has to come from the ranges instead, and a ToF sensor goes
// blind on a wall it meets at a shallow angle - the beam reflects away, so the
// robot reads "clear" while pressed against it. What gives it away is that the
// world stops changing: if all three ranges sit inside a narrow band for this
// long while cruising, the robot is not going anywhere. Only trusted when
// something is near; down an open corridor every range is saturated and still.
constexpr uint32_t kNoChangeWindowMs = 2000;
constexpr uint16_t kNoChangeSpreadMm = 25;
constexpr uint16_t kNoChangeNearMm   = 400;

constexpr uint32_t kBackupMs = 600;         // after a bump
constexpr uint32_t kTurnMinMs = 350;        // so a turn always commits
constexpr uint32_t kTurnMaxMs = 1600;       // give up and back off instead

// Escape hatches. Roaming runs untethered, and the only other way to stop the
// robot is to pull its battery, so the firmware has to give up on its own.
// kAutoStuckMs: evading this long without a single cruising tick means the
// rules are not working here (boxed in, or a sensor lying); stop and say so.
// kAutoMaxRunMs: a session cap, so a lost radio link cannot mean "runs forever".
// kCruiseRunTicks: a single cruising tick is not progress. While wedged, a
// sweeping turn keeps catching one-tick glimpses of a gap, and counting those
// resets the watchdog forever. Only an uninterrupted run counts (0.5 s @ 50 Hz).
constexpr uint32_t kAutoStuckMs     = 10000;
constexpr uint32_t kAutoMaxRunMs    = 180000;
constexpr uint16_t kCruiseRunTicks  = kLoopHz / 2;

// --- On-device Q-learning (Phase 3) -----------------------------------------
// Same sensors, same duties, same contact detector as roaming, so the two are
// compared on the robot's behaviour and not on different hardware settings.
//
// State = front band x side band x previous action = 3 x 3 x 4 = 36.
// kLearnFarMm only means something once the sensors are tilted; while they see
// the floor at ~180 mm almost everything lands in the middle band, and the
// learner can only tell "very close" from "not very close". That is a limit of
// the robot, not of the algorithm - suspect it first if learning looks weak.
constexpr uint16_t kLearnNearMm = kFrontBlockedMm;
constexpr uint16_t kLearnFarMm  = 300;

// One decision holds for this long. Shorter and a single action cannot move
// the robot far enough to change its reward; longer and it drives into walls
// before it can react.
constexpr uint32_t kLearnStepMs = 200;

constexpr float kLearnAlpha = 0.2f;   // learning rate
constexpr float kLearnGamma = 0.9f;   // discount: ~10 steps (2 s) of foresight
// Exploration starts high and decays per step; it is stored with the table, so
// a robot that has already learned does not go back to flailing after reboot.
constexpr float kEpsilonStart = 0.30f;
constexpr float kEpsilonMin   = 0.05f;
constexpr float kEpsilonDecay = 0.999f;  // ~halves every 700 steps (~2.3 min)

// Reward draft - to be tuned against measured runs, not guessed further.
// Forward progress pays per metre from the encoders, so reversing costs the
// same automatically. Turning is cheap but not free, or spinning in place
// becomes a way to never be near anything. Contact is the only big number.
constexpr float kRewardPerMeter = 30.0f;
constexpr float kTurnCost       = 0.1f;
constexpr float kNearCost       = 1.0f;
constexpr float kContactCost    = 10.0f;
// Paid by the step that runs the stuck clock out (cfg::kAutoStuckMs without
// progress). Without it, dithering in a corner cost only the turn cost per
// step - about -5 over ten seconds - which is cheaper than one contact. The
// first rule-vs-learning comparison showed exactly that trade: 0 contacts but
// 7 escapes, and 22 % less distance than the rule-based run.
constexpr float kStuckCost      = 5.0f;
// Paid for undoing the previous step: left after right, right after left, back
// after forward and forward after back. Nothing charged for it before, and the
// table found a loop - in open space it learned right after forward, left after
// right, forward after left - so the robot twitched in place every 200 ms,
// on video and in `q` alike. A reversal stops the motors and spins them the
// other way for nothing; now it costs what it wastes.
constexpr float kReverseCost    = 0.5f;

// Learning runs are long; the session cap is ten minutes instead of three.
// Getting stuck does not end a learning session: a scripted backup-and-turn
// frees the robot and learning continues, because a learner that is switched
// off every time it fails never sees what follows its worst decisions.
constexpr uint32_t kLearnMaxRunMs    = 600000;
// Its own stuck threshold, separate from kAutoStuckMs: that one is when the
// rules give up and stop, and moving it would move the baseline the learner
// is measured against. A policy run wedged in a corner spent ten seconds
// there before anything freed it - seven times in one three-minute run.
constexpr uint32_t kLearnStuckMs     = 4000;
constexpr float    kLearnProgressM   = 0.02f;   // a step that moved this far
constexpr uint32_t kLearnEscapeBackMs = 600;
constexpr uint32_t kLearnEscapeTurnMs = 700;

// Checkpoint cadence. The R4 writes EEPROM from the same thread as the control
// loop, so the motors coast for the duration of a save; `q` reports how long
// that actually took. Rare enough to keep data-flash wear negligible.
constexpr uint32_t kLearnCheckpointMs = 30000;

// How many recent decisions the "is it getting better?" number averages over.
// 50 steps is 10 s of driving: long enough to survive one unlucky turn, short
// enough that an improvement shows up while the run is still going.
constexpr uint16_t kRewardWindow = 50;

// --- Sensors ----------------------------------------------------------------
constexpr uint32_t kI2cFreqHz         = 400000;
constexpr uint32_t kTofTimingBudgetUs = 20000;  // one measurement per loop tick
constexpr uint16_t kTofOutOfRangeMm   = 8190;   // VL53L0X saturation value
// A reading older than this is stale; the control loop must treat it as unknown
// rather than as "nothing in front of me".
constexpr uint32_t kSensorStaleMs = 200;

// --- Gyro ------------------------------------------------------------------
// A gyro at rest does not read zero; it reads its own bias, and integrating
// that bias is a heading that drifts on its own. So the bias is measured with
// the robot standing still - at boot, and on `gc` - and subtracted from then
// on. It moves with temperature, which is why it is re-measurable.
constexpr uint16_t kGyroBiasSamples = 100;   // 2 s at 50 Hz
// Above this the robot is plainly moving, so a calibration started by mistake
// says so instead of baking motion into the bias.
constexpr float kGyroStillDps = 3.0f;

// --- Serial -----------------------------------------------------------------
constexpr unsigned long kSerialBaud = 115200;

// --- Telemetry --------------------------------------------------------------
constexpr size_t kTelemetryQueueLen = 32;
constexpr size_t kTelemetryLineMax  = 128;

}  // namespace cfg
